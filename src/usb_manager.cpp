#include "usb_manager.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include <bsp/board_api.h>
#include <tusb.h>

#include "pico/util/queue.h"

static ads1298 *g_ads = nullptr;

// ==================================================
// Colas de adquisición (llenadas por el core 1, drenadas por el core 0)
// ==================================================

struct EmgChannelPairFrame {
    int32_t ch_a;
    int32_t ch_b;
};

struct EmgRldStatusFrame {
    uint8_t loff_statp;
    uint8_t loff_statn;
};

static constexpr uint DATA_QUEUE_CAPACITY = 64;
static constexpr uint RLD_QUEUE_CAPACITY  = 32;
static constexpr uint DATA_CDC_COUNT      = 4; // CDC_DATA_CH12 .. CDC_DATA_CH78

static queue_t g_data_queues[DATA_CDC_COUNT];
static queue_t g_rld_queue;

// Marcador de sincronismo al inicio de cada trama binaria de datos.
static constexpr uint8_t FRAME_SYNC_BYTE = 0xAA;

static void write_channel_be24(uint8_t *out, int32_t value)
{
    out[0] = (uint8_t)((value >> 16) & 0xFF);
    out[1] = (uint8_t)((value >> 8) & 0xFF);
    out[2] = (uint8_t)(value & 0xFF);
}

static const char *interface_name(uint8_t itf)
{
    switch (itf) {
        case CDC_DEBUG:     return "DEBUG INTERFACE (CDC0)";
        case CDC_CONFIG:    return "CONFIGURATION INTERFACE (CDC1)";
        case CDC_DATA_CH12: return "DATA STREAM CH1-CH2 (CDC2)";
        case CDC_DATA_CH34: return "DATA STREAM CH3-CH4 (CDC3)";
        case CDC_DATA_CH56: return "DATA STREAM CH5-CH6 (CDC4)";
        case CDC_DATA_CH78: return "DATA STREAM CH7-CH8 (CDC5)";
        case CDC_RLD:       return "RLD INTERFACE (CDC6)";
        default:            return "UNKNOWN CDC INTERFACE";
    }
}

static void send_interface_banner(uint8_t itf)
{
    if (!tud_cdc_n_connected(itf)) {
        return;
    }

    emg_usb_write_line(itf, "=========================================");
    emg_usb_write_line(itf, "EMG MULTICHANNEL - ADS1298");
    emg_usb_write_line(itf, interface_name(itf));
    emg_usb_write_line(itf, "=========================================");

    if (itf == CDC_CONFIG) {
        emg_usb_write_line(itf, "Comandos disponibles (ver CLAUDE.md para el detalle):");
        emg_usb_write_line(itf, "  ID / READ_ID");
        emg_usb_write_line(itf, "  RREG <addr_hex> | WREG <addr_hex> <valor_hex>");
        emg_usb_write_line(itf, "  GAIN <ch> <1|2|3|4|6|8|12>");
        emg_usb_write_line(itf, "  CHPOWER <ch> <ON|OFF>");
        emg_usb_write_line(itf, "  MODE <HR|LP>");
        emg_usb_write_line(itf, "  RLD <ON|OFF> | RLD_CH <ch> <P|N> <0|1>");
        emg_usb_write_line(itf, "  TESTSIG <ON|OFF> | TESTSIG_AMP <1|2> | TESTSIG_FREQ <SLOW|FAST|DC>");
        emg_usb_write_line(itf, "  MUX <ch> <NORMAL|SHORT|RLD_MEAS|SUPPLY|TEMP|TEST|RLD_DRP|RLD_DRN>");
        emg_usb_write_line(itf, "  LOFF_CH <ch> <P|N> <0|1>");
        emg_usb_write_line(itf, "  LOFF_MODE <DC|AC> | LOFF_TYPE <RESISTOR|CURRENT>");
        emg_usb_write_line(itf, "  LOFF_CURRENT <6|12|18|24> | LOFF_THRESHOLD <0-7>");
        emg_usb_write_line(itf, "  START | STOP");
    }
}

void emg_usb_init(ads1298 *ads_device)
{
    g_ads = ads_device;

    for (uint i = 0; i < DATA_CDC_COUNT; i++) {
        queue_init(&g_data_queues[i], sizeof(EmgChannelPairFrame), DATA_QUEUE_CAPACITY);
    }
    queue_init(&g_rld_queue, sizeof(EmgRldStatusFrame), RLD_QUEUE_CAPACITY);

    board_init();
    tusb_init();

    if (board_init_after_tusb) {
        board_init_after_tusb();
    }
}

void emg_usb_push_channel_pair(uint8_t pair_index, int32_t ch_a, int32_t ch_b)
{
    if (pair_index >= DATA_CDC_COUNT) {
        return;
    }

    EmgChannelPairFrame frame = { ch_a, ch_b };
    // No bloqueante: si la cola está llena (el host no está drenando lo
    // suficientemente rápido) se descarta la muestra más nueva en vez de
    // trabar el core de adquisición.
    queue_try_add(&g_data_queues[pair_index], &frame);
}

void emg_usb_push_rld_status(uint8_t loff_statp, uint8_t loff_statn)
{
    EmgRldStatusFrame frame = { loff_statp, loff_statn };
    queue_try_add(&g_rld_queue, &frame);
}

static void drain_data_queue(uint8_t pair_index)
{
    uint8_t itf = (uint8_t)(CDC_DATA_CH12 + pair_index);
    EmgChannelPairFrame frame;

    while (true) {
        if (!tud_cdc_n_connected(itf)) {
            // Nadie leyendo este CDC: se descarta lo pendiente sin escribir.
            if (!queue_try_remove(&g_data_queues[pair_index], &frame)) {
                break;
            }
            continue;
        }

        // No saca el elemento de la cola hasta confirmar que el FIFO de TX
        // de TinyUSB tiene lugar para la trama completa. tud_cdc_n_write()
        // trunca en silencio si no hay espacio, y una trama a medio
        // escribir corrompe el framing de todas las tramas siguientes
        // (bug real observado en hardware: veía picos de ruido absurdos en
        // la señal recibida). Mejor esperar al próximo llamado a
        // emg_usb_task() antes que escribir parcialmente.
        static constexpr uint32_t FRAME_SIZE = 7; // sync + 3 + 3 bytes
        if (tud_cdc_n_write_available(itf) < FRAME_SIZE) {
            break;
        }

        if (!queue_try_remove(&g_data_queues[pair_index], &frame)) {
            break;
        }

        uint8_t buf[FRAME_SIZE];
        buf[0] = FRAME_SYNC_BYTE;
        write_channel_be24(&buf[1], frame.ch_a);
        write_channel_be24(&buf[4], frame.ch_b);

        tud_cdc_n_write(itf, buf, sizeof(buf));
    }

    if (tud_cdc_n_connected(itf)) {
        tud_cdc_n_write_flush(itf);
    }
}

static void drain_rld_queue(void)
{
    EmgRldStatusFrame frame;

    while (true) {
        if (!tud_cdc_n_connected(CDC_RLD)) {
            if (!queue_try_remove(&g_rld_queue, &frame)) {
                break;
            }
            continue;
        }

        static constexpr uint32_t FRAME_SIZE = 3; // sync + LOFF_STATP + LOFF_STATN
        if (tud_cdc_n_write_available(CDC_RLD) < FRAME_SIZE) {
            break;
        }

        if (!queue_try_remove(&g_rld_queue, &frame)) {
            break;
        }

        uint8_t buf[FRAME_SIZE];
        buf[0] = FRAME_SYNC_BYTE;
        buf[1] = frame.loff_statp;
        buf[2] = frame.loff_statn;

        tud_cdc_n_write(CDC_RLD, buf, sizeof(buf));
    }

    if (tud_cdc_n_connected(CDC_RLD)) {
        tud_cdc_n_write_flush(CDC_RLD);
    }
}

void emg_usb_task(void)
{
    tud_task();

    for (uint8_t pair_index = 0; pair_index < DATA_CDC_COUNT; pair_index++) {
        drain_data_queue(pair_index);
    }
    drain_rld_queue();
}

void emg_usb_write(uint8_t itf, const char *msg)
{
    if (itf >= CDC_COUNT || msg == nullptr) {
        return;
    }

    if (!tud_cdc_n_connected(itf)) {
        return;
    }

    size_t len = strlen(msg);
    // Si no hay lugar para el mensaje completo en el FIFO de TX, se
    // descarta entero en vez de escribirlo a medias: una escritura parcial
    // corrompe el texto de la línea siguiente (bug real observado: líneas
    // de debug concatenadas/truncadas en ráfagas de comandos).
    if (tud_cdc_n_write_available(itf) < len) {
        return;
    }

    tud_cdc_n_write(itf, msg, len);
    tud_cdc_n_write_flush(itf);
}

void emg_usb_write_line(uint8_t itf, const char *msg)
{
    if (itf >= CDC_COUNT || msg == nullptr) {
        return;
    }

    if (!tud_cdc_n_connected(itf)) {
        return;
    }

    size_t len = strlen(msg);
    if (tud_cdc_n_write_available(itf) < len + 2) {
        return;
    }

    tud_cdc_n_write(itf, msg, len);
    tud_cdc_n_write(itf, "\r\n", 2);
    tud_cdc_n_write_flush(itf);
}

void emg_usb_write_debug(const char *msg)
{
    emg_usb_write_line(CDC_DEBUG, msg);
}

// ==================================================
// API de configuración por CDC1 (comandos de texto)
// ==================================================
// Ver CLAUDE.md para la tabla completa de comandos y a qué bit de qué
// registro del ADS1298 corresponde cada uno. Nota: el ADS1298 se deja en
// SDATAC una única vez al inicio (ver EMG.cpp) y la adquisición usa
// exclusivamente el opcode RDATA (no RDATAC); por eso estos comandos pueden
// hacer RREG/WREG directamente, sin volver a mandar SDATAC ni tener que
// restablecer ningún modo de streaming después.

static void reply(const char *msg)
{
    emg_usb_write_line(CDC_CONFIG, msg);
}

static void reply_error(const char *reason)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "ERROR: %s", reason);
    emg_usb_write_line(CDC_CONFIG, buf);
}

static bool parse_channel(const char *tok, int *ch_out)
{
    if (tok == nullptr) {
        return false;
    }

    char *end = nullptr;
    long ch = strtol(tok, &end, 10);
    if (end == tok || ch < 1 || ch > 8) {
        return false;
    }

    *ch_out = (int)ch;
    return true;
}

static void handle_config_command(char *rx)
{
    // Log de depuración: qué comando llegó por CDC1, antes de que strtok()
    // mute el buffer insertando terminadores nulos en los separadores.
    char debug_msg[80];
    snprintf(debug_msg, sizeof(debug_msg), "[CFG] comando recibido: %s", rx);
    emg_usb_write_line(CDC_DEBUG, debug_msg);

    char *cmd  = strtok(rx, " \r\n");
    char *arg1 = strtok(nullptr, " \r\n");
    char *arg2 = strtok(nullptr, " \r\n");
    char *arg3 = strtok(nullptr, " \r\n");

    if (cmd == nullptr) {
        reply("OK");
        return;
    }

    if (g_ads == nullptr) {
        reply_error("ADS1298 no inicializado");
        return;
    }

    if (strcasecmp(cmd, "ID") == 0 || strcasecmp(cmd, "READ_ID") == 0) {
        uint8_t id = g_ads->readId();

        char msg[32];
        snprintf(msg, sizeof(msg), "ID=0x%02X", id);
        reply(msg);

        char dbg[64];
        snprintf(dbg, sizeof(dbg), "[CFG] ID solicitado por CDC1: 0x%02X", id);
        emg_usb_write_line(CDC_DEBUG, dbg);

    } else if (strcasecmp(cmd, "RREG") == 0) {
        if (arg1 == nullptr) { reply_error("uso: RREG <addr_hex>"); return; }

        char *end = nullptr;
        long addr = strtol(arg1, &end, 16);
        if (end == arg1 || addr < 0 || addr > 0x1F) { reply_error("direccion invalida"); return; }

        g_ads->lock();
        uint8_t value = g_ads->readRegister((uint8_t)addr);
        g_ads->unlock();

        char buf[48];
        snprintf(buf, sizeof(buf), "REG %02lX=%02X", addr, value);
        reply(buf);

    } else if (strcasecmp(cmd, "WREG") == 0) {
        if (arg1 == nullptr || arg2 == nullptr) { reply_error("uso: WREG <addr_hex> <valor_hex>"); return; }

        char *end1 = nullptr;
        char *end2 = nullptr;
        long addr  = strtol(arg1, &end1, 16);
        long value = strtol(arg2, &end2, 16);
        if (end1 == arg1 || addr < 0 || addr > 0x1F ||
            end2 == arg2 || value < 0 || value > 0xFF) {
            reply_error("parametros invalidos");
            return;
        }

        g_ads->lock();
        g_ads->writeRegister((uint8_t)addr, (uint8_t)value);
        g_ads->unlock();

        reply("OK");

    } else if (strcasecmp(cmd, "GAIN") == 0) {
        int ch;
        if (!parse_channel(arg1, &ch) || arg2 == nullptr) {
            reply_error("uso: GAIN <ch 1-8> <1|2|3|4|6|8|12>");
            return;
        }

        char *end = nullptr;
        long gain_value = strtol(arg2, &end, 10);
        int8_t code = ads1298::gainToCode((int)gain_value);
        if (end == arg2 || code < 0) { reply_error("ganancia invalida"); return; }

        uint8_t reg = (uint8_t)(ads1298::REG_CH1SET + (ch - 1));
        uint8_t masked_value = (uint8_t)(code << ads1298::CHSET_GAIN_SHIFT);

        g_ads->lock();
        g_ads->readModifyWriteRegister(reg, ads1298::CHSET_GAIN_MASK, masked_value);
        g_ads->unlock();

        reply("OK");

    } else if (strcasecmp(cmd, "CHPOWER") == 0) {
        int ch;
        if (!parse_channel(arg1, &ch) || arg2 == nullptr) {
            reply_error("uso: CHPOWER <ch 1-8> <ON|OFF>");
            return;
        }

        bool power_on;
        if (strcasecmp(arg2, "ON") == 0) power_on = true;
        else if (strcasecmp(arg2, "OFF") == 0) power_on = false;
        else { reply_error("uso: CHPOWER <ch 1-8> <ON|OFF>"); return; }

        uint8_t reg = (uint8_t)(ads1298::REG_CH1SET + (ch - 1));

        g_ads->lock();
        uint8_t current = g_ads->readRegister(reg);
        uint8_t updated = current;
        if (power_on) {
            updated = (uint8_t)(updated & ~ads1298::CHSET_PD_BIT);
        } else {
            updated = (uint8_t)(updated | ads1298::CHSET_PD_BIT);
            // Recomendado por el datasheet: cortocircuitar la entrada al apagar el canal.
            updated = (uint8_t)((updated & ~ads1298::CHSET_MUX_MASK) | 0x01);
        }
        g_ads->writeRegister(reg, updated);
        g_ads->unlock();

        reply("OK");

    } else if (strcasecmp(cmd, "MODE") == 0) {
        if (arg1 == nullptr) { reply_error("uso: MODE <HR|LP>"); return; }

        uint8_t value;
        if (strcasecmp(arg1, "HR") == 0) value = ads1298::CONFIG1_HR_BIT;
        else if (strcasecmp(arg1, "LP") == 0) value = 0x00;
        else { reply_error("uso: MODE <HR|LP>"); return; }

        g_ads->lock();
        g_ads->readModifyWriteRegister(ads1298::REG_CONFIG1, ads1298::CONFIG1_HR_BIT, value);
        g_ads->unlock();

        reply("OK");

    } else if (strcasecmp(cmd, "RLD") == 0) {
        if (arg1 == nullptr) { reply_error("uso: RLD <ON|OFF>"); return; }

        uint8_t value;
        if (strcasecmp(arg1, "ON") == 0) value = ads1298::CONFIG3_PD_RLD_BIT;
        else if (strcasecmp(arg1, "OFF") == 0) value = 0x00;
        else { reply_error("uso: RLD <ON|OFF>"); return; }

        g_ads->lock();
        g_ads->readModifyWriteRegister(ads1298::REG_CONFIG3, ads1298::CONFIG3_PD_RLD_BIT, value);
        g_ads->unlock();

        reply("OK");

    } else if (strcasecmp(cmd, "RLD_CH") == 0) {
        int ch;
        if (!parse_channel(arg1, &ch) || arg2 == nullptr || arg3 == nullptr) {
            reply_error("uso: RLD_CH <ch 1-8> <P|N> <0|1>");
            return;
        }

        uint8_t reg;
        if (strcasecmp(arg2, "P") == 0) reg = ads1298::REG_RLD_SENSP;
        else if (strcasecmp(arg2, "N") == 0) reg = ads1298::REG_RLD_SENSN;
        else { reply_error("uso: RLD_CH <ch 1-8> <P|N> <0|1>"); return; }

        bool enable = (strcmp(arg3, "1") == 0);
        uint8_t bit = (uint8_t)(1 << (ch - 1));

        g_ads->lock();
        g_ads->readModifyWriteRegister(reg, bit, (uint8_t)(enable ? bit : 0x00));
        g_ads->unlock();

        reply("OK");

    } else if (strcasecmp(cmd, "TESTSIG") == 0) {
        if (arg1 == nullptr) { reply_error("uso: TESTSIG <ON|OFF>"); return; }

        uint8_t value;
        if (strcasecmp(arg1, "ON") == 0) value = ads1298::CONFIG2_INT_TEST_BIT;
        else if (strcasecmp(arg1, "OFF") == 0) value = 0x00;
        else { reply_error("uso: TESTSIG <ON|OFF>"); return; }

        g_ads->lock();
        g_ads->readModifyWriteRegister(ads1298::REG_CONFIG2, ads1298::CONFIG2_INT_TEST_BIT, value);
        g_ads->unlock();

        reply("OK");

    } else if (strcasecmp(cmd, "TESTSIG_AMP") == 0) {
        if (arg1 == nullptr) { reply_error("uso: TESTSIG_AMP <1|2>"); return; }

        uint8_t value;
        if (strcmp(arg1, "1") == 0) value = 0x00;
        else if (strcmp(arg1, "2") == 0) value = ads1298::CONFIG2_TEST_AMP_BIT;
        else { reply_error("uso: TESTSIG_AMP <1|2>"); return; }

        g_ads->lock();
        g_ads->readModifyWriteRegister(ads1298::REG_CONFIG2, ads1298::CONFIG2_TEST_AMP_BIT, value);
        g_ads->unlock();

        reply("OK");

    } else if (strcasecmp(cmd, "TESTSIG_FREQ") == 0) {
        if (arg1 == nullptr) { reply_error("uso: TESTSIG_FREQ <SLOW|FAST|DC>"); return; }

        uint8_t value;
        if (strcasecmp(arg1, "SLOW") == 0) value = 0x00;
        else if (strcasecmp(arg1, "FAST") == 0) value = 0x01;
        else if (strcasecmp(arg1, "DC") == 0) value = 0x03;
        else { reply_error("uso: TESTSIG_FREQ <SLOW|FAST|DC>"); return; }

        g_ads->lock();
        g_ads->readModifyWriteRegister(ads1298::REG_CONFIG2, ads1298::CONFIG2_TEST_FREQ_MASK, value);
        g_ads->unlock();

        reply("OK");

    } else if (strcasecmp(cmd, "MUX") == 0) {
        int ch;
        if (!parse_channel(arg1, &ch) || arg2 == nullptr) {
            reply_error("uso: MUX <ch 1-8> <NORMAL|SHORT|RLD_MEAS|SUPPLY|TEMP|TEST|RLD_DRP|RLD_DRN>");
            return;
        }

        uint8_t code;
        if (strcasecmp(arg2, "NORMAL") == 0) code = 0x00;
        else if (strcasecmp(arg2, "SHORT") == 0) code = 0x01;
        else if (strcasecmp(arg2, "RLD_MEAS") == 0) code = 0x02;
        else if (strcasecmp(arg2, "SUPPLY") == 0) code = 0x03;
        else if (strcasecmp(arg2, "TEMP") == 0) code = 0x04;
        else if (strcasecmp(arg2, "TEST") == 0) code = 0x05;
        else if (strcasecmp(arg2, "RLD_DRP") == 0) code = 0x06;
        else if (strcasecmp(arg2, "RLD_DRN") == 0) code = 0x07;
        else { reply_error("modo de mux invalido"); return; }

        uint8_t reg = (uint8_t)(ads1298::REG_CH1SET + (ch - 1));

        g_ads->lock();
        g_ads->readModifyWriteRegister(reg, ads1298::CHSET_MUX_MASK, code);
        g_ads->unlock();

        reply("OK");

    } else if (strcasecmp(cmd, "LOFF_CH") == 0) {
        int ch;
        if (!parse_channel(arg1, &ch) || arg2 == nullptr || arg3 == nullptr) {
            reply_error("uso: LOFF_CH <ch 1-8> <P|N> <0|1>");
            return;
        }

        uint8_t reg;
        if (strcasecmp(arg2, "P") == 0) reg = ads1298::REG_LOFF_SENSP;
        else if (strcasecmp(arg2, "N") == 0) reg = ads1298::REG_LOFF_SENSN;
        else { reply_error("uso: LOFF_CH <ch 1-8> <P|N> <0|1>"); return; }

        bool enable = (strcmp(arg3, "1") == 0);
        uint8_t bit = (uint8_t)(1 << (ch - 1));

        g_ads->lock();
        g_ads->readModifyWriteRegister(reg, bit, (uint8_t)(enable ? bit : 0x00));
        g_ads->unlock();

        reply("OK");

    } else if (strcasecmp(cmd, "LOFF_MODE") == 0) {
        if (arg1 == nullptr) { reply_error("uso: LOFF_MODE <DC|AC>"); return; }

        uint8_t value;
        if (strcasecmp(arg1, "DC") == 0) value = 0x03;
        else if (strcasecmp(arg1, "AC") == 0) value = 0x01;
        else { reply_error("uso: LOFF_MODE <DC|AC>"); return; }

        g_ads->lock();
        g_ads->readModifyWriteRegister(ads1298::REG_LOFF, ads1298::LOFF_FLEAD_OFF_MASK, value);
        g_ads->unlock();

        reply("OK");

    } else if (strcasecmp(cmd, "LOFF_TYPE") == 0) {
        if (arg1 == nullptr) { reply_error("uso: LOFF_TYPE <RESISTOR|CURRENT>"); return; }

        uint8_t value;
        if (strcasecmp(arg1, "RESISTOR") == 0) value = ads1298::LOFF_VLEAD_OFF_EN_BIT;
        else if (strcasecmp(arg1, "CURRENT") == 0) value = 0x00;
        else { reply_error("uso: LOFF_TYPE <RESISTOR|CURRENT>"); return; }

        g_ads->lock();
        g_ads->readModifyWriteRegister(ads1298::REG_LOFF, ads1298::LOFF_VLEAD_OFF_EN_BIT, value);
        g_ads->unlock();

        reply("OK");

    } else if (strcasecmp(cmd, "LOFF_CURRENT") == 0) {
        if (arg1 == nullptr) { reply_error("uso: LOFF_CURRENT <6|12|18|24>"); return; }

        uint8_t code;
        if (strcmp(arg1, "6") == 0) code = 0x00;
        else if (strcmp(arg1, "12") == 0) code = 0x01;
        else if (strcmp(arg1, "18") == 0) code = 0x02;
        else if (strcmp(arg1, "24") == 0) code = 0x03;
        else { reply_error("uso: LOFF_CURRENT <6|12|18|24>"); return; }

        uint8_t value = (uint8_t)(code << ads1298::LOFF_ILEAD_OFF_SHIFT);

        g_ads->lock();
        g_ads->readModifyWriteRegister(ads1298::REG_LOFF, ads1298::LOFF_ILEAD_OFF_MASK, value);
        g_ads->unlock();

        reply("OK");

    } else if (strcasecmp(cmd, "LOFF_THRESHOLD") == 0) {
        if (arg1 == nullptr) { reply_error("uso: LOFF_THRESHOLD <0-7>"); return; }

        char *end = nullptr;
        long code = strtol(arg1, &end, 10);
        if (end == arg1 || code < 0 || code > 7) { reply_error("uso: LOFF_THRESHOLD <0-7>"); return; }

        uint8_t value = (uint8_t)(code << ads1298::LOFF_COMP_TH_SHIFT);

        g_ads->lock();
        g_ads->readModifyWriteRegister(ads1298::REG_LOFF, ads1298::LOFF_COMP_TH_MASK, value);
        g_ads->unlock();

        reply("OK");

    } else if (strcasecmp(cmd, "START") == 0) {
        g_ads->startConversions();
        reply("OK");

    } else if (strcasecmp(cmd, "STOP") == 0) {
        g_ads->stopConversions();
        reply("OK");

    } else {
        reply("OK");
    }
}

// TinyUSB llama automáticamente a este callback cuando se abre/cierra un puerto CDC.
// Cuando DTR pasa a true, el monitor serial del PC acaba de abrir ese puerto.
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)rts;

    if (dtr) {
        send_interface_banner(itf);
    }
}

// TinyUSB llama automáticamente a este callback cuando llega información por algún CDC.
void tud_cdc_rx_cb(uint8_t itf)
{
    uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE + 1];

    uint32_t count = tud_cdc_n_read(itf, buf, CFG_TUD_CDC_RX_BUFSIZE);
    buf[count] = 0;

    if (itf == CDC_CONFIG) {
        handle_config_command((char *)buf);
    } else if (itf == CDC_RLD) {
        emg_usb_write_line(CDC_RLD, "RLD_OK");
    } else {
        // Vacía RX de cualquier otro CDC para no bloquearlo.
        emg_usb_write_line(itf, "OK");
    }
}

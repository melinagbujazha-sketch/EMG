#ifndef ADS1298_H
#define ADS1298_H

#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/mutex.h"
#include "hardware/spi.h"

class ads1298 {
public:
    // Pines por defecto según el esquemático final:
    // DOUT -> GP16, CS -> GP17, SCLK -> GP18, DIN -> GP19,
    // DRDY -> GP20, RESET -> GP21, START -> GP22, PWDN -> GP26.
    ads1298(spi_inst_t *spi_port = spi0,
            uint pin_miso = 16,
            uint pin_cs = 17,
            uint pin_sck = 18,
            uint pin_mosi = 19,
            uint pin_drdy = 20,
            uint pin_reset = 21,
            uint pin_start = 22,
            uint pin_pwdn = 26);

    void init(uint32_t spi_baudrate_hz = 1000 * 1000);
    void reset();

    void sendCommand(uint8_t command);
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);

    // Lee un registro, reemplaza los bits indicados por mask (value ya debe
    // venir alineado a la posición correcta) y reescribe el registro,
    // preservando el resto de los bits. Devuelve el valor final escrito.
    uint8_t readModifyWriteRegister(uint8_t reg, uint8_t mask, uint8_t value);

    uint8_t readId();
    bool checkId();

    bool drdyIsLow();
    void startConversions();
    void stopConversions();

    // Lee un frame completo de conversión (status word de 24 bits + 8
    // canales de 24 bits en complemento a 2, ya con signo extendido a
    // int32_t) usando el opcode RDATA. Pensado para ser llamado una vez por
    // cada flanco de DRDY. Devuelve false si el registro no fue leído
    // (siempre true en la implementación actual; se deja el bool para
    // futuros chequeos de error).
    bool readDataFrame(uint8_t status_out[3], int32_t channels_out[8]);

    // Extraen LOFF_STATP/LOFF_STATN del status word de 24 bits devuelto por
    // readDataFrame (ver Figura 61 del datasheet: "Status Word Content").
    static uint8_t extractLoffStatP(const uint8_t status[3]);
    static uint8_t extractLoffStatN(const uint8_t status[3]);

    // Mapea un valor de ganancia (1,2,3,4,6,8,12) al código de 3 bits de
    // CHnSET.GAIN. Devuelve -1 si el valor no es válido.
    static int8_t gainToCode(int gain_value);

    // Mutex que protege toda transacción SPI hacia el ADS1298; se usa desde
    // el loop de adquisición (core 1) y desde el manejo de comandos de
    // configuración por USB (core 0) para que nunca se solapen dos
    // transacciones sobre el mismo bus.
    void lock();
    void unlock();

    // Comandos ADS1298
    static constexpr uint8_t CMD_WAKEUP    = 0x02;
    static constexpr uint8_t CMD_STANDBY   = 0x04;
    static constexpr uint8_t CMD_RESET     = 0x06;
    static constexpr uint8_t CMD_START     = 0x08;
    static constexpr uint8_t CMD_STOP      = 0x0A;
    static constexpr uint8_t CMD_RDATAC    = 0x10;
    static constexpr uint8_t CMD_SDATAC    = 0x11;
    static constexpr uint8_t CMD_RDATA     = 0x12;
    static constexpr uint8_t CMD_RREG      = 0x20;
    static constexpr uint8_t CMD_WREG      = 0x40;

    // Registros ADS1298 principales
    static constexpr uint8_t REG_ID         = 0x00;
    static constexpr uint8_t REG_CONFIG1    = 0x01;
    static constexpr uint8_t REG_CONFIG2    = 0x02;
    static constexpr uint8_t REG_CONFIG3    = 0x03;
    static constexpr uint8_t REG_LOFF       = 0x04;
    static constexpr uint8_t REG_CH1SET     = 0x05;
    static constexpr uint8_t REG_CH2SET     = 0x06;
    static constexpr uint8_t REG_CH3SET     = 0x07;
    static constexpr uint8_t REG_CH4SET     = 0x08;
    static constexpr uint8_t REG_CH5SET     = 0x09;
    static constexpr uint8_t REG_CH6SET     = 0x0A;
    static constexpr uint8_t REG_CH7SET     = 0x0B;
    static constexpr uint8_t REG_CH8SET     = 0x0C;
    static constexpr uint8_t REG_RLD_SENSP  = 0x0D;
    static constexpr uint8_t REG_RLD_SENSN  = 0x0E;
    static constexpr uint8_t REG_LOFF_SENSP = 0x0F;
    static constexpr uint8_t REG_LOFF_SENSN = 0x10;
    static constexpr uint8_t REG_LOFF_FLIP  = 0x11;
    static constexpr uint8_t REG_LOFF_STATP = 0x12;
    static constexpr uint8_t REG_LOFF_STATN = 0x13;
    static constexpr uint8_t REG_GPIO       = 0x14;
    static constexpr uint8_t REG_PACE       = 0x15;
    static constexpr uint8_t REG_RESP       = 0x16;
    static constexpr uint8_t REG_CONFIG4    = 0x17;
    static constexpr uint8_t REG_WCT1       = 0x18;
    static constexpr uint8_t REG_WCT2       = 0x19;

    static constexpr uint8_t EXPECTED_ID_ADS1298 = 0x92;

    // ---- Bits/máscaras usados por los comandos de configuración de alto nivel ----

    // CONFIG1 (0x01)
    static constexpr uint8_t CONFIG1_HR_BIT  = 0x80; // 1 = high-resolution, 0 = low-power
    static constexpr uint8_t CONFIG1_DR_MASK = 0x07;

    // CONFIG2 (0x02)
    static constexpr uint8_t CONFIG2_INT_TEST_BIT   = 0x10;
    static constexpr uint8_t CONFIG2_TEST_AMP_BIT   = 0x04;
    static constexpr uint8_t CONFIG2_TEST_FREQ_MASK = 0x03;

    // CONFIG3 (0x03)
    // 1 = buffer de referencia interno (2.4V/4V) habilitado. Sin este bit el
    // ADS1298 arranca con el buffer de referencia apagado (reset por
    // defecto = 0), dejando VREFP sin excitar: toda medición que dependa de
    // la referencia interna (incluida la test signal, definida como
    // (VREFP-VREFN)/2400 o /1200) queda con amplitud/offset indefinidos.
    static constexpr uint8_t CONFIG3_PD_REFBUF_BIT = 0x80;
    static constexpr uint8_t CONFIG3_PD_RLD_BIT = 0x04; // 1 = amplificador RLD habilitado

    // CHnSET (0x05-0x0C)
    static constexpr uint8_t CHSET_PD_BIT     = 0x80; // 1 = canal apagado (power-down)
    static constexpr uint8_t CHSET_GAIN_MASK  = 0x70;
    static constexpr uint8_t CHSET_GAIN_SHIFT = 4;
    static constexpr uint8_t CHSET_MUX_MASK   = 0x07;

    // LOFF (0x04)
    static constexpr uint8_t LOFF_COMP_TH_MASK     = 0xE0;
    static constexpr uint8_t LOFF_COMP_TH_SHIFT    = 5;
    static constexpr uint8_t LOFF_VLEAD_OFF_EN_BIT = 0x10; // 1 = pullup/pulldown, 0 = fuente de corriente
    static constexpr uint8_t LOFF_ILEAD_OFF_MASK   = 0x0C;
    static constexpr uint8_t LOFF_ILEAD_OFF_SHIFT  = 2;
    static constexpr uint8_t LOFF_FLEAD_OFF_MASK   = 0x03;

private:
    spi_inst_t *_spi;
    uint _pin_miso;
    uint _pin_cs;
    uint _pin_sck;
    uint _pin_mosi;
    uint _pin_drdy;
    uint _pin_reset;
    uint _pin_start;
    uint _pin_pwdn;

    mutex_t _bus_mutex;

    void csLow();
    void csHigh();
};

#endif

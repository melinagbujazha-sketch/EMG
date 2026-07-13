#include "ads1298.h"

ads1298::ads1298(spi_inst_t *spi_port,
                 uint pin_miso,
                 uint pin_cs,
                 uint pin_sck,
                 uint pin_mosi,
                 uint pin_drdy,
                 uint pin_reset,
                 uint pin_start,
                 uint pin_pwdn)
    : _spi(spi_port),
      _pin_miso(pin_miso),
      _pin_cs(pin_cs),
      _pin_sck(pin_sck),
      _pin_mosi(pin_mosi),
      _pin_drdy(pin_drdy),
      _pin_reset(pin_reset),
      _pin_start(pin_start),
      _pin_pwdn(pin_pwdn)
{
}

void ads1298::init(uint32_t spi_baudrate_hz)
{
    spi_init(_spi, spi_baudrate_hz);

    gpio_set_function(_pin_miso, GPIO_FUNC_SPI);
    gpio_set_function(_pin_sck,  GPIO_FUNC_SPI);
    gpio_set_function(_pin_mosi, GPIO_FUNC_SPI);

    // ADS1298: SPI modo 1 -> CPOL = 0, CPHA = 1.
    spi_set_format(_spi, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_init(_pin_cs);
    gpio_set_dir(_pin_cs, GPIO_OUT);
    gpio_put(_pin_cs, 1);   // CS inactivo en alto.

    gpio_init(_pin_reset);
    gpio_set_dir(_pin_reset, GPIO_OUT);
    gpio_put(_pin_reset, 1); // RESET inactivo en alto.

    gpio_init(_pin_start);
    gpio_set_dir(_pin_start, GPIO_OUT);
    gpio_put(_pin_start, 0); // START bajo: no iniciar adquisición todavía.

    gpio_init(_pin_pwdn);
    gpio_set_dir(_pin_pwdn, GPIO_OUT);
    gpio_put(_pin_pwdn, 1);  // PWDN alto: ADS encendido.

    gpio_init(_pin_drdy);
    gpio_set_dir(_pin_drdy, GPIO_IN);

    mutex_init(&_bus_mutex);

    sleep_ms(100);
}

void ads1298::reset()
{
    gpio_put(_pin_reset, 0);
    sleep_ms(10);
    gpio_put(_pin_reset, 1);
    sleep_ms(100);
}

void ads1298::sendCommand(uint8_t command)
{
    csLow();
    spi_write_blocking(_spi, &command, 1);
    sleep_us(5);
    csHigh();
    sleep_us(5);
}

uint8_t ads1298::readRegister(uint8_t reg)
{
    uint8_t tx[3];
    uint8_t rx[3];

    tx[0] = CMD_RREG | reg; // RREG + dirección inicial.
    tx[1] = 0x00;           // Cantidad de registros a leer - 1. Para 1 registro: 0.
    tx[2] = 0x00;           // Byte dummy para generar clocks de lectura.

    csLow();
    spi_write_read_blocking(_spi, tx, rx, 3);
    sleep_us(5);
    csHigh();
    sleep_us(5);

    return rx[2];
}

void ads1298::writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx[3];

    tx[0] = CMD_WREG | reg; // WREG + dirección inicial.
    tx[1] = 0x00;           // Cantidad de registros a escribir - 1. Para 1 registro: 0.
    tx[2] = value;

    csLow();
    spi_write_blocking(_spi, tx, 3);
    sleep_us(5);
    csHigh();
    sleep_us(5);
}

uint8_t ads1298::readId()
{
    sendCommand(CMD_SDATAC); // Necesario para poder usar RREG si el ADS está en RDATAC.
    return readRegister(REG_ID);
}

bool ads1298::checkId()
{
    return readId() == EXPECTED_ID_ADS1298;
}

bool ads1298::drdyIsLow()
{
    return gpio_get(_pin_drdy) == 0;
}

void ads1298::startConversions()
{
    gpio_put(_pin_start, 1);
}

void ads1298::stopConversions()
{
    gpio_put(_pin_start, 0);
}

void ads1298::csLow()
{
    gpio_put(_pin_cs, 0);
}

void ads1298::csHigh()
{
    gpio_put(_pin_cs, 1);
}

uint8_t ads1298::readModifyWriteRegister(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t current = readRegister(reg);
    uint8_t updated = (uint8_t)((current & ~mask) | (value & mask));
    writeRegister(reg, updated);
    return updated;
}

bool ads1298::readDataFrame(uint8_t status_out[3], int32_t channels_out[8])
{
    // 1 byte de opcode + 3 bytes de status word + 8 canales x 3 bytes.
    uint8_t tx[1 + 3 + 8 * 3] = {0};
    uint8_t rx[1 + 3 + 8 * 3] = {0};

    tx[0] = CMD_RDATA;

    csLow();
    spi_write_read_blocking(_spi, tx, rx, sizeof(tx));
    csHigh();
    sleep_us(5);

    status_out[0] = rx[1];
    status_out[1] = rx[2];
    status_out[2] = rx[3];

    for (int ch = 0; ch < 8; ch++) {
        int base = 4 + ch * 3;
        uint32_t raw = ((uint32_t)rx[base] << 16) |
                       ((uint32_t)rx[base + 1] << 8) |
                       (uint32_t)rx[base + 2];

        // Signo extendido de 24 bits (complemento a 2) a int32_t.
        if (raw & 0x800000) {
            raw |= 0xFF000000;
        }

        channels_out[ch] = (int32_t)raw;
    }

    return true;
}

uint8_t ads1298::extractLoffStatP(const uint8_t status[3])
{
    // Status word: [1 1 0 0][LOFF_STATP[7:0]][LOFF_STATN[7:0]][GPIO[7:4]]
    return (uint8_t)((status[0] << 4) | (status[1] >> 4));
}

uint8_t ads1298::extractLoffStatN(const uint8_t status[3])
{
    return (uint8_t)((status[1] << 4) | (status[2] >> 4));
}

int8_t ads1298::gainToCode(int gain_value)
{
    switch (gain_value) {
        case 6:  return 0b000;
        case 1:  return 0b001;
        case 2:  return 0b010;
        case 3:  return 0b011;
        case 4:  return 0b100;
        case 8:  return 0b101;
        case 12: return 0b110;
        default: return -1;
    }
}

void ads1298::lock()
{
    mutex_enter_blocking(&_bus_mutex);
}

void ads1298::unlock()
{
    mutex_exit(&_bus_mutex);
}

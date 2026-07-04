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

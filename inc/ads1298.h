#ifndef ADS1298_H
#define ADS1298_H

#include <stdint.h>
#include "pico/stdlib.h"
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

    uint8_t readId();
    bool checkId();

    bool drdyIsLow();
    void startConversions();
    void stopConversions();

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

    void csLow();
    void csHigh();
};

#endif

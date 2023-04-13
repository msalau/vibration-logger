#pragma once

#define ICM42688P
#include "Icm426xxDefs.h"
#include "clocked_input.pio.h"
#include <SPI.h>

typedef struct 
{
    int16_t x;
    int16_t y;
    int16_t z;
} Value;

typedef struct
{
    fifo_header_t header;
    Value value;
    uint8_t temp;
} FifoPacket1;

class ICM42688
{
private:
    HardwareSPI &spi;
    int csPin;

    bool extEnable;
    int extMisoPin;
    int extSckPin;
    int extCsPin;

    PIO pio;
    uint pio_sm;
    uint pio_offset;

public:
    ICM42688(HardwareSPI &spi, int csPin, int extCsPin, int extMisoPin = -1, int extSckPin = -1)
        : spi(spi), csPin(csPin), extCsPin(extCsPin), extMisoPin(extMisoPin), extSckPin(extSckPin)
    {
        extEnable = (extMisoPin != -1) && (extSckPin != -1) && ((extMisoPin + 1) == extSckPin);
    }

    void select_bank(unsigned bank)
    {
        write(MPUREG_REG_BANK_SEL, bank);
    }

    void write(uint8_t address, uint8_t value)
    {
        if (extEnable)
        {
            pio_sm_clear_fifos(pio, pio_sm);
            pio_sm_restart(pio, pio_sm);
        }

        digitalWrite(csPin, LOW);
        digitalWrite(extCsPin, LOW);

        spi.transfer(address & 0x7F);
        spi.transfer(value);

        digitalWrite(csPin, HIGH);
        digitalWrite(extCsPin, HIGH);
    }

    void read(uint8_t address, void *data0, unsigned length, void *data1 = NULL, unsigned *data1_len = NULL)
    {
        if (extEnable)
        {
            pio_sm_clear_fifos(pio, pio_sm);
            pio_sm_restart(pio, pio_sm);
        }

        digitalWrite(csPin, LOW);
        digitalWrite(extCsPin, LOW);

        spi.transfer(address | 0x80);

        if (extEnable)
        {
            // Wait for the data from PIO clkin
            delayMicroseconds(1);
            if (!pio_sm_is_rx_fifo_empty(pio, pio_sm))
                (void)pio_sm_get(pio, pio_sm);
        }

        uint8_t *pdata0 = (uint8_t *)data0;
        uint8_t *pdata1 = (uint8_t *)data1;

        while (length)
        {
            *pdata0++ = spi.transfer(0xFF);
            
            if (extEnable && pdata1 && !pio_sm_is_rx_fifo_empty(pio, pio_sm))
                *pdata1++ = pio_sm_get(pio, pio_sm);

            length--;
        }

        digitalWrite(csPin, HIGH);
        digitalWrite(extCsPin, HIGH);

        if (extEnable && pdata1)
        {
            // Wait for the data from PIO clkin
            delayMicroseconds(1);

            while (!pio_sm_is_rx_fifo_empty(pio, pio_sm))
                *pdata1++ = pio_sm_get(pio, pio_sm);
        }

        if (extEnable && data1_len)
            *data1_len = pdata1 - (uint8_t *)data1;
    }

    uint8_t read(uint8_t address)
    {
        uint8_t value;
        read(address, &value, sizeof(value));
        return value;
    }

    uint16_t read16(uint8_t address)
    {
        uint16_t value;
        read(address, &value, sizeof(value));
        return value;
    }

    bool begin()
    {
        // Drive CS signal high
        digitalWrite(csPin, HIGH);
        digitalWrite(extCsPin, HIGH);
        pinMode(csPin, OUTPUT);
        pinMode(extCsPin, OUTPUT);

        // Configure the SPI bus in mode 3, MSB first, 1 MHz
        spi.begin();
        spi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));

        if (extEnable)
        {
            // Load the clocked_input program, and configure a free state machine
            // to run the program.
            pio = pio0;
            pio_offset = pio_add_program(pio, &clocked_input_program);
            pio_sm = pio_claim_unused_sm(pio, true);
            clocked_input_program_init(pio, pio_sm, pio_offset, extMisoPin);
        }

        // Perform Software Reset
        select_bank(0);
        write(MPUREG_DEVICE_CONFIG,
            ICM426XX_CHIP_CONFIG_SPI_MODE_0_3 |
            ICM426XX_DEVICE_CONFIG_RESET_EN
            );
        delay(2);

        // Read WHO_AM_I
        unsigned whoAmI = read(MPUREG_WHO_AM_I);

        // Check WHO_AM_I value
        if (whoAmI != ICM_WHOAMI)
        {
            Log("%u: %s: Initialization failed: WHO_AM_I value doesn't match\r\n", millis(), __func__);
            return false;
        }

        // Disable I2C, switch to little-endian mode
        write(MPUREG_INTF_CONFIG0, 
            ICM426XX_INTF_CONFIG0_FIFO_COUNT_REC_RECORD |
            ICM426XX_INTF_CONFIG0_FIFO_COUNT_LITTLE_ENDIAN |
            ICM426XX_INTF_CONFIG0_DATA_LITTLE_ENDIAN |
            0x03 // Disable I2C
            );

        uint8_t data;

        // inv_icm426xx_enable_clkin_rtc()

        select_bank(1);
        data = read(MPUREG_INTF_CONFIG5_B1);
        data &= ~BIT_INTF_CONFIG5_GPIO_PAD_SEL_MASK;
        data |= (2 << BIT_INTF_CONFIG5_GPIO_PAD_SEL_POS); // PIN9_FUNCTION = CLKIN
        write(MPUREG_INTF_CONFIG5_B1, data);

        select_bank(0);
        data = read(MPUREG_INTF_CONFIG1);
        data &= ~BIT_RTC_MODE_MASK;
        data |= ICM426XX_INTF_CONFIG1_RTC_MODE_EN;
        write(MPUREG_INTF_CONFIG1, data);

	    data = read(MPUREG_TMST_CONFIG);
	    data &= ~BIT_TMST_CONFIG_RESOL_MASK;
	    data |= ICM426XX_TMST_CONFIG_RESOL_16us; // resolution is 1 RTC clock period
	    write(MPUREG_TMST_CONFIG, data);

	    // inv_icm426xx_set_accel_fsr(&icm_driver, ICM426XX_ACCEL_CONFIG0_FS_SEL_8g);
        // inv_icm426xx_set_accel_frequency(&icm_driver, ICM426XX_ACCEL_CONFIG0_ODR_2_KHZ);

        write(MPUREG_ACCEL_CONFIG0,
            ICM426XX_ACCEL_CONFIG0_FS_SEL_8g | ICM426XX_ACCEL_CONFIG0_ODR_2_KHZ
            );

	    // inv_icm426xx_set_gyro_fsr(&icm_driver, ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps);
	    // inv_icm426xx_set_gyro_frequency(&icm_driver, ICM426XX_GYRO_CONFIG0_ODR_2_KHZ);

        write(MPUREG_GYRO_CONFIG0,
            ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps | ICM426XX_GYRO_CONFIG0_ODR_2_KHZ
            );

	    // inv_icm426xx_enable_gyro_low_noise_mode(&icm_driver);
	    // inv_icm426xx_enable_accel_low_noise_mode(&icm_driver);

        write(MPUREG_PWR_MGMT_0,
            ICM426XX_PWR_MGMT_0_TEMP_EN |
            ICM426XX_PWR_MGMT_0_IDLE_EN |
            ICM426XX_PWR_MGMT_0_GYRO_MODE_LN |
            ICM426XX_PWR_MGMT_0_ACCEL_MODE_LN
            );
        delay(1);

        Log("%u: %s: Initialization complete\r\n", millis(), __func__);

        return true;
    }

    int16_t readTempData(void)
    {
        return read16(MPUREG_TEMP_DATA0_UI);
    }

    float readTemp(void)
    {
        return readTempData() / 132.48f + 25.0f;
    }

    Value readAccelData(void)
    {
        Value value;
        read(MPUREG_ACCEL_DATA_X0_UI, &value, sizeof(value));
        return value;
    }

    void readAccelData(Value *v1, Value *v2, unsigned *v2_len = NULL)
    {
        read(MPUREG_ACCEL_DATA_X0_UI, v1, sizeof(*v1), v2, v2_len);
    }

    Value readGyroData(void)
    {
        Value value;
        read(MPUREG_GYRO_DATA_X0_UI, &value, sizeof(value));
        return value;
    }

    void readGyroData(Value *v1, Value *v2, unsigned *v2_len = NULL)
    {
        read(MPUREG_GYRO_DATA_X0_UI, v1, sizeof(*v1), v2, v2_len);
    }

    void fifoBegin(void)
    {
        // inv_icm426xx_configure_fifo()
        write(MPUREG_FIFO_CONFIG1, ICM426XX_FIFO_CONFIG1_ACCEL_EN);
        write(MPUREG_FIFO_CONFIG, ICM426XX_FIFO_CONFIG_MODE_STREAM);
    }

    void fifoEnd(void)
    {
        write(MPUREG_FIFO_CONFIG, ICM426XX_FIFO_CONFIG_MODE_BYPASS);
    }

    void fifoClear(void)
    {
        write(MPUREG_SIGNAL_PATH_RESET, ICM426XX_SIGNAL_PATH_RESET_FIFO_FLUSH_EN);
    }

    uint16_t fifoReadCount(void)
    {
        return read16(MPUREG_FIFO_COUNTH);
    }

    void fifoReadCount(uint16_t *v1, uint16_t *v2 = NULL)
    {
        return read(MPUREG_FIFO_COUNTH, v1, 2, v2);
    }

    uint16_t fifoReadLostCount(void)
    {
        return read16(MPUREG_FIFO_LOST_PKT0);
    }

    void fifoReadLostCount(uint16_t *v1, uint16_t *v2 = NULL)
    {
        return read(MPUREG_FIFO_LOST_PKT0, v1, 2, v2);
    }

    FifoPacket1 fifoReadValue(void)
    {
        // inv_icm426xx_get_data_from_fifo()

        FifoPacket1 value;
        read(MPUREG_FIFO_DATA, &value, sizeof(value));
        return value;
    }

    void fifoReadValue(FifoPacket1 *pkt1, FifoPacket1 *pkt2 = NULL)
    {
        read(MPUREG_FIFO_DATA, pkt1, sizeof(*pkt1), pkt2);
    }
};

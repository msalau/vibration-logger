#pragma once
#include <DigitalIO/SoftSPI.h>

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} ImuRawValue;

#define ADXL345_REG_DEVID 0x00
#define ADXL345_REG_THRESH_TAP 0x1D
#define ADXL345_REG_OFSX 0x1E
#define ADXL345_REG_OFSY 0x1F
#define ADXL345_REG_OFSZ 0x20
#define ADXL345_REG_DUR 0x21
#define ADXL345_REG_Latent 0x22
#define ADXL345_REG_Window 0x23
#define ADXL345_REG_THRESH_ACT 0x24
#define ADXL345_REG_THRESH_INACT 0x25
#define ADXL345_REG_TIME_INACT 0x26
#define ADXL345_REG_ACT_INACT_CTL 0x27
#define ADXL345_REG_THRESH_FF 0x28
#define ADXL345_REG_TIME_FF 0x29
#define ADXL345_REG_TAP_AXES 0x2A
#define ADXL345_REG_ACT_TAP_STATUS 0x2B
#define ADXL345_REG_BW_RATE 0x2C
#define ADXL345_REG_POWER_CTL 0x2D
#define ADXL345_REG_INT_ENABLE 0x2E
#define ADXL345_REG_INT_MAP 0x2F
#define ADXL345_REG_INT_SOURCE 0x30
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_DATAX0 0x32
#define ADXL345_REG_DATAX1 0x33
#define ADXL345_REG_DATAY0 0x34
#define ADXL345_REG_DATAY1 0x35
#define ADXL345_REG_DATAZ0 0x36
#define ADXL345_REG_DATAZ1 0x37
#define ADXL345_REG_FIFO_CTL 0x38
#define ADXL345_REG_FIFO_STATUS 0x39

template<uint8_t misoPin, uint8_t mosiPin, uint8_t sckPin, uint8_t csPin>
class MyADXL345 {
private:
  SoftSPI<misoPin, mosiPin, sckPin, 3> spi;

public:
  MyADXL345() {
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, 1);
    spi.begin();
    digitalWrite(mosiPin, 1);
  }

  void readRegisters(uint8_t address, void *data, uint8_t length = 1) {
    uint8_t *pdata = (uint8_t *)data;

    digitalWrite(csPin, 0);
    spi.transfer(address | 0xC0);
    for (unsigned i = 0; i < length; i++) {
      *pdata++ = spi.transfer(0xFF);
    }
    digitalWrite(mosiPin, 1);
    digitalWrite(csPin, 1);
  }

  uint8_t readRegister(uint8_t address) {
    uint8_t value;
    readRegisters(address, &value, 1);
    return value;
  }

  void writeRegisters(uint8_t address, const void *data, uint8_t length = 1) {
    const uint8_t *pdata = (const uint8_t *)data;

    digitalWrite(csPin, 0);
    spi.transfer((address & 0x3F) | 0x40);
    for (unsigned i = 0; i < length; i++) {
      spi.transfer(*pdata++);
    }
    digitalWrite(mosiPin, 1);
    digitalWrite(csPin, 1);
  }

  void writeRegister(uint8_t address, uint8_t value) {
    writeRegisters(address, &value, 1);
  }

  bool begin(void) {
    uint8_t devid = readRegister(ADXL345_REG_DEVID);

    if (devid != 0345)
      return false;

    writeRegister(ADXL345_REG_BW_RATE, 0x0e);// 1600 Hz
    writeRegister(ADXL345_REG_DATA_FORMAT, 0x02);// 8g mode
    writeRegister(ADXL345_REG_POWER_CTL, 0x08);// Enable measurement

    return true;
  }

  bool isIrqSet(void) {
    return false;
  }

  void fifoReadValue(ImuRawValue *data) {
    readRegisters(ADXL345_REG_DATAX0, data, sizeof(data[0]));
  }

  ImuRawValue fifoReadValue(void) {
    ImuRawValue value;
    fifoReadValue(&value);
    return value;
  }

  uint8_t fifoGetStatus(void) {
    return readRegister(ADXL345_REG_FIFO_STATUS) & 0x3F;
  }

  void fifoBegin(void) {
    writeRegister(ADXL345_REG_FIFO_CTL, 0x80); // Stream Mode
  }

  void fifoEnd(void) {
    writeRegister(ADXL345_REG_FIFO_CTL, 0); // Bypass mode
  }

  void fifoClear(void) {
    ImuRawValue value;
    while (fifoGetStatus()) {
      fifoReadValue(&value);
    }
  }
};
#pragma once
#include <Wire.h>
#include "LSM6DS3.h"

typedef enum {
  IMU_SUCCESS,
  IMU_HW_ERROR,
  IMU_NOT_SUPPORTED,
  IMU_GENERIC_ERROR,
  IMU_OUT_OF_BOUNDS,
  IMU_ALL_ONES_WARNING,
  IMU_INVALID_ARGUMENT,
  IMU_INCOMPLETE_OPERATION,
} status_t;

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} ImuRawValue;

class MyLSM6DS3 {
private:
  uint8_t i2cAddress;
  TwoWire &i2cInterface;

public:
  MyLSM6DS3(TwoWire &i2cInterface = Wire1, uint8_t i2cAddress = 0x6A)
    : i2cInterface(i2cInterface),
      i2cAddress(i2cAddress) {
  }

  status_t readRegisters(uint8_t address, void *data, size_t length = 1) {
    uint8_t *p_data = (uint8_t *)data;
    i2cInterface.beginTransmission(i2cAddress);
    if (i2cInterface.write(address) != 1) {
      i2cInterface.endTransmission();
      return IMU_HW_ERROR;
    }
    if (i2cInterface.endTransmission(false) != 0) {
      return IMU_HW_ERROR;
    }
    if (i2cInterface.requestFrom(i2cAddress, length) != length) {
      return IMU_HW_ERROR;
    }
    while (length && i2cInterface.available()) {
        *p_data++ = i2cInterface.read();
        length--;
      }
    if (length) {
      return IMU_INCOMPLETE_OPERATION;
    }

    return IMU_SUCCESS;
  }

  status_t writeRegisters(uint8_t address, const void *data, size_t length = 1) {
    i2cInterface.beginTransmission(i2cAddress);
    if (i2cInterface.write(address) != 1) {
      i2cInterface.endTransmission();
      return IMU_HW_ERROR;
    }
    if (i2cInterface.write((const uint8_t *)data, length) != length) {
      i2cInterface.endTransmission();
      return IMU_HW_ERROR;
    }
    if (i2cInterface.endTransmission() != 0) {
      return IMU_HW_ERROR;
    }
    return IMU_SUCCESS;
  }

  status_t begin(void) {
    uint8_t reg_value;
    status_t status;

#ifdef PIN_LSM6DS3TR_C_POWER
    pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT_H0H1);
    digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
    delay(10);
#endif

    status = readRegisters(LSM6DS3_ACC_GYRO_WHO_AM_I_REG, &reg_value);
    if (status != IMU_SUCCESS)
      return status;

    if (reg_value != LSM6DS3_ACC_GYRO_WHO_AM_I && reg_value != LSM6DS3_C_ACC_GYRO_WHO_AM_I)
      return IMU_NOT_SUPPORTED;

    uint8_t ctrl[2] = {
      // CTRL1_XL
      LSM6DS3_ACC_GYRO_BW_XL_400Hz | LSM6DS3_ACC_GYRO_FS_XL_8g | LSM6DS3_ACC_GYRO_ODR_XL_833Hz,
      // CTRL2_G
      LSM6DS3_ACC_GYRO_ODR_G_POWER_DOWN,
    };

    return writeRegisters(LSM6DS3_ACC_GYRO_CTRL1_XL, ctrl, sizeof(ctrl));
  }

  status_t fifoReadValues(ImuRawValue *value, uint8_t length) {
    if (length > 5)
      return IMU_INVALID_ARGUMENT;

    return readRegisters(LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L, value, sizeof(value[0]) * length);
  }

  status_t fifoGetStatus(uint16_t *data) {
    return readRegisters(LSM6DS3_ACC_GYRO_FIFO_STATUS1, data, 2);
  }

  status_t fifoGetPattern(uint16_t *data) {
    return readRegisters(LSM6DS3_ACC_GYRO_FIFO_STATUS3, data, 2);
  }

  status_t fifoBegin(void) {
    const uint8_t fifo_ctrl[5] = {
      // FIFO_CTRL1
      0,
      // FIFO_CTRL2
      0,
      // FIFO_CTRL3
      LSM6DS3_ACC_GYRO_DEC_FIFO_XL_NO_DECIMATION,
      // FIFO_CTRL4
      0,
      // FIFO_CTRL5
      LSM6DS3_ACC_GYRO_FIFO_MODE_DYN_STREAM_2 | LSM6DS3_ACC_GYRO_ODR_FIFO_800Hz
    };

    return writeRegisters(LSM6DS3_ACC_GYRO_FIFO_CTRL1, fifo_ctrl, sizeof(fifo_ctrl));
  }

  status_t fifoEnd(void) {
    return writeRegisters(LSM6DS3_ACC_GYRO_FIFO_CTRL5, 0x00);
  }

  void fifoClear(void) {
    uint16_t fifoStatus = 0;
    ImuRawValue value;
    fifoGetStatus(&fifoStatus);

    while ((fifoStatus & 0x1000) == 0) {
      fifoReadValues(&value, 1);
      fifoStatus = 0;
      fifoGetStatus(&fifoStatus);
    }
  }
};
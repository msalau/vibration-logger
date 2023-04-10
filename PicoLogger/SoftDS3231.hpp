#pragma once
#include <SWI2C.h>

bool soft_ds3231_init(void)
{
    SWI2C ds3231(SOFT_DS3231_SDA, SOFT_DS3231_SCL, SOFT_DS3231_ADDR);
    ds3231.begin();
    uint8_t status = 0;
    uint8_t control = 0;
    int ret;

    ret = ds3231.readFromRegister(0x0E, control);
    if (!ret) {
        Serial.printf("%u: %s: Failed to read control\r\n", millis(), __func__);
        return false;
    }
    Serial.printf("%u: %s: RTC control: 0x%02x\r\n", millis(), __func__, control);

    ret = ds3231.readFromRegister(0x0F, status);
    if (!ret) {
        Serial.printf("%u: %s: Failed to read status\r\n", millis(), __func__);
        return false;
    }
    Serial.printf("%u: %s: RTC status: 0x%02x\r\n", millis(), __func__, status);

    ret = ds3231.writeToRegister(0x0F, 0x08);
    if (!ret) {
        Serial.printf("%u: %s: Failed to write status\r\n", millis(), __func__);
        return false;
    }

    ret = ds3231.readFromRegister(0x0F, status);
    if (!ret) {
        Serial.printf("%u: %s: Failed to read status\r\n", millis(), __func__);
        return false;
    }
    Serial.printf("%u: %s: RTC status: 0x%02x\r\n", millis(), __func__, status);

    return true;
}

#include <SdFat.h>
#include <ezButton.h>
#include "Log.hpp"
#include "ICM42688.hpp"

//#define ENABLE_DS3231

#if ENABLE_DS3231
#include <Wire.h>
#include <RTClib.h>
RTC_DS3231 rtc;
#else
#define SOFT_DS3231_ADDR 0x68
#define SOFT_DS3231_SDA D5
#define SOFT_DS3231_SCL D4
#include "SoftDS3231.hpp"
#endif

#define LED_RED D6
#define LED_GREEN D7
#define LED_BLUE D3

#define BUZZER_PIN D22
#define BUZZER_FREQ 4000

#define IMU_SPI_BUS SPI1
#define IMU_SPI_CS D13
#define IMU_EXT_CS D8
#define IMU_EXT_MISO D10
#define IMU_EXT_SCK D11

ICM42688 imu(IMU_SPI_BUS, IMU_SPI_CS, IMU_EXT_CS);

#define BUTTON_PIN D2

ezButton btn(BUTTON_PIN, INPUT_PULLUP);

#define SD_SPI_FREQ SD_SCK_MHZ(32)
#define SD_CS_PIN   D17

ExFile file;
SdExFat sd;
SdSpiConfig sdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SPI_FREQ);

void setup()
{
  Serial.begin();

  // Set Button debounce time (ms)
  btn.setDebounceTime(50);

  Log("%u: Initialize LEDs\r\n", millis());
  pinMode(LED_BUILTIN, OUTPUT_12MA);
  pinMode(LED_RED, OUTPUT_12MA);
  pinMode(LED_GREEN, OUTPUT_12MA);
  pinMode(LED_BLUE, OUTPUT_12MA);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);

#if ENABLE_DS3231
  // Initialize the I2C bus and set 400kHz frequency
  Log("%u: Initialize Wire\r\n", millis());
  Wire.begin();
  Wire.setClock(100000);
#endif

  // Initialize RTC
  while (true)
  {
    Log("%u: Initialize RTC\r\n", millis());
    bool success;
#if ENABLE_DS3231
    success = rtc.begin(&Wire);
#else
    success = soft_ds3231_init();
#endif
    if (success)
      break;
    delay(1000);
  }

  while (true)
  {
    Log("%u: Initialize IMU\r\n", millis());
    if (imu.begin())
      break;
    delay(1000);
  }
}

static void digitalToggle(int pin)
{
  digitalWrite(pin, !digitalRead(pin));
}

unsigned capture_end_time = 0;
unsigned capture_values = 0;
const unsigned capture_time = 10000;

void loop()
{
#if 0
  Value v = imu.readAccelData();
  Log("x:%d, y:%d, z:%d\r\n", v.x, v.y, v.z);
  delay(100);
  return;
#endif

  timeSetterLoop();
  btn.loop();

  if (btn.isPressed())
  {
    Log("Button pressed\r\n");

    // Turn on the buzzer for 500ms
    if (0) tone(BUZZER_PIN, BUZZER_FREQ, 500);
    if (0) sdTest();
    if (1) {
      Log("%u: Start capture (%u ms)\r\n", millis(), capture_time);
      digitalWrite(LED_BLUE, HIGH);
      capture_end_time = millis() + capture_time;
      capture_values = 0;
      imu.fifoBegin();
    }
  }

  if (capture_end_time) {
    if (millis() < capture_end_time) {
      unsigned cnt = imu.fifoReadCount();
      if (cnt) {
        //Log("%u: %u\r\n", millis(), cnt);
        while (cnt) {
          FifoPacket1 pkt = imu.fifoReadValue();
          if ((capture_values) && (pkt.header.Byte != FIFO_HEADER_ACC)) {
            Log("%u: %u: Invalid packet header: %02x\r\n", millis(), capture_values, pkt.header.Byte);
          }
          capture_values++;
          cnt--;
        }
      }
    } else {
      unsigned lost = imu.fifoReadLostCount();
      imu.fifoEnd();
      imu.fifoClear();
      Log("%u: Lost %u values\r\n", millis(), lost);
      Log("%u: Captured %u values\r\n", millis(), capture_values);
      Log("%u: Capture rate %f Hz\r\n", millis(), (capture_values * 1000.0f / capture_time));
      capture_end_time = 0;
      digitalWrite(LED_BLUE, LOW);
    }
  }
}

void sdTest(void) {
  Log("%u: sd: Start initialization\r\n", millis());
  if (!sd.begin(sdSpiConfig)) {
    Log("%u: sd: Initialization failed\r\n", millis());
    return;
  }
  Log("%u: sd: Initialization complete\r\n", millis());

  sd.ls("/");

  sd.end();

  Log("%u: sd: Test complete\r\n", millis());
}

void timeSetterLoop(void)
{
#if ENABLE_DS3231
  if (!Serial)
    return;

  if (!Serial.available())
    return;

  String buf = Serial.readStringUntil('\n');

  if (!buf.startsWith("time "))
    return;

  Log("%u: time: Command: '%s'\r\n", millis(), buf.c_str());

  unsigned day, month, year, hour, minute, second;
  int ret = sscanf(buf.c_str() + 5,
                   "%u.%u.%u %u:%u:%u",
                   &day, &month, &year,
                   &hour, &minute, &second);

  DateTime newTime(year, month, day, hour, minute, second);

  if (ret != 6)
  {
    Log("%u: time: Invalid format. Expected command: `time DD.MM.YYYY hh:mm::ss\\r\\n`\r\n", millis(), ret);
  }
  else if (!newTime.isValid())
  {
    Log("%u: time: Invalid time\r\n", millis());
  }
  else
  {
    Log("%u: time: Setting new time\r\n", millis());

    rtc.adjust(newTime);

    Log("%u: time: Time has been updated\r\n", millis());
  }
#endif
}

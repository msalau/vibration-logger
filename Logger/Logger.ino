#include <SdFat.h>
#include <ezButton.h>
#include <U8x8lib.h>
#include <RTClib.h>
#include <Wire.h>
#include "MyLSM6DS3.hpp"

typedef enum {
  STATE_IDLE,
  STATE_STARTING,
  STATE_LOGGING,
  STATE_STOPPING,
  STATE_ERROR,
  STATE_ERROR_DELAY,
} state_t;

int btn_prev = 1;
state_t state = STATE_IDLE;
uint32_t state_timer;
uint32_t imu_log_start_time;
uint32_t imu_log_stop_time;
uint32_t imu_log_data_points;

#define SD_SPI_FREQ SD_SCK_MHZ(32)
#define SD_CS_PIN D2
#define BUTTON_PIN D1
#define BUZZER_PIN D3
#define BUZZER_FREQ 1000
#define FILE_WRITE_PIN D0

ezButton btn(BUTTON_PIN, INPUT_PULLUP);
MyLSM6DS3 imu;
RTC_PCF8563 rtc;
U8X8_SSD1306_128X64_NONAME_HW_I2C lcd(/* clock=*/PIN_WIRE_SCL, /* data=*/PIN_WIRE_SDA, /* reset=*/U8X8_PIN_NONE);
ExFile file;
SdExFat sd;
SdSpiConfig sdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SPI_FREQ);

#define Log(...) \
  do { \
    if (Serial) \
      Serial.printf(__VA_ARGS__); \
  } while (0)

void setup() {
  Wire.begin();
  Wire1.begin();
  Wire.setClock(400000);
  Wire1.setClock(400000);
  Serial.begin(115200);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  digitalWrite(LED_RED, 1);
  digitalWrite(LED_GREEN, 1);
  digitalWrite(LED_BLUE, 1);

  pinMode(FILE_WRITE_PIN, OUTPUT);
  digitalWrite(FILE_WRITE_PIN, 0);

  rtc.begin();
  lcd.begin();
  lcd.setFlipMode(0);
  lcd.setFont(u8x8_font_chroma48medium8_r);
  lcd.clearDisplay();

  SdFile::dateTimeCallback(getFatDateTime);

  while (imu.begin() != IMU_SUCCESS) {
    lcd.setCursor(3, 4);
    lcd.print("IMU error!");
    digitalWrite(LED_RED, 0);
    delay(1000);
  }

  lcd.clearDisplay();

  state = STATE_IDLE;
}

void loop() {
  btn.loop();

  const int btn_new = btn.getState();
  const int btn_press = (btn_new != btn_prev) && (btn_new == 0);
  btn_prev = btn_new;

  if (btn_press)
    Log("%u: loop: btn press\r\n", millis());

  switch (state) {
    case STATE_IDLE:
      if (btn_press) {
        Log("%u: loop: start logging\r\n", millis());
        state = STATE_STARTING;
        break;
      }
      showTime();
      timeSetterLoop();
      break;
    case STATE_STARTING:
      Log("%u: loop: starting\r\n", millis());
      if (!sdLogStart()) {
        Log("%u: loop: Failed to start logging\r\n", millis());
        state = STATE_ERROR;
        break;
      }
      showRecording();
      digitalWrite(LED_GREEN, 0);
      state = STATE_LOGGING;
      break;
    case STATE_LOGGING:
      if (btn_press) {
        state = STATE_STOPPING;
      }
      if (!sdLogProcess()) {
        Log("%u: loop, Error occured while logging\r\n", millis());
        digitalWrite(LED_GREEN, 1);
        sdLogStop();
        state = STATE_ERROR;
        break;
      }
      break;
    case STATE_STOPPING:
      Log("%u: loop: stopping\r\n", millis());
      digitalWrite(LED_GREEN, 1);
      sdLogStop();
      lcd.clearDisplay();
      state = STATE_IDLE;
      break;
    case STATE_ERROR:
      Log("%u: loop: Handling error\r\n", millis());
      digitalWrite(LED_RED, 0);
      tone(BUZZER_PIN, BUZZER_FREQ);
      showError();
      state = STATE_ERROR_DELAY;
      state_timer = millis() + 1000;
      break;
    case STATE_ERROR_DELAY:
      if (millis() == state_timer) {
        Log("%u: loop: Switching back to idle\r\n", millis());
        digitalWrite(LED_RED, 1);
        noTone(BUZZER_PIN);
        state = STATE_IDLE;
      }
      break;
    default:
      Log("loop: Should not get here\r\n");
      delay(1000);
      break;
  }
}

DateTime getTime(void) {
  // rtc.getTime() is not atomic, so read it several times until identical values are read
  DateTime now1 = rtc.now();
  DateTime now2 = rtc.now();
  unsigned timeout = 10;

  while (now1 != now2 && timeout) {
    now1 = now2;
    now2 = rtc.now();
    timeout--;
  }

  return now2;
}

void getFatDateTime(uint16_t* date, uint16_t* time) {
  DateTime now = getTime();
  *date = FAT_DATE(now.year(), now.month(), now.day());
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

void showTime(void) {
  static DateTime prevTime;
  DateTime now = getTime();

  if (prevTime != now) {
    char str[16];

    snprintf(str, sizeof(str), "%02u.%02u.%04u", now.day(), now.month(), now.year());
    lcd.setCursor(3, 3);
    lcd.print(str);

    snprintf(str, sizeof(str), "%02u:%02u:%02u", now.hour(), now.minute(), now.second());
    lcd.setCursor(4, 4);
    lcd.print(str);
  }
}

void showRecording(void) {
  lcd.clearDisplay();
  lcd.setCursor(3, 4);
  lcd.print("Recording");
}

void showError(void) {
  lcd.clearDisplay();
  lcd.setCursor(5, 4);
  lcd.print("Error");
}

bool sdLogStart(void) {
  Log("%u: sd: Start initialization\r\n", millis());
  if (!sd.begin(sdSpiConfig)) {
    Log("%u: sd: Initialization failed\r\n", millis());
    return false;
  }
  Log("%u: sd: Initialization complete\r\n", millis());

  const DateTime now = getTime();
  char dirname[16];

  snprintf(dirname, sizeof(dirname),
           "%04u%02u%02u",
           now.year(), now.month(), now.day());

  if (sd.exists(dirname)) {
    Log("%u: sd: Directory %s exists\r\n", millis(), dirname);
  } else if (sd.mkdir(dirname)) {
    Log("%u: sd: Created directory %s\r\n", millis(), dirname);
  } else {
    Log("%u: sd: Failed to create directory %s\r\n", millis(), dirname);
    return false;
  }

  char filename[32];
  snprintf(filename, sizeof(filename),
           "%s/%02u%02u%02u.csv",
           dirname,
           now.hour(), now.minute(), now.second());

  Log("%u: sd: Opening %s\r\n", millis(), filename);
  bool opened = file.open(filename, (O_RDWR | O_CREAT | O_TRUNC));
  if (!opened) {
    Log("%u: sd: Failed to open %s for writing\r\n", millis(), filename);
    return false;
  }

  const char* header = "x,y,z\r\n";
  file.write(header, strlen(header));

  if (imu.fifoEnd() != IMU_SUCCESS) {
    Log("%u: log: Failed to stop FIFO\r\n", millis());
    return false;
  }

  imu.fifoClear();

  if (imu.fifoBegin() != IMU_SUCCESS) {
    Log("%u: log: Failed to start FIFO\r\n", millis());
    imu.fifoEnd();
    return false;
  }

  imu_log_start_time = millis();
  imu_log_data_points = 0;

  Log("%u: log: Started logging\r\n", millis());
  return true;
}

void sdLogError(const char* msg) {
  file.write(msg, strlen(msg));
  file.write("\r\n", 2);
}

bool sdLogProcess(void) {
  uint16_t fifoStatus;
  if (imu.fifoGetStatus(&fifoStatus) != IMU_SUCCESS) {
    Log("%u: imu: Failed to read FIFO status\r\n", millis());
    sdLogError("Failed to read FIFO status");
    return false;
  }
  if (fifoStatus & 0x6000) {
    Log("%u: imu: FIFO overflow\r\n", millis());
    sdLogError("FIFO overflow");
    return false;
  }

  unsigned fifoLength = (fifoStatus & 0x07FF) / 3;

  if (fifoLength == 0) {
    //Log("%u: log: No data\r\n", millis());
    return true;
  }

  uint16_t fifoPattern;
  if (imu.fifoGetPattern(&fifoPattern) != IMU_SUCCESS) {
    Log("%u: imu: Failed to read FIFO pattern\r\n", millis());
    sdLogError("Failed to read FIFO pattern");
    return false;
  }
  if (fifoPattern != 0) {
    Log("%u: imu: FIFO syncronization failed with pattern %u (status 0x%04x)\r\n", millis(), fifoPattern, fifoStatus);
    sdLogError("FIFO syncronization error");
    return false;
  }

  uint16_t fifoLeft = fifoLength;

  while (fifoLeft) {
    if (imu.isIrqSet()) {
      Log("%u: imu: FIFO Overflow Interrupt\r\n", millis());
      sdLogError("FIFO Overflow Interrupt");
      return false;
    }

    ImuRawValue value;
    status_t status = imu.fifoReadValue(&value);
    if (status != IMU_SUCCESS) {
      Log("%u: imu: Failed to read FIFO with code %d\r\n", millis(), status);
      sdLogError("FIFO read error");
      return false;
    }

    char str[128];
    size_t str_len = snprintf(
      str,
      sizeof(str),
      "%" PRIi16 ",%" PRIi16 ",%" PRIi16 "\r\n",
      value.x, value.y, value.z);

    digitalWrite(FILE_WRITE_PIN, 1);
    file.write(str, str_len);
    digitalWrite(FILE_WRITE_PIN, 0);
    fifoLeft--;
  }

  imu_log_data_points += fifoLength;
  if (fifoLength >= 100) {
    Log("%u: log: Received %u (+%u) data points\r\n", millis(), imu_log_data_points, fifoLength);
  }

  return true;
}

void sdLogStop(void) {
  imu_log_stop_time = millis();

  imu.fifoEnd();
  imu.fifoClear();

  if (file) {
    Log("%u: sd: Closing the file\r\n", millis());
    file.close();
  }

  Log("%u: sd: Unmounting the card\r\n", millis());
  sd.end();

  Log("%u: sd: Finished\r\n", millis());
  Log("%u: log: Captured %u data points during %u ms (%u Hz)\r\n",
      millis(),
      imu_log_data_points,
      (imu_log_stop_time - imu_log_start_time),
      (imu_log_data_points * 1000) / (imu_log_stop_time - imu_log_start_time));
}

void timeSetterLoop(void) {
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

  if (ret != 6) {
    Log("%u: time: Invalid format. Expected command: `time DD.MM.YYYY hh:mm::ss\\r\\n`\r\n", millis(), ret);
  } else if (!newTime.isValid()) {
    Log("%u: time: Invalid time\r\n", millis());
  } else {
    Log("%u: time: Setting new time\r\n", millis());

    rtc.stop();
    rtc.adjust(newTime);
    rtc.start();

    Log("%u: time: Time has been updated\r\n", millis());
  }
}

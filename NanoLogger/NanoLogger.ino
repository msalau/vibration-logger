#include <ezButton.h>
#include <RTClib.h>
#include <SdFat.h>
#include "MyADXL345.hpp"

// https://content.arduino.cc/assets/NanoV3.3_sch.pdf
// https://www.analog.com/media/en/technical-documentation/data-sheets/DS1307.pdf
// https://circuitdigest.com/sites/default/files/circuitdiagram_mic/Micro-SD-Card-Module-Schematic.png
// https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
// http://wiki.sunfounder.cc/images/thumb/a/a8/ADXL345_ad.png/731px-ADXL345_ad.png
// https://cdn.sparkfun.com/datasheets/BreakoutBoards/Logic_Level_Bidirectional.pdf

// PIN_A4 - I2C_SDA
// PIN_A5 - I2C_SCL

#define IMU_MOSI 4
#define IMU_MISO 5
#define IMU_CLK 6
#define IMU_CS 7

#define SD_CS_PIN 10
#define SD_SPI_FREQ 16000000

ezButton btn(2, INPUT_PULLUP);
RTC_DS1307 rtc;
MyADXL345<IMU_MISO, IMU_MOSI, IMU_CLK, IMU_CS> imu;
ExFile file;
SdExFat sd;
SdSpiConfig sdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SPI_FREQ);

typedef enum {
  STATE_IDLE,
  STATE_STARTING,
  STATE_LOGGING,
  STATE_STOPPING,
  STATE_ERROR,
} state_t;

state_t state;
uint32_t imu_log_start_time;
uint32_t imu_log_stop_time;
uint32_t imu_log_data_points;

#define Log(s) \
  do { \
    if (Serial) { \
      Serial.print(millis()); \
      Serial.print(": "); \
      Serial.println(s); \
    } \
  } while (0)

void setup() {
  Serial.begin(115200);

  // RTC interface
  Wire.begin();
  Wire.setClock(100000);

  // SD card interface
  SPI.begin();
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, 1);

  // Set Button debounce time (ms)
  btn.setDebounceTime(50);

  // RTC initialization
  while (!rtc.begin()) {
    Log("rtc: Initialization failed");
    delay(1000);
  }
  rtc.writeSqwPinMode(DS1307_OFF);

  // Accelerometer initialization
  while (!imu.begin()) {
    Log("imu: Initialization failed");
    delay(1000);
  }

  state = STATE_IDLE;
  Log("Started");
}

void loop() {
  btn.loop();

  const bool btn_press = btn.isPressed();

  if (btn_press)
    Log("loop: btn press");

  switch (state) {
    case STATE_IDLE:
      if (btn_press) {
        Log("loop: start logging");
        state = STATE_STARTING;
        break;
      }
      //showTime();
      //timeSetterLoop();
      break;
    case STATE_STARTING:
      //Log("loop: starting");
      //showRecording();
      //digitalWrite(LED_GREEN, 0);
      if (!sdLogStart()) {
        Log("loop: Failed to start logging");
        state = STATE_ERROR;
        break;
      }
      state = STATE_LOGGING;
      break;
    case STATE_LOGGING:
      if (btn_press) {
        state = STATE_STOPPING;
      }
      if (!sdLogProcess()) {
        Log("loop, Error occured while logging");
        //digitalWrite(LED_GREEN, 1);
        sdLogStop();
        state = STATE_ERROR;
        break;
      }
      break;
    case STATE_STOPPING:
      Log("loop: stopping");
      //digitalWrite(LED_GREEN, 1);
      sdLogStop();
      //lcd.clearDisplay();
      state = STATE_IDLE;
      break;
    case STATE_ERROR:
      //Log("loop: Handling error");
      //digitalWrite(LED_RED, 0);
      //tone(BUZZER_PIN, BUZZER_FREQ);
      //showError();
      state = STATE_IDLE;
      break;
    default:
      Log("loop: Should not get here\r\n");
      delay(1000);
      break;
  }
}

bool sdLogStart(void) {
  Log("sd: Start initialization");
  if (!sd.begin(sdSpiConfig)) {
    Log("sd: Initialization failed");
    return false;
  }
  Log("sd: Initialization complete");

  const DateTime now = rtc.now();;

#if 0
  char dirname[16];
  snprintf(dirname, sizeof(dirname),
           "%04u%02u%02u",
           now.year(), now.month(), now.day());

  if (sd.exists(dirname)) {
    Log("sd: Directory exists");
  } else if (sd.mkdir(dirname)) {
    Log("sd: Created directory");
  } else {
    Log("sd: Failed to create directory");
    return false;
  }
#endif

  char filename[16];
  snprintf(filename, sizeof(filename),
           "%02u%02u%02u.csv",
           now.hour(), now.minute(), now.second());

  Log("sd: Opening the output file");
  bool opened = file.open(filename, (O_RDWR | O_CREAT | O_TRUNC));
  if (!opened) {
    Log("sd: Failed to open the file for writing");
    return false;
  }

  const char* const header = "x,y,z\r\n";
  file.write(header, strlen(header));

  imu.fifoEnd();
  imu.fifoClear();

  imu_log_start_time = millis();
  imu_log_data_points = 0;

  Log("log: Started logging");

  imu.fifoBegin();

  return true;
}

void sdLogError(const char* msg) {
  file.write(msg, strlen(msg));
  file.write("\r\n", 2);
}

bool sdLogProcess(void) {
  unsigned fifoLength = imu.fifoGetStatus();

  if (fifoLength >= 24) {
    Log("imu: FIFO overflow");
    sdLogError("FIFO overflow");
    return false;
  }

  if (fifoLength == 0) {
    return true;
  }

  uint16_t fifoLeft = fifoLength;

  while (fifoLeft) {
    if (imu.isIrqSet()) {
      Log("imu: FIFO Overflow Interrupt");
      sdLogError("FIFO Overflow Interrupt");
      return false;
    }

    ImuRawValue value = imu.fifoReadValue();

    char str[24];
    size_t str_len = snprintf(
      str,
      sizeof(str),
      "%" PRIi16 ",%" PRIi16 ",%" PRIi16 "\r\n",
      value.x, value.y, value.z);

    //digitalWrite(FILE_WRITE_PIN, 1);
    file.write(str, str_len);
    //digitalWrite(FILE_WRITE_PIN, 0);

    fifoLeft--;
  }

  imu_log_data_points += fifoLength;

  return true;
}

void sdLogStop(void) {
  imu_log_stop_time = millis();

  imu.fifoEnd();
  imu.fifoClear();

  if (file.isOpen()) {
    Log("sd: Closing the file");
    file.close();
  }

  Log("sd: Unmounting the card");
  sd.end();

  Log("log: Captured data points:");
  Log(imu_log_data_points);
  Log("log: Capture duration (ms):");
  Log((imu_log_stop_time - imu_log_start_time));
  Log("log: Capture rate (Hz):");
  Log((imu_log_data_points * 1000) / (imu_log_stop_time - imu_log_start_time));
}

#include "Adafruit_LTR329_LTR303.h"
Adafruit_LTR303 ltr = Adafruit_LTR303();

#include <U8x8lib.h>
#include <Wire.h>
#define U8LOG_WIDTH 16
#define U8LOG_HEIGHT 8
uint8_t u8log_buffer[U8LOG_WIDTH * U8LOG_HEIGHT];
U8X8LOG u8x8log;

U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8(SCL, SDA, U8X8_PIN_NONE);

enum state { ACTIVE_LOGGING,  INTER_LOGGING,  COUNT_REPORT,  NADA } op_state = ACTIVE_LOGGING;
const char* stateStr[] = {"A-Logging", "I-Log", "Count"};
uint16_t focal_base;
uint16_t trigger_threshold;
uint16_t min_change_threshhold;
bool rising_trigger;
int avg_switching_duration;
int count_during_sleep;


// ------- Adjustable parameters -------
uint8_t change_margin = 8;                    // percentage change from focal to trigger the switching detection method
uint8_t pre_count_threshold = 20;             // Number of switches before switching from active to intermittent counting states
unsigned int timeout = 10000;                 // Time [ms] elapsed before considering the display as no longer switching
unsigned int deep_sleep_duration;             // For intermittent counting state, time period [ms] to put MCU into deep sleep
int max_count = 6000;                         // Upper limit for counting display switches

uint16_t visible_plus_ir, infrared;
uint16_t last_read;
unsigned long switching_duration;
bool timeOutFlag = false;
unsigned long switch_counter  = 0;
unsigned long sampling_period = 0;
int sampling_count = 10;



void setup() {
  Serial.begin(115200);
  Serial.println("\tSwitching Sensor Project - V3 State Machine");
  
  display_init();
  u8x8.println("Switch Counter");
  delay(500);
  u8x8.clear();

  if (!ltr.begin()) {
    u8x8.println("ERR: Light sensor");
    Serial.println("Couldn't find LTR sensor!");
    while (1) delay(10);
  }

  ltr.setGain(LTR3XX_GAIN_48);
  ltr.setIntegrationTime(LTR3XX_INTEGTIME_150);
  ltr.setMeasurementRate(LTR3XX_MEASRATE_50);

  // The LTR-303 has interrupt output support, we can enable the pin output!
  ltr.enableInterrupt(false);
  ltr.setInterruptPolarity(false);
  ltr.setLowThreshold(2000);
  ltr.setIntPersistance(2);

  calibration_seq();
  u8x8.clear(); u8x8.println("Calibration .O.");

}

void loop() {

  switch (op_state) {
    case ACTIVE_LOGGING:
      active_logging();
      break;
    case INTER_LOGGING:
      intermittent_logging();
      break;
    case COUNT_REPORT:
      count_report();
      break;
    case NADA:
      break;
  }

}


void active_logging() {
  // Wait for the first switch to start the logging session
  u8x8.println("Start test...");
  while (true) {
    readSensor();
    if (switch_change()) {
      Serial.println("Switch detected");
      u8x8.clear();
      break;
    }
  }

  Serial.println ("Counting number of switches");
  bool pre_counting_phase = true;
  bool in_switching;
  unsigned long startTime;
  unsigned long period_start = millis();
  display_pre();
  while (op_state == ACTIVE_LOGGING) {

    if (switch_change()) {
      switch_counter++;
      u8x8.setCursor(0, 1);
      u8x8.print("Count: ");
      u8x8.println(switch_counter);
      if (switch_counter % sampling_count == 0) {
        Serial.println(switch_counter);
        sampling_period = millis() - period_start;
        period_start = 0;

      }
      else {
        Serial.print(".");
      }
      in_switching = true;
      startTime = millis();

      while (in_switching) {
        if ( (switching_duration < (millis() - startTime)) && (!switch_change())) {
          in_switching = false;
        }
      }

    }

    if ( (millis() - startTime) > timeout) {
      in_switching = false;
      op_state = COUNT_REPORT;
      break;
    }

    if (switch_counter >= (pre_count_threshold - 1) ) {

      avg_switching_duration =  sampling_period / sampling_count;
      deep_sleep_duration = 5 * avg_switching_duration;
      count_during_sleep = floor(deep_sleep_duration / avg_switching_duration);    // Calc number of switches occurred during sleep

      Serial.print("Average Switching duration : "); Serial.println(avg_switching_duration);
      Serial.print("Count increment during sleep is : "); Serial.println(count_during_sleep);

      op_state = INTER_LOGGING;
    }
  }


}

void intermittent_logging() {

  unsigned long startTime;
  bool main_switching_state = true;
  bool in_switching = true;
  bool switch_check = false;

  Serial.println ("Main switch counting state initiated");
  sleep();

  while (op_state == INTER_LOGGING) {
    Serial.print('.');
    startTime = millis();
    while (!switch_check && main_switching_state && ((millis() - startTime) < (2 * avg_switching_duration) )) {

      if (switch_change()) {
        switch_counter = switch_counter + count_during_sleep;
        Serial.print("   Count: "); Serial.println(switch_counter);
        in_switching = true;
        switch_check = true;
        startTime = millis();
        while (in_switching) {
          if ( (switching_duration < (millis() - startTime)) && (!switch_change())) {
            in_switching = false;
            main_switching_state = false;
            break;
          }
        }

      }

      if ( (millis() - startTime) > timeout) {
        in_switching = false;
        op_state = COUNT_REPORT;
        break;
      }
    }

    if (!switch_check) {
      op_state = COUNT_REPORT;
      break;
    }

    main_switching_state = true;
    switch_check = false;
  }

}

void count_report() {
  Serial.print("\n ----------- Switching Timed OUT, Number of Switches: ");
  Serial.println(switch_counter);
  display_pre();
  u8x8.fillDisplay();
  for (int r = 0; r < u8x8.getRows(); r++ ){
    u8x8.clearLine(r);
    delay(100);
  }
  u8x8.setCursor(3, 2);
  u8x8.print("Count: ");
  u8x8.print(switch_counter);

  delay(5000);
  op_state = NADA;



  // check if passed the 50k mark
  // save count to eeprom

}

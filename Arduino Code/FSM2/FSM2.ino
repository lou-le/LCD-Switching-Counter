#include "Adafruit_LTR329_LTR303.h"
//#include "functions.ino"

Adafruit_LTR303 ltr = Adafruit_LTR303();

enum state {
  ACTIVE_LOGGING,
  INTER_LOGGING,
  COUNT_REPORT,
  NADA,
  TEST
} op_state = ACTIVE_LOGGING;

uint16_t focal_base;
uint16_t trigger_threshold;
uint16_t min_change_threshhold;
bool rising_trigger;
int avg_switching_duration;
int count_during_sleep;


// ------- Adjustable parameters -------
uint8_t change_margin = 8;                    // percentage change from focal to consider the output as changed
uint8_t pre_count_threshold = 11;             // Number of switches before switching from active to intermittent counting states
unsigned int timeout = 10000;                // Time [ms] elapsed before considering the display is no longer switching
unsigned int deep_sleep_duration ;    // For intermittent counting state, time period [ms] to put MCU into deep sleep
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
  Serial.println("\nSwitching Sensor Project - V3 State Machine");

  if (!ltr.begin() ) {
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

  //  Serial.print("Gain : ");
  //  Serial.print("Integration Time (ms): ");
  //  Serial.print("Measurement Rate (ms): ");
  //  Serial.print("Thresholds: ");  Serial.print(ltr.getLowThreshold());
  //  Serial.print(" & ");  Serial.println(ltr.getHighThreshold());
  //  Serial.print("Consecutive counts for IRQ: ");  Serial.println(ltr.getIntPersistance());


  calibration_seq();
  delay(500);

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
    case TEST:
      test();
    //delay(1000);
    case NADA:
      break;
  }

}


void test() {

  //  readSensor();
  //  Serial.print(visible_plus_ir); Serial.print("  -  ");
  //  Serial.println(isOutOfMargin(visible_plus_ir, focal_base, 10) );

  unsigned long startTime = 0;
  bool startFlag = false;

  if ((switch_change()) && (startFlag == false)) {
    startTime = millis();
    startFlag = true;
    Serial.print("Duration: ");
  }

  while (startFlag) {
    if (!switch_change()) {
      unsigned long duration = millis() - startTime;
      Serial.println(duration);
      startFlag = false;

    }
  }

}

void active_logging() {
  // Wait for the first switch to start the logging session
  while (true) {
    readSensor();
    if (switch_change()) {
      Serial.println("Switch detected");
      break;
    }
  }

  Serial.println ("Counting number of switches");
  bool pre_counting_phase = true;
  bool in_switching;
  unsigned long startTime;
  unsigned long period_start = millis();


  while (op_state == ACTIVE_LOGGING ) {

    if (switch_change()) {
      switch_counter++;
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
        //        if (  (3*switching_duration) <= (millis() - startTime) ) {
        //          in_switching = false;
        //           op_state = COUNT_REPORT;
        //        }
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

  Serial.println ("Main switch counting state initiated");

  while (op_state == INTER_LOGGING) {
    startTime = millis();
    while (main_switching_state && ((millis() - startTime) < (2 * avg_switching_duration) )) {

      if (switch_change()) {
        switch_counter = switch_counter + count_during_sleep;
        Serial.print("   Count: "); Serial.println(switch_counter);
        in_switching = true;
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
    
    main_switching_state = true;
    sleep();
  }

}

void sleep() {
  Serial.println("Gone to Sleep");
  delay(deep_sleep_duration);
}

void count_report() {

  Serial.print("\n ----------- Switching Timed OUT, Number of Switches: ");
  Serial.println(switch_counter);
  delay(5000);
  op_state = NADA;


  //check if passed the 50k mark
  // save count to eeprom

}

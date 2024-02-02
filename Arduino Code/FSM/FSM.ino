#include "Adafruit_LTR329_LTR303.h"

//#include "functions.ino"

Adafruit_LTR303 ltr = Adafruit_LTR303();

enum state {
  ACTIVE_LOGGING,
  INTER_LOGGING,
  COUNT_REPORT,
  NADA
}op_state = ACTIVE_LOGGING;



uint16_t focal_base;
uint16_t trigger_threshold;
uint16_t min_change_threshhold;
bool rising_trigger;


// ------- Adjustable parameters ------- 
uint8_t pre_count_threshold = 12;   // Number of switches before switching from active to intermittent counting states
unsigned int timeout = 10000;                // Time [ms] elapsed before considering the display is no longer switching
unsigned int deep_sleep_duration = 10000;    // For intermittent counting state, time period [ms] to put MCU into deep sleep
int max_count =6000 ;                      // Upper limit for counting display switches

uint16_t visible_plus_ir, infrared;
unsigned long switching_duration;

uint32_t visible_plus_IR;

bool timeOutFlag = false;

unsigned int switch_counter  = 0;


//void calibration_seq();
//void calibrate_dark();
//void set_switching_threshhold();

void setup() {
  Serial.begin(115200);
  Serial.println("\nSwitching Sensor Project - V2 State Machine");

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
  //ltr.setIntPersistance(2);

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
    case NADA:
      break;
  }

}


void active_logging() {
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

  while (op_state == ACTIVE_LOGGING ) {
    if (switch_change()) {
      switch_counter++;
      if (switch_counter % 10 == 0) {
        Serial.println(switch_counter);
      }
      else {
        Serial.print(".");
      }

      in_switching = true;
      startTime = millis();

      while (in_switching) {
        if ( (switching_duration < (millis() - startTime) ) && (!switch_change()) ) {
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
      op_state = INTER_LOGGING;
    }
  }


}

void intermittent_logging() {
  unsigned long startTime;

  Serial.println ("Main switch counting state initiated");
  bool main_switching_state = true;
  int count_during_sleep = floor(deep_sleep_duration / switching_duration);    // Calc number of switches occurred during sleep

  while (main_switching_state) {
    startTime = millis();
    while ( (millis() - startTime) < (2 * switching_duration) ) {

      if (switch_change()) {
        switch_counter = switch_counter + count_during_sleep + 1;
        break;
      }

    }

  }


  //LowPower.sleep(deep_sleep_duration);    // go to sleep for deep_sleep_duration

}

void count_report() {
  
  Serial.print("\n ----------- Switching Timed OUT, Number of Switches: ");
  Serial.println(switch_counter);
  delay(5000);
  op_state = NADA;
  

  //check if passed the 50k mark
  // save count to eeprom

}

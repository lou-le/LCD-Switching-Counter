#include "Adafruit_LTR329_LTR303.h"
Adafruit_LTR303 ltr = Adafruit_LTR303();


uint16_t focal_base;
uint16_t trigger_threshold;
uint16_t min_change_threshhold = 25;   // testing at 4% of the base value
bool switchRise;

uint16_t visible_plus_ir, infrared;
unsigned long duration;

int timeout = 10000;
bool timeOutFlag = false;

void calibration_seq();
void calibrate_dark();
void set_switching_threshhold();

void setup() {
  Serial.begin(115200);
  Serial.println("\nSwitching Sensor Project - V2 State Machine");

  if (!ltr.begin() ) {
    Serial.println("Couldn't find LTR sensor!");
    while (1) delay(10);
  }

  ltr.setGain(LTR3XX_GAIN_96);
  ltr.setIntegrationTime(LTR3XX_INTEGTIME_50);
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
}

void loop() {

  bool switching_start = true;
  while (switching_start) {
    readSensor();
    //int change = focal_base - visible_plus_ir;
    if (visible_plus_ir <= trigger_threshold) {
      //Serial.print("Lumen switch dropped below threshhold:  -- "); Serial.println(visible_plus_ir);
      switching_start = false;
      Serial.println("Switch detected");
    }
  }

  Serial.println ("Counting number of switches");

  bool counting_phase = true;
  bool in_switching;
  int switch_counter = 0;
  unsigned long startTime;
  while (counting_phase) {
    readSensor();
    // int change = focal_base - visible_plus_ir;
    if (visible_plus_ir <= trigger_threshold) {
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

        if ( ( duration < (millis() - startTime) ) and (visible_plus_ir < trigger_threshold)  ) {
          in_switching = false;
        }


      }
    }

    if (timeout < (millis() - startTime) ) {
      in_switching = false;
      timeOutFlag = true;
      counting_phase = false;
    }
  }

  if (timeOutFlag) {
    Serial.println("\nSwitching Timed OUT");
  }

}

void calibration_seq() {
  //----- Calibration Seq -------
  // read baseline
  // Tell user to start switching test
  // Display number of switches
  // Measure time intervals between switches
  // Display time interval

  calibrate_dark();
  set_switching_threshhold();


}

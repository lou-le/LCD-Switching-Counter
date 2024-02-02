// 1. Run calibration seq
// 2. Initial active logging
// 3. Sleep
// 4. Active logging
// 5. End Test detection


#include "Adafruit_LTR329_LTR303.h"
Adafruit_LTR303 ltr = Adafruit_LTR303();


uint16_t base_dark;
uint16_t trigger_threshold;
uint16_t margin = 20;   // testing at 4% of the base value

bool switching_start = false;
uint16_t visible_plus_ir, infrared;
unsigned long duration;

int timeout = 10000;

void setup() {
  Serial.begin(115200);
  Serial.println("Switching Sensor Project");

  if ( ! ltr.begin() ) {
    Serial.println("Couldn't find LTR sensor!");
    while (1) delay(10);
  }

  ltr.setGain(LTR3XX_GAIN_48);
  switch (ltr.getGain()) {
    case LTR3XX_GAIN_1: Serial.println(1); break;
    case LTR3XX_GAIN_2: Serial.println(2); break;
    case LTR3XX_GAIN_4: Serial.println(4); break;
    case LTR3XX_GAIN_8: Serial.println(8); break;
    case LTR3XX_GAIN_48: Serial.println(48); break;
    case LTR3XX_GAIN_96: Serial.println(96); break;
  }

  ltr.setIntegrationTime(LTR3XX_INTEGTIME_50);
  switch (ltr.getIntegrationTime()) {
    case LTR3XX_INTEGTIME_50: Serial.println(50); break;
    case LTR3XX_INTEGTIME_100: Serial.println(100); break;
    case LTR3XX_INTEGTIME_150: Serial.println(150); break;
    case LTR3XX_INTEGTIME_200: Serial.println(200); break;
    case LTR3XX_INTEGTIME_250: Serial.println(250); break;
    case LTR3XX_INTEGTIME_300: Serial.println(300); break;
    case LTR3XX_INTEGTIME_350: Serial.println(350); break;
    case LTR3XX_INTEGTIME_400: Serial.println(400); break;
  }

  ltr.setMeasurementRate(LTR3XX_MEASRATE_50);
  switch (ltr.getMeasurementRate()) {
    case LTR3XX_MEASRATE_50: Serial.println(50); break;
    case LTR3XX_MEASRATE_100: Serial.println(100); break;
    case LTR3XX_MEASRATE_200: Serial.println(200); break;
    case LTR3XX_MEASRATE_500: Serial.println(500); break;
    case LTR3XX_MEASRATE_1000: Serial.println(1000); break;
    case LTR3XX_MEASRATE_2000: Serial.println(2000); break;
  }

  // The LTR-303 has interrupt output support, we can enable the pin output!
  ltr.enableInterrupt(false);
  // The INT pin also has a polarity setting. For active LOW set to 'false',
  // for active HIGH set to 'true'
  ltr.setInterruptPolarity(false);

  // Then set the low threshold (values BELOW this trigger an interrupt)
  ltr.setLowThreshold(2000);
  // and set the high threshold (values ABOVE this trigger an interrupt)
  ltr.setHighThreshold(30000);


  // Finally, default is an interrupt on every value that is under/over the
  // threshold ranges. However, you're more likely to get spurious IRQs, so
  // we can set it to require "N counts in a row" before an IRQ. 1 count is
  // IRQ for each reading, 2 count means we need two outside readings in a row, etc
  // up to 16.
  ltr.setIntPersistance(4);

  //  Serial.print("Gain : ");
  //  Serial.print("Integration Time (ms): ");
  //  Serial.print("Measurement Rate (ms): ");
  //  Serial.print("Thresholds: ");  Serial.print(ltr.getLowThreshold());
  //  Serial.print(" & ");  Serial.println(ltr.getHighThreshold());
  //  Serial.print("Consecutive counts for IRQ: ");  Serial.println(ltr.getIntPersistance());


  calibration_seq();


}

void loop() {

  while (!switching_start) {
    readSensor();
    //int change = base_dark - visible_plus_ir;
    if (visible_plus_ir <= trigger_threshold) {
      //Serial.print("Lumen switch dropped below threshhold:  -- "); Serial.println(visible_plus_ir);
      switching_start = true;
      Serial.println("Switch detected");
    }
  }

  Serial.println ("Counting number of switches");

  bool counting_phase = true;
  bool in_switching;
  int switch_counter = 0;
  while (counting_phase) {
    readSensor();
    // int change = base_dark - visible_plus_ir;
    if (visible_plus_ir <= trigger_threshold) {
      switch_counter++;
      Serial.println(switch_counter);
      
      in_switching = true;
      unsigned long startTime = millis();
      while (in_switching) {
        if ( ((millis() - startTime) > duration) && (visible_plus_ir > trigger_threshold)  ) in_switching = false;

      }

    }
  }

}

uint16_t readSensor() {
  if (ltr.newDataAvailable()) {
    bool valid = ltr.readBothChannels(visible_plus_ir, infrared);
    if (valid) {
      delay(100);
      //Serial.println(visible_plus_ir);
      return visible_plus_ir;
    }
  }
}

void calibration_seq() {

  //----- Calibration Seq -------
  // read baseline
  // Tell user to start switching test
  // Display number of switches
  // Measure time intervals between switches
  // Display time interval

  uint16_t sum = 0;
  uint16_t count = 5;


  // Measure Dark State
  for (int i = 0; i <= count; i++) {
    readSensor();
    sum += visible_plus_ir;
    delay(200);
  }



  base_dark = sum / count;
  Serial.print("Base dark value: "); Serial.println(base_dark);
  Serial.print("Margin: "); Serial.println(margin);
  //Assign threshhold for swtiching
  trigger_threshold = base_dark - margin;
  Serial.print("Trigger Threshold value: "); Serial.println(trigger_threshold);

  // Tell user to switch display once
  Serial.println("--- Switch display for pulse width measurement");

  // Method to measure duration of switching state
  while (1) {
    unsigned long startTime;
    unsigned long endTime;
    uint16_t vmin;
    readSensor();
    if (visible_plus_ir <= trigger_threshold) {
      startTime = millis();
      Serial.print("Switch started");
      vmin = visible_plus_ir;

      //trigger_threshold = base_dark - ((base_dark - vmin) / 2);
      trigger_threshold = vmin + margin;


      Serial.print("New trigger_threshold is: ");
      Serial.println(trigger_threshold);

      while (visible_plus_ir <= trigger_threshold) {
        readSensor();
        Serial.print(".");
        //Serial.println(visible_plus_ir);
      }
      endTime = millis(); Serial.println("Done");

      duration = endTime - startTime;
      Serial.print("Switch State duration(ms): "); Serial.println(duration);
      delay(2000);
      break;
    }

  }

}

void readSensor() {
  if (ltr.newDataAvailable()) {
    bool valid = ltr.readBothChannels(visible_plus_ir, infrared);
    if (valid) {
      delay(100);
      //Serial.println(visible_plus_ir);
    }
  }
}

bool switch_change() {
  readSensor();
  if (rising_trigger) {
    return (visible_plus_ir >= trigger_threshold) ? true : false;
  } else { // falling trigger case
    return (visible_plus_ir <= trigger_threshold) ? true : false;
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

void calibrate_dark() {
  uint16_t sum = 0;
  uint16_t count = 5;

  // Measure Focal Conic State
  for (int i = 0; i < count; i++) {
    readSensor();
    sum += visible_plus_ir;
    delay(200);
  }
  focal_base = sum / count;
  Serial.print("Focal Conic value: "); Serial.println(focal_base);
  min_change_threshhold = 0.1 * focal_base;
  Serial.print("Min_change_threshhold: "); Serial.println(min_change_threshhold);

}

void set_switching_threshhold() {
  unsigned long startTime;
  unsigned long endTime;

  // Tell user to switch display once
  Serial.println("--- Switch display for pulse width measurement");
  delay(500);

  // Method to measure duration of switching state
  bool measuringSwitchRange = true;
  uint16_t vmin = visible_plus_ir;
  uint16_t vmax = visible_plus_ir;
  int some_count = 0;
  int val_diff;

  while (measuringSwitchRange) {
    readSensor();
    val_diff = abs( (int)(visible_plus_ir - focal_base) );

    if ( val_diff >= min_change_threshhold) {
      startTime = millis();
      Serial.print("Switch change detected");

      while (measuringSwitchRange) {
        readSensor();
        if (visible_plus_ir > vmax) {
          vmax = visible_plus_ir;
        }
        if (visible_plus_ir < vmin) {
          vmin = visible_plus_ir;
        }
        if (some_count > 3 ) {
          endTime = millis(); Serial.println("... Done");
          switching_duration = endTime - startTime;

          if ( (vmax - focal_base) > (focal_base - vmin) ) {
            rising_trigger = true;
          } else {
            rising_trigger = false;
          }

          measuringSwitchRange = false;
          Serial.print("Switching duration (ms): "); Serial.println(switching_duration);
          Serial.print("Waveform Max -- Min = "); Serial.print(vmax); Serial.print("--"); Serial.println(vmin);
          delay(2000);

        }
        if (min_change_threshhold >= abs( (int)(visible_plus_ir - focal_base) ) ) {
          some_count++;
        }
      }

    }


  }

  //Assign threshhold for switching
  if ( (vmax - focal_base) > (focal_base - vmin) ) {
    trigger_threshold = ((vmax - focal_base) * 0.3) + focal_base;
    rising_trigger = true;
  } else {
    trigger_threshold = focal_base - ((focal_base - vmin) * 0.3);
    rising_trigger = false;
  }
  Serial.print("Trigger Threshold value: "); Serial.println(trigger_threshold);
  Serial.print("Rising trigger is "); Serial.println(rising_trigger); Serial.println();

}

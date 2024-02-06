void display_init() {
  u8x8.setBusClock(100000);
  u8x8.begin();
  u8x8log.begin(u8x8, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
  u8x8log.setRedrawMode(1);    // 0: Update screen with newline, 1: Update screen for every char
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.setFlipMode(0);
  u8x8.clear();
}

void display_pre() {
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
  u8x8.clear();

  u8x8.inverse();
  u8x8.print(" ");  u8x8.print(stateStr[op_state]);  u8x8.print(" ");
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.noInverse();
  u8x8.setCursor(0, 1);
}

void readSensor() {
  digitalWrite(Emitter, HIGH);
  //  delay(100);
  if (ltr.newDataAvailable()) {
    bool valid = ltr.readBothChannels(visible_plus_ir, infrared);
    if (valid) {
      delay(100);
    }
  }
  digitalWrite(Emitter, LOW);
}

bool switch_change() {
  readSensor();
  return isOutOfMargin(visible_plus_ir, waveform.focal_base, change_margin);
}

void calibrate_dark() {     // Measure base values for Focal Conic State
  uint16_t focal_base;
  uint16_t min_change_threshhold;

  u8x8.clear();  u8x8.println("Dark Values.. ");
  uint16_t sum = 0;
  uint16_t count = 5;
  delay(500);

  for (int i = 0; i < count; i++) {
    readSensor();
    sum += visible_plus_ir;
    delay(100);
  }
  focal_base = sum / count;
  min_change_threshhold = 0.08 * focal_base;
  Serial.print("Focal Conic Base: "); Serial.println(focal_base);
  Serial.print("Min change threshhold: "); Serial.println(min_change_threshhold);
  u8x8.print("   Captured");

  waveform.focal_base = focal_base;

  delay(800);

}

void set_switching_threshhold() {
  unsigned long startTime;
  unsigned long endTime;
  uint16_t focal_base = waveform.focal_base;

  // Tell user to switch display once
  Serial.println("--- Switch display for pulse width measurement ---");
  u8x8.clear();  u8x8.println("Switch unit.. ");

  // Method to measure duration of switching state
  bool measuringSwitchRange = true;
  uint16_t vmin = visible_plus_ir;
  uint16_t vmax = visible_plus_ir;
  int some_count = 0;
  int val_diff;

  while (measuringSwitchRange) {
    readSensor();
    //val_diff = abs( (int)(visible_plus_ir - focal_base) );

    if ( switch_change()) {
      startTime = millis();
      Serial.print("Switch change detected");
      u8x8.print("  Detected");

      while (measuringSwitchRange) {
        readSensor();
        if (visible_plus_ir > vmax) {
          vmax = visible_plus_ir;
        }
        if (visible_plus_ir < vmin) {
          vmin = visible_plus_ir;
        }
        if (some_count > 3 ) {
          endTime = millis();
          Serial.println("... Done");
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
        if ( !switch_change() ) {
          some_count++;
        }
      }

    }


  }

  //    Rising/falling edge characterization
  if ( (vmax - focal_base) > (focal_base - vmin) ) {
    rising_trigger = true;
  } else {
    rising_trigger = false;
  }
  Serial.print("isRising: "); Serial.println(rising_trigger); Serial.println();

}

bool isOutOfMargin(int value, int target, int percentage) {
  double difference = abs(target - value);
  double percentDifference = (difference / target) * 100;
  return (percentDifference > percentage);
}

void sleep() {
  Serial.print(F("Gone to Sleep.."));
  digitalWrite(Indicator, HIGH);
  delay(deep_sleep_duration);
  Serial.print(F("Awake"));
  digitalWrite(Indicator, LOW);
}

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM6DSOX.h>
#include <vl53lx_class.h>
#include <arducam.h>

#include <rust_lib.h>


void setup() {
  Serial.begin(115200);
}

int count = 0;

void loop() {
  count++;
  Serial.print("Tick: ");
  Serial.print(double_input(count));
  Serial.println();
  sleep_ms(1000);
}

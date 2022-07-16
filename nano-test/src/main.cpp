#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM6DSOX.h>
#include <vl53lx_class.h>
#include <arducam.h>

#include <rust_lib.h>

extern "C" void ffi_usb_serial_write(const char* buffer, size_t size) {
  Serial.write(buffer, size);
}

void setup() {
  Serial.begin(115200);
}

int count = 0;

void loop() {
  count++;
  print_tick(count);
  sleep_ms(1000);
}

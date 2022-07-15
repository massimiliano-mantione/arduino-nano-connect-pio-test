#include <Arduino.h>

void setup() {
  Serial.begin(115200);
}

int count = 0;

void loop() {
  count++;
  Serial.print("Tick: ");
  Serial.print(count);
  Serial.println();
  sleep_ms(1000);
}
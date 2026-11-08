#include <Arduino.h>

const byte playCommand[] = {0xAA, 0x02, 0x00, 0xAC};
const byte stopCommand[] = {0xAA, 0x04, 0x00, 0xAE};

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.write(playCommand, sizeof(playCommand));
  delay(5000);
  Serial.write(stopCommand, sizeof(stopCommand));
}

void loop() {
}
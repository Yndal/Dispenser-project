#include <EEPROM.h>

#define I2C_ADDRESS 0

void setup() {
  // put your setup code here, to run once:
  EEPROM.write(I2C_ADDRESS, 0);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:

}

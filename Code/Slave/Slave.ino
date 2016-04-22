#include <EEPROM.h>
#include <Wire.h>

#define I2C_ADDRESS 0

#define CMD_SET_ADDR 0




//Dispenser slave
void setup() {
  // put your setup code here, to run once:
  int address = EEPROM.read(I2C_ADDRESS);

  Wire.begin(address);
  .Wire.onReceive(onReceiveEvent);
  
  if(address == 0){
    //Address has not been set
  }
}

void loop() {
  // put your main code here, to run repeatedly:



}


//NOTE This method will reset the arduino
void configureI2C(int address){
  EEPROM.write(I2C_ADDRESS, address);
  reset();
}

void onReceiveEvent(int value){



}
}


void reset(){
  //TODO
}


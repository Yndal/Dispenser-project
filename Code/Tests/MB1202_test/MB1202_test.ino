#include <i2c_t3.h>

//#include <i2c_t3.h>

// Wire Master Reader
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Reads data from an I2C/TWI slave device
// Refer to the "Wire Slave Sender" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


//#include <Wire.h>

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  delay(1000);
  pinMode(13, OUTPUT);
}

boolean led = true;
void loop()
{
  digitalWrite(13, led);
  led = !led;

  //Serial.println("1");

  Wire.beginTransmission(112); // transmit to device #44 (0x2c)
  Wire.send(224);             // sends value byte  
  Wire.send(81);             // sends value byte  
  Wire.endTransmission();
  delay(100);

  /*Serial.println("2");
  Wire.beginTransmission(112); // transmit to device #44 (0x2c)
  Wire.write(81);             // sends value byte  
  Wire.endTransmission();
  delay(1000);  */
  
  
  /*Serial.println("2");
  Wire.beginTransmission(112); // transmit to device #44 (0x2c)
  Wire.send(225);             // sends value byte  
  Wire.endTransmission();
  delay(100);*/

  //Serial.println("3");
  Wire.requestFrom(112,2);
 //  delay(200);
  int byteCounter = 0;
  int range = 0;
  while(Wire.available()){
    int value = Wire.read();
    Serial.print(byteCounter);Serial.print(": ");Serial.println(value);
    if(byteCounter == 0)
      range = value;
    else
      range += 256*value;//Wire.read();
    Serial.print("range: ");Serial.println(range);
    
  }

  Serial.print("byteCounter: "); Serial.println(byteCounter);
  /*byte high = Wire.read();
  byte low = Wire.read();

  int range = (high * 256) + low;  //compile the range integer from the two bytes received.
  */
  Serial.print("4: ");
  Serial.println(range);
  /*while (Wire.available())   // slave may send less than requested
  {
    byte c = Wire.read(); // receive a byte as character
    Serial.print(c);         // print the character
  }*/
  Serial.println("\n");

  delay(1000);
}

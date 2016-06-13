#include <Servo.h>

/*****************************************************************************
 * Local classes as tab environments can be troubled in Arduino and Teensy   *
 *****************************************************************************/

Servo servo;
#define SERVO_PIN 10
#define SERVO_PRESSURE_READ_PIN A9
#define SERVO_OPEN_VALUE 10
#define SERVO_CLOSE_VALUE 170
#define SERVO_INTERVAL_DEGREE 5
#define SERVO_PRESSURE_LIMIT 15

int currentServoDegree;



/***************************
 * Available commands
 **************************/
#define CMD_DISPENSE  'd'
#define CMD_REMOVE    'r'
#define CMD_CALIBRATE 'c'
#define CMD_ORIGO     'o'
#define CMD_LAYERS    'l'
#define CMD_WIDTH     'w'
#define CMD_HEIGHT    'h'
#define CMD_POSITION  'p'
#define CMD_STACKS    's'


#define CMD_OK        "ok"
#define CMD_UKNOWN    "unknown cmd"




/**********************************
 * System state variable - start
 *********************************/


void setup() {
  Serial.begin(9600);
  delay(1000); //Wait for serial to be ready
  
  pinMode(13, OUTPUT);

  servo.attach(SERVO_PIN);
  //pinMode(SERVO_PRESSURE_READ_PIN, INPUT);  Apparently not...

  currentServoDegree = SERVO_OPEN_VALUE;
  servo.write(currentServoDegree); 
  Serial.println("Dispener module loaded...");

}


long lastTime = 0;
boolean ledOn = true;
void loop() {
  //Make the LED blink to show that it is alive
  if (millis() - lastTime > 500) {
    ledOn = !ledOn;
    lastTime = millis();
    digitalWrite(13, ledOn);
  }


  if(Serial.available()){
    char c = Serial.read();
    if(c == 'o')
      openGripper();
    else if (c =='c')
      closeGripper();
  }
  //Serial.println(analogRead(SERVO_PRESSURE_READ_PIN));
}



void openGripper(){
  currentServoDegree = SERVO_CLOSE_VALUE;
  servo.write(currentServoDegree);
  Serial.println("Servo is open.\n");
}


void closeGripper(){
  Serial.print("Servo closing.... ");

  currentServoDegree = SERVO_OPEN_VALUE;
  servo.write(currentServoDegree);
  
/*
  int pressure = analogRead(SERVO_PRESSURE_READ_PIN);
  while(pressure < SERVO_PRESSURE_LIMIT && currentServoDegree < SERVO_CLOSE_VALUE){
    Serial.print("Pressure: ");
    Serial.println(pressure);

    int newValue = currentServoDegree + SERVO_INTERVAL_DEGREE;
    currentServoDegree = newValue;
    
    Serial.print("Value: ");
    Serial.println(newValue);

    servo.write(currentServoDegree);

    delay(150);
    pressure = analogRead(SERVO_PRESSURE_READ_PIN);
  } */
  
       
  Serial.println("Servo closed!\n");
}




#include <Servo.h>

Servo servo;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

  servo.attach(23);
  servo.write(90);
  
  pinMode(14, INPUT);
  pinMode(13, OUTPUT);
}

long lastTime = 0;
boolean on = true;
void loop() {
  if(millis() - lastTime > 500){
    on = !on;
    lastTime = millis();
    digitalWrite(13, on);
  }
  
 
  
  // put your main code here, to run repeatedly:
//  double val = analogRead(14);

 // Serial.println(val);


  String s = "";
  while(Serial.available()){
    s += (char) Serial.read();    
  }
  if(!s.equals("")){
    servo.write(s.toInt());
    Serial.println(s.toInt());
  }
  
}

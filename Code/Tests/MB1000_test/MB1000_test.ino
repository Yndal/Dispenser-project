void setup() {
  Serial.begin(9600);  // put your setup code here, to run once:
  delay(1000);
}

void loop() {
  int value = analogRead(A0);
  //value = (vcc/512)*inch
  double inch = value/(5.0/512.0/4*1000);//NB This line needs to be corrected!!
  double cm = inch*2.54;
  
  // put your main code here, to run repeatedly:
  Serial.println(value);//cm);
  delay(100);
}

#define PIN_A A5
#define PIN_D 2



void setup() {
  Serial.begin(9600);
  pinMode(PIN_D, INPUT);
  delay(1000);
}

void loop() {
  int val1 = analogRead(PIN_A);
  
  Serial.print("1: ");
  Serial.println(val1);
  

  delay(30);
}

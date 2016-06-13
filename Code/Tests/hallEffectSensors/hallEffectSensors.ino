const int hall49e = A0;
const int hall3144 = A1;

int window = 10;

void setup() {
  pinMode(hall49e, INPUT);
  pinMode(hall3144, INPUT);
  Serial.begin(9600);
}

void loop() {
  int sum49e = 0;
  int sum3144 = 0;
  for(int i = 0; i < window; i++){
      int value49e = analogRead(hall49e);
      int value3144 = analogRead(hall3144);
      
      sum49e += value49e;
      sum3144 += value3144;
  }
  sum49e = sum49e / window;
  sum3144 = sum3144 / window;
  Serial.print("hall 49E = ");
  Serial.println(sum49e);
  
  Serial.print("hall 3144 = ");
  Serial.println(sum3144);
  
  Serial.print("\n");
  delay(100);
}

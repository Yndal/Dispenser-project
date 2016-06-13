//End stops
#define VERT_END_STOP_START_PIN 18
#define VERT_END_STOP_END_PIN 19

#define HORI_END_STOP_START_PIN 20
#define HORI_END_STOP_END_PIN 21

#define EJECT_END_STOP_START_PIN 22
#define EJECT_END_STOP_END_PIN 23



void setup() {
  Serial.begin(9600);
  delay(1000);
  pinMode(VERT_END_STOP_START_PIN, INPUT);
  pinMode(VERT_END_STOP_END_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(VERT_END_STOP_START_PIN), stopVertAxisOrigo, FALLING);
  attachInterrupt(digitalPinToInterrupt(VERT_END_STOP_END_PIN), stopVertAxisExtrema, FALLING);

  pinMode(HORI_END_STOP_START_PIN, INPUT);
  pinMode(HORI_END_STOP_END_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(HORI_END_STOP_START_PIN), stopHoriAxisOrigo, FALLING);
  attachInterrupt(digitalPinToInterrupt(HORI_END_STOP_END_PIN), stopHoriAxisExtrema, FALLING);

  pinMode(EJECT_END_STOP_START_PIN, INPUT);
  pinMode(EJECT_END_STOP_END_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(EJECT_END_STOP_START_PIN), stopEjectAxisOrigo, FALLING);
  attachInterrupt(digitalPinToInterrupt(EJECT_END_STOP_END_PIN), stopEjectAxisExtrema, FALLING);
  Serial.println("Started");
}

void loop() {
  // put your main code here, to run repeatedly:
/*  int vertStart = digitalRead(VERT_END_STOP_START_PIN);
  int vertEnd = digitalRead(VERT_END_STOP_END_PIN);

  int horiStart = digitalRead(HORI_END_STOP_START_PIN);
  int horiEnd = digitalRead(HORI_END_STOP_END_PIN);

  int ejectStart = digitalRead(EJECT_END_STOP_START_PIN);
  int ejectEnd = digitalRead(EJECT_END_STOP_END_PIN);

  Serial.print("Values: ");
  Serial.print(vertStart);
  Serial.print(" ");
  Serial.print(vertEnd);
  Serial.print(" ");
  
  Serial.print(horiStart);
  Serial.print(" ");
  Serial.print(horiEnd);
  Serial.print(" ");
  
  Serial.print(ejectStart);
  Serial.print(" ");
  Serial.print(ejectEnd);
  Serial.print(" ");
  Serial.println();
*/
  //delay(10);
}


void stopVertAxisOrigo() {
  Serial.println("Vert origo");
}

void stopVertAxisExtrema() {
  Serial.println("Vert extrema");
}

void stopHoriAxisOrigo() {
  Serial.println("Hori origo");
}

void stopHoriAxisExtrema() {
  Serial.println("Hori extrema");
}

void stopEjectAxisOrigo() {
  Serial.println("Eject Origo");
}

void stopEjectAxisExtrema() {
  Serial.println("Eject extrema");
}

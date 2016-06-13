//End stops
#define VERT_END_STOP_ORIGO_PIN 18 //Z
#define VERT_END_STOP_MAX_PIN 19

#define HORI_END_STOP_ORIGO_PIN 3 //X
#define HORI_END_STOP_MAX_PIN 2

#define EJECT_END_STOP_ORIGO_PIN 14 //Y
#define EJECT_END_STOP_MAX_PIN 15



void setup() {
  Serial.begin(9600);
  delay(1000);
  pinMode(VERT_END_STOP_ORIGO_PIN, INPUT);
   pinMode(VERT_END_STOP_MAX_PIN, INPUT);
  // attachInterrupt(digitalPinToInterrupt(VERT_END_STOP_START_PIN), stopVertAxisOrigo, LOW);
   //attachInterrupt(digitalPinToInterrupt(VERT_END_STOP_END_PIN), stopVertAxisExtrema, LOW);

   pinMode(HORI_END_STOP_ORIGO_PIN, INPUT);
   pinMode(HORI_END_STOP_MAX_PIN, INPUT);
   //attachInterrupt(digitalPinToInterrupt(HORI_END_STOP_START_PIN), stopHoriAxisOrigo, LOW);
//   attachInterrupt(digitalPinToInterrupt(HORI_END_STOP_END_PIN), stopHoriAxisExtrema, LOW);

   pinMode(EJECT_END_STOP_ORIGO_PIN, INPUT);
   pinMode(EJECT_END_STOP_MAX_PIN, INPUT);
  // attachInterrupt(digitalPinToInterrupt(EJECT_END_STOP_START_PIN), stopEjectAxisOrigo, LOW);
   //attachInterrupt(digitalPinToInterrupt(EJECT_END_STOP_END_PIN), stopEjectAxisExtrema, LOW);
  Serial.println("Started");
}

void loop() {
  // put your main code here, to run repeatedly:
  int vertStart = digitalRead(VERT_END_STOP_ORIGO_PIN);
  int vertEnd = digitalRead(VERT_END_STOP_MAX_PIN);

  int horiStart = digitalRead(HORI_END_STOP_ORIGO_PIN);
  int horiEnd = digitalRead(HORI_END_STOP_MAX_PIN);

  int ejectStart = digitalRead(EJECT_END_STOP_ORIGO_PIN);
  int ejectEnd = digitalRead(EJECT_END_STOP_MAX_PIN);

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

  delay(10);
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

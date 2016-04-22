/*
 * <-- basic functions -->
 * moveVertAxis(boolean forward)
 * moveHoriAxis(boolean forward)
 * moveEjectAxis(boolean forward)
 * grabDish()
 * releaseDish()
 * 
 * 
 * <-- interrupts -->
 * detect change in stackers
 * 
 * 
 * 
 * <-- currentl unknown functionalities -->
 * Measure amount of dishes in stack
 * Amount of stacks (return int array with amount of dishes in each stack)
 * 
 * 
 * 
 */





#include <AccelStepper.h>
#include <Servo.h>



/********************************
 * Motion constants
 *******************************/
//Z axis
#define VERT_STEP_PIN 0 
#define VERT_DIR_PIN 1
#define VERT_ENA_PIN 2
#define VERT_MAX_SPEED 200//steps per seconds
#define VERT_MAX_ACCEL 10//steps per second squared

//X axis
#define HORI_STEP_PIN 4
#define HORI_DIR_PIN 5
#define HORI_ENA_PIN 6
#define HORI_MAX_SPEED 200//steps per seconds
#define HORI_MAX_ACCEL 10//steps per second squared

//Y axis
#define EJECT_STEP_PIN 8
#define EJECT_DIR_PIN 9
#define EJECT_ENA_PIN 10
#define EJECT_MAX_SPEED 200//steps per seconds
#define EJECT_MAX_ACCEL 10//steps per second squared


//End stops
#define VERT_END_STOP_START_PIN 1
#define VERT_END_STOP_END_PIN 2

#define HORI_END_STOP_START_PIN 3
#define HORI_END_STOP_END_PIN 4

#define EJECT_END_STOP_START_PIN 5
#define EJECT_END_STOP_END_PIN 6

AccelStepper vertAxis(1, VERT_STEP_PIN, VERT_DIR_PIN);
AccelStepper horiAxis(1, HORI_STEP_PIN, HORI_DIR_PIN);
AccelStepper ejectAxis(1, EJECT_STEP_PIN, EJECT_DIR_PIN);

Servo servo;
#define SERVO_PIN 23
#define SERVO_PRESSURE_READ_PIN 14
#define SERVO_LOWEST_DEGREE 10
#define SERVO_HIGHEST_DEGREE 170
#define SERVO_INTERVAL_DEGREE 5
#define SERVO_PRESSURE_LIMIT 512
int servoCurrentDegree = SERVO_LOWEST_DEGREE;


/***************************
 * Rx/Tx variables
 **************************/
#define RXTX_BAUD 9600 
#define MSG_SEP_CHAR ';'
String msg =  "";
boolean msgComplete = false;



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


#define CMD_OK "ok"
#define CMD_UKNOWN "unknown cmd"

/**********************************
 * System state variable - start
 *********************************/
//#ifndef Queue_h
  #include "Queue.h"
//#endif

#define STATE
#define STATE_INITIAL      0
#define STATE_MOVE_UP      1
#define STATE_MOVE_DOWN    2
#define STATE_MOVE_LEFT    3
#define STATE_MOVE_RIGHT   4
#define STATE_MOVE_EJECT   5
#define STATE_MOVE_RETRACT 6
#define STATE_RELEASE      7
#define STATE_GRAB         8

#define STATE_ERROR       -1

int state = STATE_INITIAL;
Queue cmdQueue = Queue();
/**********************************
 * System state variable - END
 *********************************/



/***************************
 * Testing parameters
 **************************/
#define TESTING true


void setup() {
  Serial.begin(RXTX_BAUD);
  msg.reserve(8*8); //Reserve enough bits for 8 chars
  cmdQueue.enqueue(3);
  pinMode(13, OUTPUT);

  servo.attach(SERVO_PIN);
  //pinMode(SERVO_PRESSURE_READ_PIN, INPUT);  Apparently not...
  
 /* pinMode(VERT_END_STOP_START_PIN, INPUT);
  pinMode(VERT_END_STOP_END_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(VERT_END_STOP_START_PIN), stopVertAxis, FALLING);
  attachInterrupt(digitalPinToInterrupt(VERT_END_STOP_END_PIN), stopVertAxis, FALLING);
  
  pinMode(HORI_END_STOP_START_PIN, INPUT);
  pinMode(HORI_END_STOP_END_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(HORI_END_STOP_START_PIN), stopHoriAxis, FALLING);
  attachInterrupt(digitalPinToInterrupt(HORI_END_STOP_END_PIN), stopHoriAxis, FALLING);
  
  pinMode(EJECT_END_STOP_START_PIN, INPUT);
  pinMode(EJECT_END_STOP_END_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(EJECT_END_STOP_START_PIN), stopEjectAxis, FALLING);
  attachInterrupt(digitalPinToInterrupt(EJECT_END_STOP_END_PIN), stopEjectAxis, FALLING);
*/
  pinMode(VERT_ENA_PIN, OUTPUT);
  pinMode(HORI_ENA_PIN, OUTPUT);
  pinMode(EJECT_ENA_PIN, OUTPUT);

  digitalWrite(VERT_ENA_PIN, LOW);
  digitalWrite(HORI_ENA_PIN, LOW);
  digitalWrite(EJECT_ENA_PIN, LOW);
  
  initialize();
  
  vertAxis.setMaxSpeed(VERT_MAX_SPEED);
  vertAxis.setAcceleration(VERT_MAX_ACCEL);

  horiAxis.setMaxSpeed(HORI_MAX_SPEED);
  horiAxis.setAcceleration(HORI_MAX_ACCEL);

  ejectAxis.setMaxSpeed(EJECT_MAX_SPEED);
  ejectAxis.setAcceleration(EJECT_MAX_ACCEL);

  vertAxis.setSpeed(VERT_MAX_SPEED0);
  horiAxis.setSpeed(HORI_MAX_SPEED);
  ejectAxis.setSpeed(EJECT_MAX_SPEED);
  
}

void initialize(){
  int initMaxSpeed = 500;
  int initMaxAccel = 50;

  vertAxis.setMaxSpeed(initMaxSpeed);
  vertAxis.setAcceleration(initMaxAccel);

  horiAxis.setMaxSpeed(initMaxSpeed);
  horiAxis.setAcceleration(initMaxAccel);

  ejectAxis.setMaxSpeed(initMaxSpeed);
  ejectAxis.setAcceleration(initMaxAccel);

  //Move to origo

  //Detect size of platform

  //Search for dish stacks
  
  //Find platform layers
  //Make sure to measure from a position where there is free stack

  
}

void stopVertAxis(){
  vertAxis.stop();
}

void stopHoriAxis(){
  horiAxis.stop();
}

void stopEjectAxis(){
  ejectAxis.stop();
}

void executeNextCmd(){
  if(cmdQueue.isEmpty())
    return;
  executingCmd = true;
  //cmdQueue;
  
  
}


/********************************
 * Command calls - start
 *******************************/
void dispense(int stack){
  Serial.print("Dispensing plate from stack: ");
  Serial.println(stack);
}

void remove(int stack){
  Serial.print("Removing plate to stack: ");
  Serial.println(stack);
}

void calibrate(){
  Serial.println("Calibrating. Please hold...");  
}
         
void goToOrigo(){
  Serial.println("Going to origo");

  vertAxis.setSpeed(-VERT_MAX_SPEED);

  vertAxis.setCurrentPosition(0);
  horiAxis.setCurrentPosition(0);
  ejectAxis.setCurrentPosition(0);
  
}

void returnLayers(){
  Serial.println("l[1,2,3,4]");
}

void returnWidth(){
  Serial.println("w42");
}

void returnHeight(){
  Serial.println("h43");
}

void returnPosition(){
  int v = vertAxis.currentPosition();
  int h = horiAxis.currentPosition();
  int e = ejectAxis.currentPosition();

  Serial.print('(');
  Serial.print(v);
  Serial.print(',');
  Serial.print(h);
  Serial.print(',');
  Serial.print(e);
  Serial.println(')');
}

void returnStacks(){
  Serial.println("[1,2,3,4,5,6,7,8]");
}

/********************************
 * Command calls - END
 *******************************/

/********************************
 * Primitive motion functions - start
 *******************************/
void moveVertial(long steps){
  
}

void moveHorizontal(long stops){
  
}

void moveGripper(long steps){
  
}

void releaseDish(){
  servo.write(SERVO_LOWEST_DEGREE);
}

void grabDish(){
  while(analogRead(SERVO_PRESSURE_READ_PIN) < SERVO_PRESSURE_LIMIT){
    servo.write(servoCurrentDegree + SERVO_INTERVAL_DEGREE);
    delay(100);
  }
}

/********************************
 * Primitive motion functions - END
 *******************************/


long lastTime = 0;
boolean ledOn = true;
boolean executingCmd = false;
long currentPosition[] = {0,0,0};
long targetPosition[] = {0,0,0};
void loop() {
  //Make the LED blink to show that it is alive
  if(millis() - lastTime > 500){
    ledOn = !ledOn;
    lastTime = millis();
    digitalWrite(13, ledOn);
  }

  /*vertAxis.run();
  horiAxis.run();
  ejectAxis.run();*/
  
  /*vertAxis.runSpeed();
  horiAxis.runSpeed();
  ejectAxis.runSpeed();
*/

  //If a command has finished
/*  if(executingCmd &&
    !vertAxis.isRunning() &&
    !horiAxis.isRunning() &&
    !ejectAxis.isRunning()){
      executingCmd = false;
    }*/

  if(!executingCmd && 
    !cmdQueue.isEmpty())
    executeNextCmd();

  if(msgComplete){
    handleMsg();
  }
}

void executeRunToPosition(){
  
}

void executeRunSpeed(){
  
}

void executeRun(){
  
}

boolean isDigit(char c){
  switch(c){
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      return true;

    default:
      return false;
  }
}

//NB Belt drive vs. lead screw
/*double stepsToMm(long steps, int stepsPerRound, double pulleyDiameterMm){
  double rounds = steps/stepsPerRound;
  double roundMm = pulleyDiameterMm*3.1415;

  return rounds * roundMm;
}*/

void handleMsg(){
  int index = msg.indexOf(MSG_SEP_CHAR);

  //Is there at least one full command
  if(index < 0)
    return;

  while(-1 < index){
    if(index == 0){ 
      //if msg starts with MSG_SEP_CHAR (empty command) then remove this
      msg = msg.substring(1);
      Serial.println(CMD_UKNOWN);
    } else {
      
      String cmd = msg.substring(0,index);
      msg = msg.substring(index+1); //Remove MSG_SEP_CHAR
    
      //Serial.print("Got command: "); Serial.println(cmd);

      boolean isValidCmd = false;
      switch(cmd[0]){
        case CMD_DISPENSE:{
            if(!isDigit(cmd[1]))
              break;

            isValidCmd = true;
            int stack = -1;
            stack = String(cmd[1]).toInt() ;           
            dispense(stack);
          break;
        }
        case CMD_REMOVE:{
          if(!isDigit(cmd[1]))
              break;

            isValidCmd = true;
            int stack = -1;
            stack = String(cmd[1]).toInt() ;           
            remove(stack);
          break;
        }
        case CMD_CALIBRATE:{
          isValidCmd = true;
          calibrate();
          break;
        }
        case CMD_ORIGO:{
          isValidCmd = true;
          goToOrigo();        
          break;
        }
        case CMD_LAYERS:{
          isValidCmd = true;
          returnLayers();
          break;
        }
        case CMD_WIDTH:{
          isValidCmd = true;
          returnWidth();
          break;
        }
        case CMD_HEIGHT:{
          isValidCmd = true;
          returnHeight();
          break;
        }
        case CMD_POSITION:{
          isValidCmd = true;
          returnPosition();
          break;
        }
        case CMD_STACKS:{
          isValidCmd = true;
          returnStacks();
          break;
        }
        default:{
          //Nothing...
        }
      }
      if(!isValidCmd){
        Serial.print("Invalid command: ");
        Serial.println(cmd);
      } else {
        Serial.println(CMD_OK);
      }
    }
    index = msg.indexOf(MSG_SEP_CHAR);
  }

  
}

/*
 SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the msg:
    msg += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == MSG_SEP_CHAR) {
      msgComplete = true;
    }
  }
}


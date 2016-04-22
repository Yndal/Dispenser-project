#include <AccelStepper.h>
#include <Servo.h>

/*****************************************************************************
 * Local classes as tab environments can be troubled in Arduino and Teensy   *
 *****************************************************************************/
struct Command {
  public:
    typedef enum {
      DISPENSE,
      REMOVE,
      CALIBRATE,
      TO_ORIGO,
      ___EMPTY___
    } Action;

    Action action;
    int stack;//Number

    Command() {
      Command(___EMPTY___);
    }

    Command(Action action) {
      Command(action, -1);
    }

    Command(Action action, int stackNumber) {
      this->action = action;
      this->stack = stackNumber;
    }

    

};

class Motion {
  public:
    typedef enum {
      VERTICAL,
      HORIZONTAL,
      EJECT,
      GRIP,
      ___EMPTY
    } Axis;

    Axis axis;
    long absPosition;

    Motion(){
      Motion(Motion::___EMPTY, 0);
    }
    Motion(Axis axis, long absPosition) {
      this->axis = axis;
      this->absPosition  = absPosition;
    }
};



/********************************
 * Motion constants
 *******************************/
//Z axis
#define VERT_STEP_PIN 0
#define VERT_DIR_PIN 1
#define VERT_ENA_PIN 2
#define VERT_MAX_SPEED 1500//steps per seconds
#define VERT_MAX_ACCEL 300//steps per second squared

//X axis
#define HORI_STEP_PIN 4
#define HORI_DIR_PIN 5
#define HORI_ENA_PIN 6
#define HORI_MAX_SPEED 1500//steps per seconds
#define HORI_MAX_ACCEL 300//steps per second squared

//Y axis
#define EJECT_STEP_PIN 8
#define EJECT_DIR_PIN 9
#define EJECT_ENA_PIN 10
#define EJECT_MAX_SPEED 1500//steps per seconds
#define EJECT_MAX_ACCEL 300//steps per second squared


//End stops
#define VERT_END_STOP_START_PIN 1
#define VERT_END_STOP_END_PIN 2

#define HORI_END_STOP_START_PIN 3
#define HORI_END_STOP_END_PIN 4

#define EJECT_END_STOP_START_PIN 5
#define EJECT_END_STOP_END_PIN 6

AccelStepper vertAxis(AccelStepper::DRIVER, VERT_STEP_PIN, VERT_DIR_PIN);
AccelStepper horiAxis(AccelStepper::DRIVER, HORI_STEP_PIN, HORI_DIR_PIN);
AccelStepper ejectAxis(AccelStepper::DRIVER, EJECT_STEP_PIN, EJECT_DIR_PIN);

Servo servo;
#define SERVO_PIN 23
#define SERVO_PRESSURE_READ_PIN 14
#define SERVO_OPEN 10
#define SERVO_CLOSET 170
#define SERVO_INTERVAL_DEGREE 5
#define SERVO_PRESSURE_LIMIT 512
#define GRIPPER_CLOSE 0
#define GRIPPER_OPEN  1
int servoCurrentDegree;

/*************************************
 * Distance related
 ************************************/
#include <Wire.h>
#include <VL6180X.h>

VL6180X vl6180x;

#define SWITCH_PIN 14





/*************************************
 * Motion variables
 ************************************/
#define MOVE_FARHTER_THAN_ORIGO   -1
#define MOVE_FARHTER_THAN_EXTREMA -2
#define MOTION_QUEUE_LENGTH 32
 
boolean executingCmd   = false;
long currentPosition[] = {0, 0, 0, 0};
long targetPosition[]  = {0, 0, 0, 0};
long maxPosition[]  = {100, 100, 100, 1};
Motion* motionQueue[MOTION_QUEUE_LENGTH];
int motionQueueStart = 0;
int motionQueueEnd   = motionQueueStart;

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


#define CMD_OK        "ok"
#define CMD_UKNOWN    "unknown cmd"




/**********************************
 * System state variable - start
 *********************************/
#define CMD_QUEUE_LENGTH 64
Command* cmdQueue[CMD_QUEUE_LENGTH];
int cmdQueueStart = 0;
int cmdQueueEnd = cmdQueueStart;

#define STATE_ERROR               0
#define STATE_IDLE                1
#define STATE_RUNNING_TO_POSITION 2
#define STATE_RUNNING_TO_SPEED    3
#define STATE_RUNNING             4
#define STATE_GRIPPING            5

int _state = STATE_IDLE;


/**********************************
 * System state variable - END
 *********************************/

/***************************************
 * Stack related (Dish stacks) - start *
 **************************************/
#define MAX_STACKS 8
int stacks[MAX_STACKS]            = {42,-1,-1,-1,-1,-1,-1,-1}; //Amount of dishes in each stack
long stackPositions[MAX_STACKS][3] = {{50,50,50},{0,0,0},{0,0,0},{0,0,0},
                                      {0,0,0},{0,0,0},{0,0,0},{0,0,0}};


/***************************************
 * Stack related (Dish stacks) - end   *
 **************************************/

long platformInsertionPos[3] = {20,20,20};
long layers[] = {-1,-1,-1,-1,
                 -1,-1,-1,-1};




/***************************
 * Testing parameters
 **************************/
//#define TESTING true


void setup() {
  Serial.begin(RXTX_BAUD);
  delay(1000); //Wait for serial to be ready
  msg.reserve(64 * 8); //Reserve enough bits for 64 chars
  //cmdQueue[0] = new Command(Command::CALIBRATE);
  //cmdQueue.enqueue(3);
  pinMode(13, OUTPUT);

  servo.attach(SERVO_PIN);
  //pinMode(SERVO_PRESSURE_READ_PIN, INPUT);  Apparently not...

  /* pinMode(VERT_END_STOP_START_PIN, INPUT);
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
  */
  pinMode(VERT_ENA_PIN, OUTPUT);
  pinMode(HORI_ENA_PIN, OUTPUT);
  pinMode(EJECT_ENA_PIN, OUTPUT);

  digitalWrite(VERT_ENA_PIN, LOW);
  digitalWrite(HORI_ENA_PIN, LOW);
  digitalWrite(EJECT_ENA_PIN, LOW);

  initDistanceSensor();
  initialize();

  vertAxis.setMaxSpeed(VERT_MAX_SPEED);
  vertAxis.setAcceleration(VERT_MAX_ACCEL);

  horiAxis.setMaxSpeed(HORI_MAX_SPEED);
  horiAxis.setAcceleration(HORI_MAX_ACCEL);

  ejectAxis.setMaxSpeed(EJECT_MAX_SPEED);
  ejectAxis.setAcceleration(EJECT_MAX_ACCEL);

  /*vertAxis.setSpeed(VERT_MAX_SPEED);
  horiAxis.setSpeed(HORI_MAX_SPEED);
  ejectAxis.setSpeed(EJECT_MAX_SPEED);*/

}

void initDistanceSensor(){
  Wire.begin();
  
  vl6180x.init();
  vl6180x.configureDefault();
  vl6180x.setTimeout(500);

  pinMode(SWITCH_PIN, INPUT);
}

void initialize() {
  /*int initMaxSpeed = 50;
  int initMaxAccel = 5;

  vertAxis.setMaxSpeed(initMaxSpeed);
  vertAxis.setAcceleration(initMaxAccel);

  horiAxis.setMaxSpeed(initMaxSpeed);
  horiAxis.setAcceleration(initMaxAccel);

  ejectAxis.setMaxSpeed(initMaxSpeed);
  ejectAxis.setAcceleration(initMaxAccel);
*/
  //Release gripper and move to origo
  targetPosition[3] = GRIPPER_OPEN;
  openCloseGripper();
  // cmdQueue
  vertAxis.setCurrentPosition(0);
  horiAxis.setCurrentPosition(0);
  ejectAxis.setCurrentPosition(0);


  //while(!cmdQueue.isEmpty())
  //;//    executeCmd();

  //Detect size of platform

  //Search for dish stacks

  //Find platform layers
  //Make sure to measure from a position where there is free stack


}

void stopVertAxisOrigo() {
  targetPosition[0] = 0;
  vertAxis.setCurrentPosition(0);
  vertAxis.moveTo(0);
  //vertAxis.stop();
}

void stopVertAxisExtrema() {
  long pos = vertAxis.currentPosition();
  targetPosition[0] = pos;
  maxPosition[0] = pos;
  vertAxis.moveTo(pos);
  //vertAxis.stop();
}

void stopHoriAxisOrigo() {
  targetPosition[1] = 0;
  horiAxis.setCurrentPosition(0);
  horiAxis.moveTo(0);
  //vertAxis.stop();
}

void stopHoriAxisExtrema() {
  long pos = horiAxis.currentPosition();
  targetPosition[1] = pos;
  maxPosition[1] = pos;
  horiAxis.moveTo(pos);
  //vertAxis.stop();
}

void stopEjectAxisOrigo() {
  targetPosition[2] = 0;
  ejectAxis.setCurrentPosition(0);
  ejectAxis.moveTo(0);
  //vertAxis.stop();
}

void stopEjectAxisExtrema() {
  long pos = ejectAxis.currentPosition();
  targetPosition[2] = pos;
  maxPosition[2] = pos;
  ejectAxis.moveTo(pos);
  //vertAxis.stop();
}



/********************************
 * Command calls - start
 *******************************/
void dispense(int stack) {
  Serial.print("Dispensing plate from stack: ");
  Serial.println(stack);
  //Command* c = new Command(Command::DISPENSE, stack);
  
  cmdQueue[cmdQueueEnd++] = new Command(Command::DISPENSE, stack);
  if (CMD_QUEUE_LENGTH - 1 < cmdQueueEnd)
    cmdQueueEnd = 0;
}

void remove(int stack) {
  Serial.print("Removing plate to stack: ");
  Serial.println(stack);

  cmdQueue[cmdQueueEnd++] = new Command(Command::REMOVE, stack);
  if (CMD_QUEUE_LENGTH - 1 < cmdQueueEnd)
    cmdQueueEnd = 0;
}

void calibrate() {
  Serial.println("Calibrating. Please hold...");

  cmdQueue[cmdQueueEnd++] = new Command(Command::CALIBRATE,0);
  if (CMD_QUEUE_LENGTH - 1 < cmdQueueEnd)
    cmdQueueEnd = 0;
}

void goToOrigo() {
  Serial.println("Going to origo");

  //  servo.write
 // vertAxis.setSpeed(-VERT_MAX_SPEED);

  cmdQueue[cmdQueueEnd++] = new Command(Command::TO_ORIGO);
  if (CMD_QUEUE_LENGTH - 1 < cmdQueueEnd)
    cmdQueueEnd = 0;

}

void returnLayers() {
  Serial.print('[');
  int l = sizeof(layers)/sizeof(long);
  for(int i=0; i<l; i++){
    Serial.print(layers[i]);
    if(i < l-1)
      Serial.print(',');
  }
  Serial.print(']');
  //Serial.println("l[1,2,3,4]");
}

void returnWidth() {
  Serial.println(maxPosition[1]);
  //Serial.println("w42");
}

void returnHeight() {
  Serial.println(maxPosition[0]);
  //Serial.println("h43");
}

void returnPosition() {
  long v = vertAxis.currentPosition();
  long h = horiAxis.currentPosition();
  long e = ejectAxis.currentPosition();

  Serial.print('(');
  Serial.print(v);
  Serial.print(',');
  Serial.print(h);
  Serial.print(',');
  Serial.print(e);
  Serial.println(')');
}

void returnStacks() {
  Serial.print("[");
  for(int i=0; i<MAX_STACKS; i++){
    Serial.print(stacks[i]);
    if(i!= MAX_STACKS-1)
      Serial.print(',');
  }
  Serial.println(']');
}

/********************************
 * Command calls - END
 *******************************/

/********************************
 * Primitive motion functions - start
 *******************************/

void openCloseGripper(){
  long pos = targetPosition[3];
  if(pos == GRIPPER_CLOSE){
    //while(analogRead(SERVO_PRESSURE_READ_PIN) < SERVO_PRESSURE_LIMIT){
    //  servo.write(servoCurrentDegree + SERVO_INTERVAL_DEGREE);
    //  delay(100);
    
    Serial.print("Grabbing dish (");
    Serial.print(SERVO_CLOSET);
    Serial.println(')');
    
    servo.write(SERVO_CLOSET); 
    currentPosition[3] = GRIPPER_CLOSE; 
    
    delay(1000);
    
    //}        
   
  } else if (pos == GRIPPER_OPEN) {
    Serial.print("Releasing dish (");
    Serial.print(SERVO_OPEN);
    Serial.println(')');
    
    servo.write(SERVO_OPEN);
    
    currentPosition[3] = GRIPPER_OPEN;
    
    delay(1000);  
  } else {
    //ignore
  }
  //currentPosition[3] = targetPosition[3];
  //printPosition();
}


/********************************
 * Primitive motion functions - END
 *******************************/


long lastTime = 0;
boolean ledOn = true;
void loop() {
  //Make the LED blink to show that it is alive
  if (millis() - lastTime > 500) {
    ledOn = !ledOn;
    lastTime = millis();
    digitalWrite(13, ledOn);
  }

 
  handleCommands();
  handleMotions();

  getSerial();
  if (msgComplete) {
    handleMsg();
  }

  getDistance();
}


void getDistance(){
  //VL6180X
  int vl6180xValue = vl6180x.readRangeSingle();
  
  //MB1202
  int mb1202Value = 42;
  
  //Lever switch
  int switchValue = digitalRead(SWITCH_PIN);

  Serial.print("Distances: ");
  
  Serial.print("VL6180X (");
  Serial.print(vl6180xValue);
  Serial.print(") - ");
  
  Serial.print("MB1202 (");
  Serial.print(mb1202Value);
  Serial.print(") - ");
  
  Serial.print("Lever switch (");
  Serial.print(switchValue);
  Serial.print(")");

  Serial.println("\n");



  
}


void handleMotions(){
  //Return if motion queue is empty
  if(motionQueueStart == motionQueueEnd){
    //Serial.println("STATE_IDLE");
    _state = STATE_IDLE;
    return;
  }
    
  Motion* motion = motionQueue[motionQueueStart];

  boolean motionDone = false;
  switch(motion->axis){
    case Motion::VERTICAL:{
      long pos = vertAxis.currentPosition();

      //Dynamically update max and min positions
      if(pos < 0){
        vertAxis.setCurrentPosition(0);
        pos = 0;
      } else if (maxPosition[0] < pos){
        maxPosition[0] = pos;
        targetPosition[0] = pos + 1;
        vertAxis.moveTo(pos + 1);
      }
      
      if(pos == targetPosition[0]){
        //Serial.println("Reached vertical");
        motionDone = true;
      }
      break;
    }
    case Motion::HORIZONTAL:{
      long pos = horiAxis.currentPosition();

      //Dynamically update max and min positions
      if(pos < 0){
        horiAxis.setCurrentPosition(0);
        pos = 0;
      } else if (maxPosition[1] < pos){
        maxPosition[1] = pos;
        targetPosition[1] = pos + 1;
        horiAxis.moveTo(pos + 1);
      }
      
      if(pos == targetPosition[1]){
        //Serial.println("Reached horizontal");
        motionDone = true;
      }
      break;
    }
    case Motion::EJECT:{
      long pos = ejectAxis.currentPosition();

      //Dynamically update max and min positions
      if(pos < 0){
        ejectAxis.setCurrentPosition(0);
        pos = 0;
      } else if (maxPosition[2] < pos){
        maxPosition[2] = pos;
        targetPosition[2] = pos + 1;
        ejectAxis.moveTo(pos + 1);
      }
      
      if(pos == targetPosition[2]){
        //Serial.println("Reached ejection");
        motionDone = true;
      }
      break;
    }
    case Motion::GRIP:{
      printPosition();
      if(currentPosition[3] == targetPosition[3]){
        motionDone = true;
        //Serial.println("Reached gripper");
      }
      break;
    }
    default:
      Serial.println("Fails here!!!");
      //Move on from the failure...
      motionDone=true;
  }

  //If current motion is done, start the next one
  if(motionDone){
   // Serial.print("Motion "); Serial.print(motionQueueStart); Serial.println(" done");
    motionQueueStart++;
    if(MOTION_QUEUE_LENGTH - 1 <motionQueueStart)
      motionQueueStart = 0;

    //Return if motion queue is empty
    if(motionQueueStart == motionQueueEnd){
      _state = STATE_IDLE;
      return;
    }

    Motion* m = motionQueue[motionQueueStart];
    switch(m->axis){
      case Motion::VERTICAL:{
        long pos = m->absPosition;
        if(pos == MOVE_FARHTER_THAN_ORIGO){
         /* pos = -1;
          Serial.println("Vert: Farther than origo");
        } else if(pos == MOVE_FARHTER_THAN_EXTREMA)
          pos = maxPosition[0] + 1;
          */Serial.println("FAILS HERE");// sdfghj;
          }
        Serial.print("Vert: Go to ");
        Serial.println(pos);
        vertAxis.moveTo(pos);
        targetPosition[0] = pos;
        //TODO May need to call setSpeed() to keep a constant speed. Line 343 at: http://www.airspayce.com/mikem/arduino/AccelStepper/AccelStepper_8h_source.html
        _state = STATE_RUNNING;      
        
        break;
      }
      case Motion::HORIZONTAL:{
        Serial.print("Hori: Go to ");
        Serial.println(m->absPosition);
        horiAxis.moveTo(m->absPosition);
        targetPosition[1] = m->absPosition;
        //TODO May need to call setSpeed() to keep a constant speed. Line 343 at: http://www.airspayce.com/mikem/arduino/AccelStepper/AccelStepper_8h_source.html
        _state = STATE_RUNNING;   
        
        break;
      }
      case Motion::EJECT:{
        Serial.print("Eject: Go to ");
        Serial.println(m->absPosition);
        ejectAxis.moveTo(m->absPosition);
        targetPosition[2] = m->absPosition;
        //TODO May need to call setSpeed() to keep a constant speed. Line 343 at: http://www.airspayce.com/mikem/arduino/AccelStepper/AccelStepper_8h_source.html
        _state = STATE_RUNNING;   
        break;
      }
      case Motion::GRIP:{
        Serial.print("Gripper: Go to ");
        Serial.println(m->absPosition);
        long pos = m->absPosition;
        targetPosition[3] = pos;
        _state = STATE_GRIPPING;
        
        break;
      }
      default:
      ;//Nothing
    }
  }
}


void handleCommands() {
  //Return if empty
  if (cmdQueueStart == cmdQueueEnd &&
      _state == STATE_IDLE){    
        //Serial.println("no commands");    
      return;
    }

  switch (_state) {
    case STATE_IDLE:
      //Start next comamnd
      startNextCommand();
      break;
    case STATE_RUNNING_TO_POSITION:
      executeRunToPosition();
      break;
    case STATE_RUNNING_TO_SPEED:
      executeRunSpeed();
      break;
    case STATE_RUNNING:
      executeRun();
      break;
    case STATE_GRIPPING:
      executeGrip();
      break;
    default:
      ;//Nothing
  }
}




void startNextCommand() {
  //Serial.println("Starting next command");

  //Command queue is empty
  if (cmdQueueStart == cmdQueueEnd)
    return;

  //Get command
  Command* cmd = cmdQueue[cmdQueueStart];

  //Increase starting point of queue
  cmdQueueStart++;
  if (CMD_QUEUE_LENGTH - 1 < cmdQueueStart)
    cmdQueueStart = 0;

  Serial.print("Next command is: ");
  switch (cmd->action) {
    case Command::___EMPTY___:
      //Do nothing
      Serial.println("___EMPTY___");
      break;
    case Command::DISPENSE: {
      Serial.print("DISPENSER");
      Serial.print(" (");
      Serial.print(cmd->stack);
      Serial.println(")");
      int stack = cmd->stack;
      
      if(MAX_STACKS <= stack || stacks[stack] == -1){
        Serial.println("Stack is unavailable - ignoring command!!!");
        break;
      }

      //TODO Missing some points here, when sliding down into dish stack, etc
      Motion* ms[] ={new Motion(Motion::GRIP,        GRIPPER_OPEN),             //Release grip
                     new Motion(Motion::EJECT,       0),                        //Retract c
                     new Motion(Motion::VERTICAL,    stackPositions[stack][0]), //Move a
                     new Motion(Motion::HORIZONTAL,  stackPositions[stack][1]), //Move b
                     new Motion(Motion::EJECT,       stackPositions[stack][2]), //Eject c
                     new Motion(Motion::GRIP,        GRIPPER_CLOSE),            //Grap
                     new Motion(Motion::EJECT,       0),                        //Retract C
                     new Motion(Motion::VERTICAL,    platformInsertionPos[0]),  //Move A
                     new Motion(Motion::HORIZONTAL,  platformInsertionPos[1]),  //Move B
                     new Motion(Motion::EJECT,       platformInsertionPos[2]),  //Eject C
                     new Motion(Motion::GRIP,        GRIPPER_OPEN),             //Release
                     new Motion(Motion::EJECT,       0)                         //Retract C
                    };
      //Serial.println(sizeof(ms));
      //Serial.println(sizeof(ms)/sizeof(Motion));

      int l = sizeof(ms) / sizeof(Motion*);
      /*Serial.print("Adding ");
      Serial.print(l);
      Serial.println(" command to motion queue");delay(5000);*/
      for (int i = 0; i < l; i++) {
        motionQueue[motionQueueEnd++] = ms[i];
        //Serial.print(ms[i]->absPosition); Serial.print(",");
        if (MOTION_QUEUE_LENGTH - 1 < motionQueueEnd)
          motionQueueEnd = 0;
      }
      break;
    }

    case Command::REMOVE:{
      Serial.print("REMOVE");
      Serial.print(" (");
      Serial.print(cmd->stack);
      Serial.println(")");
      int stack = cmd->stack;
      
      if(MAX_STACKS <= stack || stacks[stack] == -1){
        Serial.println("Stack is unavailable - ignoring command!!!");
        break;
      }

      //TODO Missing some points here, when sliding down into dish stack, etc
      Motion* ms[] ={new Motion(Motion::GRIP,        GRIPPER_OPEN),             //Release grip
                     new Motion(Motion::EJECT,       0),                        //Retract c
                     new Motion(Motion::VERTICAL,    platformInsertionPos[0]),  //Move A
                     new Motion(Motion::HORIZONTAL,  platformInsertionPos[1]),  //Move B
                     new Motion(Motion::EJECT,       platformInsertionPos[2]),  //Eject C
                     new Motion(Motion::GRIP,        GRIPPER_CLOSE),            //Grap
                     new Motion(Motion::EJECT,       0),                        //Retract C
                     new Motion(Motion::VERTICAL,    stackPositions[stack][0]), //Move a
                     new Motion(Motion::HORIZONTAL,  stackPositions[stack][1]), //Move b
                     new Motion(Motion::EJECT,       stackPositions[stack][2]), //Eject c
                     new Motion(Motion::GRIP,        GRIPPER_OPEN),             //Release
                     new Motion(Motion::EJECT,       0),                        //Retract C
                    };
      
      int l = sizeof(ms) / sizeof(Motion*);
      for (int i = 0; i < l; i++) {
        motionQueue[motionQueueEnd++] = ms[i];
        Serial.print(ms[i]->absPosition); Serial.print(",");
        if (MOTION_QUEUE_LENGTH - 1 < motionQueueEnd)
          motionQueueEnd = 0;
      }
      break;
    }

    case Command::CALIBRATE:{
      Serial.println("CALIBRATE");

                     //Move to what we think is origo
      Motion* ms[] ={new Motion(Motion::GRIP,         GRIPPER_OPEN),       //Release grip
                     new Motion(Motion::EJECT,        0),                  //Retract c
                     new Motion(Motion::VERTICAL,     0),                  //Move a
                     new Motion(Motion::HORIZONTAL,   0),                   //Move b
                     
                     //Try to move farther than origo - and log coordinates
                     new Motion(Motion::EJECT,       MOVE_FARHTER_THAN_ORIGO),                  //Retract c
                     new Motion(Motion::VERTICAL,    MOVE_FARHTER_THAN_ORIGO),                  //Move a
                     new Motion(Motion::HORIZONTAL,  MOVE_FARHTER_THAN_ORIGO),                  //Move b
                     
                     //Move to the extremes of each axis - and log coordinates
                     //Note that Eject axis will do this at the lowest possible height as this is supposed to be free
                     new Motion(Motion::EJECT,       MOVE_FARHTER_THAN_EXTREMA),                  //Eject c
                     new Motion(Motion::EJECT,       0),                                          //Retract c
                     new Motion(Motion::VERTICAL,    MOVE_FARHTER_THAN_EXTREMA),                  //Move a
                     new Motion(Motion::HORIZONTAL,  MOVE_FARHTER_THAN_EXTREMA)                   //Move b
                    };

      int l = sizeof(ms) / sizeof(Motion*);
      for (int i = 0; i < l; i++) {
        motionQueue[motionQueueEnd++] = ms[i];
        if (MOTION_QUEUE_LENGTH - 1 < motionQueueEnd)
          motionQueueEnd = 0;           
      }


      break;
    }

    case Command::TO_ORIGO:{ 
      Serial.println("TO_ORIGO");

      Motion* ms[] ={new Motion(Motion::GRIP,       GRIPPER_OPEN),       //Release grip
                     new Motion(Motion::EJECT,      0),                  //Retract c
                     new Motion(Motion::VERTICAL,   0),                  //Move a
                     new Motion(Motion::HORIZONTAL, 0)                   //Move b
                    };

      //Serial.println(sizeof(ms));
      //Serial.println(sizeof(ms)/sizeof(Motion));

      int l = sizeof(ms) / sizeof(Motion*);
      for (int i = 0; i < l; i++) {
        motionQueue[motionQueueEnd++] = ms[i];
        if (MOTION_QUEUE_LENGTH - 1 < motionQueueEnd)
          motionQueueEnd = 0;           
      }
    
      break;
    }

    default:
      Serial.print("Unknown command action: ");
      Serial.println(cmd->action);
      ;//Nothing
  }
}


void executeRunToPosition() {
  //Serial.println("Running to position");
  vertAxis.runToPosition();
  horiAxis.runToPosition();
  ejectAxis.runToPosition();
  //printPosition();
}

void executeRunSpeed() {
  //Serial.println("Running to speed");
  vertAxis.runSpeed();
  horiAxis.runSpeed();
  ejectAxis.runSpeed();
  //printPosition();
}

void executeRun() {
  //Serial.println("Running");
  vertAxis.run();
  horiAxis.run();
  ejectAxis.run();
  //printPosition();
}

void executeGrip(){
  //Serial.println("Opening/closing gripper");
  openCloseGripper();
  //printPosition();
}

void printPosition(){
  Serial.print("Position [");
  Serial.print(vertAxis.currentPosition());
  Serial.print(',');
  Serial.print(horiAxis.currentPosition());
  Serial.print(',');
  Serial.print(ejectAxis.currentPosition());
  Serial.print(',');
  Serial.print(currentPosition[3]);
  Serial.print("] - Target [");
  Serial.print(targetPosition[0]);
  Serial.print(',');
  Serial.print(targetPosition[1]);
  Serial.print(',');
  Serial.print(targetPosition[2]);
  Serial.print(',');
  Serial.print(targetPosition[3]);
  Serial.println("]");
  
}

boolean isDigit(char c) {
  switch (c) {
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

void handleMsg() {
  int index = msg.indexOf(MSG_SEP_CHAR);

  //Is there at least one full command
  if (index < 0)
    return;
  while (-1 < index) {
    if (index == 0) {
      //if msg starts with MSG_SEP_CHAR (empty command) then remove this
      msg = msg.substring(1);
      Serial.println(CMD_UKNOWN);
    } else {

      String cmd = msg.substring(0, index);
      msg = msg.substring(index + 1); //Remove MSG_SEP_CHAR

      //Serial.print("Got command: "); Serial.println(cmd);

      boolean isValidCmd = false;
      switch (cmd[0]) {
        case CMD_DISPENSE: {
            if (!isDigit(cmd[1]))
              break;

            isValidCmd = true;
            int stack = -1;
            stack = String(cmd[1]).toInt() ;
            dispense(stack);
            break;
          }
        case CMD_REMOVE: {
            if (!isDigit(cmd[1]))
              break;

            isValidCmd = true;
            int stack = -1;
            stack = String(cmd[1]).toInt() ;
            remove(stack);
            break;
          }
        case CMD_CALIBRATE: {
            isValidCmd = true;
            calibrate();
            break;
          }
        case CMD_ORIGO: {
            isValidCmd = true;
            goToOrigo();
            break;
          }
        case CMD_LAYERS: {
            isValidCmd = true;
            returnLayers();
            break;
          }
        case CMD_WIDTH: {
            isValidCmd = true;
            returnWidth();
            break;
          }
        case CMD_HEIGHT: {
            isValidCmd = true;
            returnHeight();
            break;
          }
        case CMD_POSITION: {
            isValidCmd = true;
            returnPosition();
            break;
          }
        case CMD_STACKS: {
            isValidCmd = true;
            returnStacks();
            break;
          }
        default: {
            //Nothing...
          }
      }
      if (!isValidCmd) {
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
//void serialEvent() {
void getSerial() {
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


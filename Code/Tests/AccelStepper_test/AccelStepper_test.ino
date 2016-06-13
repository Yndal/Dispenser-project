#include <AccelStepper.h>
#include <Servo.h>

/*****************************************************************************
   Local classes as tab environments can be troubled in Arduino and Teensy
 *****************************************************************************/
struct Command {
  public:
    typedef enum {
      DISPENSE,
      REMOVE,
      CALIBRATE,
      TO_ORIGO,
      INSERTION_POINT,
      GOTO_POINT,
      ___EMPTY___
    } Action;

    Action action;
    int valueA = -1; //Stack number or steps
    int valueB = -1; //Steps
    int valueC = -1; //Steps
    int valueD = -1; //Gripper open/closed

    Command() {
      //Command(___EMPTY___);
      this->action = ___EMPTY___;
    }

    Command(Action action) {
      //Command(action, -1, -1, -1, -1);
      this->action = action;
    }

    Command(Action action, int stackNumber) {
      //Command(action, stackNumber, -1, -1, -1);
      this->action = action;
      this->valueA = stackNumber;
    }

    Command(Action action, int a, int b, int c) {
      //Command(action, a, b, c, -1);
      this->action = action;
      this->valueA = a;
      this->valueB = b;
      this->valueC = c;
    }

    Command(Action action, int a, int b, int c, int d) {
      this->action = action;
      //Serial.print("Adding to new Command: ");
      this->valueA = a;
      this->valueB = b;
      this->valueC = c;
      this->valueD = d;
      /*for(int i=0; i<4; i++){
        this->stack[i] = stackNumber[i];
        //Serial.print(stackNumber[i]);
        Serial.print(",");
        }*/
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
    long absPosition = 0;

    Motion() {
      axis = Motion::___EMPTY;
      //Motion(Motion::___EMPTY, 0);
    }
    Motion(Axis axis, long absPosition) {
      this->axis = axis;
      this->absPosition  = absPosition;
    }
};



/********************************
   Motion constants
 *******************************/

//purple - dir
//yellow - enable
//orange - step
//Z axis
#define VERT_STEP_PIN 46
#define VERT_DIR_PIN 48
#define VERT_ENA_PIN A8
#define VERT_MICRO_STEPS         1
#define VERT_MAX_SPEED         (VERT_MICRO_STEPS*600)//steps per seconds
#define VERT_SPEED             (VERT_MICRO_STEPS*600)//steps per seconds
#define VERT_CALIBRATION_SPEED (VERT_MICRO_STEPS*200)//steps per seconds
#define VERT_ACCELERATION      (VERT_MICRO_STEPS* 80)//steps per second squared

//X axis
#define HORI_STEP_PIN A0
#define HORI_DIR_PIN A1
#define HORI_ENA_PIN 38
#define HORI_MICRO_STEPS         1
#define HORI_MAX_SPEED         (HORI_MICRO_STEPS*400)//steps per seconds
#define HORI_SPEED             (HORI_MICRO_STEPS*400)//steps per seconds
#define HORI_CALIBRATION_SPEED (HORI_MICRO_STEPS* 50)//steps per seconds
#define HORI_ACCELERATION      (HORI_MICRO_STEPS* 40)//steps per second squared

//Y axis
#define EJECT_STEP_PIN A6
#define EJECT_DIR_PIN A7
#define EJECT_ENA_PIN A2
#define EJECT_MICRO_STEPS         1
#define EJECT_MAX_SPEED         (EJECT_MICRO_STEPS*400)//steps per seconds
#define EJECT_SPEED             (EJECT_MICRO_STEPS*400)//steps per seconds
#define EJECT_CALIBRATION_SPEED (EJECT_MICRO_STEPS* 50)//steps per seconds
#define EJECT_ACCELERATION      (EJECT_MICRO_STEPS* 40)//steps per second squared

//End stops
#define VERT_END_STOP_ORIGO_PIN 18 //Z
#define VERT_END_STOP_MAX_PIN 19

#define HORI_END_STOP_ORIGO_PIN 3 //X
#define HORI_END_STOP_MAX_PIN 2

#define EJECT_END_STOP_ORIGO_PIN 14 //Y
#define EJECT_END_STOP_MAX_PIN 15

AccelStepper vertAxis(AccelStepper::DRIVER, VERT_STEP_PIN, VERT_DIR_PIN);
AccelStepper horiAxis(AccelStepper::DRIVER, HORI_STEP_PIN, HORI_DIR_PIN);
AccelStepper ejectAxis(AccelStepper::DRIVER, EJECT_STEP_PIN, EJECT_DIR_PIN);

Servo servo;
#define SERVO_PIN 11
//#define SERVO_PRESSURE_READ_PIN 9
#define SERVO_OPEN 10
#define SERVO_CLOSET 170
//#define SERVO_INTERVAL_DEGREE 5
//#define SERVO_PRESSURE_LIMIT 512
#define GRIPPER_CLOSE 0
#define GRIPPER_OPEN  1
//int servoCurrentDegree;

/*************************************
   Sensor related
 ************************************/
 
#define MAX_VERT_STEPS 32767 //Highest possible number


/*************************************
   Motion variables
 ************************************/
#define MOVE_FARHTER_THAN_ORIGO   -1
#define MOVE_FARHTER_THAN_EXTREMA -2
#define MOTION_QUEUE_LENGTH 32

boolean executingCmd   = false;
long currentPosition[] = {0, 0, 0, 0};
long targetPosition[]  = {0, 0, 0, 0};
long maxPosition[]  = {0, 0, 0, 1};
Motion* motionQueue[MOTION_QUEUE_LENGTH];
int motionQueueStart = 0;
int motionQueueEnd   = motionQueueStart;

/***************************
   Rx/Tx variables
 **************************/
#define RXTX_BAUD 9600
#define MSG_SEP_CHAR ';'
String msg =  "";
boolean msgComplete = false;



/***************************
   Available commands
 **************************/
#define CMD_DISPENSE  'd'
#define CMD_REMOVE    'r'
#define CMD_CALIBRATE 'c'
#define CMD_ORIGO     'o'
#define CMD_LAYERS    'l'
#define CMD_I_POINT   'i'
#define CMD_GOTO      'g'
#define CMD_WIDTH     'w'
#define CMD_HEIGHT    'h'
#define CMD_POSITION  'p'
#define CMD_STACKS    's'


#define CMD_OK        "ok"
#define CMD_UKNOWN    "unknown cmd"




/**********************************
   System state variable - start
 *********************************/
#define CMD_QUEUE_LENGTH 64
Command* cmdQueue[CMD_QUEUE_LENGTH];
int cmdQueueStart = 0;
int cmdQueueEnd = cmdQueueStart;

#define STATE_ERROR               0
#define STATE_IDLE                1
//#define STATE_RUNNING_TO_POSITION 2
#define STATE_RUNNING_TO_SPEED    3
#define STATE_RUNNING             4
#define STATE_GRIPPING            5

int _state = STATE_IDLE;


/**********************************
   System state variable - END
 *********************************/

/***************************************
   Stack related (Dish stacks) - start
 **************************************/
const int STACK_PINS[] = {A0, A1, A2, A3}; //,A4,A5,A6,A7};
#define STACK_56MM_PETRI_DISH_VALUE 250
#define STACK_94MM_PETRI_DISH_VALUE 500
#define STACK_WELLS_DISH_VALUE      750
#define STACK_EMPTY_VALUE             0
#define STACK_CHANGE_MAX_DELTA       20

const int STACKS_AMOUNT = sizeof(STACK_PINS) / sizeof(int); //8
#define CHECK_FOR_STACK_CHANGE_FREQUENZY 100000 //This can be rather high, but shouldn't exceed 10000.
int checkForStackChangeCounter = 0;
int stackChangeValues[STACKS_AMOUNT] = { -100};
int stacks[STACKS_AMOUNT]            = {42, -1, -1, -1/*,-1,-1,-1,-1*/}; //Amount of dishes in each stack
long stackPositions[STACKS_AMOUNT][3] = {{50, 50, 50}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}/*,
                                      {0,0,0},{0,0,0},{0,0,0},{0,0,0}*/
};


/***************************************
   Stack related (Dish stacks) - end
 **************************************/
#define MAX_LAYERS 8
long platformInsertionPos[3] = { -1, -1, -1};
long layers[MAX_LAYERS] = { -1 };


/***************************
   Testing parameters
 **************************/
//#define TESTING true


void setup() {
  Serial.begin(RXTX_BAUD);
  delay(1000); //Wait for serial to be ready

  msg.reserve(CMD_QUEUE_LENGTH * 8); //Reserve enough bits for 64 chars
  //cmdQueue[0] = new Command(Command::CALIBRATE);
  //cmdQueue.enqueue(3);
  pinMode(13, OUTPUT);

  servo.attach(SERVO_PIN);
  //pinMode(SERVO_PRESSURE_READ_PIN, INPUT);  Apparently not...

  pinMode(VERT_END_STOP_ORIGO_PIN, INPUT);
  pinMode(VERT_END_STOP_MAX_PIN, INPUT);
  //attachInterrupt(digitalPinToInterrupt(VERT_END_STOP_START_PIN), stopVertAxisOrigo, FALLING);
  //attachInterrupt(digitalPinToInterrupt(VERT_END_STOP_END_PIN), stopVertAxisExtrema, FALLING);

  pinMode(HORI_END_STOP_ORIGO_PIN, INPUT);
  pinMode(HORI_END_STOP_MAX_PIN, INPUT);
  //attachInterrupt(digitalPinToInterrupt(HORI_END_STOP_START_PIN), stopHoriAxisOrigo, FALLING);
  //attachInterrupt(digitalPinToInterrupt(HORI_END_STOP_END_PIN), stopHoriAxisExtrema, FALLING);

  pinMode(EJECT_END_STOP_ORIGO_PIN, INPUT);
  pinMode(EJECT_END_STOP_MAX_PIN, INPUT);
  //attachInterrupt(digitalPinToInterrupt(EJECT_END_STOP_START_PIN), stopEjectAxisOrigo, FALLING);
  //attachInterrupt(digitalPinToInterrupt(EJECT_END_STOP_END_PIN), stopEjectAxisExtrema, FALLING);

  pinMode(VERT_ENA_PIN, OUTPUT);
  pinMode(HORI_ENA_PIN, OUTPUT);
  pinMode(EJECT_ENA_PIN, OUTPUT);

  digitalWrite(VERT_ENA_PIN, LOW);
  digitalWrite(HORI_ENA_PIN, LOW);
  digitalWrite(EJECT_ENA_PIN, LOW);

  initialize();

  //Release gripper
  targetPosition[3] = GRIPPER_OPEN;
  openCloseGripper();
  
 /* vertAxis.setMaxSpeed(VERT_SPEED);
  horiAxis.setMaxSpeed(HORI_SPEED);
  ejectAxis.setMaxSpeed(EJECT_SPEED);
  
  vertAxis.setSpeed(VERT_SPEED);
  horiAxis.setSpeed(HORI_SPEED);
  ejectAxis.setSpeed(EJECT_SPEED);
  */
  long pos = 2000;
      vertAxis.moveTo(pos);
      horiAxis.moveTo(pos);
      ejectAxis.moveTo(pos);
      
  Serial.println("Dispener module loaded...");
}


void initialize() {
  //Calibrate the dispenser
  calibrate();

  /*vertAxis.setMaxSpeed(VERT_MAX_SPEED);
  vertAxis.setSpeed(VERT_SPEED);
  vertAxis.setAcceleration(VERT_ACCELERATION);

  horiAxis.setMaxSpeed(HORI_MAX_SPEED);
  horiAxis.setSpeed(HORI_SPEED);
  horiAxis.setAcceleration(HORI_ACCELERATION);

  ejectAxis.setMaxSpeed(EJECT_MAX_SPEED);
  ejectAxis.setSpeed(EJECT_SPEED);
  ejectAxis.setAcceleration(EJECT_ACCELERATION);*/

  //Release gripper and move to origo
  targetPosition[3] = GRIPPER_OPEN;
  openCloseGripper();
  // cmdQueue
  // vertAxis.setCurrentPosition(0);
  //horiAxis.setCurrentPosition(0);
  //ejectAxis.setCurrentPosition(0);


  //while(!cmdQueue.isEmpty())
  //;//    executeCmd();

  //Detect size of platform

  //Search for dish stacks

  //Find platform layers
  //Make sure to measure from a position where there is free stack


}

void stopVertAxisOrigo() {
  Serial.println("Vert origo reached!!");
  targetPosition[0] = 0;
  vertAxis.setCurrentPosition(0);
  vertAxis.moveTo(0);
  //vertAxis.stop();
}

void stopVertAxisExtrema() {
  Serial.println("Vert extrema reached!!");
  long pos = vertAxis.currentPosition();
  targetPosition[0] = pos;
  maxPosition[0] = pos;
  vertAxis.moveTo(pos);
  //vertAxis.stop();
}

void stopHoriAxisOrigo() {
  Serial.println("Horizontal origo reached!!");
  targetPosition[1] = 0;
  horiAxis.setCurrentPosition(0);
  horiAxis.moveTo(0);
  //vertAxis.stop();
}

void stopHoriAxisExtrema() {
  Serial.println("Horizontal extrema reached!!");
  long pos = horiAxis.currentPosition();
  targetPosition[1] = pos;
  maxPosition[1] = pos;
  horiAxis.moveTo(pos);
  //vertAxis.stop();
}

void stopEjectAxisOrigo() {
  Serial.println("Eject origo reached!!");
  targetPosition[2] = 0;
  ejectAxis.setCurrentPosition(0);
  ejectAxis.moveTo(0);
  //vertAxis.stop();
}

void stopEjectAxisExtrema() {
  Serial.println("eject extrema reached!!");
  long pos = ejectAxis.currentPosition();
  targetPosition[2] = pos;
  maxPosition[2] = pos;
  ejectAxis.moveTo(pos);
  //vertAxis.stop();
}



/********************************
   Command calls - start
 *******************************/
void dispense(int stack) {
  //Serial.print("Dispensing plate from stack: ");
  //Serial.println(stack);
  //Command* c = new Command(Command::DISPENSE, stack);

  cmdQueue[cmdQueueEnd++] = new Command(Command::DISPENSE, stack);
  if (CMD_QUEUE_LENGTH - 1 < cmdQueueEnd)
    cmdQueueEnd = 0;
}

void remove(int stack) {
  //Serial.print("Removing plate to stack: ");
  //Serial.println(stack);

  cmdQueue[cmdQueueEnd++] = new Command(Command::REMOVE, stack);
  if (CMD_QUEUE_LENGTH - 1 < cmdQueueEnd)
    cmdQueueEnd = 0;
}

void checkDishAmountInStack(int stack) {
  Serial.println("This is NOT implemented:");
  Serial.print("Go to stack ");
  Serial.print(stack);
  Serial.println(" and measure amount!");
}


void calibrate() {
  Serial.println("Calibrating. Please hold...");

  //Release gripper
  targetPosition[3] = GRIPPER_OPEN;
  openCloseGripper();
  
  ejectAxis.runToNewPosition(0);
  horiAxis.runToNewPosition(0);
  vertAxis.runToNewPosition(0);

  vertAxis.setMaxSpeed(VERT_CALIBRATION_SPEED);
  horiAxis.setMaxSpeed(HORI_CALIBRATION_SPEED);
  ejectAxis.setMaxSpeed(EJECT_CALIBRATION_SPEED);
  
  vertAxis.setSpeed(-VERT_CALIBRATION_SPEED);
  horiAxis.setSpeed(-HORI_CALIBRATION_SPEED);
  ejectAxis.setSpeed(-EJECT_CALIBRATION_SPEED);
  
  while (digitalRead(EJECT_END_STOP_ORIGO_PIN) == 0) {
    ejectAxis.runSpeed();
  }
  //Should already be zero, but just to make sure!
  ejectAxis.setCurrentPosition(0);

  boolean hDone = false;
  boolean vDone = false;
  while(!hDone || !vDone){
    hDone = digitalRead(HORI_END_STOP_ORIGO_PIN) != 0;
    vDone = digitalRead(VERT_END_STOP_ORIGO_PIN) != 0;
    
    if(!hDone)
      horiAxis.runSpeed();
    if(!vDone)
      vertAxis.runSpeed();  
  }
  horiAxis.setCurrentPosition(0);
  vertAxis.setCurrentPosition(0);

  //Serial.println("All axes are at their origo");

  /*
   * Now try to find the maximum positions
   */
    
  vertAxis.setSpeed(VERT_CALIBRATION_SPEED);
  horiAxis.setSpeed(HORI_CALIBRATION_SPEED);
  ejectAxis.setSpeed(EJECT_CALIBRATION_SPEED);
  
  while (digitalRead(EJECT_END_STOP_MAX_PIN) == 0) {
    ejectAxis.runSpeed();
  }
  maxPosition[2] = ejectAxis.currentPosition();
  ejectAxis.setSpeed(EJECT_SPEED);
  ejectAxis.setAcceleration(EJECT_ACCELERATION);
  ejectAxis.runToNewPosition(0); //Retract it
  
  byte hallArray[MAX_VERT_STEPS] = {0};
  int sensorThreshold = 900; //Should start at ~800, so 900 is fine!
  long lastPosition = 0;
  byte stepCounter = 0;
  byte stepsBeforeMeasure = 16;
  int validValues = 0;
  int validValuesMinimum = 128;
  int HALL_EFFECT_SENSOR = -1;
  long HALL_VALUE_DEDUCT = sensorThreshold;

/*
 * Notes:
 * Only measure when actually moved a step
 * Save values as bytes by starting at... 800?... so that 800 is stores as 0, 801 as 1, etc.
 * Smooth the data by a windows of size 5-10 (using a leadscrew, so a single step is not *that* much of a physical step)
 *    Besides... We are also going to use microstepping...)
 * Leadscrew has a lead of 8mm
 * 
 * Only start storing data, when above a certain threshold
 */

  
  while (digitalRead(VERT_END_STOP_MAX_PIN) == 0) {
    vertAxis.runSpeed();
    //Serial.println(vertAxis.currentPosition());
    
    //If stepped
    if(lastPosition != vertAxis.currentPosition()){
      stepCounter++;

      if(stepsBeforeMeasure <= stepCounter){// moved XXX times //This is to not measure at each individual step, which would be too much because of microstepping
        stepCounter = 0;
        int hallValue = analogRead(HALL_EFFECT_SENSOR);

        if(sensorThreshold <= hallValue){
          //add value to array
          hallArray[validValues++] = (hallValue - HALL_VALUE_DEDUCT);            
        } else if(validValuesMinimum <= validValues){// got more than XXX valid values
            //calculate position - and reset
            long position = calculateLayerPosition(hallArray, validValues);
            validValues = 0;
        } else {
          //Reset
          validValues = 0;
        }
         
      }
    }
    
  }
  maxPosition[0] = vertAxis.currentPosition();
  //vertAxis.setSpeed(VERT_SPEED);

  while (digitalRead(HORI_END_STOP_MAX_PIN) == 0) {
    horiAxis.runSpeed();
    //TODO Measure hall effect sensors here
    
  }
  maxPosition[1] = horiAxis.currentPosition();
  //horiAxis.setSpeed(HORI_SPEED);

  
  
  
  printPosition();

  /*
   * Clearing any (stupid!!) position issues
   */
  /*vertAxis.moveTo(vertAxis.currentPosition());
  horiAxis.moveTo(horiAxis.currentPosition());
  ejectAxis.moveTo(ejectAxis.currentPosition());
  vertAxis.stop();
  horiAxis.stop();
  ejectAxis.stop();*/
  
  /*
   * Go back to default settings
   */
  vertAxis.setMaxSpeed(VERT_MAX_SPEED);
  vertAxis.setSpeed(VERT_SPEED);
  vertAxis.setAcceleration(VERT_ACCELERATION);

  horiAxis.setMaxSpeed(HORI_MAX_SPEED);
  horiAxis.setSpeed(HORI_SPEED);
  horiAxis.setAcceleration(HORI_ACCELERATION);

  ejectAxis.setMaxSpeed(EJECT_MAX_SPEED);
  ejectAxis.setSpeed(EJECT_SPEED);
  ejectAxis.setAcceleration(EJECT_ACCELERATION);

  Serial.print("After setting all speed to standart: ");
  printPosition();
  //delay(10000);
}

void setInsertionPoint(long xValue, long yValue, long zValue) {
  Serial.println("setInsertion point is not fully implemented yet");
  cmdQueue[cmdQueueEnd++] = new Command(Command::INSERTION_POINT, xValue, yValue, zValue);
  if (CMD_QUEUE_LENGTH - 1 < cmdQueueEnd)
    cmdQueueEnd = 0;
}

void goToPoint(long xValue, long yValue, long zValue, long gripValue) {
  Serial.println("goToPoint point is not fully implemented yet");
  cmdQueue[cmdQueueEnd++] = new Command(Command::GOTO_POINT, xValue, yValue, zValue, gripValue);
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
  int l = sizeof(layers) / sizeof(long);
  for (int i = 0; i < l; i++) {
    Serial.print(layers[i]);
    if (i < l - 1)
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
  for (int i = 0; i < STACKS_AMOUNT; i++) {
    Serial.print(stacks[i]);
    if (i != STACKS_AMOUNT - 1)
      Serial.print(',');
  }
  Serial.println(']');
}

/********************************
   Command calls - END
 *******************************/

/********************************
   Primitive motion functions - start
 *******************************/

void openCloseGripper() {
  long pos = targetPosition[3];
  if (pos == GRIPPER_CLOSE) {
    /*while(analogRead(SERVO_PRESSURE_READ_PIN) < SERVO_PRESSURE_LIMIT){
      servo.write(servoCurrentDegree + SERVO_INTERVAL_DEGREE);
      delay(100);
    */
   /* Serial.print("Grabbing dish (");
    Serial.print(SERVO_CLOSET);
    Serial.println(')');*/

    servo.write(SERVO_CLOSET);
    currentPosition[3] = GRIPPER_CLOSE;

    delay(1000);

    //}

  } else if (pos == GRIPPER_OPEN) {
    /*Serial.print("Releasing dish (");
    Serial.print(SERVO_OPEN);
    Serial.println(')');
*/
    servo.write(SERVO_OPEN);

    currentPosition[3] = GRIPPER_OPEN;

    delay(1000);
  } else {
    //ignore
    Serial.print("Invalid gripper command: ");
    Serial.println(targetPosition[3]);
  }
  //currentPosition[3] = targetPosition[3];
  //printPosition();
}


/********************************
   Primitive motion functions - END
 *******************************/


bool moving = false;
bool forward = true;
long pos = 2000;
void loop() {
  /*if(!moving){
    moving = true;
    if(forward){
      vertAxis.moveTo(pos);
      horiAxis.moveTo(pos);
      ejectAxis.moveTo(pos);
    //  forward = false;
    } else {
      vertAxis.moveTo(0);
      //horiAxis.moveTo(0);
      ejectAxis.moveTo(0);
      //forward = true;
    }
  } 

  if(vertAxis.currentPosition() == pos &&
    horiAxis.currentPosition() == pos &&
    ejectAxis.currentPosition() == pos){
    moving = false;
    forward = false;
  } else if(vertAxis.currentPosition() == 0 &&
    horiAxis.currentPosition() == 0 &&
    ejectAxis.currentPosition() == 0){
    moving = false;
    forward = true;
  }*/
  vertAxis.run();
  horiAxis.run();
  ejectAxis.run();
  
}



long lastTime = 0;
boolean ledOn = true;
void heartBeat() {
  if (millis() - lastTime > 500) {
    ledOn = !ledOn;
    lastTime = millis();
    digitalWrite(13, ledOn);
  }
}

void handleCommands() {
  //Return if empty
  if (cmdQueueStart == cmdQueueEnd &&
      _state == STATE_IDLE) {
    //Serial.println("no commands");
    return;
  }

  switch (_state) {
    case STATE_IDLE:
      //Start next comamnd
      startNextCommand();
      break;
    /*case STATE_RUNNING_TO_POSITION:
      executeRunToPosition();
      break;*/
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
  /*Serial.print("cmdQueueStart == cmdQueueEnd: ");
  Serial.print(cmdQueueStart);
  Serial.print(" == ");
  Serial.println(cmdQueueEnd);*/

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
        Serial.print(cmd->valueA);
        Serial.println(")");
        int stack = cmd->valueA;

        if (STACKS_AMOUNT <= stack || stacks[stack] == -1) {
          Serial.print("Stack ");
          Serial.print(stack);
          Serial.println(" is unavailable - ignoring command!!");
          break;
        }

        //TODO Missing some points here, when sliding down into dish stack, etc
        Motion* ms[] = {new Motion(Motion::GRIP,        GRIPPER_OPEN),            //Release grip
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

    case Command::REMOVE: {
        Serial.print("REMOVE");
        Serial.print(" (");
        Serial.print(cmd->valueA);
        Serial.println(")");
        int stack = cmd->valueA;

        if (STACKS_AMOUNT <= stack || stacks[stack] == -1) {
          Serial.println("Stack is unavailable - ignoring command!!");
          break;
        }

        //TODO Missing some points here, when sliding down into dish stack, etc
        Motion* ms[] = {new Motion(Motion::GRIP,        GRIPPER_OPEN),            //Release grip
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

    case Command::CALIBRATE: {
        Serial.println("CALIBRATE");

        //Move to what we think is origo
        Motion* ms[] = {new Motion(Motion::GRIP,         GRIPPER_OPEN),      //Release grip
                  new Motion(Motion::EJECT,        50),                  //Retract c
                  new Motion(Motion::VERTICAL,     50),                  //Move a
                  new Motion(Motion::HORIZONTAL,   50),                   //Move b

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

    case Command::TO_ORIGO: {
        Serial.println("TO_ORIGO");

        Motion* ms[] = {new Motion(Motion::GRIP,       GRIPPER_OPEN),      //Release grip
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

    case Command::INSERTION_POINT: {
        Serial.print("INSERTION_POINT");
        Serial.print(" (");
        Serial.print(cmd->valueA);
        Serial.print(',');
        Serial.print(cmd->valueB);
        Serial.print(',');
        Serial.print(cmd->valueC);
        Serial.println(')');

        long x = cmd->valueA;
        long y = cmd->valueB;
        long z = cmd->valueC;


        if (-1 < x && -1 < y && -1 < z) {
          Serial.print("Defining new insertion point - ");
          platformInsertionPos[0] = x;
          platformInsertionPos[1] = y;
          platformInsertionPos[2] = z;
        } else {
          Serial.print("Invalid insertion point provided - ");
        }

        Serial.print('(');
        Serial.print(x);
        Serial.print(',');
        Serial.print(y);
        Serial.print(',');
        Serial.print(z);
        Serial.println(')');

        break;
      }

    case Command::GOTO_POINT: {
        Serial.print("GOTO_POINT");
        Serial.print(" (");
        Serial.print(cmd->valueA);
        Serial.print(',');
        Serial.print(cmd->valueB);
        Serial.print(',');
        Serial.print(cmd->valueC);
        Serial.print(',');
        Serial.print(cmd->valueD);
        Serial.println(')');

        long x = cmd->valueA;
        long y = cmd->valueB;
        long z = cmd->valueC;
        long g = cmd->valueD;


        if (-1 < x && -1 < y && -1 < z) {
          Serial.print("Going to point - ");

          Motion* ms[] = {
            new Motion(Motion::GRIP,        GRIPPER_OPEN), //Release grip
            new Motion(Motion::EJECT,       0),            //Retract c
            new Motion(Motion::VERTICAL,    x),            //Move A
            new Motion(Motion::HORIZONTAL,  y),            //Move B
            new Motion(Motion::EJECT,       z),            //Eject C
            new Motion(Motion::GRIP,        g),            //Grap
          };

          int l = sizeof(ms) / sizeof(Motion*);
          for (int i = 0; i < l; i++) {
            motionQueue[motionQueueEnd++] = ms[i];
            Serial.print(ms[i]->absPosition); Serial.print(",");
            if (MOTION_QUEUE_LENGTH - 1 < motionQueueEnd)
              motionQueueEnd = 0;
          }
        } else {
          Serial.print("Invalid goto point provided - ");
        }

        Serial.print('(');
        Serial.print(x);
        Serial.print(',');
        Serial.print(y);
        Serial.print(',');
        Serial.print(z);
        Serial.print(',');
        Serial.print(g);
        Serial.println(')');

        break;
      }

    default:
      Serial.print("Unknown command action: ");
      Serial.println(cmd->action);
      ;//Nothing
  }
}

void handleMotions() {
  //Return if motion queue is empty
  if (motionQueueStart == motionQueueEnd) {
    //Serial.println("STATE_IDLE");
    _state = STATE_IDLE;
    return;
  }

  Motion* motion = motionQueue[motionQueueStart];

  boolean motionDone = false;
  switch (motion->axis) {
    case Motion::VERTICAL: {
        long pos = vertAxis.currentPosition();
        /*Serial.print("Vert pos: ");
        Serial.println(pos);*/

        //Dynamically update max and min positions
        /*if(pos < 0){
          vertAxis.setCurrentPosition(0);
          pos = 0;
          } else if (maxPosition[0] < pos){
          maxPosition[0] = pos;
          targetPosition[0] = pos + 1;
          vertAxis.moveTo(pos + 1);
          }*/

        if (pos == targetPosition[0]) {
          vertAxis.stop();
          //Serial.println("Reached vertical");
          motionDone = true;
        }
        break;
      }
    case Motion::HORIZONTAL: {
        long pos = horiAxis.currentPosition();
        /*Serial.print("Hori pos: ");
        Serial.println(pos);*/

        //Dynamically update max and min positions
        /*if(pos < 0){
          horiAxis.setCurrentPosition(0);
          pos = 0;
          } else if (maxPosition[1] < pos){
          maxPosition[1] = pos;
          targetPosition[1] = pos + 1;
          horiAxis.moveTo(pos + 1);
          }*/

        if (pos == targetPosition[1]) {
          horiAxis.stop();
          //Serial.println("Reached horizontal");
          motionDone = true;
        }
        break;
      }
    case Motion::EJECT: {
        long pos = ejectAxis.currentPosition();
        /*Serial.print("EJECT pos: ");
        Serial.println(pos);*/

        //Dynamically update max and min positions
        /*if(pos < 0){
          ejectAxis.setCurrentPosition(0);
          pos = 0;
          } else if (maxPosition[2] < pos){
          maxPosition[2] = pos;
          targetPosition[2] = pos + 1;
          ejectAxis.moveTo(pos + 1);
          }*/

        if (pos == targetPosition[2]) {
          ejectAxis.stop();
          Serial.println("Reached ejection");
          motionDone = true;
        }
        break;
      }
    case Motion::GRIP: {
        //printPosition();
        if (currentPosition[3] == targetPosition[3]) {
          motionDone = true;
          //Serial.println("Reached gripper");
        }
        break;
      }
    default:
      //Serial.println("Fails here!!");
      //Move on from the failure...
      motionDone = true;
  }

  //If current motion is done, start the next one
  if (motionDone) {
    // Serial.print("Motion "); Serial.print(motionQueueStart); Serial.println(" done");
    motionQueueStart++;
    if (MOTION_QUEUE_LENGTH - 1 < motionQueueStart)
      motionQueueStart = 0;

    //Return if motion queue is empty
    if (motionQueueStart == motionQueueEnd) {
      _state = STATE_IDLE;
      return;
    }

    Motion* m = motionQueue[motionQueueStart];
    switch (m->axis) {
      case Motion::VERTICAL: {
          long tarPos = m->absPosition;
          if (tarPos == MOVE_FARHTER_THAN_ORIGO) {
            /* pos = -1;
              Serial.println("Vert: Farther than origo");
              } else if(pos == MOVE_FARHTER_THAN_EXTREMA)
              pos = maxPosition[0] + 1;
            */Serial.println("FAILS HERE");// sdfghj;
          }
          long deltaPos = tarPos - vertAxis.currentPosition();
          Serial.print("Vert: Go to ");
          Serial.println(tarPos);
          vertAxis.move(deltaPos);
          //vertAxis.computeNewSpeed();
          if(deltaPos < vertAxis.currentPosition())
            vertAxis.setSpeed(-VERT_SPEED);
          else
            vertAxis.setSpeed(VERT_SPEED);
          targetPosition[0] = tarPos;
          //TODO May need to call setSpeed() to keep a constant speed. Line 343 at: http://www.airspayce.com/mikem/arduino/AccelStepper/AccelStepper_8h_source.html
          _state = STATE_RUNNING_TO_SPEED;

          break;
        }
      case Motion::HORIZONTAL: {
          long tarPos = m->absPosition;
          Serial.print("Hori: Go to ");
          Serial.println(m->absPosition);
          long deltaPos = tarPos - horiAxis.currentPosition();
          horiAxis.move(deltaPos);
          //horiAxis.computeNewSpeed();
          if(deltaPos < horiAxis.currentPosition())
            vertAxis.setSpeed(-VERT_SPEED);
          else
            vertAxis.setSpeed(VERT_SPEED);
          targetPosition[1] = tarPos;
          //TODO May need to call setSpeed() to keep a constant speed. Line 343 at: http://www.airspayce.com/mikem/arduino/AccelStepper/AccelStepper_8h_source.html
          _state = STATE_RUNNING_TO_SPEED;

          break;
        }
      case Motion::EJECT: {
          long tarPos = m->absPosition;
          Serial.print("Eject: Go to ");
          Serial.println(m->absPosition);
          long deltaPos = tarPos - ejectAxis.currentPosition();
          ejectAxis.move(deltaPos);
          //ejectAxis.computeNewSpeed();
          if(deltaPos < ejectAxis.currentPosition())
            ejectAxis.setSpeed(-VERT_SPEED);
          else
            ejectAxis.setSpeed(VERT_SPEED);
          targetPosition[2] = tarPos;
          //TODO May need to call setSpeed() to keep a constant speed. Line 343 at: http://www.airspayce.com/mikem/arduino/AccelStepper/AccelStepper_8h_source.html
          _state = STATE_RUNNING_TO_SPEED;
          break;
        }
      case Motion::GRIP: {
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

/*void executeRunToPosition() {
  //Serial.println("Running to position");
  vertAxis.runToPosition();
  horiAxis.runToPosition();
  ejectAxis.runToPosition();
  //printPosition();
}*/

void executeRunSpeed() {
  //Serial.println("Running to speed");
  vertAxis.runSpeed();
  horiAxis.runSpeed();
  ejectAxis.runSpeed();
  printPosition();
}

void executeRun() {
  //Serial.println("Running");
  vertAxis.run();
  horiAxis.run();
  ejectAxis.run();
  printPosition();
}

void executeGrip() {
  //Serial.println("Opening/closing gripper");
  openCloseGripper();
  //printPosition();
}

long calculateLayerPosition(byte hallArray[], int validValues){

//  askldfj

  
  return 42;
}

void checkForChangesInStack() {
  //Measure all values
  int values[STACKS_AMOUNT] = { -1};
  for (int i = 0; i < STACKS_AMOUNT; i++) {
    values[i] = analogRead(STACK_PINS[i]);
  }

  //Compare values
  //React if differences are found
  //Serial.println("Values:");
  for (int i = 0; i < STACKS_AMOUNT; i++) {
    /*Serial.print(i);
      Serial.print(": ");
      Serial.print(values[i]);
      if(i != STACKS_AMOUNT)
      Serial.print(", ");*/

    //If new value differ to much from old the old values
    if (STACK_CHANGE_MAX_DELTA < abs(stackChangeValues[i] - values[i])) {
      stackChangeValues[i] = values[i];
      Serial.print("Stack ");
      Serial.print(i);
      Serial.println(" has changed!");

      //If it's the "empty value" - just set as empty
      if (abs(STACK_EMPTY_VALUE - values[i]) < STACK_CHANGE_MAX_DELTA) {
        setStackAsEmty(i);
      } else {
        checkDishAmountInStack(i);
      }
    }
  }


  /*  Serial.print("A0=");
    Serial.print(val0);
    Serial.print(", A1=");
    Serial.print(val1);
    Serial.print(", A2=");
    Serial.print(val2);
    Serial.print(", A3=");
    Serial.print(val3);
    Serial.println();

    Serial.print("A4=");
    Serial.print(val4);
    Serial.print(", A5=");
    Serial.print(val5);
    Serial.print(", A6=");
    Serial.print(val6);
    Serial.print(", A7=");
    Serial.print(val7);*/
  //Serial.println();

}


void setStackAsEmty(int stack) {
  //If out of range...
  if (STACKS_AMOUNT <= stack)
    return;

  stacks[stack] = -1;
  Serial.print("Stack ");
  Serial.print(stack);
  Serial.println(" set as empty");
}

/*void getDistance(){
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




  }*/


void printPosition() {
  Serial.print("Position [");
  Serial.print(vertAxis.currentPosition());
  Serial.print("->");
  Serial.print(vertAxis.targetPosition());
  Serial.print(',');
  Serial.print(horiAxis.currentPosition());
  Serial.print("->");
  Serial.print(horiAxis.targetPosition());
  Serial.print(',');
  Serial.print(ejectAxis.currentPosition());
  Serial.print("->");
  Serial.print(ejectAxis.targetPosition());
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
      //if msg starts with MSG_SEP_CHAR (empty command) then reject "command"
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
            stack = String(cmd[1]).toInt();
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
        case CMD_I_POINT: {
            int i = cmd.indexOf(',');
            int xValue = -1;
            xValue = cmd.substring(1, i).toInt();
            cmd = cmd.substring(i + 1);
            
            i = cmd.indexOf(',');
            int yValue = -1;
            yValue = cmd.substring(0, i).toInt();
            cmd = cmd.substring(i + 1);
            
            i = cmd.indexOf(',');
            int zValue = -1;
            zValue = cmd.substring(0, i).toInt();
            cmd = cmd.substring(i + 1);
            
            if (-1 < xValue && -1 < yValue && -1 < zValue) {
              isValidCmd = true;
              setInsertionPoint(xValue, yValue, zValue);
            }
            break;
          }
        case CMD_GOTO: {
            int i = cmd.indexOf(',');
            int xValue = -1;
            xValue = cmd.substring(1, i).toInt();
            cmd = cmd.substring(i + 1);
            
            i = cmd.indexOf(',');
            int yValue = -1;
            yValue = cmd.substring(0, i).toInt();
            cmd = cmd.substring(i + 1);
            
            i = cmd.indexOf(',');
            int zValue = -1;
            zValue = cmd.substring(0, i).toInt();
            cmd = cmd.substring(i + 1);
            
            i = cmd.indexOf(',');
            int gripValue = -1;
            gripValue = cmd.substring(0, i).toInt();
            cmd = cmd.substring(i + 1);
            
            if (-1 < xValue && -1 < yValue && -1 < zValue) {
              isValidCmd = true;
              goToPoint(xValue, yValue, zValue, gripValue);
            }

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
  if (Serial.available()) {
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


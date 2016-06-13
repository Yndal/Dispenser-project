#include <AccelStepper.h>
#include <Servo.h>

/*****************************************************************************
   Local classes as tab environments can be troublesome in Arduino
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
    long valueA = -1; //Stack number or steps
    long valueB = -1; //Steps
    long valueC = -1; //Steps
    long valueD = -1; //Gripper open/closed

    Command() {
      //Command(___EMPTY___);
      this->action = ___EMPTY___;
    }

    Command(Action action) {
      //Command(action, -1, -1, -1, -1);
      this->action = action;
    }

    Command(Action action, long stackNumber) {
      //Command(action, stackNumber, -1, -1, -1);
      this->action = action;
      this->valueA = stackNumber;
    }

    Command(Action action, long a, long b, long c) {
      //Command(action, a, b, c, -1);
      this->action = action;
      this->valueA = a;
      this->valueB = b;
      this->valueC = c;
    }

    Command(Action action, long a, long b, long c, long d) {
      this->action = action;
      //Serial.print("Adding to new Command: ");
      this->valueA = a;
      this->valueB = b;
      this->valueC = c;
      this->valueD = d;
    }
};

class Motion {
  public:
    typedef enum {
      VERTICAL,
      HORIZONTAL,
      EJECT,
      GRIP,
      CALIBRATE,
      MEASURE_STACK,
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
#define VERT_MICRO_STEPS         1.0
#define VERT_MAX_SPEED         (VERT_MICRO_STEPS*4000.0)//steps per seconds
#define VERT_SPEED             (VERT_MICRO_STEPS*4000.0)//steps per seconds
#define VERT_CALIBRATION_SPEED (VERT_MICRO_STEPS*800.0)//steps per seconds
#define VERT_ACCELERATION      (VERT_MICRO_STEPS*400.0)//steps per second squared

//X axis
#define HORI_STEP_PIN A0
#define HORI_DIR_PIN A1
#define HORI_ENA_PIN 38
#define HORI_MICRO_STEPS         1.0
#define HORI_MAX_SPEED         (HORI_MICRO_STEPS*800.0)//steps per seconds
#define HORI_SPEED             (HORI_MICRO_STEPS*800.0)//steps per seconds
#define HORI_CALIBRATION_SPEED (HORI_MICRO_STEPS*150.0)//steps per seconds
#define HORI_ACCELERATION      (HORI_MICRO_STEPS*200.0)//steps per second squared

//Y axis
#define EJECT_STEP_PIN A6
#define EJECT_DIR_PIN A7
#define EJECT_ENA_PIN A2
#define EJECT_MICRO_STEPS         1.0
#define EJECT_MAX_SPEED         (EJECT_MICRO_STEPS*400.0)//steps per seconds
#define EJECT_SPEED             (EJECT_MICRO_STEPS*400.0)//steps per seconds
#define EJECT_CALIBRATION_SPEED (EJECT_MICRO_STEPS*200.0)//steps per seconds
#define EJECT_ACCELERATION      (EJECT_MICRO_STEPS*200.0)//steps per second squared

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
#define SERVO_OPEN 10
#define SERVO_CLOSET 170
#define GRIPPER_CLOSE 0
#define GRIPPER_OPEN  1

/*************************************
   Sensor related
 ************************************/
#define MAX_VERT_STEPS 32767 //Highest possible number


/*************************************
   Motion variables
 ************************************/
#define MOVE_FARHTER_THAN_ORIGO   -1
#define MOVE_FARHTER_THAN_EXTREMA -2
#define MOTION_QUEUE_LENGTH 64

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


#define CMD_OK        "OK"
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
const int STACK_RESI_PINS[] = {A9, A11, A5, A12};
const int STACK_DIST_PINS[] = {40, 42,  64,  44};
const double DISH_MEASURE_DELTA = 0.5; //This is not used yet
const double STACK_56MM_DISTANCES[] = {19.40, 17.83, 16.26, 15.20, 13.60, 12.07, 10.45, 8.90, 7.30, 5.90};
const double STACK_93MM_DISTANCES[] = {19.40, 17.83, 16.26, 15.20, 13.60, 12.07, 10.45, 8.90, 7.30, 5.90};
const double STACK_WELL_DISTANCES[] = {23.10, 18, 16, 14, 12, 10, 8, 6, 4, 2, 0};


#define STACK_56MM_PETRI_DISH_VALUE 100 //Not defined
#define STACK_94MM_PETRI_DISH_VALUE 510
#define STACK_WELLS_DISH_VALUE      930
#define STACK_EMPTY_VALUE             0
#define STACK_CHANGE_MAX_DELTA       60


#define DISH_56_MM_STACK_ENTRY_HEIGHT 19400LL
#define DISH_56_MM_STACK_ENTRY_DEPTH  4915LL
#define DISH_93_MM_STACK_ENTRY_HEIGHT 19400LL
#define DISH_93_MM_STACK_ENTRY_DEPTH  4915LL
#define WELL_STACK_ENTRY_HEIGHT       21700LL
#define WELL_STACK_ENTRY_DEPTH        5600LL
#define STEPS_PER_CM_VERT             996LL
#define MOTION_DISTANCE_TO_PLATFORM   (3*STEPS_PER_CM_VERT) //How much it shall move above the layer when inserting the dish
#define WELL_PLATE_HEIGHT                   (2.0 * STEPS_PER_CM_VERT)
#define DISH_HEIGHT                   (1.6 * STEPS_PER_CM_VERT)

const long ALL_STACKS_ENTRY_HORI_POSITION[] = {85LL, 2525LL, 4900LL, 0LL};

const int STACKS_AMOUNT = sizeof(STACK_RESI_PINS) / sizeof(int); //8
#define CHECK_FOR_STACK_CHANGE_FREQUENZY 1000 //This can be rather high, but shouldn't exceed 10000.
int checkForStackChangeCounter = 0;
int stackChangeValues[STACKS_AMOUNT];
int stacks[STACKS_AMOUNT];  //Amount of dishes in each stack
const long stackPositions[STACKS_AMOUNT][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};


#define UNKNOWN_STACK_TYPE   -1
#define EMPTY_STACK_TYPE      0
#define DISH_56_MM_STACK_TYPE 1
#define DISH_93_MM_STACK_TYPE 2
#define WELLS_STACK_TYPE      3
int stackTypes[STACKS_AMOUNT];
long stackEntryPoints[STACKS_AMOUNT][3];
long topDishPositions[STACKS_AMOUNT]; //Vertical position only (the rest can be extracted from stackEntryPoints-variable)
long stackLayerPosition;
int lidParkingPoint[] = {500, 5600, 5000};


/***************************************
   Stack related (Dish stacks) - end
 **************************************/
#define MAX_LAYERS 8
long platformInsertionPos[3] = { 1000, 2500, 5000};
long layers[MAX_LAYERS] = { -1 };
int totalLayers;

#define HALL_EFFECT_SENSOR A3
#define SENSOR_THRESHOLD 550 //Should start at ~800, so 900 is fine!
#define STEPS_BEFORE_HALL_EFFECT_MEASURE 16
#define HAL_SENSOR_MIN_AMOUNT_OF_VALUES_BEFORE_VALID 80



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


  Serial.println(F("Dispener module loaded..."));
}


void initialize() {
  vertAxis.setMaxSpeed(VERT_MAX_SPEED);
  horiAxis.setMaxSpeed(HORI_MAX_SPEED);
  ejectAxis.setMaxSpeed(EJECT_MAX_SPEED);

  vertAxis.setAcceleration(VERT_ACCELERATION);
  horiAxis.setAcceleration(HORI_ACCELERATION);
  ejectAxis.setAcceleration(EJECT_ACCELERATION);

  //Calibrate the dispenser
  //calibrate();

  //  vertAxis.setMaxSpeed(VERT_MAX_SPEED);
  //  vertAxis.setSpeed(VERT_SPEED);
  //  vertAxis.setAcceleration(VERT_ACCELERATION);
  //
  //  horiAxis.setMaxSpeed(HORI_MAX_SPEED);
  //  horiAxis.setSpeed(HORI_SPEED);
  //  horiAxis.setAcceleration(HORI_ACCELERATION);
  //
  //  ejectAxis.setMaxSpeed(EJECT_MAX_SPEED);
  //  ejectAxis.setSpeed(EJECT_SPEED);
  //  ejectAxis.setAcceleration(EJECT_ACCELERATION);


  //Release gripper and move to origo
  targetPosition[3] = GRIPPER_OPEN;
  openCloseGripper();

  for (int i = 0; i < STACKS_AMOUNT; i++)
    stackChangeValues[i] = -2 * STACK_CHANGE_MAX_DELTA;

  for (int i = 0; i < STACKS_AMOUNT; i++)
    stackTypes[i] = UNKNOWN_STACK_TYPE;

  for (int i = 0; i < STACKS_AMOUNT; i++)
    stacks[i] = -1;
}

void stopVertAxisOrigo() {
  //Serial.println("Vert origo reached!!");
  targetPosition[0] = 0;
  vertAxis.setCurrentPosition(0);
  vertAxis.moveTo(0);
  //vertAxis.stop();
}

void stopVertAxisExtrema() {
  //Serial.println("Vert extrema reached!!");
  long pos = vertAxis.currentPosition();
  targetPosition[0] = pos;
  maxPosition[0] = pos;
  vertAxis.moveTo(pos);
  //vertAxis.stop();
}

void stopHoriAxisOrigo() {
  Serial.println(F("Horizontal origo reached!!"));
  targetPosition[1] = 0;
  horiAxis.setCurrentPosition(0);
  horiAxis.moveTo(0);
  //horiAxis.stop();
}

void stopHoriAxisExtrema() {
  //Serial.println("Horizontal extrema reached!!");
  long pos = horiAxis.currentPosition();
  targetPosition[1] = pos;
  maxPosition[1] = pos;
  horiAxis.moveTo(pos);
  //horiAxis.stop();
}

void stopEjectAxisOrigo() {
  //Serial.println("Eject origo reached!!");
  targetPosition[2] = 0;
  ejectAxis.setCurrentPosition(0);
  ejectAxis.moveTo(0);
  //ejectAxis.stop();
}

void stopEjectAxisExtrema() {
  //Serial.println("eject extrema reached!!");
  long pos = ejectAxis.currentPosition();
  targetPosition[2] = pos;
  maxPosition[2] = pos;
  ejectAxis.moveTo(pos);
  //ejectAxis.stop();
}



/********************************
   Command calls - start
 *******************************/
void dispense(int stack) {
  cmdQueue[cmdQueueEnd++] = new Command(Command::DISPENSE, stack);
  if (CMD_QUEUE_LENGTH - 1 < cmdQueueEnd)
    cmdQueueEnd = 0;
}

void remove(int stack) {
  cmdQueue[cmdQueueEnd++] = new Command(Command::REMOVE, stack);
  if (CMD_QUEUE_LENGTH - 1 < cmdQueueEnd)
    cmdQueueEnd = 0;
}




void checkDishAmountInStack(int stack) {
  //Measure distance
  delay(900); //Allow for the user to insert the stack properly and have contact with all connections
  //Serial.print("Dist pin: ");
  //Serial.println(STACK_DIST_PINS[stack]);
  double distance = measureStackDistance(STACK_DIST_PINS[stack]);

  //Serial.print(F("Distance: "));
  //Serial.println(distance);

  //Get dish stack types
  int type = stackTypes[stack];
  //Serial.print(F("Stack type: "));
  //Serial.println(type);

  //Collect amount from distance array with respect to the dish types
  int dishes = 0;
  switch (type) {
    case (DISH_56_MM_STACK_TYPE): {
        int i = 0;
        int maxSize = sizeof(STACK_56MM_DISTANCES) / sizeof(int);
        for (; i <= maxSize; i) {
          if (STACK_56MM_DISTANCES[i] <= distance) {
            dishes = i;
            break;
          }
          i++;
        }

        /*while(STACK_56MM_DISTANCES[i] < distance)
          i++;*/
        //if(
        //dishes = i;
        break;
      }
    case (DISH_93_MM_STACK_TYPE): {
        int i = 0;
        int maxSize = sizeof(STACK_93MM_DISTANCES) / sizeof(int);
        for (; i <= maxSize; i) {
          if (STACK_93MM_DISTANCES[i] <= distance) {
            dishes = i;
            break;
          }
          i++;
        }
        break;
      }
    case (WELLS_STACK_TYPE): {
        int i = 0;
        int maxSize = sizeof(STACK_WELL_DISTANCES) / sizeof(int);
        for (; i <= maxSize; i) {
          if (STACK_WELL_DISTANCES[i] <= distance) {
            dishes = i;
            break;
          }
          i++;
        }
        break;
      }
    default:
      Serial.println(F("Unknown stack type"));
      dishes = 0;//Set to empty
  }
  Serial.print(F("Found "));
  Serial.print(dishes);
  Serial.print(F(" dishes in stack "));
  Serial.println(stack);

  stacks[stack] = dishes;


  //Define the location of the top dish
  topDishPositions[stack] = stackEntryPoints[stack][0] - distance * STEPS_PER_CM_VERT;

  /*Serial.println("This is NOT implemented:");
    Serial.print("Go to stack ");
    Serial.print(stack);
    Serial.println(" and measure amount!");*/
}

double measureStackDistance(int stack) {
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration;
  double inches, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(stack, OUTPUT);
  digitalWrite(stack, LOW);
  delayMicroseconds(2);
  digitalWrite(stack, HIGH);
  delayMicroseconds(5);
  digitalWrite(stack, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(stack, INPUT);
  duration = pulseIn(stack, HIGH);

  // convert the time into a distance
  //inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);

  return cm;
}

double microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

double microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

void calibrate() {
  Serial.println(F("Calibrating..."));


  ejectAxis.runToNewPosition(0);
  ejectAxis.setSpeed(EJECT_CALIBRATION_SPEED / 4);
  if(digitalRead(EJECT_END_STOP_ORIGO_PIN) == 1){
    while (digitalRead(EJECT_END_STOP_ORIGO_PIN) == 1)
      ejectAxis.runSpeed();
    delay(1000);
  }

  //Release gripper
  targetPosition[3] = GRIPPER_OPEN;
  openCloseGripper();

  vertAxis.runToNewPosition(0);
  vertAxis.setSpeed(VERT_CALIBRATION_SPEED / 4);
  if(digitalRead(VERT_END_STOP_ORIGO_PIN) == 1){
    while (digitalRead(VERT_END_STOP_ORIGO_PIN) == 1)
      vertAxis.runSpeed();
    delay(1000);
  }

  horiAxis.runToNewPosition(0);
  horiAxis.setSpeed(HORI_CALIBRATION_SPEED / 4);
  if(digitalRead(HORI_END_STOP_ORIGO_PIN) == 1){
    while(digitalRead(HORI_END_STOP_ORIGO_PIN) == 1)
      horiAxis.runSpeed();
    delay(1000);
  }
  //Serial.println("All axis moved back from origo");

  ejectAxis.setSpeed(-EJECT_CALIBRATION_SPEED);
  if(digitalRead(EJECT_END_STOP_ORIGO_PIN) == 0){
    while(digitalRead(EJECT_END_STOP_ORIGO_PIN) == 0) {
      ejectAxis.runSpeed();
    }
  }
  //Should already be zero, but just to make sure!
  ejectAxis.setCurrentPosition(0);

  boolean hDone = false;
  boolean vDone = false;
  vertAxis.setSpeed(-VERT_CALIBRATION_SPEED);
  horiAxis.setSpeed(-HORI_CALIBRATION_SPEED);
  while (!hDone || !vDone) {
    hDone = digitalRead(HORI_END_STOP_ORIGO_PIN) != 0;
    vDone = digitalRead(VERT_END_STOP_ORIGO_PIN) != 0;

    if (!hDone)
      horiAxis.runSpeed();
    if (!vDone)
      vertAxis.runSpeed();
  }
  horiAxis.setCurrentPosition(0);
  vertAxis.setCurrentPosition(0);


  /*
     Now try to find the maximum positions
  */

  ejectAxis.setSpeed(EJECT_CALIBRATION_SPEED);
  while (digitalRead(EJECT_END_STOP_MAX_PIN) == 0) {
    ejectAxis.runSpeed();
  }
  maxPosition[2] = ejectAxis.currentPosition();
  ejectAxis.setSpeed(EJECT_SPEED);
  //ejectAxis.setAcceleration(EJECT_ACCELERATION);
  ejectAxis.runToNewPosition(0); //Retract it

  //Position where the first value is detected
  long firstHallEffect = -1;

  //Steps needed before the first values is considered valid
  int validCounter = 0;

  //Used to measure IF a step was performed
  long lastPosition = -1;
  long measureCounter = 0;


  /*
     Notes:
     Only measure when actually moved a step
     Save values as bytes by starting at... 800?... so that 800 is stores as 0, 801 as 1, etc.
     Smooth the data by a windows of size 5-10 (using a leadscrew, so a single step is not *that* much of a physical step)
        Besides... We are also going to use microstepping...)
     Leadscrew has a lead of 8mm

     Only start storing data, when above a certain threshold
  */
  totalLayers = 0;

  vertAxis.setSpeed(VERT_CALIBRATION_SPEED);
  while (digitalRead(VERT_END_STOP_MAX_PIN) == 0) {
    vertAxis.runSpeed();
    //Serial.println(vertAxis.currentPosition());

    //If stepped
    if (lastPosition != vertAxis.currentPosition()) {
      // stepCounter++;
      lastPosition = vertAxis.currentPosition();

      //If stepped enough to do a measurement
      if (STEPS_BEFORE_HALL_EFFECT_MEASURE <= measureCounter++) { //Moved XXX times - this is to not measure at each individual step, which would be too much because of microstepping
        measureCounter = 0;

        //Measure
        int hallValue = analogRead(HALL_EFFECT_SENSOR);

        //If high enough to be considered a valid measurement
        if (SENSOR_THRESHOLD <= hallValue) {
          if (firstHallEffect < 0) {
            firstHallEffect = vertAxis.currentPosition();
          }
          //lastHallEffect = vertAxis.currentPosition();
          validCounter++;
          //Serial.println(validCounter);

          //If this is the last value +1
        } else if (-1 < firstHallEffect) {
          long layerPosition = firstHallEffect + (validCounter / 2);

          if (HAL_SENSOR_MIN_AMOUNT_OF_VALUES_BEFORE_VALID <= validCounter) {
            Serial.print(F("Found layer at: "));
            Serial.println(layerPosition);
            layers[totalLayers++] = layerPosition;

            //Last layer will *always* be the stack layer
            stackLayerPosition = layerPosition;
            validCounter = 0;
          }

          firstHallEffect = -1;
          //lastHallEfect = -1;
        }
      }
    }
  }
  maxPosition[0] = vertAxis.currentPosition();

  horiAxis.setSpeed(HORI_CALIBRATION_SPEED);
  while (digitalRead(HORI_END_STOP_MAX_PIN) == 0) {
    horiAxis.runSpeed();
  }
  maxPosition[1] = horiAxis.currentPosition();



  /*
     Clearing any (stupid!!) position issues
  */
  vertAxis.moveTo(vertAxis.currentPosition());
  horiAxis.moveTo(horiAxis.currentPosition());
  ejectAxis.moveTo(ejectAxis.currentPosition());
}

void setInsertionPoint(long xValue, long yValue, long zValue) {
  cmdQueue[cmdQueueEnd++] = new Command(Command::INSERTION_POINT, xValue, yValue, zValue);
  if (CMD_QUEUE_LENGTH - 1 < cmdQueueEnd)
    cmdQueueEnd = 0;
}

void goToPoint(long xValue, long yValue, long zValue, long gripValue) {
  cmdQueue[cmdQueueEnd++] = new Command(Command::GOTO_POINT, xValue, yValue, zValue, gripValue);
  if (CMD_QUEUE_LENGTH - 1 < cmdQueueEnd)
    cmdQueueEnd = 0;
}


void goToOrigo() {
  cmdQueue[cmdQueueEnd++] = new Command(Command::TO_ORIGO);
  if (CMD_QUEUE_LENGTH - 1 < cmdQueueEnd)
    cmdQueueEnd = 0;
}

void returnLayers() {
  Serial.print('[');
  for (int i = 0; i < totalLayers; i++) {
    Serial.print(layers[i]);
    if (i < totalLayers - 1)
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
  Serial.print('[');
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
    Serial.println(targetPosition[3]);
    targetPosition[3] = currentPosition[3];;
  }
}


/********************************
   Primitive motion functions - END
 *******************************/



void loop() {
  //Make the LED blink to show that it is alive
  heartBeat();

  handleCommands();
  handleMotions();

  getSerial();
  if (msgComplete)
    handleMsg();

  if(/*_state == IDLE_STATE && */CHECK_FOR_STACK_CHANGE_FREQUENZY <= checkForStackChangeCounter++) {
    //Serial.println("Checking for new stacks");
    checkForStackChangeCounter = 0;
    checkForChangesInStack();
  }
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

  Serial.print(F("Next command is: "));
  switch (cmd->action) {
    case Command::___EMPTY___:
      //Do nothing
      Serial.println(F("___EMPTY___"));
      break;
    case Command::DISPENSE: {
        Serial.print(F("DISPENSER"));
        Serial.print(F(" ("));
        Serial.print(cmd->valueA);
        Serial.println(F(")"));
        int stack = cmd->valueA;

        if (STACKS_AMOUNT <= stack || stacks[stack] == -1) {
          Serial.print(F("Stack "));
          Serial.print(stack);
          Serial.println(F(" doesn't exist or is empty."));
          return;
        }

       
        switch (stackTypes[stack]) {
          case (DISH_56_MM_STACK_TYPE):
          case (DISH_93_MM_STACK_TYPE): {
           /* Serial.print("Stack entry point: ");
            Serial.print(stackLayerPosition + stackEntryPoints[stack][0]);
            Serial.print(',');
            Serial.print(stackEntryPoints[stack][1]);
            Serial.print(',');
            Serial.println(stackEntryPoints[stack][2]);
            
            Serial.print("Top dish position: ");
            Serial.println(topDishPositions[stack]);*/

            Serial.print(F("Lid parking point: "));
            Serial.print(lidParkingPoint[0]);
            Serial.print(',');
            Serial.print(lidParkingPoint[1]);
            Serial.print(',');
            Serial.println(lidParkingPoint[2]);
            
               
              Motion* ms[] = {
                //Get in a movable state...
                new Motion(Motion::GRIP,        GRIPPER_OPEN),                //Release grip
                new Motion(Motion::EJECT,       0),                           //Retract

                //Move to stack entry point
                new Motion(Motion::VERTICAL,    stackLayerPosition + stackEntryPoints[stack][0]),  //Move a
                new Motion(Motion::HORIZONTAL,  stackEntryPoints[stack][1]),  //Move b
                new Motion(Motion::EJECT,       stackEntryPoints[stack][2]),  //Eject c

                //Lower to first plate and grab lid (and lid ONLY)
                new Motion(Motion::VERTICAL,    stackLayerPosition + topDishPositions[stack]),     //Move a
                new Motion(Motion::GRIP,        GRIPPER_CLOSE),               //Grab
                
                //Move to stack entry point and retract
                new Motion(Motion::VERTICAL,    stackLayerPosition + stackEntryPoints[stack][0]),  //Move a
                new Motion(Motion::EJECT,       0),                           //Retract

                //Move to lid parking and release
                new Motion(Motion::VERTICAL,    lidParkingPoint[0] + MOTION_DISTANCE_TO_PLATFORM), //Move A
                new Motion(Motion::HORIZONTAL,  lidParkingPoint[1]),          //Move B
                new Motion(Motion::EJECT,       lidParkingPoint[2]),          //Eject C
                new Motion(Motion::VERTICAL,    lidParkingPoint[0]),          //Move A
                new Motion(Motion::GRIP,        GRIPPER_OPEN),                //Release

                //Retract to a movable position
                new Motion(Motion::VERTICAL,    lidParkingPoint[0] + MOTION_DISTANCE_TO_PLATFORM), //Move A
                new Motion(Motion::EJECT,       0),                           //Retract

                //Move to stack entry point to get the bottom of the dish
                new Motion(Motion::VERTICAL,    stackLayerPosition + stackEntryPoints[stack][0]),  //Move a
                new Motion(Motion::HORIZONTAL,  stackEntryPoints[stack][1]),  //Move b
                new Motion(Motion::EJECT,       stackEntryPoints[stack][2]),  //Eject c

                //Lower to first plate's bottom and grab this
                new Motion(Motion::VERTICAL,    stackLayerPosition + topDishPositions[stack]),     //Move a
                new Motion(Motion::GRIP,        GRIPPER_CLOSE),               //Grab

                //Move to stack entry point and retract
                new Motion(Motion::VERTICAL,    stackLayerPosition + stackEntryPoints[stack][0]),  //Move a
                new Motion(Motion::EJECT,       0),                           //Retract

                //Move to insertions point and release
                new Motion(Motion::VERTICAL,    platformInsertionPos[0] + MOTION_DISTANCE_TO_PLATFORM),     //Move A + dishHeight
                new Motion(Motion::HORIZONTAL,  platformInsertionPos[1]),     //Move B
                new Motion(Motion::EJECT,       platformInsertionPos[2]),     //Eject C
                new Motion(Motion::VERTICAL,    platformInsertionPos[0]),     //Move A
                new Motion(Motion::GRIP,        GRIPPER_OPEN),                //Release

                /*
                   Now the bottom has been placed - all we need now is the lid from the lid parking point...
                */
                //Retract to a movable position
                new Motion(Motion::VERTICAL,    platformInsertionPos[0] + MOTION_DISTANCE_TO_PLATFORM),     //Move A + dishHeight
                new Motion(Motion::EJECT,       0),                           //Retract

                //Move to lid parking and grab
                new Motion(Motion::VERTICAL,    lidParkingPoint[0] + MOTION_DISTANCE_TO_PLATFORM),          //Move A
                new Motion(Motion::HORIZONTAL,  lidParkingPoint[1]),          //Move B
                new Motion(Motion::EJECT,       lidParkingPoint[2]),          //Eject C
                new Motion(Motion::VERTICAL,    lidParkingPoint[0] ),          //Move A
                new Motion(Motion::GRIP,        GRIPPER_CLOSE),               //Grab

                //Retract
                new Motion(Motion::VERTICAL,    lidParkingPoint[0] + MOTION_DISTANCE_TO_PLATFORM),          //Move A
                new Motion(Motion::EJECT,       0),                           //Retract

                //Move to insertion point and place lid
                new Motion(Motion::VERTICAL,    platformInsertionPos[0] + MOTION_DISTANCE_TO_PLATFORM),     //Move A
                new Motion(Motion::HORIZONTAL,  platformInsertionPos[1]),     //Move B
                new Motion(Motion::EJECT,       platformInsertionPos[2]),     //Eject C
                new Motion(Motion::VERTICAL,    platformInsertionPos[0]),     //Move A
                new Motion(Motion::GRIP,        GRIPPER_OPEN),                //Release

                //Retract
                new Motion(Motion::VERTICAL,    platformInsertionPos[0] + MOTION_DISTANCE_TO_PLATFORM),     //Move A
                new Motion(Motion::EJECT,       0)                            //Retract
                
              };

              int l = sizeof(ms) / sizeof(Motion*);
              //Serial.print(F("Adding motions: "));
              //Serial.println(l);

              /*Serial.print(F("Size of ms: "));
              Serial.println(sizeof(ms));*/
              for (int i = 0; i < l; i++) {
                motionQueue[motionQueueEnd++] = ms[i];
                //Serial.print(ms[i]->absPosition); Serial.print(",");
                if (MOTION_QUEUE_LENGTH - 1 < motionQueueEnd)
                  motionQueueEnd = 0;
              }
              break;
            }

          case (WELLS_STACK_TYPE): {
              Motion* ms[] = {
                //Get in a movable state...
                new Motion(Motion::GRIP,        GRIPPER_OPEN),                //Release grip
                new Motion(Motion::EJECT,       0),                           //Retract

                //Move to stack entry point
                new Motion(Motion::VERTICAL,    stackLayerPosition + stackEntryPoints[stack][0]),  //Move a
                new Motion(Motion::HORIZONTAL,  stackEntryPoints[stack][1]),  //Move b
                new Motion(Motion::EJECT,       stackEntryPoints[stack][2]),  //Eject c

                //Lower to first plate and grab
                new Motion(Motion::VERTICAL,    stackLayerPosition + topDishPositions[stack]),     //Move a
                new Motion(Motion::GRIP,        GRIPPER_CLOSE),               //Grab

                //Move to stack entry point and retract
                new Motion(Motion::VERTICAL,    stackLayerPosition + stackEntryPoints[stack][0]),  //Move a
                new Motion(Motion::EJECT,       0),                           //Retract

                //Move to insertions point and release
                new Motion(Motion::VERTICAL,    platformInsertionPos[0] + MOTION_DISTANCE_TO_PLATFORM),     //Move A + dishHeight
                new Motion(Motion::HORIZONTAL,  platformInsertionPos[1]),     //Move B
                new Motion(Motion::EJECT,       platformInsertionPos[2]),     //Eject C
                new Motion(Motion::VERTICAL,    platformInsertionPos[0]),     //Move A
                new Motion(Motion::GRIP,        GRIPPER_OPEN),                //Release

                //Retract to a movable position
                new Motion(Motion::VERTICAL,    platformInsertionPos[0] + MOTION_DISTANCE_TO_PLATFORM),     //Move A + dishHeight
                new Motion(Motion::EJECT,       0)                           //Retract
              };

              int l = sizeof(ms) / sizeof(Motion*);
              for (int i = 0; i < l; i++) {
                motionQueue[motionQueueEnd++] = ms[i];
                //Serial.print(ms[i]->absPosition); Serial.print(",");
                if (MOTION_QUEUE_LENGTH - 1 < motionQueueEnd)
                  motionQueueEnd = 0;
              }

              break;
            }

          case (EMPTY_STACK_TYPE): {
              Serial.println(F("Stack position is empty"));
              return;
            }
          case (UNKNOWN_STACK_TYPE):
          default:
            Serial.println(F("Unknown stack type"));
            return;
        }

        break;
      }

    case Command::REMOVE: {
        Serial.print(F("REMOVE"));
        Serial.print(F(" ("));
        Serial.print(cmd->valueA);
        Serial.println(F(")"));
        int stack = cmd->valueA;

        if (STACKS_AMOUNT <= stack || stacks[stack] == -1) {
          Serial.println(F("Stack unavailable"));
          break;
        }

        switch (stackTypes[stack]) {
          case (DISH_56_MM_STACK_TYPE):
          case (DISH_93_MM_STACK_TYPE): {
           /* Serial.print("Stack entry point: ");
            Serial.print(stackLayerPosition + stackEntryPoints[stack][0]);
            Serial.print(',');
            Serial.print(stackEntryPoints[stack][1]);
            Serial.print(',');
            Serial.println(stackEntryPoints[stack][2]);
            
            Serial.print("Top dish position: ");
            Serial.println(topDishPositions[stack]);*/

            Serial.print(F("Lid parking point: "));
            Serial.print(lidParkingPoint[0]);
            Serial.print(',');
            Serial.print(lidParkingPoint[1]);
            Serial.print(',');
            Serial.println(lidParkingPoint[2]);
            
               
              Motion* ms[] = {
                //Get in a movable state...
                new Motion(Motion::GRIP,        GRIPPER_OPEN),                //Release grip
                new Motion(Motion::EJECT,       0),                           //Retract

                //Move to insertions point and grab
                new Motion(Motion::VERTICAL,    platformInsertionPos[0] + MOTION_DISTANCE_TO_PLATFORM),     //Move A + dishHeight
                new Motion(Motion::HORIZONTAL,  platformInsertionPos[1]),     //Move B
                new Motion(Motion::EJECT,       platformInsertionPos[2]),     //Eject C
                new Motion(Motion::VERTICAL,    platformInsertionPos[0]),     //Move A
                new Motion(Motion::GRIP,        GRIPPER_CLOSE),               //Grab

                //Retract to a movable position
                new Motion(Motion::VERTICAL,    platformInsertionPos[0] + MOTION_DISTANCE_TO_PLATFORM),     //Move A + dishHeight
                new Motion(Motion::EJECT,       0),                           //Retract

                //Move to lid parking and release
                new Motion(Motion::VERTICAL,    lidParkingPoint[0] + MOTION_DISTANCE_TO_PLATFORM), //Move A
                new Motion(Motion::HORIZONTAL,  lidParkingPoint[1]),          //Move B
                new Motion(Motion::EJECT,       lidParkingPoint[2]),          //Eject C
                new Motion(Motion::VERTICAL,    lidParkingPoint[0]),          //Move A
                new Motion(Motion::GRIP,        GRIPPER_OPEN),                //Release

                //Lift and retract
                new Motion(Motion::VERTICAL,    lidParkingPoint[0] + MOTION_DISTANCE_TO_PLATFORM), //Move A
                new Motion(Motion::EJECT,       0),                           //Retract

                //Now go to insertion point for bottom
                new Motion(Motion::VERTICAL,    platformInsertionPos[0] + MOTION_DISTANCE_TO_PLATFORM),     //Move A + dishHeight
                new Motion(Motion::HORIZONTAL,  platformInsertionPos[1]),     //Move B
                new Motion(Motion::EJECT,       platformInsertionPos[2]),     //Eject C
                new Motion(Motion::VERTICAL,    platformInsertionPos[0]),     //Move A
                new Motion(Motion::GRIP,        GRIPPER_CLOSE),               //Grab

                //Retract to a movable position
                new Motion(Motion::VERTICAL,    platformInsertionPos[0] + MOTION_DISTANCE_TO_PLATFORM),     //Move A + dishHeight
                new Motion(Motion::EJECT,       0),                           //Retract

                //Move to stack entry point
                new Motion(Motion::VERTICAL,    stackLayerPosition + stackEntryPoints[stack][0]),  //Move a
                new Motion(Motion::HORIZONTAL,  stackEntryPoints[stack][1]),  //Move b
                new Motion(Motion::EJECT,       stackEntryPoints[stack][2]),  //Eject c

                //Lower to first plate and grab lid (and lid ONLY)
                new Motion(Motion::VERTICAL,    stackLayerPosition + topDishPositions[stack] + DISH_HEIGHT),     //Move a
                new Motion(Motion::GRIP,        GRIPPER_OPEN),               //Grab

                //Move to stack entry point and retract
                new Motion(Motion::VERTICAL,    stackLayerPosition + stackEntryPoints[stack][0]),  //Move a
                new Motion(Motion::EJECT,       0),                           //Retract

                //Move to lid parking and release
                new Motion(Motion::VERTICAL,    lidParkingPoint[0] + MOTION_DISTANCE_TO_PLATFORM), //Move A
                new Motion(Motion::HORIZONTAL,  lidParkingPoint[1]),          //Move B
                new Motion(Motion::EJECT,       lidParkingPoint[2]),          //Eject C
                new Motion(Motion::VERTICAL,    lidParkingPoint[0]),          //Move A
                new Motion(Motion::GRIP,        GRIPPER_CLOSE),                //Release

                //Lift and retract
                new Motion(Motion::VERTICAL,    lidParkingPoint[0] + MOTION_DISTANCE_TO_PLATFORM), //Move A
                new Motion(Motion::EJECT,       0),                           //Eject C
                
                //Move to stack entry point
                new Motion(Motion::VERTICAL,    stackLayerPosition + stackEntryPoints[stack][0]),  //Move a
                new Motion(Motion::HORIZONTAL,  stackEntryPoints[stack][1]),  //Move b
                new Motion(Motion::EJECT,       stackEntryPoints[stack][2]),  //Eject c

                //Lower to first plate and release
                new Motion(Motion::VERTICAL,    stackLayerPosition + topDishPositions[stack] + DISH_HEIGHT),     //Move a
                new Motion(Motion::GRIP,        GRIPPER_OPEN),               //Grab

                //Move to stack entry point and retract
                new Motion(Motion::VERTICAL,    stackLayerPosition + stackEntryPoints[stack][0]),  //Move a
                new Motion(Motion::EJECT,       0)                          //Retract
              };

              int l = sizeof(ms) / sizeof(Motion*);
              //Serial.print(F("Adding motions: "));
              //Serial.println(l);

              /*Serial.print(F("Size of ms: "));
              Serial.println(sizeof(ms));*/
              for (int i = 0; i < l; i++) {
                motionQueue[motionQueueEnd++] = ms[i];
                //Serial.print(ms[i]->absPosition); Serial.print(",");
                if (MOTION_QUEUE_LENGTH - 1 < motionQueueEnd)
                  motionQueueEnd = 0;
              }
              break;
            }

          case (WELLS_STACK_TYPE): {
              Motion* ms[] = {
                //Get in a movable state...
                new Motion(Motion::GRIP,        GRIPPER_OPEN),                //Release grip
                new Motion(Motion::EJECT,       0),                           //Retract

                //Move to insertions point and grab plate
                new Motion(Motion::VERTICAL,    platformInsertionPos[0] + MOTION_DISTANCE_TO_PLATFORM),     //Move A + dishHeight
                new Motion(Motion::HORIZONTAL,  platformInsertionPos[1]),     //Move B
                new Motion(Motion::EJECT,       platformInsertionPos[2]),     //Eject C
                new Motion(Motion::VERTICAL,    platformInsertionPos[0]),     //Move A
                new Motion(Motion::GRIP,        GRIPPER_CLOSE),               //Grab
               
                //Lift a bit and retract
                new Motion(Motion::VERTICAL,    platformInsertionPos[0] + MOTION_DISTANCE_TO_PLATFORM),     //Move A + dishHeight
                new Motion(Motion::EJECT,       0),                           //Retract

                //Move to stack entry point
                new Motion(Motion::VERTICAL,    stackLayerPosition + stackEntryPoints[stack][0]),  //Move a
                new Motion(Motion::HORIZONTAL,  stackEntryPoints[stack][1]),  //Move b
                new Motion(Motion::EJECT,       stackEntryPoints[stack][2]),  //Eject c

                //Lower to first plate and release
                new Motion(Motion::VERTICAL,    stackLayerPosition + topDishPositions[stack] + WELL_PLATE_HEIGHT),     //Move a
                new Motion(Motion::GRIP,        GRIPPER_OPEN),               //Grab

                //Move to stack entry point and retract
                new Motion(Motion::VERTICAL,    stackLayerPosition + stackEntryPoints[stack][0]),  //Move a
                new Motion(Motion::EJECT,       0),                           //Retract
              };

              int l = sizeof(ms) / sizeof(Motion*);
              for (int i = 0; i < l; i++) {
                motionQueue[motionQueueEnd++] = ms[i];
                //Serial.print(ms[i]->absPosition); Serial.print(",");
                if (MOTION_QUEUE_LENGTH - 1 < motionQueueEnd)
                  motionQueueEnd = 0;
              }

              break;
            }

          case (EMPTY_STACK_TYPE): {
              Serial.println(F("Stack position is empty"));
              return;
            }
          case (UNKNOWN_STACK_TYPE):
          default:
            Serial.println(F("Unknown stack type"));
            return;
        }
        
        break;
      }

    case Command::CALIBRATE: {
        Serial.println(F("CALIBRATE"));
        calibrate();

        break;
      }

    case Command::TO_ORIGO: {
        Serial.println(F("TO_ORIGO"));

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
        Serial.print(F("INSERTION_POINT"));
        Serial.print(F(" ("));
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
          Serial.print(F("New insertion point set"));
          platformInsertionPos[0] = x;
          platformInsertionPos[1] = y;
          platformInsertionPos[2] = z;
        } else {
          Serial.print(F("Invalid new insertion"));
        }

        break;
      }

    case Command::GOTO_POINT: {
        Serial.print(F("GOTO_POINT"));
        Serial.print(F(" ("));
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
          //Serial.print("Going to point - ");

          Motion* ms[] = {
            new Motion(Motion::GRIP,        GRIPPER_OPEN), //Release grip
            new Motion(Motion::EJECT,       0),            //Retract
            new Motion(Motion::VERTICAL,    x),            //Move X
            new Motion(Motion::HORIZONTAL,  y),            //Move Y
            new Motion(Motion::EJECT,       z),            //Eject
            new Motion(Motion::GRIP,        g),            //Grap
          };

          int l = sizeof(ms) / sizeof(Motion*);
          for (int i = 0; i < l; i++) {
            motionQueue[motionQueueEnd++] = ms[i];
            //Serial.print(ms[i]->absPosition); Serial.print(",");
            if (MOTION_QUEUE_LENGTH - 1 < motionQueueEnd)
              motionQueueEnd = 0;
          }
        } else {
          Serial.println(F("Invalid goto point provided."));
        }

        /*Serial.print('(');
        Serial.print(x);
        Serial.print(',');
        Serial.print(y);
        Serial.print(',');
        Serial.print(z);
        Serial.print(',');
        Serial.print(g);
        Serial.println(')');*/

        break;
      }

    default:
      Serial.print(F("Unknown command action: "));
      Serial.println(cmd->action);
  }
}

static volatile boolean motionDone = false;
void handleMotions() {
  //Serial.print('m');
  //Return if motion queue is empty
  if (motionQueueStart == motionQueueEnd) {
    //Serial.println("STATE_IDLE");
    _state = STATE_IDLE;
    return;
  }
  //Serial.println("ASDFA");
  Motion* motion = motionQueue[motionQueueStart];

  motionDone = false;
  switch (motion->axis) {
    case Motion::VERTICAL: {
        long pos = vertAxis.currentPosition();
        /*Serial.print("Vert pos: ");
          Serial.println(pos);*/
        if (pos == targetPosition[0]) {
          //Serial.println("Reached vertical");
          motionDone = true;
        }
        break;
      }
    case Motion::HORIZONTAL: {
        long pos = horiAxis.currentPosition();
        /*Serial.print("Hori pos: ");
          Serial.println(pos);*/


        if (pos == targetPosition[1]) {
          //Serial.println("Reached horizontal");
          motionDone = true;
        }
        break;
      }
    case Motion::EJECT: {
        long pos = ejectAxis.currentPosition();
        /*Serial.print("EJECT pos: ");
          Serial.println(pos);*/

        if (pos == targetPosition[2]) {
          //          Serial.println("Reached ejection");
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
    case Motion::MEASURE_STACK: {
        //Serial.print("Measure stack!!");
        int stack = motion->absPosition;
        checkDishAmountInStack(stack);
        motionDone = true;
        break;
      }
    /*case Motion::CALIBRATE: {
      //This method is blocking, so it has been taken care of
      motionDone = true;
      }*/

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
    //Serial.print("Next motion: ");
    Motion* m = motionQueue[motionQueueStart];
    switch (m->axis) {
      case Motion::VERTICAL: {
      //  Serial.print("Vert - ");
          long tarPos = m->absPosition;
          vertAxis.moveTo(tarPos);
          targetPosition[0] = tarPos;
          _state = STATE_RUNNING;
      //    Serial.println(tarPos);

          break;
        }
      case Motion::HORIZONTAL: {
        //  Serial.print("Hori - ");
          long tarPos = m->absPosition;
          horiAxis.moveTo(tarPos);
          targetPosition[1] = tarPos;
          _state = STATE_RUNNING;
       //   Serial.println(tarPos);
          
          break;
        }
      case Motion::EJECT: {
       //   Serial.print("Eject - ");
          long tarPos = m->absPosition;
          ejectAxis.moveTo(tarPos);
          targetPosition[2] = tarPos;
          //TODO May need to call setSpeed() to keep a constant speed. Line 343 at: http://www.airspayce.com/mikem/arduino/AccelStepper/AccelStepper_8h_source.html
          _state = STATE_RUNNING;
        //  Serial.println(tarPos);
          
          break;
        }
      case Motion::GRIP: {
       //   Serial.print("Grib - ");
          long pos = m->absPosition;
          targetPosition[3] = pos;
          _state = STATE_GRIPPING;
        //  Serial.println(pos);

          break;
        }
      case Motion::MEASURE_STACK: {
          //Serial.print("Measure stack!!");
          int stack = m->absPosition;
          checkDishAmountInStack(stack);
          break;
        }
      /*case Motion::CALIBRATE: {
        calibrate();
        break;
        }*/
      default:
        ;//Nothing
    }
  }
}

void executeRun() {
  //Serial.println("Running");
  //Serial.print('r');
  vertAxis.run();
  horiAxis.run();
  ejectAxis.run();
  // printPosition();
}

void executeGrip() {
  //Serial.println("Opening/closing gripper");
  openCloseGripper();
  //printPosition();
}

void checkForChangesInStack() {
  //Serial.print('c');
  //Measure all values
  //Serial.print("STACKS_AMOUNT: ");
  //Serial.println(STACKS_AMOUNT);
  int values;
  for (int i = 0; i < STACKS_AMOUNT; i++) {
    values = analogRead(STACK_RESI_PINS[i]);
 
    //If new value differ to much from old the old values
    if (STACK_CHANGE_MAX_DELTA < abs(stackChangeValues[i] - values) ||
        stackChangeValues[i] < (0 - STACK_CHANGE_MAX_DELTA)) {
      stackChangeValues[i] = values;
      Serial.print(F("Stack "));
      Serial.print(i);
      Serial.println(F(" has changed!"));


      /*
         Figure out which type it is
      */
      int type;
      //Empty
      if (abs(STACK_EMPTY_VALUE - values) < STACK_CHANGE_MAX_DELTA) {
        //setStackAsEmty(i);
        type = EMPTY_STACK_TYPE;

        //56 mm dish
      } else if (abs(STACK_56MM_PETRI_DISH_VALUE - values) < STACK_CHANGE_MAX_DELTA) {
        type = DISH_56_MM_STACK_TYPE;

        //94 mm dish
      } else if (abs(STACK_94MM_PETRI_DISH_VALUE - values) < STACK_CHANGE_MAX_DELTA) {
        type = DISH_93_MM_STACK_TYPE;

        //Wells
      } else if (abs(STACK_WELLS_DISH_VALUE - values) < STACK_CHANGE_MAX_DELTA) {
        type = WELLS_STACK_TYPE;

        //Default to unknown...
      } else {
        type = UNKNOWN_STACK_TYPE;
      }

      stackTypes[i] = type;
      if (type == EMPTY_STACK_TYPE || type == UNKNOWN_STACK_TYPE) {
        stacks[i] = 0;

        //Update entry points
        stackEntryPoints[i][0] = -1;
        stackEntryPoints[i][1] = -1;
        stackEntryPoints[i][2] = -1;

        //Update top positions of dish
        topDishPositions[i] = -1;
      } else {
        //Create measuring motion
        Motion* motion = new Motion(Motion::MEASURE_STACK, i);

        //Insert motion as the next to be executed
        //NB Max size of motion queue is 64, so (motionQueueStart <= motionQueueEnd) should ALWAYS be true!!
        int queueLength = motionQueueEnd - motionQueueStart;
        //Serial.print("Queue length: ");
        //Serial.println(queueLength);

        if (0 < queueLength) {
          //Move all motions one to the right, but leave the current motion that is being executed
          for (int qI = queueLength; 1 < qI; qI--) {
            motionQueue[motionQueueStart + qI + 1] = motionQueue[motionQueueStart + qI];
          }
          motionQueueEnd++;

          motionQueue[motionQueueStart + 1] = motion;
        } else {
          motionQueue[motionQueueStart] = motion;
          motionQueueEnd++;
        }
        //Serial.println("Motion inserted");


        /*
         * Define entry points
         */
        switch (type) {
          case (DISH_56_MM_STACK_TYPE): {
              stackEntryPoints[i][0] = stackLayerPosition + DISH_56_MM_STACK_ENTRY_HEIGHT;
              stackEntryPoints[i][2] = DISH_56_MM_STACK_ENTRY_DEPTH;

              break;
            }
          case (DISH_93_MM_STACK_TYPE): {
              stackEntryPoints[i][0] = stackLayerPosition + DISH_93_MM_STACK_ENTRY_HEIGHT;
              stackEntryPoints[i][2] = DISH_93_MM_STACK_ENTRY_DEPTH;

              break;
            }
          case (WELLS_STACK_TYPE): {
              stackEntryPoints[i][0] = stackLayerPosition + WELL_STACK_ENTRY_HEIGHT;
              stackEntryPoints[i][2] = WELL_STACK_ENTRY_DEPTH;

              break;
            }

        }
        //The horizontal coordinate will always be the same, no-matter which stack type that is present
        stackEntryPoints[i][1] = ALL_STACKS_ENTRY_HORI_POSITION[i];
      }
    }
  }
}

void printPosition() {
  Serial.print(F("Position ["));
  Serial.print(vertAxis.currentPosition());
  Serial.print(F("->"));
  Serial.print(vertAxis.targetPosition());
  Serial.print(',');
  Serial.print(horiAxis.currentPosition());
  Serial.print(F("->"));
  Serial.print(horiAxis.targetPosition());
  Serial.print(',');
  Serial.print(ejectAxis.currentPosition());
  Serial.print(F("->"));
  Serial.print(ejectAxis.targetPosition());
  Serial.print(',');
  Serial.print(currentPosition[3]);
  Serial.print(F("] - Target ["));
  Serial.print(targetPosition[0]);
  Serial.print(',');
  Serial.print(targetPosition[1]);
  Serial.print(',');
  Serial.print(targetPosition[2]);
  Serial.print(',');
  Serial.print(targetPosition[3]);
  Serial.println(']');

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

void handleMsg() {
  //Serial.print('h');
  int index = msg.indexOf(MSG_SEP_CHAR);

  //Is there at least one full command
  if (index < 0)
    return;
  //Serial.print("Msg: ");
  //Serial.println(msg);
  while (-1 < index) {
    if (index == 0) {
      //if msg starts with MSG_SEP_CHAR (empty command) then reject "command"
      msg = msg.substring(1);
      Serial.println(CMD_UKNOWN);
    } else {

      String cmd = msg.substring(0, index);
      msg = msg.substring(index + 1); //Remove MSG_SEP_CHAR

      Serial.print(F("Got command: ")); Serial.println(cmd);

      boolean isValidCmd = false;
      switch (cmd[0]) {
        case CMD_DISPENSE: {
            if (!isDigit(cmd[1]))
              break;

            isValidCmd = true;
            int stack = -1;
            stack = String(cmd[1]).toInt();
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
            long xValue = -1;
            char array[16]; 
            cmd.substring(1, i).toCharArray(array, sizeof(array));
            xValue = atol(array);  
            cmd = cmd.substring(i + 1);

            i = cmd.indexOf(',');
            long yValue = -1;
            cmd.substring(0, i).toCharArray(array, sizeof(array));
            yValue = atol(array);  
            cmd = cmd.substring(i + 1);

            i = cmd.indexOf(',');
            long zValue = -1;
            cmd.substring(0, i).toCharArray(array, sizeof(array));
            zValue = atol(array);  
            cmd = cmd.substring(i + 1);

            Serial.print(F("Values: "));
            Serial.print(xValue);
            Serial.print(',');
            Serial.print(yValue);
            Serial.print(',');
            Serial.println(zValue);
            

            if (-1 < xValue && -1 < yValue && -1 < zValue) {
              isValidCmd = true;
              setInsertionPoint(xValue, yValue, zValue);
            }
            break;
          }
        case CMD_GOTO: {
            int i = cmd.indexOf(',');
            long xValue = -1;

            char array[16]; 
            cmd.substring(1, i).toCharArray(array, sizeof(array));
            xValue = atol(array);  
            cmd = cmd.substring(i + 1);

            i = cmd.indexOf(',');
            long yValue = -1;
            cmd.substring(0, i).toCharArray(array, sizeof(array));
            yValue = atol(array);  
            cmd = cmd.substring(i + 1);

            i = cmd.indexOf(',');
            long zValue = -1;
            cmd.substring(0, i).toCharArray(array, sizeof(array));
            zValue = atol(array);  
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
        Serial.print(F("Invalid command: "));
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
  //Serial.print('s');
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


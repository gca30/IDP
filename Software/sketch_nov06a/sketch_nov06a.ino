#include <Adafruit_MotorShield.h>

// SENSOR PINS
const int redButtonPin = 2;
const int frontLeftLSP = 6; // LSP = line sensor pin
const int frontRightLSP = 4;
const int axleLeftLSP = 5;
const int axleRightLSP = 3;
const int magneticPin = 999;
const int ultrasonicPin = A0;
const int tofPin = 999;
const int errorLEDPin = 999;

// SENSOR VALUES
int frontLeft, frontRight, axleLeft,axleRight;
int button = LOW, prevButton = LOW;

// HARD CODED DISTANCES
const int movetime = 1450;
const int rotatetime = 1300;

// STATE
char points[] = "x\
ooooo\
ooooo\
xooox\
xoxox";

enum State {
    INACTIVE, // completely stopped
    LINE_FOLLOWING, // we line follow until one of the back sensor is white
    CHANGE_DIRECTION, // we change our direction at a junction until the desiredDir is hit
    DEPOSIT // depositing of the cube from the starting position
};
void setState(State newState);
State state = INACTIVE;
enum Direction {
    NORTH = -5,
    SOUTH = 5,
    WEST = -1,
    EAST = 1
};
enum CubeState {
    NO_CUBE,
    NOT_MAGNETIC,
    MAGNETIC
};
CubeState cubeState = NO_CUBE;
int cubesCollected = 0;
int ourPoint = 13;
Direction direct = NORTH;
bool backwards = false;
Direction desiredDir = NORTH;
int clearedRotation = 0;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* motor1 = AFMS.getMotor(1);
Adafruit_DCMotor* motor2 = AFMS.getMotor(3);

enum Movement {
  STOPPED,
  R_LEFT,
  R_RIGHT,
  M_FORWARD,
  M_BACKWARD
};
static inline void setMovement(Movement m) {
    switch(m) {
      case STOPPED:
        motor1->run(RELEASE);
        motor2->run(RELEASE);
        break;
      case R_LEFT:
        motor1->setSpeed(150);
        motor2->setSpeed(150);
        motor1->run(FORWARD);
        motor2->run(FORWARD);
        break;
      case R_RIGHT:
        motor1->setSpeed(150);
        motor2->setSpeed(150);
        motor1->run(BACKWARD);
        motor2->run(BACKWARD);
        break;
      case M_FORWARD:
        motor1->setSpeed(200);
        motor2->setSpeed(200);
        motor1->run(FORWARD);
        motor2->run(BACKWARD);
        break;
      case M_BACKWARD:
        motor1->setSpeed(200);
        motor2->setSpeed(200);
        motor1->run(BACKWARD);
        motor2->run(FORWARD);
        break;
    }
}

float getUltrasonicDistance() {
  float sensity_t = analogRead(ultrasonicPin);
  float dist_t = sensity_t * 520 / 1023.0;
  return dist_t;
}

void setup() {
    Serial.begin(9600); // Init the serial port
    if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
    }
    Serial.println("Starting!!!!1!");
    motor1->setSpeed(150);
    motor1->run(RELEASE);
    motor2->setSpeed(150);
    motor2->run(RELEASE);
      
    pinMode(frontLeftLSP, INPUT); 
    pinMode(frontRightLSP, INPUT);
    pinMode(axleLeftLSP, INPUT); 
    pinMode(axleRightLSP, INPUT);
    pinMode(redButtonPin, INPUT);
}

Direction getDesiredDirection() {
  if(cubeState == NO_CUBE) {
    switch(ourPoint) {
      case 1:
        return EAST;
      case 2:
        if(direct == NORTH)
          return WEST;
        else if(direct == EAST)
          return EAST;
        break;
      case 3:
        return EAST;
      case 4:
        if(direct == EAST)
          return EAST;
        else if(direct == WEST)
          return SOUTH;
        break;
      case 5:
        return WEST;
      case 6:
        return EAST;
      case 7:
        if(direct == WEST)
          return WEST;
        else if(direct == EAST)
          return NORTH;
        break;
      case 8:
        if(direct == NORTH)
          return WEST;
        else if(direct == EAST)
          return EAST;
        else if(direct == WEST)
          return NORTH;
        else if(direct == SOUTH)
          return SOUTH;
        break;
       case 9:
        if(direct == SOUTH)
          return EAST;
        else if(direct == WEST)
          return WEST;
        break;
      case 10:
        return WEST;
    }
  }
  //digitalWrite(errorLEDPin, HIGH);
  setState(INACTIVE);
  return SOUTH;
  
  // we know ourPoint, current direction, cubeState
  if(cubeState == NO_CUBE) {
    switch(ourPoint) {
      case 8:
        return direct;
      case 10:
        return NORTH;
      case 3:
        if(direct == NORTH)
          return WEST;
        else
          return SOUTH;
      case 2:
      case 5:
      case 4:
        return WEST;
      case 1:
        return SOUTH;
      case 6:
      case 7:
      case 9:
        return EAST;
    }
  }
}

bool shouldChangeState = false;
bool outOfJunction;

void lineFollowingSetup() {
  outOfJunction = false;
}

void lineFollowingLoop() {
    
   if (frontLeft == 0 && frontRight == 1)
      setMovement(R_RIGHT);
   else if (frontLeft == 1 && frontRight == 0)
      setMovement(R_LEFT);
   else if (frontLeft == 1 && frontRight == 1)
      setMovement(M_FORWARD);
   else if(frontLeft == 0 && frontRight == 0)
      setMovement(M_FORWARD);

    // we can start in a junction and we must first ignore it
    if(axleRight == 0 && axleLeft == 0)
      outOfJunction = true;
    if(outOfJunction && (axleRight == 1 || axleLeft == 1))
        shouldChangeState = true;
}

int rotationIterations;

void changeDirectionSetup() {
    desiredDir = getDesiredDirection();
    outOfJunction = false;
    if(desiredDir == direct) {
      shouldChangeState = true;
       return;
    }
    rotationIterations = direct == -desiredDir ? 2 : 1;
    if(
        (direct == NORTH && desiredDir == WEST) ||
        (direct == WEST && desiredDir == SOUTH) ||
        (direct == SOUTH && desiredDir == EAST) ||
        (direct == EAST && desiredDir == NORTH)
    )
      setMovement(R_LEFT);
    else if(direct == -desiredDir && (ourPoint-1)%5 > 2) // special case when turing at point 6
      setMovement(R_LEFT);
    else
      setMovement(R_RIGHT);
}

void changeDirectionLoop() {

    // it can only exit the loop if it has cleared the line
    if(frontLeft == LOW && frontRight == LOW)
      outOfJunction = true;

    if(frontLeft != HIGH && frontRight != HIGH)
       return; //loop back
    if(!outOfJunction)
        return;
    if(rotationIterations > 1) {
      rotationIterations--;
      outOfJunction = false;  
      return;
    }
    shouldChangeState = true;
}

State prevState = LINE_FOLLOWING;
void setState(State newState) {
  if(newState == state)
    return;
  prevState = state;
  state = newState;
  switch(state) {
    case LINE_FOLLOWING:
      lineFollowingSetup();
      break;
    case CHANGE_DIRECTION:
      changeDirectionSetup();
      break;
    default:
      break;
  }
}

void stateLoop() {

    // Button shuts down
    if(prevButton == HIGH && button == LOW) {
        if(state == INACTIVE) {
            state = prevState;
            prevState = INACTIVE;
        }
        else {
            prevState = state;
            state = INACTIVE;
        }
    }
    prevButton = button;

    // Ultrasonic sensor changes LED state
    // TO DO
    

    // 
    if(!shouldChangeState)
      return;
    shouldChangeState = false;
    
    switch(state) {
      case LINE_FOLLOWING:
          ourPoint += direct;
          if(ourPoint == 13)
            setState(DEPOSIT);
          else
            setState(CHANGE_DIRECTION);
          break;
      case CHANGE_DIRECTION:
         direct = desiredDir;
         if(ourPoint == 13)
            setState(DEPOSIT);
         else
            setState(LINE_FOLLOWING);
        break;
    }
}

void loop() {

    // SENSOR READING
    button = digitalRead(redButtonPin);
    frontLeft = digitalRead(frontLeftLSP);
    frontRight = digitalRead(frontRightLSP);
    axleLeft = digitalRead(axleLeftLSP);
    axleRight = digitalRead(axleRightLSP);

    stateLoop();

    // ACTION BASED ON STATE
    switch(state) {
       case INACTIVE:
       case DEPOSIT:
          setMovement(STOPPED);
          break;
       case LINE_FOLLOWING:
          lineFollowingLoop();
          break;
       case CHANGE_DIRECTION:
          changeDirectionLoop();
          break;
   }
          
  delay(5);
}

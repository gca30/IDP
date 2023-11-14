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

// SENSOR VALUES
int button = LOW, prevButton = LOW;

// HARD CODED DISTANCES
const int movetime = 1450;
const int rotatetime = 1300;

// STATE
char points[] = "x\
ooooo\
ooooo\
xxoxx";
enum State {
    INACTIVE, // completely stopped
    LINE_FOLLOWING, // we line follow until one of the back sensor is white
    CHANGE_DIRECTION, // we change our direction at a junction until the desiredDir is hit
    DEPOSIT // depositing of the cube from the starting position
};
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
Direction direct = NORTH, desiredDir = NORTH;
int clearedLine = 0;

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
        motor1->run(FORWARD);
        motor2->run(FORWARD);
        break;
      case R_RIGHT:
        motor1->run(BACKWARD);
        motor2->run(BACKWARD);
        break;
      case M_FORWARD:
        motor1->run(FORWARD);
        motor2->run(BACKWARD);
        break;
      case M_BACKWARD:
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

void lineFollowingLoop() {
    int frontLeft = digitalRead(frontLeftLSP);
    int frontRight = digitalRead(frontRightLSP);
    int axleLeft = digitalRead(axleLeftLSP);
    int axleRight = digitalRead(axleRightLSP);
    
   if (frontLeft == 0 && frontRight == 1)
      setMovement(R_RIGHT);
   else if (frontLeft == 1 && frontRight == 0)
      setMovement(R_LEFT);
   else if (frontLeft == 1 && frontRight == 1)
      setMovement(M_FORWARD);
   else if(frontLeft == 0 && frontRight == 0)
      setMovement(M_FORWARD);

    if(axleRight == 0 && axleLeft == 0)
      clearedLine = false;
   if(!clearedLine && (axleRight == 1 || axleLeft == 1)) {
        clearedLine = true;
        ourPoint += direct;
        if(ourPoint == 13) { //arrived at start square
          state = DEPOSIT;
        }
        else {
          desiredDir = getDesiredDirection();
          state = CHANGE_DIRECTION;
        }
   }
}

void changeDirectionLoop() {
    int frontLeft = digitalRead(frontLeftLSP);
    int frontRight = digitalRead(frontRightLSP);
    int axleLeft = digitalRead(axleLeftLSP);
    int axleRight = digitalRead(axleRightLSP);

    if(desiredDir == direct) {
       if(ourPoint == 13)
          state = DEPOSIT;
       else
          state = LINE_FOLLOWING;
      return;
    }

    // current direct, desiredDir
    Movement m = R_RIGHT;
    if(
      (direct == NORTH && desiredDir == WEST) ||
      (direct == WEST && desiredDir == SOUTH) ||
      (direct == SOUTH && desiredDir == EAST) ||
      (direct == EAST && desiredDir == NORTH)
    )
      setMovement(R_LEFT);
    else
      setMovement(R_RIGHT);
    
    if(frontLeft != HIGH && frontRight != HIGH)
       return;

    setMovement(STOPPED);

    if(desiredDir == -direct) {
      switch(direct) {
        case NORTH: direct = EAST;  break;
        case EAST:  direct = SOUTH; break;
        case SOUTH: direct = WEST;  break;
        case WEST:  direct = NORTH; break;
      }
    }
    else
      direct = desiredDir;
}

void loop() {

    // SENSOR READING
    button = digitalRead(redButtonPin);

    // BUTTON
    if(prevButton == HIGH && button == LOW) {
       if(state == INACTIVE)
          state = LINE_FOLLOWING;
       else
          state = INACTIVE;
    }
    prevButton = button;

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
          
  delay(10);
}

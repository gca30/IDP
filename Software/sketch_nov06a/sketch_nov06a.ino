#include <Adafruit_MotorShield.h>

// ----------------
// VARIABLES AND CONSTANTS
// ----------------

// SENSOR PINS
const int redButtonPin = 2;
const int frontLeftLSP = 6; // LSP = line sensor pin
const int frontRightLSP = 4;
const int axleLeftLSP = 5;
const int axleRightLSP = 3;
const int magneticPin = 999;
const int ultrasonicPin = A0;
const int redLEDPin = 999;
const int greenLEDPin = 999;
const int blueLEDPin = 999;
const int tofPin = 999;
const int magneticSensorPin = 999;
const int errorLEDPin = 999;

// SENSOR VALUES
int frontLeft, frontRight, axleLeft, axleRight;
float ultrasonic = 500.0, prevUltrasonic;
int magnetic = LOW;
int button = LOW, prevButton = LOW;

// HARD CODED DISTANCES
// const int movetime = 1450; ~ length of the car
// const int rotatetime = 1300; ~ 90 degrees

// STATE
char points[] = "x\
ooooo\
ooooo\
xooox\
xoxox";

int delayCounter = 1000;

enum State {
    INACTIVE, // completely stopped
    INITIAL, // the initial state of the car, in which it needs to move forward until it hits the line
    LINE_FOLLOWING, // we line follow until one of the back sensor is white
    CHANGE_DIRECTION, // we change our direction at a junction until the desiredDir is hit
    DEPOSIT // depositing of the cube from the starting position
    DELAY,
};

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

enum TaskState {
    FIRST_CUBE,
    SECOND_CUBE,
    SCANNING_FIRST,
    SCANNING_SECOND
};

enum Movement {
    STOPPED,
    R_LEFT,
    R_RIGHT,
    M_FORWARD,
    M_BACKWARD
};

void setState(State newState);
State state = INACTIVE;
TaskState taskState = FIRST_CUBE;
CubeState cubeState = NO_CUBE;

int positon = 18;
Direction directn = NORTH;
bool backwards = false;

Direction desiredDir = NORTH;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* motor1 = AFMS.getMotor(1);
Adafruit_DCMotor* motor2 = AFMS.getMotor(3);



// ----------------
// HELPER FUNCTIONS
// ----------------

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

#define MEDIAN_SIZE 13
float getUltrasonicDistance() {
    static float dists[MEDIAN_SIZE];
    static float sorted[MEDIAN_SIZE];
    float sensity_t = analogRead(ultrasonicPin);
    float dist_t = sensity_t * 520 / 1023.0;
    
    for(int i = 0; i < MEDIAN_SIZE-1; i++)
        dists[i] = dists[i+1];
    dists[MEDIAN_SIZE-1] = dist_t;
    for(int i = 0; i < MEDIAN_SIZE; i++)
        sorted[i] = dists[i];
    
    for(int i = 1; i < MEDIAN_SIZE; i++) {
        int j = i;
        while(j > 0 && sorted[j-1] > sorted[j]) {
            float temp = sorted[j];
            sorted[j] = sorted[j-1];
            sorted[j-1] = temp;
            j--;
        }
    }
    return sorted[MEDIAN_SIZE/2];
}
#undef MEDIAN_SIZE

void setCubeState(CubeState cb) {
    if(cb == cubeState)
        return;
    cubeState = cb;
    switch (cubeState) {
        case NO_CUBE:
            digitalWrite(redLEDPin, LOW);
            digitalWrite(greenLEDPin, LOW);
            break;
        case NOT_MAGNETIC:
            digitalWrite(redLEDPin, LOW);
            digitalWrite(greenLEDPin, HIGH);
            break;
        case MAGNETIC:
            digitalWrite(redLEDPin, HIGH);
            digitalWrite(greenLEDPin, LOW);
            break;
    }
}



// ----------------
// ALGORITHM FOR THE ROBOT
// ----------------

Direction getDesiredDirection() {
    if(cubeState == NO_CUBE) {
        Serial.print("at pos ");
            Serial.print(positon);
            Serial.print(" keep direction ");
            Serial.println(directn);
        switch(positon) {
        case 1:
            return EAST;
        case 2:
            if(directn == NORTH)
                return WEST;
            else if(directn == EAST)
                return EAST;
            break;
        case 3:
            return EAST;
        case 4:
            if(directn == EAST)
                return EAST;
            else if(directn == WEST)
                return SOUTH;
            break;
        case 5:
            return WEST;
        case 6:
            return EAST;
        case 7:
            if(directn == WEST)
                return WEST;
            else if(directn == EAST)
                return NORTH;
            break;
        case 8:
            if(directn == NORTH)
                return WEST;
            else if(directn == EAST)
                return EAST;
            else if(directn == WEST)
                return NORTH;
            else if(directn == SOUTH)
                return SOUTH;
            break;
        case 9:
            if(directn == SOUTH)
                return EAST;
            else if(directn == WEST)
                return WEST;
            break;
        case 10:
            return WEST;
        case 13:
            return directn;
        default:
            return directn;
        }
    }
    else {
        Serial.println("CUBE GOT");
        switch(positon % 5) {
          case 1: case 2:
              return EAST;
          case 3:
              return SOUTH;
          case 4: case 0:
              return WEST;
        }
        return SOUTH;
    }

    // unreachable 
    // digitalWrite(errorLEDPin, HIGH);
    setState(INACTIVE);
    return NORTH;
}

CubeState csAtLastJunction = NO_CUBE;

void updateOnJunction() {
    positon += directn;
    Serial.print("pos = ");
    Serial.print(positon);
    Serial.print(", dir = ");
    Serial.print(directn);
    Serial.print(", desired dir = ");
    Serial.print(desiredDir);
    Serial.println(" !!!");
    desiredDir = getDesiredDirection();
    if(csAtLastJunction == NO_CUBE)
        points[positon] = 'e'; // now the position is are empty
    csAtLastJunction = cubeState;
}



// ----------------
// SPECIFIC STATE FUNCTIONS
// ----------------

// LINE FOLLOWING STATE

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
    if(outOfJunction && (axleRight == 1 || axleLeft == 1)) {
        updateOnJunction();
        if(positon == 13 && directn == SOUTH)
            setState(DEPOSIT);
        else if(desiredDir != directn)
            setState(CHANGE_DIRECTION);
        else
            setState(LINE_FOLLOWING);
    }
}

// CHANGE DIRECTION STATE

int rotationIterations;

void changeDirectionSetup() {
    outOfJunction = false;
    rotationIterations = directn == -desiredDir ? 2 : 1;
    if(
        (directn == NORTH && desiredDir == WEST) ||
        (directn == WEST && desiredDir == SOUTH) ||
        (directn == SOUTH && desiredDir == EAST) ||
        (directn == EAST && desiredDir == NORTH)
    )
        setMovement(R_LEFT);
    else if(directn == -desiredDir && (positon-1)%5 > 2) // special case when turing at point 6
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
    directn = desiredDir;
    setState(LINE_FOLLOWING);
}

// INITIAL STATE

void initialSetup() {
  positon = 18;
}

void initialLoop() {
    // initial wait time to get ultrasonic sensor median readings
    static int counter = 13;
    if(counter > 0) {
        counter--;
        if(counter == 0)
            setMovement(M_FORWARD);
        return;
    }
    // move forward until we hit the line
    if(frontLeft == HIGH && frontRight == HIGH)
        setState(LINE_FOLLOWING);
}


// DELAY STATE

void delaySetup() {}

void delayLoop() {
    if(delayCounter == 0) {
        setState(prevState);
        return;
    }
    delayCounter--;
}

// INACTIVE STATE

void inactiveSetup() {
    setMovement(STOPPED);
}

void inactiveLoop() {
  ;;
}

// DEPOSIT STATE

const int depositRotate = 260;
const int depositBack = 500;
const int depositForward = 3000;
int depositProgress = 0;
int depositTimer = 0;
CubeState initialCubeState = 0;

void depositSetup() {
    depositProgress = 0;  
    initialCubeState = cubeState;
}

void depositLoop() {
    if(depositTimer > 0) {
        depositTimer--;
        return;
    }
    switch(depositProgress) {
        case 0:
            setMovement(M_FORWARD);
            depositTimer = 20;
            break;
        case 1:
            setMovement(initialCubeState == MAGNETIC ? M_RIGHT : M_LEFT);
            depositTimer = depositRotate;
            break;
        case 2:
            setMovement(M_FORWARD);
            depositTimer = depositForward;
            break;
        case 3:
            setMovement(M_RELEASE);
            depositTimer = 50;
            break;
        case 4:
            setMovement(M_BACKWARD);
            depositTimer = depositBack;
            break;
        case 5:
            setMovement(M_BACKWARD);
            depositTimer = depositForward - depositBack;
            break;
        case 6:
            setMovement(initialCubeState == MAGNETIC ? M_LEFT : M_RIGHT);
            depositTimer = depositRotate;
            break;
        case 7:
            taskState++;
            setState(INITIAL);
            return;
    }
    depositProgress++;
}


// ----------------
// STATE MANAGEMENT
// ----------------

State prevState = INITIAL;

// STATE SETUP
void setState(State newState) {
    if(newState == state)
        return;
    prevState = state;
    state = newState;
    // do setup for the new state
    switch(state) {
        case LINE_FOLLOWING:
            lineFollowingSetup();
            break;
        case CHANGE_DIRECTION:
            changeDirectionSetup();
            break;
        case INACTIVE:
            inactiveSetup();
            break;
        case DELAY:
            delaySetup();
        default:
            break;
    }
}

void stateSensors() {
    // Button sets inactive state
    if(prevButton == HIGH && button == LOW)
        setState(state == INACTIVE ? prevState : INACTIVE);

    // Ultrasonic sensor changes LED state
    if(cubeState == NO_CUBE)
        Serial.println(ultrasonic);
    if(
        (frontRight == HIGH || frontLeft == HIGH) && 
        state != INACTIVE && state != INITIAL && cubeState == NO_CUBE
        (ultrasonic < 6.0 || ultrasonic > 480.0)
    ) {
        if(magnetic == HIGH)
            setCubeState(MAGNETIC);
        else
            setCubeState(NOT_MAGNETIC);
        setMovement(STOPPED);
        delayCounter = 1000;
        setState(DELAY);
    }
}

void statesLoop() {
    // ACTION BASED ON STATE
    switch(state) {
        case INACTIVE:
            inactiveLoop();
            break;
        case DEPOSIT:
            depositLoop();
            break;
        case INITIAL:
            initialLoop();
            break;
        case DELAY:
            delayLoop();
            break;
        case LINE_FOLLOWING:
            lineFollowingLoop();
            break;
        case CHANGE_DIRECTION:
            changeDirectionLoop();
            break;
    }
}


// ----------------
// ARDUINO FUNCTIONS
// ----------------

void setup() {
    Serial.begin(9600);
    if (!AFMS.begin()) {
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
    }
    Serial.println("Begin");
    motor1->setSpeed(150);
    motor1->run(RELEASE);
    motor2->setSpeed(150);
    motor2->run(RELEASE);
      
    pinMode(frontLeftLSP, INPUT); 
    pinMode(frontRightLSP, INPUT);
    pinMode(axleLeftLSP, INPUT); 
    pinMode(axleRightLSP, INPUT);
    pinMode(redButtonPin, INPUT);
    // pinMode(magneticSensorPin, INPUT);
    // pinMode(ultrasonicPin, INPUT);
    // pinMode(redLEDPin, OUTPUT);
    // pinMode(greenLEDPin, OUTPUT);
    // pinMode(blueLEDPin, OUTPUT);
    // pinMode(errorLEDPin, INPUT);
    // pinMode(tofPin, INPUT);
}

void loop() {

    // SENSOR READING
    prevButton = button;
    button = digitalRead(redButtonPin);
    frontLeft = digitalRead(frontLeftLSP);
    frontRight = digitalRead(frontRightLSP);
    axleLeft = digitalRead(axleLeftLSP);
    axleRight = digitalRead(axleRightLSP);
    ultrasonic = getUltrasonicDistance();
    magnetic = digitalRead(magneticSensorPin);

    setState(DELAY);

    statesSensors();
    statesChange();
    statesLoop();      
    delay(5);
}

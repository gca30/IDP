#include <Adafruit_MotorShield.h>
#include <DFRobot_VL53L0X.h>
#include <Wire.h>

// ----------------
// VARIABLES AND CONSTANTS
// ----------------

// SENSOR PINS
const int redButtonPin = 0;
const int frontLeftLSP = 2; // LSP = line sensor pin
const int frontRightLSP = 1;
const int axleLeftLSP = 4;
const int axleRightLSP = 3;
const int ultrasonicPin = A0;
const int redLEDPin = 7;
const int greenLEDPin = 8;
const int blueLEDPin = 6;
const int magneticSensorPin = 5;

// SENSOR VALUES
int frontLeft, frontRight, axleLeft, axleRight;
float ultrasonic = 500.0, prevUltrasonic;
float tof;
int magnetic = LOW;
int button = LOW, prevButton = LOW;

// HARD CODED DISTANCES
// const int movetime = 1450; ~ length of the car
// const int rotatetime = 1250; ~ 90 degrees

// STATE
char pointsMatrix[] = "\
o---o\
ooooo\
ooooo\
xooox\
xoxox";
char* points = &pointsMatrix[4];
#define ARRAY_SIZE(ARRAY) (sizeof(ARRAY) / sizeof(ARRAY[0]) - 1)
char commands1[] = "mcWcmcNcmWcEmcmcScmEc";
char commands2[] = "mWmmNmEqqqqSmWmmSm";
int commandsCount = ARRAY_SIZE(commands1);
char* commands = commands1;
int commandsIndex = 0;


// Each state has a setup and a loop function

// There is a stateSwitch function that controls the transitions between states
void switchState();

enum State {
    INACTIVE, // The car is completely stopped
    INITIAL, // The initial state of the car, in which it needs to move forward until it hits the line
    LINE_FOLLOWING, // The car follows the line until one of the back sensor is white
    CHANGE_DIRECTION, // The car rotates at a junction until the desiredDirection is hit
    DEPOSIT, // Depositing of the cube from the starting position
    PICKUP_DELAY, // Delay when the cube is picked up, the car stops, and LEDs are turned on
    VIBE_CHECK, // Stopping and checking if there is a cube at the next junction
    DEPOSIT_2, // Second part of depositing (waiting 5 seconds on start square)
    WALKING_HERE, // Line following while looking for a block on the side
    CATCH_BLOCK
};

// The setup function is called when the state is entered
// The loop function is called every loop of the main loop
void setState(State newState);

enum Direction {
    NORTH = -5,
    SOUTH = 5,
    WEST = -1,
    EAST = 1,
    INVALID_DIR = 0
};

enum CubeState {
    NO_CUBE,
    CUBE_NEXT_JUNCTION,
    NOT_MAGNETIC,
    MAGNETIC
};

enum TaskState: int {
    LINE_FIRST_CUBE = 0,
    LINE_SECOND_CUBE = 1,
    SCANNING_FIRST_CUBE = 2,
    SCANNING_SECOND_CUBE = 3,
    DONE = 4
};

enum Movement {
    STOPPED,
    R_LEFT,
    R_RIGHT,
    M_FORWARD,
    M_BACKWARD,
    M_SLOW,
    PIVOTB_RIGHT,
    PIVOTB_LEFT,
    PIVOTF_RIGHT,
    PIVOTF_LEFT
};

enum BlueLEDState {
    BLED_OFF,
    BLED_ON,
    BLED_BLINKING,
};

BlueLEDState blueLEDState = BLED_OFF;

State state = INACTIVE;
TaskState taskState = SCANNING_FIRST_CUBE;
CubeState cubeState = NO_CUBE;

int positon = 18;
Direction directon = NORTH;

Direction desiredDir = NORTH;
State prevState = INITIAL;
const int fiveSeconds = 1000;
bool shouldBlinkBlueLED = false;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* motor1 = AFMS.getMotor(4);
Adafruit_DCMotor* motor2 = AFMS.getMotor(1);
DFRobot_VL53L0X tofSensor;

// ----------------
// HELPER FUNCTIONS
// ----------------

static inline void setMovement(Movement m) {
    // MOTOR1 = LEFT
    switch(m) {
        case STOPPED:
            motor1->run(RELEASE);
            motor2->run(RELEASE);
            break;
        case R_LEFT:
            motor1->setSpeed(200);
            motor2->setSpeed(200);
            motor1->run(BACKWARD);
            motor2->run(FORWARD);
            break;
        case R_RIGHT:
            motor1->setSpeed(200);
            motor2->setSpeed(200);
            motor1->run(FORWARD);
            motor2->run(BACKWARD);
            break;
        case M_FORWARD:
            motor1->setSpeed(250);
            motor2->setSpeed(250);
            motor1->run(FORWARD);
            motor2->run(FORWARD);
            break;
        case M_SLOW:
            motor1->setSpeed(150);
            motor2->setSpeed(150);
            motor1->run(FORWARD);
            motor2->run(FORWARD);
            break;
        case M_BACKWARD:
            motor1->setSpeed(250);
            motor2->setSpeed(250);
            motor1->run(BACKWARD);
            motor2->run(BACKWARD);
            break;
        case PIVOTB_RIGHT:
            motor1->setSpeed(200);
            motor2->setSpeed(200);
            motor1->run(RELEASE);
            motor2->run(BACKWARD);
            break;
        case PIVOTB_LEFT:
            motor1->setSpeed(200);
            motor2->setSpeed(200);
            motor1->run(BACKWARD);
            motor2->run(RELEASE);
            break;
        case PIVOTF_RIGHT:
            motor1->setSpeed(200);
            motor2->setSpeed(200);
            motor1->run(RELEASE);
            motor2->run(FORWARD);
            break;
        case PIVOTF_LEFT:
            motor1->setSpeed(200);
            motor2->setSpeed(200);
            motor1->run(FORWARD);
            motor2->run(RELEASE);
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


#define MEDIAN_SIZE 9
float time_of_flight() {
    static float dists[MEDIAN_SIZE];
    static float sorted[MEDIAN_SIZE];
    float flight = tofSensor.getDistance();
    return flight;

    for(int i = 0; i < MEDIAN_SIZE-1; i++)
        dists[i] = dists[i+1];
    dists[MEDIAN_SIZE-1] = flight;
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
        case CUBE_NEXT_JUNCTION:
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

Direction backDirection(int point) {
    switch(point % 5) {
        case 1: case 2:
            return EAST;
        case 4: case 0:
            return WEST;
        case 3:
            return SOUTH;
    }
}

bool validNext(int point, Direction facing) {
    if(!(0 < point + facing && point + facing < 11))
        return false;
    if((facing == EAST || facing == WEST) && (point-1)/5 != (point+facing-1)/5)
        return false;
    return true;
}

void admitDefeat() {
    setMovement(R_RIGHT); 
    while(1);
}

Direction rightOf(Direction d) {
    switch(d) {
        case NORTH: return EAST;
        case EAST: return SOUTH;
        case SOUTH: return WEST;
        case WEST: return NORTH;
    }
}

Direction directionFromChar(char c) {
    switch(c) {
        case 'N': return NORTH;
        case 'E': return EAST;
        case 'S': return SOUTH;
        case 'W': return WEST;
        default:  return INVALID_DIR;
    }
}

Direction flipVertically(Direction d) {
    switch(d) {
        case EAST: return WEST;
        case WEST: return EAST;
        default:   return d;
    }
}

bool hasValidRotJunction(int point, Direction d) {
    switch(d) {
        case NORTH: return !(point == 2 || point == 3 || point == 4);
        case EAST:  return !(point == 6 || point == 7 || point == 9 || point == 10);
        case SOUTH: return !(point == 5 || point == 10);
        case WEST:  return !(point == 1 || point == 6);
    }
}

bool isInTOFRange(float t) {
    return t < 900.0 && t > 30.0;
} 

// ----------------
// SPECIFIC STATE FUNCTIONS
// ----------------

// LINE FOLLOWING + TOF
bool reachedTheEnd;
bool outOfJunction;
bool whDelay = 0;

void walkingHereSetup() {
    blueLEDState = BLED_BLINKING;
    outOfJunction = false;
    whDelay = 0;
}

void walkingHereLoop() {
    if(whDelay < 10)
        whDelay++;
    if(isInTOFRange(tof)) {
        Serial.print(whDelay);
        Serial.print(" ");
        Serial.println(tof);
    }
    if(isInTOFRange(tof)) {
        Serial.println("GOT IT PLEASE");
        setState(CATCH_BLOCK);
        return;
    }
    Serial.println(tof);
    
    if (frontLeft == 0 && frontRight == 1)
        setMovement(R_RIGHT);
    else if (frontLeft == 1 && frontRight == 0)
        setMovement(R_LEFT);
    else if (frontLeft == 1 && frontRight == 1)
        setMovement(M_SLOW);
    else if(frontLeft == 0 && frontRight == 0) {
        reachedTheEnd = true;
        setMovement(M_SLOW);
    }

    // we can start in a junction and we must first ignore it
    if(axleRight == 0 && axleLeft == 0)
        outOfJunction = true;
    if(outOfJunction && (axleRight == 1 || axleLeft == 1)) {
        //updateOnJunction();
        positon += directon;
        switchState();
    }
}

// CATCH_BLOCK
int catchDelay = 200;
int catchStage = 0;
int catchCount = 0;

void catchBlockSetup() {
    catchDelay = 20;
    catchStage = 0;
    catchCount = 0;
    setMovement(M_FORWARD);
    blueLEDState = BLED_OFF;
}

void catchBlockLoop() {
    switch(catchStage) {
        case 0:
            catchDelay--;
            if(catchDelay == 0) {
                setMovement(M_FORWARD);
                catchStage++;
                catchDelay = 100;
            }
            break;
        case 1:
            if(isInTOFRange(tof))
                catchCount++;
            catchDelay--;
            if(catchDelay == 0) {
                if(catchCount < 50) {
                    Serial.println("NOT ENOUGH!!!!!!");
                    setState(WALKING_HERE);
                    return;
                }
                catchCount = 0;
                catchDelay = 120;
                catchStage++;
                setMovement(M_FORWARD);
            }
            break;
        case 2:
            catchDelay--;
            if(catchDelay == 0) {
                setMovement(R_LEFT);
                catchStage++;
            }
            break;
        case 3:
            if(axleRight == 1 && axleLeft == 1) {
                setMovement(M_FORWARD);
                catchStage++;
            }
            break;
        case 4:
            if(axleRight == 0 && axleLeft == 0)
                catchCount = 1;
            if(catchCount != 1)
              return;
            if(axleRight == 1 && axleLeft == 1) {
                catchStage++;
                catchCount = 0;
                setMovement(R_RIGHT);
            }
            else if(axleRight == 1)
                setMovement(PIVOTF_RIGHT);
            else if(axleLeft == 1)
                setMovement(PIVOTF_LEFT);
            break;
          case 5:
              if(axleRight == 0 && axleLeft == 0)
                  catchCount = 1;
              if(catchCount != 1)
                  return;
              if(axleRight == 1 && axleLeft == 1) {
                  catchStage++;
                  catchCount = 0;
                  directon = SOUTH;
                  positon = -3;
                  desiredDir = EAST;
                  setState(CHANGE_DIRECTION);
              }
              break;
    }
}

// LINE FOLLOWING STATE
void lineFollowingSetup() {
    blueLEDState = BLED_BLINKING;
    outOfJunction = false;
    reachedTheEnd = false;
}

void lineFollowingLoop() {
    
    if (frontLeft == 0 && frontRight == 1)
        setMovement(R_RIGHT);
    else if (frontLeft == 1 && frontRight == 0)
        setMovement(R_LEFT);
    else if (frontLeft == 1 && frontRight == 1)
        setMovement(M_FORWARD);
    else if(frontLeft == 0 && frontRight == 0) {
        reachedTheEnd = true;
        setMovement(M_FORWARD);
    }

    // we can start in a junction and we must first ignore it
    if(axleRight == 0 && axleLeft == 0)
        outOfJunction = true;
    if(outOfJunction && (axleRight == 1 || axleLeft == 1)) {
        //updateOnJunction();
        positon += directon;
        switchState();
    }
}

// CHANGE DIRECTION STATE
int rotationIterations;

void changeDirectionSetup() {
    if(directon == desiredDir) {
        switchState();
        return;
    }
    blueLEDState = BLED_OFF;
    outOfJunction = false;
    rotationIterations = directon == -desiredDir ? 2 : 1;
    if(desiredDir == -rightOf(directon))
        setMovement(R_LEFT);
    else if(directon == -desiredDir) {
        if(hasValidRotJunction(positon, rightOf(directon)))
            setMovement(R_RIGHT);
        else
            setMovement(R_LEFT);
    }
    else
      setMovement(R_RIGHT);
}

void changeDirectionLoop() {

    // it can only exit the loop if it has cleared the line
    if(frontLeft == LOW && frontRight == LOW)
      outOfJunction = true;

    if(frontLeft != HIGH || frontRight != HIGH)
        return; //loop back
    if(!outOfJunction)
        return;
    if(rotationIterations > 1) {
        rotationIterations--;
        outOfJunction = false;  
        return;
    }
    directon = desiredDir;
    switchState();
}

// VIBE CHECK DELAY
int vibeCheckDelay;

void vibeCheckSetup() {
    blueLEDState = BLED_OFF;
    setMovement(STOPPED);
    vibeCheckDelay = 70;
}

void vibeCheckLoop() {
    if(vibeCheckDelay > 0) {
        vibeCheckDelay--;
        return;
    }
    if(ultrasonic > 25.0 && ultrasonic < 40.0) {
        setCubeState(CUBE_NEXT_JUNCTION);
    }
    else
        points[positon + directon] = 'e';
    switchState();
}

// INITIAL STATE
int initialStateCounter = 100;

void initialSetup() {
    blueLEDState = BLED_OFF;
    positon = 18;
    directon = NORTH;
    commandsIndex = 0;
    initialStateCounter = 13;
    setMovement(STOPPED);
}

void initialLoop() {
    // initial wait time to get ultrasonic sensor median readings
    if(initialStateCounter > 0) {
        initialStateCounter--;
        if(initialStateCounter == 0)
            setMovement(M_FORWARD);
        return;
    }
    // move forward until we hit the line
    if(frontLeft == HIGH || frontRight == HIGH)
        setState(LINE_FOLLOWING);
}


// DELAY STATE
int delayCounter;

void pickupDelaySetup() {
    blueLEDState = BLED_OFF;
    setMovement(STOPPED);
    setCubeState(magnetic ? MAGNETIC : NOT_MAGNETIC);
    delayCounter = fiveSeconds;
}

void pickupDelayLoop() {
    if(delayCounter > 0) {
        delayCounter--;
        return;
    }
    switchState();
}

// INACTIVE STATE
void inactiveSetup() {
    blueLEDState = BLED_ON;
    setMovement(STOPPED);
}

void inactiveLoop() {
    ;;
}

// DEPOSIT STATE
const int depositSmallForward = 160;
const int depositRotateM = 240; 
const int depositRotateNM = 265;
const int depositForward = 850;
const int depositBackward = 450;
const int depositSmallBackward = 150;
const int depositFinalBack = 200;
int depositProgress = 0;
int depositTimer = 0;
CubeState initialCubeState = NO_CUBE;

void depositSetup() {
    depositProgress = 0;
    initialCubeState = cubeState;
}

void depositLoop() {
    if(depositTimer > 0) {
        depositTimer--;
        return;
    }
    switch(depositProgress+1) {
        case 1:
            blueLEDState = BLED_BLINKING;
            setMovement(M_FORWARD);
            depositTimer = depositSmallForward;
            break;
        case 2:
            blueLEDState = BLED_OFF;
            setMovement(initialCubeState == MAGNETIC ? R_RIGHT : R_LEFT);
            depositTimer = initialCubeState == MAGNETIC ? depositRotateM : depositRotateNM;
            break;
        case 3:
            blueLEDState = BLED_BLINKING;
            setMovement(M_FORWARD);
            depositTimer = depositForward;
            break;
        case 4:
            blueLEDState = BLED_ON;
            setMovement(STOPPED);
            depositTimer = 200; // CHECK LATER
            break;
        case 5:
            blueLEDState = BLED_BLINKING;
            setMovement(M_BACKWARD);
            setCubeState(NO_CUBE);
            depositTimer = depositBackward;
            break;
        case 6:
            blueLEDState = BLED_BLINKING;
            setMovement(M_BACKWARD);
            depositTimer = 0;
            break;
        case 7:
            blueLEDState = BLED_OFF;
            if(axleRight == HIGH && axleLeft == HIGH) {
                depositProgress++;
                setMovement(initialCubeState == MAGNETIC ? R_RIGHT : R_LEFT);
                depositTimer = initialCubeState == MAGNETIC ? depositRotateM : depositRotateM;
                return;
            }
            else if(axleRight == HIGH || axleLeft == HIGH) {
                if(axleRight == HIGH)
                    setMovement(PIVOTB_LEFT);
                else
                    setMovement(PIVOTB_RIGHT);
            }
            return;
        case 8:
            directon = NORTH;
            positon = initialCubeState == MAGNETIC ? 17 : 19;
            setState(LINE_FOLLOWING);
            break;
    }
    depositProgress++;
}

// DEPOSIT_2
void deposit2Setup() {
    depositProgress = 0;
    depositTimer = 0;
    setMovement(STOPPED);
}

void deposit2Loop() {
    if(depositTimer > 0) {
        depositTimer--;
        return;
    }
    depositProgress++;
    switch(depositProgress) {
        case 1:
            blueLEDState = BLED_BLINKING;
            setMovement(M_BACKWARD);
            depositTimer = depositFinalBack;
            break;
        case 2:
            blueLEDState = BLED_ON;
            setMovement(STOPPED);
            digitalWrite(blueLEDPin, HIGH);
            depositTimer = fiveSeconds;
            break;
        case 3:
            taskState = taskState + 1;
            if(taskState == DONE || taskState == SCANNING_FIRST_CUBE) {
                setState(INACTIVE);
                return;
            }
            if(taskState == SCANNING_FIRST_CUBE) {
                commandsCount = ARRAY_SIZE(commands2);
                commands = commands2;
            }
            setState(INITIAL);
            break;
    }
}

// ----------------
// STATE MANAGEMENT
// ----------------

// STATE SWITCH
// for the 4 states that will be found in the board

void switchState() {
    if(cubeState == NO_CUBE) {
        // special cases
        if(state == LINE_FOLLOWING && positon == 12) {
            desiredDir = EAST;
            setState(CHANGE_DIRECTION);
            return;
        }
        if(state == LINE_FOLLOWING && positon == 14) {
            desiredDir = WEST;
            setState(CHANGE_DIRECTION);
            return;
        }
        if(state == CHANGE_DIRECTION && (positon == 12 || positon == 14)) {
            setState(LINE_FOLLOWING);
            return;
        }
        if(state == LINE_FOLLOWING && positon == 13 && (directon == EAST || directon == WEST)) {
            desiredDir = NORTH;
            setState(CHANGE_DIRECTION);
            return;
        }
        if(state == CHANGE_DIRECTION && positon == 13 && directon == NORTH) {
            setState(DEPOSIT_2);
            return;
        }
        // following commands
        if(commands[commandsIndex] == 'c' && points[positon + directon] == 'e')
            commandsIndex++;
        if(commandsIndex == commandsCount)
            admitDefeat();
        commandsIndex++;
        switch(commands[commandsIndex-1]) {
            case 'm':
                setState(LINE_FOLLOWING);
                return;
            case 'c':
                setState(VIBE_CHECK);
                return;
            case 'q':
                setState(WALKING_HERE);
                return;
            case 'W':
            case 'E':
            case 'N':
            case 'S':
                desiredDir = directionFromChar(commands[commandsIndex-1]);
                if(taskState == LINE_SECOND_CUBE)
                    desiredDir = flipVertically(desiredDir);
                setState(CHANGE_DIRECTION);
                return;
        }
    }
    else if(cubeState == CUBE_NEXT_JUNCTION) {
        if(state == VIBE_CHECK)
            setState(LINE_FOLLOWING);
        else if(state == LINE_FOLLOWING)
            setState(PICKUP_DELAY);
        else admitDefeat(); /* impossible */ 
    }
    else {
        if(state == LINE_FOLLOWING && positon == 13)
            setState(DEPOSIT);
        else if(state == PICKUP_DELAY || state == LINE_FOLLOWING) {
            desiredDir = backDirection(positon);
            if(desiredDir == directon)
                setState(LINE_FOLLOWING);
            else
                setState(CHANGE_DIRECTION);
        }
        else setState(LINE_FOLLOWING);
    }
}

// STATE SETUP
void setState(State newState) {
    if(newState != state) {
        prevState = state;
        state = newState;
    }
    // do setup for the new state
    switch(state) {
        case CATCH_BLOCK:
            catchBlockSetup();
            break;
        case LINE_FOLLOWING:
            lineFollowingSetup();
            break;
        case CHANGE_DIRECTION:
            changeDirectionSetup();
            break;
        case DEPOSIT:
            depositSetup();
            break;
        case DEPOSIT_2:
            deposit2Setup();
            break;
        case INACTIVE:
            inactiveSetup();
            break;
        case PICKUP_DELAY:
            pickupDelaySetup();
            break;
        case INITIAL:
            initialSetup();
            break;
        case VIBE_CHECK:
            vibeCheckSetup();
            break;
        case WALKING_HERE:
            walkingHereSetup();
            break;
        default:
            break;
    }
}

int blueLEDTimer = 0;

void stateSensors() {
    // Button sets inactive state
    if(prevButton == HIGH && button == LOW)
        setState(state == INACTIVE ? prevState : INACTIVE);

    // Ultrasonic sensor changes LED state
    if(blueLEDState == BLED_BLINKING) {
        if(blueLEDTimer == 0) {
            blueLEDTimer = 50;
            digitalWrite(blueLEDPin, HIGH);
        }
        else if(blueLEDTimer == 25)
            digitalWrite(blueLEDPin, LOW);
        blueLEDTimer--;
    }
    else if(blueLEDState == BLED_ON)
        digitalWrite(blueLEDPin, HIGH);
    else if(blueLEDState == BLED_OFF)
        digitalWrite(blueLEDPin, LOW);
}

void statesLoop() {
    // ACTION BASED ON STATE
    switch(state) {
        case CATCH_BLOCK:
            catchBlockLoop();
            break;
        case INACTIVE:
            inactiveLoop();
            break;
        case DEPOSIT:
            depositLoop();
            break;
        case DEPOSIT_2:
            deposit2Loop();
            break;
        case INITIAL:
            initialLoop();
            break;
        case PICKUP_DELAY:
            pickupDelayLoop();
            break;
        case LINE_FOLLOWING:
            lineFollowingLoop();
            break;
        case CHANGE_DIRECTION:
            changeDirectionLoop();
            break;
        case VIBE_CHECK:
            vibeCheckLoop();
            break;
        case WALKING_HERE:
            walkingHereLoop();
            break;
    }
}


// ----------------
// ARDUINO FUNCTIONS
// ----------------

void setup() {
    Serial.begin(115200);
    Wire.begin();
    tofSensor.begin(0x50);
    tofSensor.setMode(tofSensor.eContinuous, tofSensor.eHigh);
    tofSensor.start();
    if (!AFMS.begin()) {
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
    }
    Serial.println("Begin");
    setMovement(STOPPED);
      
    pinMode(frontLeftLSP, INPUT); 
    pinMode(frontRightLSP, INPUT);
    pinMode(axleLeftLSP, INPUT); 
    pinMode(axleRightLSP, INPUT);
    pinMode(redButtonPin, INPUT);
    pinMode(magneticSensorPin, INPUT);
    pinMode(ultrasonicPin, INPUT);
    pinMode(redLEDPin, OUTPUT);
    pinMode(greenLEDPin, OUTPUT);
    pinMode(blueLEDPin, OUTPUT);
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
    //tof = time_of_flight();
    magnetic = digitalRead(magneticSensorPin);
    
    stateSensors();
    statesLoop();      
    delay(5);
}

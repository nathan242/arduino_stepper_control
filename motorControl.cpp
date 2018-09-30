// Add those defines to your program

// Set the maximum number of motors that the class will support
#define MAX_MOTORS        2

// Motor modes
#define MOTOR_STEP         0
#define MOTOR_CONSTANT     1

// Motor states
#define MOTOR_STOP         0
#define MOTOR_ACCEL        1
#define MOTOR_DECEL        2


class motorControl {
  public:
    motorControl();
    uint8_t addMotor(uint8_t mode, uint16_t stepDelay, uint8_t dirPin, uint8_t stepPin);
    void setDirection(uint8_t motor, uint8_t direction);
    void setSteps(uint8_t motor, uint16_t steps);
    void setAccel(uint8_t motor, uint16_t startDelay, uint16_t accelDelay);
    void setAccelState(uint8_t motor, uint8_t state);
    uint16_t getSteps(uint8_t motor);
    void run();
  private:
    uint8_t motorCount;
    uint8_t motorModes[MAX_MOTORS];
    uint8_t motorPins[MAX_MOTORS][2];
    uint16_t motorConfigs[MAX_MOTORS][4];
    unsigned long motorTimers[MAX_MOTORS];
    unsigned long motorAccelTimers[MAX_MOTORS];
    uint16_t motorStates[MAX_MOTORS][3];
};

motorControl::motorControl()
{
  motorCount = 0;
}

uint8_t motorControl::addMotor(uint8_t mode, uint16_t stepDelay, uint8_t dirPin, uint8_t stepPin)
{
  if (motorCount >= MAX_MOTORS || mode > 1) {
    return 0;
  }
  
  // Set pin modes for direction and step
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  
  // Motor mode
  motorModes[motorCount] = mode; // STEP or CONSTANT
  
  // Motor pins
  motorPins[motorCount][0] = dirPin; // Direction pin
  motorPins[motorCount][1] = stepPin; // Step pin
  
  // Motor configurations
  motorConfigs[motorCount][0] = stepDelay; // Delay for step signalling
  motorConfigs[motorCount][1] = 0; // Steps to move (STEP mode)
  motorConfigs[motorCount][2] = 0; // Start step signalling (CONSTANT mode)
  motorConfigs[motorCount][3] = 0; // Acceleration delay (CONSTANT mode)
  
  // Motor timers
  motorTimers[motorCount] = micros(); // Previous time
  motorAccelTimers[motorCount] = micros();  // Acceleration timer
  
  // Motor state variables
  motorStates[motorCount][0] = 0; // HIGH or LOW signalling state
  motorStates[motorCount][1] = 0; // Accelerate/decelerate/stop mode (CONSTANT mode)
  motorStates[motorCount][2] = 0; // Acceleration current speed (CONSTANT mode)
  
  // Set initial direction
  digitalWrite(dirPin, LOW);
  
  return ++motorCount;
}

// Set motor direction pin to HIGH or LOW
void motorControl::setDirection(uint8_t motor, uint8_t direction)
{
  if (motor == 0 || motor > motorCount) {
    return;
  }
  
  uint8_t index = motor-1;
  
  digitalWrite(motorPins[index][0], direction);
  
  return;
}

// Set steps that motor in STEP mode will make
void motorControl::setSteps(uint8_t motor, uint16_t steps)
{
  if (motor == 0 || motor > motorCount) {
    return;
  }
  
  uint8_t index = motor-1;
  
  motorConfigs[index][1] = steps;
  
  return;
}

// Set motor start speed and acceleration rate for motor in CONSTANT mode
void motorControl::setAccel(uint8_t motor, uint16_t startDelay, uint16_t accelDelay)
{
  if (motor == 0 || motor > motorCount) {
    return;
  }
  
  uint8_t index = motor-1;
  
  motorConfigs[index][2] = startDelay;
  motorConfigs[index][3] = accelDelay;
  
  return;
}

// Set motor acceleration/deceleration mode
void motorControl::setAccelState(uint8_t motor, uint8_t state)
{
  if (motor == 0 || motor > motorCount || state > 2) {
    return;
  }

  uint8_t index = motor-1;

  motorStates[index][1] = state;

  if (state == MOTOR_ACCEL) {
    motorStates[index][2] = motorConfigs[index][2];
  }

  return;
}

// Return number of remaining steps for motor in STEP mode
uint16_t motorControl::getSteps(uint8_t motor)
{
  if (motor == 0 || motor > motorCount) {
    return 0;
  }
  
  uint8_t index = motor-1;
  
  return motorConfigs[index][1];
}

// Motor scheduler. Run this in a loop
void motorControl::run()
{
  unsigned long now;
  
  for (uint8_t i = 0; i < motorCount; i++) {
    now = micros();
    
    if (motorModes[i] == MOTOR_STEP && motorConfigs[i][1] > 0) { // If we are set to step and there is at least 1 step on the counter
      if ((now - motorTimers[i]) > motorConfigs[i][0]) { // Check if enough time has passed to advance
        if (motorStates[i][0] == 0) {
          digitalWrite(motorPins[i][1], HIGH);
          motorStates[i][0] = 1;
        } else if (motorStates[i][0] == 1) {
          digitalWrite(motorPins[i][1], LOW);
          motorStates[i][0] = 0;
          motorConfigs[i][1]--; // Decrement step count
        }
        motorTimers[i] = micros(); // Update timestamp of last action
      }
    } else if (motorModes[i] == MOTOR_CONSTANT && motorStates[i][1] > 0) { // If we are set to constant and our state is not stopped
        if (motorStates[i][1] == MOTOR_ACCEL) { // Accelerate mode
          if ((now - motorTimers[i]) > motorStates[i][2]) { // Has enough time passed in the current state to advance
            if (motorStates[i][0] == 0) {
              digitalWrite(motorPins[i][1], HIGH);
              motorStates[i][0] = 1;
            } else if (motorStates[i][0] == 1) {
              digitalWrite(motorPins[i][1], LOW);
              motorStates[i][0] = 0;
              if (motorStates[i][2] > motorConfigs[i][0] && (now - motorAccelTimers[i]) > motorConfigs[i][3]) { // If we have passed enough time continue acceleration
                motorStates[i][2]--;
                motorAccelTimers[i] = micros();
              }
            }
            motorTimers[i] = micros(); // Update timestamp of last action  
          }
        } else if (motorStates[i][1] == MOTOR_DECEL) { // Decelerate mode
          if ((now - motorTimers[i]) > motorStates[i][2]) { // Has enough time passed in the current state to advance
            if (motorStates[i][0] == 0) {
              digitalWrite(motorPins[i][1], HIGH);
              motorStates[i][0] = 1;
            } else if (motorStates[i][0] == 1) {
              digitalWrite(motorPins[i][1], LOW);
              motorStates[i][0] = 0;
              if (motorStates[i][2] < motorConfigs[i][2] && (now - motorAccelTimers[i]) > motorConfigs[i][3]) { // If we have passed enough time continue deceleration
                motorStates[i][2]++;
                motorAccelTimers[i] = micros();
              }
            }
            motorTimers[i] = micros(); // Update timestamp of last action  
          }
        }
    }
  }
}


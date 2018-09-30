# Stepper motor controller class for Arduino

This is a C++ class for controlling stepper motors that use a step and direction pin.

Multiple motors can be run at the same time and a scheduler is used to determine when the step signal should be sent.

It can also support acceleration and deceleration of a constantly running stepper motor.

I have only tested this on an Arduino Uno but it should also work on other models.

## How to use the motorControl class.

Instantiate the class:

```motorControl motor = motorControl();```


Add motors with addMotors:

```motor.addMotor(MODE, SPEED, STEP PIN, DIRECTION PIN);```

MODE can be:

* STEP - Motor will only move a set number of steps (set via setSteps)
* CONSTANT - Motor will move constantly and can accelerate to speed
    
SPEED is the number of microseconds between each HIGH and LOW on the step pin.  
Each motor added will get a number starting at 1.
  
### Motor in STEP mode

```motor.setSteps (1, 200); // Sets motor 1 to 200 steps```
  
### Motor in CONSTANT mode

Set acceleration speed:

```motor.setAccel(MOTOR, START SPEED, ACCEL DELAY);```
  
Set mode to accelerate/decelerate:

```motor.setAccelState(MOTOR, STATE);```

STATE can be MOTOR\_ACCEL, MOTOR\_DECEL or MOTOR\_STOP

Set motor direction:

```motor.setDirection(MOTOR, HIGH/LOW);```
  
You must call motor.run() in a loop

### Example Program

```void setup() {```  
```  motorControl motor = motorControl();```  
```  motor.addMotor(MOTOR_STEP, 5000, 5, 4);```  

```  motor.addMotor(MOTOR_CONSTANT, 200, 7, 6);```  
  
```  motor.setSteps(1, 200);```  
```  motor.setAccel(2, 1000, 1000);```  
```  motor.setAccelState(2, 1);```  
  
```  unsigned long time = millis();```  
```  unsigned long now;```  
```  int d = 0;```  
```  int s = 0;```  
  
```  while (true) {```  
```    motor.run();```  
```    now = millis();```  
```    if ((now - time) > 5000) {```  
```      if (s == 0) {```  
```        motor.setAccelState(2,2);```  
```        s = 1;```  
```      } else {```  
```        motor.setAccelState(2,1);```  
```        s = 0;```  
```      }```  
```      time = millis();```  
```    }```  
```    if (motor.getSteps(1) < 1) {```  
```      if (d == 0) {```  
```        motor.setDirection(1, HIGH);```  
```        d = 1;```  
```      } else {```  
```        motor.setDirection(1, LOW);```  
```        d = 0;```  
```      }```  
```      motor.setSteps(1, 200);```  
```    }```  
```  }```  
```}```  

```void loop() {```  
  
```}```  


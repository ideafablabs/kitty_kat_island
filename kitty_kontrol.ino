// kitty_kontrol.ino - code to kontrol operation of stepper motor in the cats head
// 1/31/2024 - Idea Fab Labs, Chico
// current issue: runSpeed() doesn't support acceleration ...

#include <AccelStepper.h>

// Pin usage defines
#define PUL   9   // Stepper Pulse pin
#define DIR   8   // Stepper Direction pin
#define JUP   6   // Joystick Up pin
#define JDN   4   // Joystick Down pin
#define JLT   5   // Joystick Left pin
#define JRT   7   // Joystick Right pin
#define LLS   3   // Left Limit Switch pin
#define RLS   2   // Right Limit Switch pin
#define SPD   A0  // Potentiometer for speed pin
//#define ACC   A1  // Potentiometer for acceleration pin

// direction
#define STOP  0
#define CW    1
#define CCW   2

// Stepper variables
int MaxSpeed          = 500; // maximum speed for stepper
int MinSpeed          = 25;   // minimum speed for stepper
int Speed             = STOP; // initial speed stopped
//int MaxAcceleration = 200;  // maximum acceleration for stepper
//int MinAcceleration = 20;   // minimum acceleration for stepper
//int Acceleration    = 20;   // initial acceleration rate
int RUNNING           = STOP; // are we moving? and in what direction?
bool AUTOMATIC        = false;// are we in automatic (bounce) mode

// Joystick variables
int JoyLeft_curr    = HIGH;
int JoyLeft_last    = HIGH;
int JoyRight_curr   = HIGH;
int JoyRight_last   = HIGH;
int JoyUp_curr      = HIGH;
int JoyUp_last      = HIGH;
int JoyDown_curr    = HIGH;
int JoyDown_last    = HIGH;

// Interrupts variables
int stateRight_curr = HIGH;
int stateLeft_curr  = HIGH;
int stateRight_last = HIGH;
int stateLeft_last  = HIGH;

// Define the stepper and the pins used
AccelStepper stepper(AccelStepper::DRIVER, PUL, DIR);

// The setup
void setup() {
  Serial.begin(115200); // Initialize Serial
  Serial.println("Startup ...");

// Initialize digital pins
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(JUP, INPUT_PULLUP); 
  pinMode(JDN, INPUT_PULLUP);
  pinMode(JLT, INPUT_PULLUP);
  pinMode(JRT, INPUT_PULLUP);

// Initialize Limit-switch Interrupts
  pinMode(LLS, INPUT_PULLUP);
  pinMode(RLS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LLS), limitLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(RLS), limitRight, FALLING);

// Initialize stepper settings
  //stepper.setAcceleration(Acceleration);
  stepper.setMaxSpeed(MaxSpeed);
  stepper.setSpeed(Speed);
}

// The loop 
void loop() {
  checkLimits();
  readJoystick();
  moveStepper();
}

// Move stepper / update speed
void moveStepper() {
  if(RUNNING > STOP) {
    readSpeed();
    //readAcceleration();
    //stepper.setAcceleration(Acceleration);
    if(RUNNING == CCW) {
      Speed = -Speed; // CCW
    }
    stepper.setSpeed(Speed);
  }
  stepper.runSpeed();
}

// Check limit-switches & Process 
void checkLimits() {
  if(stateRight_curr != stateRight_last) {
    Serial.println("Right Limit Hit! ...");
    RUNNING = STOP;
    //stepper.setAcceleration(STOP); // hard stop!
    stepper.setSpeed(STOP);
    stepper.runSpeed(); // stop asap
    digitalWrite(LED_BUILTIN, LOW);    
    stateRight_last = stateRight_curr;
    if(AUTOMATIC) {
      stateRight_last = HIGH;
      stateRight_curr = HIGH;      
      RUNNING = CW;
      digitalWrite(LED_BUILTIN, HIGH);
    }
  } else {
    if(stateLeft_curr != stateLeft_last) {
      Serial.println("Left Limit Hit! ...");
      RUNNING = STOP;
      //stepper.setAcceleration(STOP); // hard stop!
      stepper.setSpeed(STOP);
      stepper.runSpeed(); // stop asap
      digitalWrite(LED_BUILTIN, LOW);    
      stateLeft_last = stateLeft_curr;
      if(AUTOMATIC) {
        stateLeft_last = HIGH;
        stateLeft_curr = HIGH;        
        RUNNING = CCW;
        digitalWrite(LED_BUILTIN, HIGH);
      }
    }
  }
}

// Read joystick switches & Process
void readJoystick() {
  JoyLeft_curr  = digitalRead(JLT);
  JoyRight_curr = digitalRead(JRT);
  JoyUp_curr    = digitalRead(JUP);
  JoyDown_curr  = digitalRead(JDN);

// Joystick left switch
  if(JoyLeft_curr != JoyLeft_last) {
    if(JoyLeft_curr == HIGH) {
      Serial.println("Stop ...");
      RUNNING = STOP;
      //stepper.setAcceleration(Acceleration);
      stepper.setSpeed(STOP);
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      if(AUTOMATIC) {
        AUTOMATIC = false;
        RUNNING = STOP;
        stepper.setSpeed(STOP);
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("Automatic stopped ...");
      }  
      if(stateLeft_last == LOW) { // left limit already reached!
        Serial.println("Left limit active!");
        JoyLeft_last = JoyLeft_curr;
        return;
      }
      if(stateRight_last == LOW) { // reset right limit switch...
        stateRight_last = HIGH;
        stateRight_curr = HIGH;        
      }
      Serial.println("Left ...");
      RUNNING = CW;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    JoyLeft_last = JoyLeft_curr;
  } 

// Joystick right switch
  if(JoyRight_curr != JoyRight_last) {
    if(JoyRight_curr == HIGH) {
      Serial.println("Stop ...");
      RUNNING = STOP;
      //stepper.setAcceleration(Acceleration);
      stepper.setSpeed(STOP);
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      if(AUTOMATIC) {
        AUTOMATIC = false;
        RUNNING = STOP;
        stepper.setSpeed(STOP);
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("Automatic stopped ...");
      }      
      if(stateRight_last == LOW) { // Right limit already reached!
        Serial.println("Right limit active!");
        JoyRight_last = JoyRight_curr;
        return;
      }
      if(stateLeft_last == LOW) { // reset left limit switch...
        stateLeft_last = HIGH;
        stateLeft_curr = HIGH;        
      }
      Serial.println("Right ...");
      RUNNING = CCW;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    JoyRight_last = JoyRight_curr;
  }

// Joystick up switch  
  if(JoyUp_curr != JoyUp_last) {
    if(JoyUp_curr == LOW) {
      if(AUTOMATIC) {
        Serial.println("Automatic already running ...");
        return;
      }
      AUTOMATIC = true;
      RUNNING = CCW; // FIXME test for tripped limit switches and go the other way ...
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("Automatic started ...");
    }
    JoyUp_last = JoyUp_curr;
  }
  
// Joystick down switch  
  if(JoyDown_curr != JoyDown_last) {
    if(JoyDown_curr == LOW) {
      if(AUTOMATIC) {
        AUTOMATIC = false;
        RUNNING = STOP;
        stepper.setSpeed(STOP);
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("Automatic stopped ...");
      }
    }
    JoyDown_last = JoyDown_curr;
  }  
}

// Read analog potentiometer / calculate speed
void readSpeed() {
 Speed = map((analogRead(SPD)), 0, 1023, MinSpeed, MaxSpeed);
}
/*
// Read analog potentiometer / calculate acceleration
void readAcceleration() {
  //Acceleration = map((analogRead(ACC)), 0, 1023, MinAcceleration, MaxAcceleration);
  Acceleration = 200;
}
*/

// Left limit-switch ISR
void limitLeft() {
  stateLeft_curr = LOW;  // slim and clean ISR
}

// Right limit-switch ISR
void limitRight() {
  stateRight_curr = LOW;  // slim and clean ISR
}


// sfranzyshen

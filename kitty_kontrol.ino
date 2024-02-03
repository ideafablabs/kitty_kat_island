// kitty_kontrol.ino - code to kontrol operation of stepper motor in the cats head
// 1/31/2024 - Idea Fab Labs, Chico

#include <AccelStepper.h>

// Pin usage
#define PUL 9    // Stepper Pulse pin
#define DIR 8    // Stepper Direction pin
//#define JUP 6  // Joystick Up pin
//#define JDN 4  // Joystick Down pin
#define JLT 5    // Joystick Left pin
#define JRT 7    // Joystick Right pin
#define LLS 3   // Left Limit Switch pin
#define RLS 2   // Right Limit Switch pin

// Speeds
#define STOP 0
#define FORW 1000
#define REVE -1000

// Variables
int MaxSpeed = 2000;
//int Acceleration = 20;
int Speed = STOP;  // Initial speed
int JoyLeft_curr = HIGH;
int JoyLeft_last = HIGH;
int JoyRight_curr = HIGH;
int JoyRight_last = HIGH;
//int JoyUp_curr = HIGH;
//int JoyUp_last = HIGH;
//int JoyDown_curr = HIGH;
//int JoyDown_last = HIGH;
int stateRight_curr = HIGH;
int stateLeft_curr = HIGH;
int stateRight_last = HIGH;
int stateLeft_last = HIGH;

// Define the stepper and the pins used
AccelStepper stepper(AccelStepper::DRIVER, PUL, DIR);

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  Serial.println("Startup ...");

// Initialize digital pins
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  //pinMode(JUP, INPUT_PULLUP); 
  //pinMode(JDN, INPUT_PULLUP);
  pinMode(JLT, INPUT_PULLUP);
  pinMode(JRT, INPUT_PULLUP);

// Initialize Limit-switch Interrupts
  pinMode(LLS, INPUT_PULLUP);
  pinMode(RLS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LLS), limitLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(RLS), limitRight, FALLING);

// Initialize stepper settings
  //stepper.setAcceleration(20.0);
  stepper.setMaxSpeed(MaxSpeed);
  stepper.setSpeed(Speed);
}

void loop() {
  checkLimits();
  readJoystick();
  stepper.runSpeed();
}

void checkLimits() {
  if(stateRight_curr != stateRight_last) {
    Serial.println("Right Limit Hit! ...");
    Speed = STOP;
    //stepper.setAcceleration(0); // hard stop!
    stepper.setSpeed(Speed);
    stepper.runSpeed(); // stop asap
    digitalWrite(LED_BUILTIN, LOW);    
    stateRight_last = stateRight_curr;
  } else {
    if(stateLeft_curr != stateLeft_last) {
      Serial.println("Left Limit Hit! ...");
      Speed = STOP;
      //stepper.setAcceleration(0); // hard stop!
      stepper.setSpeed(Speed);
      stepper.runSpeed(); // stop asap
      digitalWrite(LED_BUILTIN, LOW);    
      stateLeft_last = stateLeft_curr;
    }
  }
}

void readJoystick() {
  JoyLeft_curr = digitalRead(JLT);
  JoyRight_curr = digitalRead(JRT);
  //JoyUp_curr = digitalRead(JUP);
  //JoyDown_curr = digitalRead(JDN);

  if(JoyLeft_curr != JoyLeft_last) {
    if(JoyLeft_curr == HIGH) {
      Serial.println("Stop ...");
      Speed = STOP;
      //stepper.setAcceleration(Acceleration);
      stepper.setSpeed(Speed);
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      if(stateLeft_last == LOW) { // left limit already reached!
        Serial.println("Limit Hit!");
        JoyLeft_last = JoyLeft_curr;
        return;
      }
      if(stateRight_last == LOW) { // reset right limit switch...
        stateRight_last = HIGH;
        stateRight_curr = HIGH;        
      }
      Serial.println("Left ...");
      Speed = FORW;
      //stepper.setAcceleration(Acceleration);
      stepper.setSpeed(Speed);
      digitalWrite(LED_BUILTIN, HIGH);
    }
    JoyLeft_last = JoyLeft_curr;
  } 

  if(JoyRight_curr != JoyRight_last) {
    if(JoyRight_curr == HIGH) {
      Serial.println("Stop ...");
      Speed = STOP;
      //stepper.setAcceleration(Acceleration);
      stepper.setSpeed(Speed);
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      if(stateRight_last == LOW) { // Right limit already reached!
        Serial.println("Limit Hit!");
        JoyRight_last = JoyRight_curr;
        return;
      }
      if(stateLeft_last == LOW) { // reset left limit switch...
        stateLeft_last = HIGH;
        stateLeft_curr = HIGH;        
      }
      Serial.println("Right ...");
      Speed = REVE;
      //stepper.setAcceleration(Acceleration);
      stepper.setSpeed(Speed);
      digitalWrite(LED_BUILTIN, HIGH);
    }
    JoyRight_last = JoyRight_curr;
  }
}

void limitLeft() {
  stateLeft_curr = LOW;
}

void limitRight() {
  stateRight_curr = LOW;
}

// sfranzyshen

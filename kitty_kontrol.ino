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
#define LLS A0   // Left Limit Switch pin
#define RLS A1   // Right Limit Switch pin

// Speeds
#define STOP 0
#define FORW 1000
#define REVE -1000

// Variables
int MaxSpeed = 2000;
//int Acceleration = 0;
int Speed = STOP;  // Initial speed
int JoyLeft_curr = 0;
int JoyLeft_last = HIGH;
int JoyRight_curr = 0;
int JoyRight_last = HIGH;
//int JoyUp_curr = 0;
//int JoyUp_last = HIGH;
//int JoyDown_curr = 0;
//int JoyDown_last = HIGH;

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
  pinMode(LLS, INPUT_PULLUP);
  pinMode(RLS, INPUT_PULLUP);

  // Initialize stepper settings
  stepper.setMaxSpeed(MaxSpeed);
  stepper.setSpeed(Speed);
}

void loop() {
  readJoystick();
  stepper.setSpeed(Speed);
  stepper.runSpeed();
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
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      Serial.println("Left ...");
      Speed = FORW;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    JoyLeft_last = JoyLeft_curr;
    return;
  }

  if(JoyRight_curr != JoyRight_last) {
    if(JoyRight_curr == HIGH) {
      Serial.println("Stop ...");
      Speed = STOP;
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      Serial.println("Right ...");
      Speed = REVE;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    JoyRight_last = JoyRight_curr;
    return;
  }
}

// sfranzyshen

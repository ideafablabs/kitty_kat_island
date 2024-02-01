// kitty_kontrol.ino - code to kontrol operation of stepper motor in the cats head
// 1/31/2024 - Idea Fab Labs, Chico

#include <AccelStepper.h>

#define PUL 9  // Stepper Pulse pin
#define DIR 8  // Stepper Direction pin
#define JUP 6  // Joystick Up pin
#define JDN 4  // Joystick Down pin
#define JLT 5  // Joystick Left pin
#define JRT 7  // Joystick Right pin
#define LLS A0  // Left Limit Switch pin
#define RLS A1  // Right Limit Switch pin

int MaxSpeed = 1000;
int Acceleration = 20;

// Define the stepper and the pins used
AccelStepper stepper(1, PUL, DIR);

void setup() {
  Serial.begin(115200);
  Serial.println("Startup ...");

  pinMode(JUP, INPUT_PULLUP); 
  pinMode(JDN, INPUT_PULLUP);
  pinMode(JLT, INPUT_PULLUP);
  pinMode(JRT, INPUT_PULLUP);
  pinMode(LLS, INPUT_PULLUP);
  pinMode(RLS, INPUT_PULLUP);
  
  stepper.setMaxSpeed(MaxSpeed);
  stepper.setAcceleration(Acceleration);
}

void loop() {
  // we're getting there ...
}



// sfranzyshen
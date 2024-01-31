// simple bounce code to test basic operation of stepper
#include <AccelStepper.h>

#define PUL 9    // PUL- pin
#define DIR 8    // DIR- pin

// Define a stepper and the pins it will use
AccelStepper stepper(1, PUL, DIR);

void setup()
{  
  // Change these to suit your stepper if you want
  stepper.setMaxSpeed(100);
  stepper.setAcceleration(20);
  stepper.moveTo(500);
}

void loop()
{
    // If at the end of travel go to the other end
    if (stepper.distanceToGo() == 0)
      stepper.moveTo(-stepper.currentPosition());

    stepper.run();
}

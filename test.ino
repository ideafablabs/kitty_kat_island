
#include <AccelStepper.h>

#define PUL 9   // PUL- pin
#define DIR 8   // DIR- pin

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, PUL, DIR);

void setup()
{  
  // Change these to suit your stepper if you want
   stepper.setMaxSpeed(2000);
   stepper.setSpeed(1000);  
}

void loop()
{
   stepper.runSpeed();
}

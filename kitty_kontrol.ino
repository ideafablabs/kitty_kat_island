// kitty_kontrol.ino - code to kontrol operation of stepper motor in the cats head (and more)
// 1/31/2024 - Idea Fab Labs, Chico

// current issue: runSpeed() doesn't support acceleration ...
// see https://github.com/bblanchon/ArduinoContinuousStepper for future solution

#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TiCoServo.h>

// Pin usage defines (* = required to be this pin do not move)
#define PUL   9   // Stepper Pulse pin (output)
#define DIR   8   // Stepper Direction pin (output)
#define JUP   6   // Joystick Up pin (input)
#define JDN   4   // Joystick Down pin (input)
#define JLT   5   // Joystick Left pin (input)
#define JRT   7   // Joystick Right pin (input)
#define SRV   10  // Servo pin (output) *
#define ABA   A2  // Arcade button #A pin (input)
#define ABB   11  // Arcade button #B pin (input)
#define LLS   3   // Left Limit Switch pin (input ~ interrupt) *
#define RLS   2   // Right Limit Switch pin (input ~ interrupt) *
#define SPD   A0  // Potentiometer for speed pin (input ~ analog) *
//#define ACC   A7  // Potentiometer for acceleration pin (input ~ analog)

// Neopixel defines
#define RGB   A1  // Neopixel pin (output)
#define LEDS  8   // Number of Neopixels
#define BRGT  50  // Set brightness to about 1/5 (max = 255)

// servo predefined positions
#define SRVO_ON   180   // servo laser On position
#define SRVO_OFF  20    // servo laser Off position
#define SRVO_MIN  1000  // 1 ms pulse
#define SRVO_MAX  2000  // 2 ms pulse

// Stepper speed variables
int MaxSpeed  = 500;   // maximum speed for stepper
int MinSpeed  = 50;    // minimum speed for stepper

//---------------------------------------------------------------------------------------------------------------------------------------------------

// Stepper predefined direction
#define STOP  0
#define CW    1
#define CCW   2

// Predefined RGB Led patterns
#define CWR  0  // colorWipeRed
#define CWG  1  // colorWipeGreen
#define CWB  2  // colorWipeBlue
#define RNB  3  // rainbow
#define TCRB 4  // theaterChaseRainbow
#define BLK  5  // colorWipeBlack (off)

// Stepper more variables
int Speed             = STOP;  // initial speed stopped
//int MaxAcceleration = 200;   // maximum acceleration for stepper
//int MinAcceleration = 20;    // minimum acceleration for stepper
//int Acceleration    = 20;    // initial acceleration rate
int RUNNING           = STOP;  // are we moving? and in what direction?
bool AUTOMATIC        = false; // are we in automatic (bounce) mode

// Joystick  & Arcade button variables
int JoyLeft_curr    = HIGH;
int JoyLeft_last    = HIGH;
int JoyRight_curr   = HIGH;
int JoyRight_last   = HIGH;
int JoyUp_curr      = HIGH;
int JoyUp_last      = HIGH;
int JoyDown_curr    = HIGH;
int JoyDown_last    = HIGH;
int ArcadeA_curr    = HIGH;
int ArcadeA_last    = HIGH;
int ArcadeB_curr    = HIGH;
int ArcadeB_last    = HIGH;

// Interrupts variables
int stateRight_curr = HIGH;
int stateLeft_curr  = HIGH;
int stateRight_last = HIGH;
int stateLeft_last  = HIGH;

// neopixel variables
unsigned long    pixelPrevious     = 0;        // Previous Pixel Millis
//unsigned long    patternPrevious   = 0;        // Previous Pattern Millis
int              patternCurrent    = RNB;      // Current Pattern Number (Rainbow)
//int              patternInterval   = 5000;     // Pattern Interval (ms)
int              pixelInterval     = 50;       // Pixel Interval (ms)
int              pixelQueue        = 0;        // Pattern Pixel Queue
int              pixelCycle        = 0;        // Pattern Pixel Cycle
uint16_t         pixelCurrent      = 0;        // Pattern Current Pixel Number
uint16_t         pixelNumber       = LEDS;     // Total Number of Pixels

// Define the stepper and the pins used
AccelStepper stepper(AccelStepper::DRIVER, PUL, DIR);

// Define ws2812 leds
Adafruit_NeoPixel strip(LEDS, RGB, NEO_GRB + NEO_KHZ800);

// Define servo object to control a servo
Adafruit_TiCoServo servo;
int Servo_curr = SRVO_OFF;
int Servo_last = SRVO_OFF;

// The setup
void setup() {
  Serial.begin(115200); // Initialize Serial
  Serial.println("Startup ...");

// Initialize NeoPixel driver  
  strip.begin();             // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();              // Turn OFF all pixels ASAP
  strip.setBrightness(BRGT); // Set BRGT to about 1/5 (max = 255)
  
// Initialize digital pins
  pinMode(LED_BUILTIN, OUTPUT);   // onboard LED
  digitalWrite(LED_BUILTIN, LOW); 
  pinMode(JUP, INPUT_PULLUP);     // Joystick up
  pinMode(JDN, INPUT_PULLUP);     // Joystick down
  pinMode(JLT, INPUT_PULLUP);     // Joystick left
  pinMode(JRT, INPUT_PULLUP);     // Joystick right
  pinMode(ABA, INPUT_PULLUP);     // Arcade button A
  pinMode(ABB, INPUT_PULLUP);     // Arcade button B
  
// Initialize Limit-switch Interrupts
  pinMode(LLS, INPUT_PULLUP);
  pinMode(RLS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LLS), limitLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(RLS), limitRight, FALLING);

// Initialize stepper settings
  //stepper.setAcceleration(Acceleration);
  stepper.setMaxSpeed(MaxSpeed);
  stepper.setSpeed(Speed);

// Initialize servo settings
  servo.attach(SRV, SRVO_MIN, SRVO_MAX);  // attaches the servo to the servo object
  servo.write(Servo_curr);
}

// The loop 
void loop() {
  checkLimits();
  readJoystick();
  moveStepper();
  readArcade();
  moveServo();
  Neoloop();
}

// Move servo
void moveServo() {
  if(Servo_curr != Servo_last) {
    servo.write(Servo_curr);
    Servo_last = Servo_curr;
  }
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

// Read Arcade buttons & Process
void readArcade() {
  ArcadeA_curr = digitalRead(ABA);
  ArcadeB_curr = digitalRead(ABB);
  
  if(ArcadeA_curr != ArcadeA_last) {
    if(ArcadeA_curr == HIGH) {
      Serial.println("Arcade A off");
      // do nothing
    } else {
      Serial.println("Arcade A on");
      if(patternCurrent + 1 > 5) {
        patternCurrent = 0;
      } else {
        patternCurrent = patternCurrent + 1;
      }
    }
    ArcadeA_last = ArcadeA_curr;
  }

  if(ArcadeB_curr != ArcadeB_last) {
    if(ArcadeB_curr == HIGH) {
      Serial.println("Arcade B off");
      Servo_curr = SRVO_OFF;
    } else {
      Serial.println("Arcade B on");
      Servo_curr = SRVO_ON;
    }
    ArcadeB_last = ArcadeB_curr;
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

// Neopixel routines
void Neoloop() {
  unsigned long currentMillis = millis();                     //  Update current time
  
  if(currentMillis - pixelPrevious >= pixelInterval) {        //  Check for expired time
    pixelPrevious = currentMillis;                            //  Run current frame
    switch (patternCurrent) {
      case BLK:
        colorWipe(strip.Color(0, 0, 0), 50); // Blackout wipe
        break;
      case TCRB:
        theaterChaseRainbow(50); // Rainbow-enhanced theaterChase variant
        break;
      case RNB:
        rainbow(10); // Flowing rainbow cycle along the whole strip
        break;     
      case CWB:
        colorWipe(strip.Color(0, 0, 255), 50); // Blue
        break;
      case CWG:
        colorWipe(strip.Color(0, 255, 0), 50); // Green
        break;        
      default:
        colorWipe(strip.Color(255, 0, 0), 50); // Red
        break;
    }
  }
}

void colorWipe(uint32_t color, int wait) {
  if(pixelInterval != wait)
    pixelInterval = wait;                   //  Update delay time
  strip.setPixelColor(pixelCurrent, color); //  Set pixel's color (in RAM)
  strip.show();                             //  Update strip to match
  pixelCurrent++;                           //  Advance current pixel
  if(pixelCurrent >= pixelNumber)           //  Loop the pattern from the first LED
    pixelCurrent = 0;
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(uint8_t wait) {
  if(pixelInterval != wait)
    pixelInterval = wait;                   
  for(uint16_t i=0; i < pixelNumber; i++) {
    strip.setPixelColor(i, Wheel((i + pixelCycle) & 255)); //  Update delay time  
  }
  strip.show();                             //  Update strip to match
  pixelCycle++;                             //  Advance current cycle
  if(pixelCycle >= 256)
    pixelCycle = 0;                         //  Loop the cycle back to the begining
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  if(pixelInterval != wait)
    pixelInterval = wait;                   //  Update delay time  
  for(int i=0; i < pixelNumber; i+=3) {
    strip.setPixelColor(i + pixelQueue, Wheel((i + pixelCycle) % 255)); //  Update delay time  
  }
  strip.show();
  for(int i=0; i < pixelNumber; i+=3) {
    strip.setPixelColor(i + pixelQueue, strip.Color(0, 0, 0)); //  Update delay time  
  }      
  pixelQueue++;                           //  Advance current queue  
  pixelCycle++;                           //  Advance current cycle
  if(pixelQueue >= 3)
    pixelQueue = 0;                       //  Loop
  if(pixelCycle >= 256)
    pixelCycle = 0;                       //  Loop
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}




// sfranzyshen

// kitty_testing.ino - code to test operation switches
// 2/15/2024 - Idea Fab Labs, Chico

// Pin usage defines (* = required to be this pin do not move)
#define JUP   6   // Joystick Up pin (input)
#define JDN   4   // Joystick Down pin (input)
#define JLT   5   // Joystick Left pin (input)
#define JRT   7   // Joystick Right pin (input)
#define ABA   A2  // Arcade button #A pin (input)
#define ABB   A3  // Arcade button #B pin (input)
#define LLS   3   // Left Limit Switch pin (input ~ interrupt) *
#define RLS   2   // Right Limit Switch pin (input ~ interrupt) *

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

// The setup
void setup() {
  Serial.begin(115200); // Initialize Serial
  Serial.println("Startup ...");

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
  attachInterrupt(digitalPinToInterrupt(LLS), limitLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RLS), limitRight, CHANGE);

  // Poll current limit switch state ...
  stateRight_curr = digitalRead(RLS);
  stateLeft_curr = digitalRead(LLS);
}

// The loop 
void loop() {
  checkLimits();
  readJoystick();
  readArcade();
}

// Check limit-switches & Process 
void checkLimits() {
  if(stateRight_curr != stateRight_last) {
    if(stateRight_curr == HIGH) {
      Serial.println("Right Limit off");
    } else {
      Serial.println("Right Limit on");
    }
    stateRight_last = stateRight_curr;
  }
  
  if(stateLeft_curr != stateLeft_last) {
    if(stateLeft_curr == HIGH) {
      Serial.println("Left Limit off");
    } else {
      Serial.println("Left Limit on");
    }
    stateLeft_last = stateLeft_curr;
  }
}

// Read Arcade buttons & Process
void readArcade() {
  ArcadeA_curr = digitalRead(ABA);
  ArcadeB_curr = digitalRead(ABB);
  
  if(ArcadeA_curr != ArcadeA_last) {
    if(ArcadeA_curr == HIGH) {
      Serial.println("Arcade A off");
    } else {
      Serial.println("Arcade A on");
    }
    ArcadeA_last = ArcadeA_curr;
  }

  if(ArcadeB_curr != ArcadeB_last) {
    if(ArcadeB_curr == HIGH) {
      Serial.println("Arcade B off");
    } else {
      Serial.println("Arcade B on");
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
    if(JoyLeft_curr == LOW) {
      Serial.println("JoyLeft On");
    } else {
      Serial.println("JoyLeft Off");
      JoyLeft_last = JoyLeft_curr;
    }
  }

  // Joystick right switch
  if(JoyRight_curr != JoyRight_last) {
    if(JoyRight_curr == LOW) {
      Serial.println("JoyRight On");
    } else {
      Serial.println("JoyRight Off");
      JoyUp_last = JoyRight_curr;
    }
  }

  // Joystick up switch  
  if(JoyUp_curr != JoyUp_last) {
    if(JoyUp_curr == LOW) {
      Serial.println("JoyUp On");
    } else {
      Serial.println("JoyUp Off");
      JoyUp_last = JoyUp_curr;
    }
  }  
  
  // Joystick down switch  
  if(JoyDown_curr != JoyDown_last) {
    if(JoyDown_curr == LOW) {
      Serial.println("JoyDown On ");
    } else {
      Serial.println("JoyDown Off");
      JoyDown_last = JoyDown_curr;
    }
  }
}

// Left limit-switch ISR
void limitLeft() {
  if(digitalRead(LLS)) {
    stateLeft_curr = HIGH;  // slim and clean ISR
  } else {
    stateLeft_curr = LOW;  // slim and clean ISR
  }
}

// Right limit-switch ISR
void limitRight() {
  if(digitalRead(RLS)) {
    stateRight_curr = HIGH;  // slim and clean ISR
  } else {
    stateRight_curr = LOW;  // slim and clean ISR
  }
}

// sfranzyshen

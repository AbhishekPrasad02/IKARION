// #include <AccelStepper.h>

// // Stepper motor 1 pins (Joint 1)
// #define STEP1_PIN 44
// #define DIR1_PIN 42
// #define ENABLE1_PIN 46

// // Stepper motor 2 pins (Joint 2)
// #define STEP2_PIN 38
// #define DIR2_PIN 36
// #define ENABLE2_PIN 40

// #define DIR_PIN A14
// #define STEP_PIN 32
// #define ENABLE_PIN 34
// #define BRAKE_PIN A12

// // Create stepper objects
// AccelStepper stepper1(AccelStepper::DRIVER, STEP1_PIN, DIR1_PIN);
// AccelStepper stepper2(AccelStepper::DRIVER, STEP2_PIN, DIR2_PIN);
// AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// // Add these variables
// bool motorRunning = false;
// unsigned long motorStartTime = 0;
// unsigned long motorRunDuration = 0;
// const int as=1200;
// long stepsArray[as];
// int arrayIndex = 0;
// String command;
// long previousPosition = 0;
// long stepsSinceLastCommand = 0;
// int r=0;

// void setup() {
//   Serial2.begin(115200);
  
//   // Initialize enable pins
//   pinMode(ENABLE1_PIN, OUTPUT);
//   pinMode(ENABLE2_PIN, OUTPUT);
  
//   // Enable stepper motors (LOW = enabled for most drivers)
//   digitalWrite(ENABLE1_PIN, HIGH);
//   digitalWrite(ENABLE2_PIN, HIGH);
  
//   pinMode(ENABLE_PIN, OUTPUT);
//   pinMode(BRAKE_PIN, OUTPUT);
  
//   // Safety: Keep enable low and brakes on
//   digitalWrite(ENABLE_PIN, LOW);
//   delay(100);
//   digitalWrite(BRAKE_PIN, HIGH);

//   // Set maximum speed and acceleration
//   stepper1.setMaxSpeed(600);
//   stepper1.setAcceleration(1000);
  
//   stepper2.setMaxSpeed(600);
//   stepper2.setAcceleration(1000);
  
//   stepper.setMaxSpeed(1000);
//   stepper.setAcceleration(500);

//   previousPosition = stepper.currentPosition();

//   Serial2.println("Arduino Motor Controller Ready");
// }

// void loop() {
//   // Run the steppers
//   stepper1.run();
//   stepper2.run();
//   if (r==1)
//   {
//     stepper.runSpeed();
//   }
//   else
//   {
//     stepper.run();
//   }
  
//   // Handle Serial22 commands from ESP32
//   handleSerial2Commands();
// }

// void handleSerial2Commands() {
//   if (Serial2.available()) {
//     command = Serial2.readStringUntil('\n');
//     command.trim();
    
//     if (command.startsWith("REL:")) {
//       // Relative movement command
//       digitalWrite(ENABLE1_PIN, LOW);
//       digitalWrite(ENABLE2_PIN, LOW);
//       handleRelativeMovement(command);
//     }
//     else if (command == "FETCH") {
//       // Send steps array back to ESP32
//       // delay(200);
//       r=0;
//       sendStepsArray();
//     }
//     else if (command == "STOP") {
//       // Emergency stop
//       stepper1.stop();
//       stepper2.stop();
//       Serial2.println("Motors stopped");
//     }

//     else if (command == "DISABLE") {
//       // Disable motors
//       digitalWrite(ENABLE1_PIN, HIGH);
//       digitalWrite(ENABLE2_PIN, HIGH);
//       Serial2.println("Motors disabled");
//     }
//     else if (command == "ENABLE") {
//       // Enable motors
//       digitalWrite(ENABLE1_PIN, LOW);
//       digitalWrite(ENABLE2_PIN, LOW);
//       Serial2.println("Motors enabled");
//     }
//     else if (command == "s") {
//       printStepsArray();
//     }
//     else if (command.startsWith("FORCE_SPEED:")) {
//       int speed = command.substring(12).toInt();
//       //speed=speed*(-1);
//       r=1;
//       digitalWrite(BRAKE_PIN, LOW);
//       long currentPosition = stepper.currentPosition();
//       stepsSinceLastCommand = currentPosition - previousPosition;
//       previousPosition = currentPosition;

//       // Store in array if there's space
//       if (arrayIndex < as) {
//           stepsArray[arrayIndex] = stepsSinceLastCommand;
//           arrayIndex++;
//       } else {
//           // Array is full, optionally wrap around or handle overflow
//           // For wrap-around: arrayIndex = 0; stepsArray[arrayIndex] = stepsSinceLastCommand; arrayIndex++;
//           Serial2.println("Steps array full!");
//       }
//       // Set speed and start motor
//       if(speed==0)
//       {
//         digitalWrite(BRAKE_PIN, HIGH);
//         stepper.setSpeed(0);
//         return;
//       }
//       stepper.setSpeed(speed);
//      // Serial2.println(speed);
//     }
//     else
//     {
//       digitalWrite(ENABLE1_PIN, LOW);
//       digitalWrite(ENABLE2_PIN, LOW);
//       digitalWrite(BRAKE_PIN, HIGH);
//       stepper.setSpeed(0);
//     }
  
//   }
// }


// void handleRelativeMovement(String command) {
//   // Parse: REL:steps1,steps2,speed1,speed2
//   command.remove(0, 4); // Remove "REL:"
  
//   int firstComma = command.indexOf(',');
//   int secondComma = command.indexOf(',', firstComma + 1);
//   int thirdComma = command.indexOf(',', secondComma + 1);
//   int fourthComma = command.indexOf(',', thirdComma + 1);
//   int fifthComma = command.indexOf(',', fourthComma + 1);
  
//   if (firstComma == -1 || secondComma == -1 || thirdComma == -1 || 
//       fourthComma == -1 || fifthComma == -1) {
//     Serial2.println("Invalid REL command format");
//     return;
//   }
  
//   long steps1 = command.substring(0, firstComma).toInt();
//   long steps2 = command.substring(firstComma + 1, secondComma).toInt();
//   long steps3 = command.substring(secondComma + 1, thirdComma).toInt();
//   int speed1 = command.substring(thirdComma + 1, fourthComma).toInt();
//   int speed2 = command.substring(fourthComma + 1, fifthComma).toInt();
//   int speed3 = command.substring(fifthComma + 1).toInt();
//   // Set individual speeds
//   stepper1.setMaxSpeed(speed1);
//   stepper2.setMaxSpeed(-speed2);
//   if(steps3>0 || steps3<0)
//   {
//     digitalWrite(BRAKE_PIN, LOW);
//   }
//   else
//   {
//     digitalWrite(BRAKE_PIN, HIGH);
//   }
//   stepper.setMaxSpeed(speed3);
//   // Move relative to current positions
//   stepper1.move(steps1);
//   stepper2.move(-steps2);
//   stepper.move(steps3);
//   Serial2.print("REL Move - Motor1: ");
//   Serial2.print(steps1);
//   Serial2.print(" Speed1: ");
//   Serial2.print(speed1);
//   Serial2.print(" Motor2: ");
//   Serial2.print(steps2);
//   Serial2.print(" Speed2: ");
//   Serial2.println(speed2);
// }

// void printStepsArray() {
//   Serial2.println("=== Steps Array ===");
//   Serial2.print("Total entries: ");
//   Serial2.println(arrayIndex);
  
//   for (int i = 0; i < arrayIndex; i++) {
//     Serial2.print("Index ");
//     Serial2.print(i);
//     Serial2.print(": ");
//     Serial2.println(stepsArray[i]);
//   }
//   Serial2.println("=== End Array ===");
// }
// void sendStepsArray() {
//   Serial2.print("STEPS_ARRAY:");
//   Serial2.print(arrayIndex);
//   Serial2.print(":");
//   for (int i = 0; i < arrayIndex; i++) {
//     Serial2.print(stepsArray[i]);
//     if (i < arrayIndex - 1) {
//       Serial2.print(",");
//     }
//   }
//   arrayIndex=0;
//   Serial2.println();
// }

#include <AccelStepper.h>

// Stepper motor 1 pins (Joint 1)
#define STEP1_PIN 44
#define DIR1_PIN 42
#define ENABLE1_PIN 46

// Stepper motor 2 pins (Joint 2)
#define STEP2_PIN 38
#define DIR2_PIN 36
#define ENABLE2_PIN 40

#define DIR_PIN A14
#define STEP_PIN 32
#define ENABLE_PIN 34
#define BRAKE_PIN A12

// Create stepper objects
AccelStepper stepper1(AccelStepper::DRIVER, STEP1_PIN, DIR1_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEP2_PIN, DIR2_PIN);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Add these variables
bool motorRunning = false;
unsigned long motorStartTime = 0;
unsigned long motorRunDuration = 0;
const int as=1200;
long stepsArray[as];
int arrayIndex = 1; // Start from index 1 instead of 0
String command;
long previousPosition = 0;
long stepsSinceLastCommand = 0;
int r=0;

void setup() {
  Serial2.begin(9600);
  
  // Initialize enable pins
  pinMode(ENABLE1_PIN, OUTPUT);
  pinMode(ENABLE2_PIN, OUTPUT);
  
  // Enable stepper motors (LOW = enabled for most drivers)
  digitalWrite(ENABLE1_PIN, HIGH);
  digitalWrite(ENABLE2_PIN, HIGH);
  
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(BRAKE_PIN, OUTPUT);
  
  // Safety: Keep enable low and brakes on
  digitalWrite(ENABLE_PIN, LOW);
  delay(100);
  digitalWrite(BRAKE_PIN, HIGH);

  // Set maximum speed and acceleration
  stepper1.setMaxSpeed(2000);
  stepper1.setAcceleration(10000);
  
  stepper2.setMaxSpeed(2000);
  stepper2.setAcceleration(10000);
  
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(5000);

  previousPosition = stepper.currentPosition();

  // Initialize array with zero at index 0
  stepsArray[0] = 0;

  Serial2.println("Arduino Motor Controller Ready");
}

void loop() {
  // Run the steppers
  stepper1.run();
  stepper2.run();
  if (r==1)
  {
    stepper.runSpeed();
  }
  else
  {
    stepper.run();
  }
  
  // Handle Serial22 commands from ESP32
  handleSerial2Commands();
}

void handleSerial2Commands() {
  // Non-blocking serial reading
  while (Serial2.available() && command.length() < 50) {
    char c = Serial2.read();
    if (c == '\n') {
      processCommand();
      command = "";
      break;
    } else if (c != '\r') {
      command += c;
    }
  }
}

void processCommand() {
  command.trim();
  
  if (command.startsWith("REL:")) {
    // Relative movement command
    digitalWrite(ENABLE1_PIN, LOW);
    digitalWrite(ENABLE2_PIN, LOW);
    handleRelativeMovement(command);
  }
  else if (command == "FETCH") {
    // Send steps array back to ESP32
    r=0;
    calculateAndUpdateSum();
    sendStepsArray();
  }
  else if (command == "pm") {
    digitalWrite(ENABLE1_PIN, LOW);
    digitalWrite(ENABLE2_PIN, LOW);
  }
  else if (command == "STOP") {
    // Emergency stop
    stepper1.stop();
    stepper2.stop();
    Serial2.println("Motors stopped");
  }
  else if (command == "DISABLE") {
    // Disable motors
    digitalWrite(ENABLE1_PIN, HIGH);
    digitalWrite(ENABLE2_PIN, HIGH);
    Serial2.println("Motors disabled");
  }
  else if (command == "ENABLE") {
    // Enable motors
    digitalWrite(ENABLE1_PIN, LOW);
    digitalWrite(ENABLE2_PIN, LOW);
    Serial2.println("Motors enabled");
  }
  else if (command == "s") {
    printStepsArray();
  }
  else if (command.startsWith("FORCE_SPEED:")) {
    int speed = command.substring(12).toInt();
    r=1;
    digitalWrite(BRAKE_PIN, LOW);
    long currentPosition = stepper.currentPosition();
    stepsSinceLastCommand = currentPosition - previousPosition;
    previousPosition = currentPosition;

    // Store in array starting from index 1 if there's space
    if (arrayIndex < as) {
        stepsArray[arrayIndex] = stepsSinceLastCommand;
        arrayIndex++;
    } else {
        Serial2.println("Steps array full!");
    }
    
    // Set speed and start motor
    if(speed==0) {
      digitalWrite(BRAKE_PIN, HIGH);
      stepper.setSpeed(0);
      return;
    }
    stepper.setSpeed(speed);
  }
  else {
    digitalWrite(BRAKE_PIN, HIGH);
    stepper.setSpeed(0);
  }
}

void handleRelativeMovement(String command) {
  // Parse: REL:steps1,steps2,speed1,speed2
  command.remove(0, 4); // Remove "REL:"
  
  int firstComma = command.indexOf(',');
  int secondComma = command.indexOf(',', firstComma + 1);
  int thirdComma = command.indexOf(',', secondComma + 1);
  int fourthComma = command.indexOf(',', thirdComma + 1);
  int fifthComma = command.indexOf(',', fourthComma + 1);
  
  if (firstComma == -1 || secondComma == -1 || thirdComma == -1 || 
      fourthComma == -1 || fifthComma == -1) {
    Serial2.println("Invalid REL command format");
    return;
  }
  
  long steps1 = command.substring(0, firstComma).toInt();
  long steps2 = command.substring(firstComma + 1, secondComma).toInt();
  long steps3 = command.substring(secondComma + 1, thirdComma).toInt();
  int speed1 = command.substring(thirdComma + 1, fourthComma).toInt();
  int speed2 = command.substring(fourthComma + 1, fifthComma).toInt();
  int speed3 = command.substring(fifthComma + 1).toInt();
  // Set individual speeds
  int w=15;
  // stepper1.setAcceleration(speed1*w);
  // stepper2.setAcceleration(speed2*w);
  // stepper.setAcceleration(speed3*w);
  stepper1.setMaxSpeed(speed1);
  stepper2.setMaxSpeed(-speed2);
  if(steps3>0 || steps3<0)
  {
    digitalWrite(BRAKE_PIN, LOW);
  }
  else
  {
    digitalWrite(BRAKE_PIN, HIGH);
  }
  stepper.setMaxSpeed(speed3);
  // Move relative to current positions
  stepper1.move(steps1);
  stepper2.move(-steps2);
  stepper.move(steps3);
  Serial2.print("REL Move - Motor1: ");
  Serial2.print(steps1);
  Serial2.print(" Speed1: ");
  Serial2.print(speed1);
  Serial2.print(" Motor2: ");
  Serial2.print(steps2);
  Serial2.print(" Speed3: ");
  Serial2.println(speed3);
  
}

void calculateAndUpdateSum() {
  long totalSum = 0;
  
  // Calculate sum of all values from index 1 to arrayIndex-1
  for (int i = 1; i < arrayIndex; i++) {
    totalSum += stepsArray[i];
  }
  
  // Reverse the sign and store at index 0
  stepsArray[0] = -totalSum;
}

void printStepsArray() {
  Serial2.println("=== Steps Array ===");
  Serial2.print("Total entries: ");
  Serial2.println(arrayIndex);
  
  for (int i = 0; i < arrayIndex; i++) {
    Serial2.print("Index ");
    Serial2.print(i);
    Serial2.print(": ");
    Serial2.println(stepsArray[i]);
  }
  Serial2.println("=== End Array ===");
}

void sendStepsArray() {
  Serial2.print("STEPS_ARRAY:");
  Serial2.print(arrayIndex);
  Serial2.print(":");
  for (int i = 0; i < arrayIndex; i++) {
    Serial2.print(stepsArray[i]);
    if (i < arrayIndex - 1) {
      Serial2.print(",");
    }
  }
  // Reset array after sending - initialize index 0 with zero and start from index 1
  stepsArray[0] = 0;
  arrayIndex = 1;
  Serial2.println();
}
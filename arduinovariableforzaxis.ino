#include <AccelStepper.h>

// Stepper motor 1 pins (Joint 1)
#define STEP1_PIN 44
#define DIR1_PIN 42
#define ENABLE1_PIN 46

// Stepper motor 2 pins (Joint 2)
#define STEP2_PIN 38
#define DIR2_PIN 36
#define ENABLE2_PIN 40

// Stepper motor 3 pins (Third motor with force control)
#define STEP3_PIN 32
#define DIR3_PIN A14
#define ENABLE3_PIN 34
#define BRAKE_PIN A12

// Create AccelStepper objects
AccelStepper stepper1(AccelStepper::DRIVER, STEP1_PIN, DIR1_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEP2_PIN, DIR2_PIN);
AccelStepper stepper3(AccelStepper::DRIVER, STEP3_PIN, DIR3_PIN);

// Data arrays
int stepsData1[1000];    // Motor 1 steps
int stepsData2[1000];    // Motor 2 steps
int dataCount = 0;

// Motion control variables
bool motionActive = false;
int currentPointIndex = 0;
unsigned long lastMoveTime = 0;
const unsigned long MOVE_INTERVAL = 100; // 100ms between points

// Homing variables
bool homingActive = false;
int homingSteps1 = 0;
int homingSteps2 = 0;

// Third motor force control variables
bool thirdMotorActive = false;
float currentForceValue = 0.0;
int currentSpeed = 0;
const float MIN_FORCE = 1.0;   // Minimum force threshold
const float MAX_FORCE = 10.0;  // Maximum force for speed mapping
const int MIN_SPEED = 50;      // Minimum speed
const int MAX_SPEED = 600;     // Maximum speed

// Speed settings
const int NORMAL_MAX_SPEED = 2400;
const int NORMAL_ACCELERATION = 4800;
const int HOMING_MAX_SPEED = 500;
const int HOMING_ACCELERATION = 1000;

// Communication variables
String inputBuffer = "";
bool receivingArray1 = false;
bool receivingArray2 = false;
bool dataReady = false;

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  
  // Initialize enable pins
  pinMode(ENABLE1_PIN, OUTPUT);
  pinMode(ENABLE2_PIN, OUTPUT);
  pinMode(ENABLE3_PIN, OUTPUT);
  pinMode(BRAKE_PIN, OUTPUT);
  
  // Enable steppers (LOW = enabled for most drivers)
  digitalWrite(ENABLE1_PIN, HIGH);
  digitalWrite(ENABLE2_PIN, HIGH);
  digitalWrite(ENABLE3_PIN, HIGH);
  
  // Set brake engaged (HIGH = brake engaged for safety)
  digitalWrite(BRAKE_PIN, HIGH);
  
  // Configure stepper parameters (start with normal speeds)
  setNormalSpeed();
  
  // Initialize third motor
  stepper3.setMaxSpeed(MAX_SPEED);
  stepper3.setAcceleration(1000);
  
  Serial.println("Arduino Mega with Third Motor - Waiting for ESP32 data...");
}

void loop() {
  // Read data from ESP32
  while (Serial2.available()) {
    char c = Serial2.read();
    
    if (c == '\n') {
      processLine(inputBuffer);
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }
  
  // Handle any manual serial commands (non-blocking)
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') { // Simple stop command
      stopMotion();
      stopHoming();
      stopThirdMotor();
    }
  }
  
  // Handle motion execution
  updateMotion();
  updateHoming();
  updateThirdMotor();
}

void processLine(String line) {
  line.trim();
  
  // Check for force control commands
  if (line == "FORCE_START") {
    startThirdMotor();
    return;
  }
  
  if (line == "FORCE_STOP") {
    stopThirdMotor();
    return;
  }
  
  if (line.startsWith("FORCE_VALUE ")) {
    float forceValue = line.substring(12).toFloat();
    updateThirdMotorSpeed(forceValue);
    return;
  }
  
  // Check for homing command
  if (line.startsWith("HOMING ")) {
    parseHomingCommand(line);
    return;
  }
  
  if (line == "Steps Array 1 (Motor 1):") {
    receivingArray1 = true;
    return;
  }
  
  if (line == "Steps Array 2 (Motor 2):") {
    receivingArray1 = false;
    receivingArray2 = true;
    return;
  }
  
  if (line == "PRINT_DATA") {
    if (dataReady) {
      setNormalSpeed(); // Ensure normal speed settings before motion
      startMotion();
    } else {
      Serial.println("No data available to execute");
    }
    return;
  }
  
  if(line == "rec") {
    digitalWrite(ENABLE1_PIN, HIGH);
    digitalWrite(ENABLE2_PIN, HIGH);
  }
  
  if(line == "stop") {
    digitalWrite(ENABLE1_PIN, LOW);
    digitalWrite(ENABLE2_PIN, LOW);
    stopThirdMotor(); // Also stop third motor when recording stops
  }

  if (receivingArray1) {
    parseArray(line, stepsData1);
    receivingArray1 = false;
  } else if (receivingArray2) {
    parseArray(line, stepsData2);
    receivingArray2 = false;
    dataReady = true;
    Serial.println("Data received! Ready for homing and motion execution");
  }
}

// Third motor control functions
void startThirdMotor() {
  if (!thirdMotorActive) {
    thirdMotorActive = true;
    digitalWrite(BRAKE_PIN, LOW);  // Release brake
    digitalWrite(ENABLE3_PIN, LOW); // Enable motor (assuming LOW = enabled)
    Serial.println("Third motor activated - brake released");
  }
}

void stopThirdMotor() {
  if (thirdMotorActive) {
    thirdMotorActive = false;
    stepper3.stop();
    digitalWrite(BRAKE_PIN, HIGH);  // Engage brake
    digitalWrite(ENABLE3_PIN, HIGH); // Disable motor
    Serial.println("Third motor stopped - brake engaged");
  }
}

void updateThirdMotorSpeed(float forceValue) {
  if (!thirdMotorActive) return;
  
  currentForceValue = forceValue;
  int speed = mapForceToSpeed(abs(forceValue));
  
  // Set direction based on force sign
  if (forceValue > 0) {
    stepper3.setSpeed(speed);
  } else {
    stepper3.setSpeed(-speed);
  }
  
  currentSpeed = speed;
}

void updateThirdMotor() {
  if (thirdMotorActive) {
    stepper3.runSpeed();
  }
}

int mapForceToSpeed(float forceValue) {
  // Clamp force value to our defined range
  if (forceValue < MIN_FORCE) forceValue = MIN_FORCE;
  if (forceValue > MAX_FORCE) forceValue = MAX_FORCE;
  
  // Map force range to speed range
  int speed = map(forceValue * 100, MIN_FORCE * 100, MAX_FORCE * 100, MIN_SPEED, MAX_SPEED);
  
  Serial.print("Force: ");
  Serial.print(forceValue);
  Serial.print(" -> Speed: ");
  Serial.println(speed);
  
  return speed;
}

void parseHomingCommand(String command) {
  // Format: "HOMING steps1 steps2"
  int firstSpace = command.indexOf(' ', 7); // Skip "HOMING "
  int secondSpace = command.indexOf(' ', firstSpace + 1);
  
  if (firstSpace != -1 && secondSpace != -1) {
    homingSteps1 = command.substring(7, firstSpace).toInt();
    homingSteps2 = command.substring(firstSpace + 1, secondSpace).toInt();
  } else if (firstSpace != -1) {
    homingSteps1 = command.substring(7, firstSpace).toInt();
    homingSteps2 = command.substring(firstSpace + 1).toInt();
  }
  
  Serial.print("Homing command received - M1: ");
  Serial.print(homingSteps1);
  Serial.print(" steps, M2: ");
  Serial.print(homingSteps2);
  Serial.println(" steps");
  
  startHoming();
}

void startHoming() {
  if (homingActive) {
    Serial.println("Already homing");
    return;
  }
  
  Serial.println("Starting homing movement...");
  
  // Set homing speed parameters
  setHomingSpeed();
  
  // Calculate target positions for homing
  long targetPos1 = stepper1.currentPosition() + homingSteps1;
  long targetPos2 = stepper2.currentPosition() + homingSteps2;
  
  // Set target positions
  stepper1.moveTo(targetPos1);
  stepper2.moveTo(targetPos2);
  
  homingActive = true;
  
  Serial.print("Moving to homing positions - M1: ");
  Serial.print(targetPos1);
  Serial.print(", M2: ");
  Serial.println(targetPos2);
}

void updateHoming() {
  if (!homingActive) return;
  
  // Run both steppers
  stepper1.run();
  stepper2.run();
  
  // Check if both steppers have reached their targets
  if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0) {
    Serial.println("Homing movement completed");
    homingActive = false;
    
    // Send completion signal to ESP32
    Serial2.println("HOMING_COMPLETE");
  }
}

void stopHoming() {
  if (homingActive) {
    homingActive = false;
    stepper1.stop();
    stepper2.stop();
    Serial.println("Homing stopped");
  }
}

void setHomingSpeed() {
  stepper1.setMaxSpeed(HOMING_MAX_SPEED);
  stepper1.setAcceleration(HOMING_ACCELERATION);
  stepper2.setMaxSpeed(HOMING_MAX_SPEED);
  stepper2.setAcceleration(HOMING_ACCELERATION);
  
  Serial.println("Speed set to homing parameters");
}

void setNormalSpeed() {
  stepper1.setMaxSpeed(NORMAL_MAX_SPEED);
  stepper1.setAcceleration(NORMAL_ACCELERATION);
  stepper2.setMaxSpeed(NORMAL_MAX_SPEED);
  stepper2.setAcceleration(NORMAL_ACCELERATION);
  
  Serial.println("Speed set to normal parameters");
}

void parseArray(String data, int* array) {
  dataCount = 0;
  int start = 0;
  int comma = 0;
  
  while (comma != -1 && dataCount < 1000) {
    comma = data.indexOf(',', start);
    
    String value;
    if (comma != -1) {
      value = data.substring(start, comma);
    } else {
      value = data.substring(start);
    }
    
    value.trim();
    if (value.length() > 0) {
      array[dataCount] = value.toInt();
      dataCount++;
    }
    
    start = comma + 1;
  }
}

void startMotion() {
  if (dataCount > 0) {
    Serial.println("Starting motion sequence...");
    
    // Set initial positions (first data point)
    stepper1.setCurrentPosition(stepsData1[0]);
    stepper2.setCurrentPosition(stepsData2[0]);
    
    // Start from second point if available
    currentPointIndex = 1;
    motionActive = true;
    lastMoveTime = millis();
    
    if (dataCount > 1) {
      // Move to second point
      stepper1.moveTo(stepsData1[1]);
      stepper2.moveTo(stepsData2[1]);
      Serial.print("Moving to point ");
      Serial.print(currentPointIndex + 1);
      Serial.print(": Motor1=");
      Serial.print(stepsData1[1]);
      Serial.print(", Motor2=");
      Serial.println(stepsData2[1]);
    } else {
      Serial.println("Only one data point - motion complete");
      motionActive = false;
    }
  }
}

void updateMotion() {
  if (!motionActive) return;
  
  // Always run the steppers for smooth motion
  stepper1.run();
  stepper2.run();
  
  // Check if it's time to move to next point
  if (millis() - lastMoveTime >= MOVE_INTERVAL) {
    currentPointIndex++;
    
    if (currentPointIndex < dataCount) {
      // Move to next point
      stepper1.moveTo(stepsData1[currentPointIndex]);
      stepper2.moveTo(stepsData2[currentPointIndex]);
      
      Serial.print("Moving to point ");
      Serial.print(currentPointIndex + 1);
      Serial.print(": Motor1=");
      Serial.print(stepsData1[currentPointIndex]);
      Serial.print(", Motor2=");
      Serial.println(stepsData2[currentPointIndex]);
      
      lastMoveTime = millis();
    } else {
      // Motion sequence complete
      Serial.println("Motion sequence completed!");
      motionActive = false;
    }
  }
}

void stopMotion() {
  motionActive = false;
  stepper1.stop();
  stepper2.stop();
  Serial.println("Motion stopped");
}

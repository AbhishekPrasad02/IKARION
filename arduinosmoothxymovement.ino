#include <AccelStepper.h>

// Stepper motor 1 pins (Joint 1)
#define STEP1_PIN 44
#define DIR1_PIN 42
#define ENABLE1_PIN 46

// Stepper motor 2 pins (Joint 2)
#define STEP2_PIN 38
#define DIR2_PIN 36
#define ENABLE2_PIN 40

// Create AccelStepper objects
AccelStepper stepper1(AccelStepper::DRIVER, STEP1_PIN, DIR1_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEP2_PIN, DIR2_PIN);

// Data arrays
int stepsData1[1000];    // Motor 1 steps
int stepsData2[1000];    // Motor 2 steps
int dataCount = 0;

// Motion control variables
bool motionActive = false;
int currentPointIndex = 0;
unsigned long lastMoveTime = 0;
const unsigned long MOVE_INTERVAL = 100; // 100ms between points

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
  
  // Enable steppers (LOW = enabled for most drivers)
  digitalWrite(ENABLE1_PIN, LOW);
  digitalWrite(ENABLE2_PIN, LOW);
  
  // Configure stepper parameters
  stepper1.setMaxSpeed(4800);      // Adjust based on your motor specs
  stepper1.setAcceleration(1000);   // Smooth acceleration
  stepper2.setMaxSpeed(4800);      
  stepper2.setAcceleration(1000);
  
  Serial.println("Arduino Mega - Waiting for ESP32 data...");
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
    }
  }
  
  // Handle motion execution
  updateMotion();
}

void processLine(String line) {
  line.trim();
  
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
      startMotion();
    } else {
      Serial.println("No data available to execute");
    }
    return;
  }

  if (receivingArray1) {
    parseArray(line, stepsData1);
    receivingArray1 = false;
  } else if (receivingArray2) {
    parseArray(line, stepsData2);
    receivingArray2 = false;
    dataReady = true;
    Serial.println("Data received! Type 'print' to start motion");
  }
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

void printStepsData() {
  Serial.println("\n=== RECORDED STEPS DATA ===");
  Serial.print("Data points: ");
  Serial.println(dataCount);
  Serial.println("Step Motor1, Motor2");
  Serial.println("-------------------");
  
  for (int i = 0; i < dataCount; i++) {
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(stepsData1[i]);
    Serial.print(", ");
    Serial.println(stepsData2[i]);
  }
  
  Serial.println("=== END OF DATA ===\n");
}

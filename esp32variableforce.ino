#include <Wire.h>
#include <AS5600.h>
#include <HX711.h>

// TCA9548A I2C Multiplexer
#define TCA_ADDRESS 0x70
#define ENCODER1_CHANNEL 7  // First joint
#define ENCODER2_CHANNEL 6  // Second joint

// HX711 Force Sensor
#define FORCE_DATA_PIN 19
#define FORCE_CLOCK_PIN 23

AS5600 encoder1, encoder2;
HX711 forceSensor;

// Force sensor variables
float currentForce = 0.0;
float forceThreshold = 1.0;  // Adjust this threshold as needed
bool forceControlActive = false;
unsigned long lastForceRead = 0;
const unsigned long FORCE_READ_INTERVAL = 100; // 50Hz sampling rate

// Recording variables
float angleData1[1000];  // Store angle readings for motor 1
float angleData2[1000];  // Store angle readings for motor 2
int stepsData1[1000];    // Store converted steps for motor 1
int stepsData2[1000];    // Store converted steps for motor 2
int dataIndex = 0;
bool isRecording = false;

// Homing variables
float homeAngle1 = 0.0;  // Home position for motor 1
float homeAngle2 = 0.0;  // Home position for motor 2
bool isHoming = false;
bool waitingForHomingComplete = false;
const float HOMING_TOLERANCE = 2.0; // Degrees tolerance for homing accuracy

unsigned long lastRecordTime = 0;
const unsigned long RECORD_INTERVAL = 100; // 100ms for 10Hz sampling rate

// Steps per revolution for your stepper motors
const int STEPS_PER_REV = 4800; // Adjust based on your stepper motor

void setup() {
  Serial.begin(9600);   // For debugging/commands
  Serial2.begin(9600);  // For communication with Arduino Mega
  Wire.begin();
  
  // Initialize force sensor
  forceSensor.begin(FORCE_DATA_PIN, FORCE_CLOCK_PIN);
  forceSensor.set_scale(700);  // Set calibration factor (you may need to calibrate)
  forceSensor.tare();       // Zero the scale
  
  // Initialize encoders
  selectTCAChannel(ENCODER1_CHANNEL);
  encoder1.begin();
  delay(10);
  
  selectTCAChannel(ENCODER2_CHANNEL);
  encoder2.begin();
  delay(10);
  
  Serial.println("SCARA Recording System with Force Control Ready");
  Serial.println("Commands: rec, stop, print");
  Serial2.println("ESP32 SCARA System with Force Control Connected");
}

void loop() {
  handleSerialCommands();
  handleRecording();
  handleHomingResponse();
  handleForceControl();
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Convert to lowercase for comparison
    String lowerCommand = command;
    lowerCommand.toLowerCase();
    
    if (lowerCommand == "rec") {
      Serial2.println("rec");
      startRecording();
    } else if (lowerCommand == "stop") {
      Serial2.println("stop");
      stopRecording();
    } else if (lowerCommand == "print") {
      if (dataIndex > 0) {
        startHomingSequence();
      } else {
        Serial.println("No recorded data to print. Record first using 'rec' command.");
      }
    } else {
      Serial.println("Unknown command. Available: rec, stop, print");
    }
  }
}

void handleForceControl() {
  if (!isRecording) return; // Only active during recording
  
  unsigned long currentTime = millis();
  if ((currentTime - lastForceRead) >= FORCE_READ_INTERVAL) {
    // Read force sensor
    if (forceSensor.is_ready()) {
      currentForce = forceSensor.get_units(1); // Average of 1 reading for speed
      
      // Check if force exceeds threshold
      if (abs(currentForce) > forceThreshold) {
        if (!forceControlActive) {
          forceControlActive = true;
          Serial2.println("FORCE_START"); // Signal Arduino to release brake
        }
        
        // Send force value to Arduino for speed control
        Serial2.print("FORCE_VALUE ");
        Serial2.println(currentForce);
        
        Serial.print("Force detected: ");
        Serial.println(currentForce);
      } else {
        if (forceControlActive) {
          forceControlActive = false;
          Serial2.println("FORCE_STOP"); // Signal Arduino to engage brake and stop motor
          Serial.println("Force below threshold - stopping third motor");
        }
      }
    }
    
    lastForceRead = currentTime;
  }
}

void handleHomingResponse() {
  if (Serial2.available()) {
    String response = Serial2.readStringUntil('\n');
    response.trim();
    
    if (response == "HOMING_COMPLETE" && waitingForHomingComplete) {
      waitingForHomingComplete = false;
      
      // Check if we've actually reached the home position
      float currentAngle1 = readEncoder1();
      float currentAngle2 = readEncoder2();
      
      float error1 = abs(currentAngle1 - homeAngle1);
      float error2 = abs(currentAngle2 - homeAngle2);
      
      Serial.print("Homing check - Current: M1=");
      Serial.print(currentAngle1);
      Serial.print("°, M2=");
      Serial.print(currentAngle2);
      Serial.print("° | Target: M1=");
      Serial.print(homeAngle1);
      Serial.print("°, M2=");
      Serial.print(homeAngle2);
      Serial.print("° | Error: M1=");
      Serial.print(error1);
      Serial.print("°, M2=");
      Serial.print(error2);
      Serial.println("°");
      
      if (error1 <= HOMING_TOLERANCE && error2 <= HOMING_TOLERANCE) {
        Serial.println("Homing successful! Starting recorded motion...");
        isHoming = false;
        convertAndSendSteps();
        // Send the motion data to Arduino
        Serial2.println("PRINT_DATA");
      } else {
        Serial.println("Homing accuracy not achieved, retrying...");
        performHoming(currentAngle1, currentAngle2);
      }
    }
  }
}

void startRecording() {
  if (isRecording) {
    Serial.println("Already recording");
    return;
  }
  
  isRecording = true;
  dataIndex = 0;
  lastRecordTime = millis();
  forceControlActive = false;
  
  // Record the home position (starting position)
  homeAngle1 = readEncoder1();
  homeAngle2 = readEncoder2();
  
  Serial.println("Recording started...");
  Serial.print("Home position set - M1: ");
  Serial.print(homeAngle1);
  Serial.print("°, M2: ");
  Serial.print(homeAngle2);
  Serial.println("°");
  Serial.println("Force control activated for third motor");
  
  Serial2.println("Recording started...");
}

void handleRecording() {
  if (isRecording && dataIndex < 1000) {
    unsigned long currentTime = millis();
    if (currentTime - lastRecordTime >= RECORD_INTERVAL) {
      recordDataPoint();
      lastRecordTime = currentTime;
    }
  }
}

void recordDataPoint() {
  if (dataIndex >= 1000) {
    Serial.println("Recording buffer full, stopping recording");
    stopRecording();
    return;
  }
  
  float currentAngle1 = readEncoder1();
  float currentAngle2 = readEncoder2();
  
  angleData1[dataIndex] = currentAngle1;
  angleData2[dataIndex] = currentAngle2;
  
  dataIndex++;
}

void stopRecording() {
  if (!isRecording) {
    Serial.println("Not recording");
    return;
  }
  
  isRecording = false;
  
  // Stop force control and third motor
  if (forceControlActive) {
    Serial2.println("FORCE_STOP");
    forceControlActive = false;
  }
  
  Serial.print("Recording stopped. Recorded ");
  Serial.print(dataIndex);
  Serial.println(" data points.");
  Serial.println("Force control deactivated");
  
  Serial2.print("Recording stopped. Recorded ");
  Serial2.print(dataIndex);
  Serial2.println(" data points.");
  
  // Convert angles to steps for later use
  convertAnglesToSteps();
}

void convertAnglesToSteps() {
  Serial.println("Converting angles to steps...");
  
  // Convert all recorded angles to steps and store in arrays
  for (int i = 0; i < dataIndex; i++) {
    stepsData1[i] = (int)(angleData1[i] * (STEPS_PER_REV / 360.0));
    stepsData2[i] = (int)(angleData2[i] * (STEPS_PER_REV / 360.0));
  }
  
  Serial.println("Conversion complete. Use 'print' to execute motion.");
}

void startHomingSequence() {
  if (isHoming) {
    Serial.println("Already homing");
    return;
  }
  
  Serial.println("Starting homing sequence...");
  
  // Get current position
  float currentAngle1 = readEncoder1();
  float currentAngle2 = readEncoder2();
  
  Serial.print("Current position - M1: ");
  Serial.print(currentAngle1);
  Serial.print("°, M2: ");
  Serial.print(currentAngle2);
  Serial.print("° | Target home - M1: ");
  Serial.print(homeAngle1);
  Serial.print("°, M2: ");
  Serial.print(homeAngle2);
  Serial.println("°");
  
  isHoming = true;
  performHoming(currentAngle1, currentAngle2);
}

void performHoming(float currentAngle1, float currentAngle2) {
  // Calculate angle differences
  float angleDiff1 = homeAngle1 - currentAngle1;
  float angleDiff2 = homeAngle2 - currentAngle2;
  
  // Handle angle wrapping (shortest path)
  if (angleDiff1 > 180) angleDiff1 -= 360;
  if (angleDiff1 < -180) angleDiff1 += 360;
  if (angleDiff2 > 180) angleDiff2 -= 360;
  if (angleDiff2 < -180) angleDiff2 += 360;
  
  // Convert angle differences to steps
  int steps1 = (int)(angleDiff1 * (STEPS_PER_REV / 360.0));
  int steps2 = (int)(angleDiff2 * (STEPS_PER_REV / 360.0));
  
  Serial.print("Homing steps required - M1: ");
  Serial.print(steps1);
  Serial.print(", M2: ");
  Serial.println(steps2);
  
  // Send homing commands to Arduino
  Serial2.print("HOMING ");
  Serial2.print(steps1);
  Serial2.print(" ");
  Serial2.println(steps2);
  
  waitingForHomingComplete = true;
}

void selectTCAChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delayMicroseconds(100); // Minimal delay for channel switching
}

float readEncoder1() {
  selectTCAChannel(ENCODER1_CHANNEL);
  return encoder1.rawAngle() * 0.087890625;
}

float readEncoder2() {
  selectTCAChannel(ENCODER2_CHANNEL);
  return encoder2.rawAngle() * 0.087890625;
}

void convertAndSendSteps() {
  Serial.println("Converting angles to steps...");
  Serial2.println("Converting angles to steps...");
  
  // Send the steps arrays to Arduino via Serial2
  Serial2.println("Steps Array 1 (Motor 1):");
  for (int i = 0; i < dataIndex; i++) {
    Serial2.print(stepsData1[i]);
    if (i < dataIndex - 1) Serial2.print(", ");
  }
  Serial2.println();
  
  Serial2.println("Steps Array 2 (Motor 2):");
  for (int i = 0; i < dataIndex; i++) {
    Serial2.print(stepsData2[i]);
    if (i < dataIndex - 1) Serial2.print(", ");
  }
  Serial2.println();
  
  Serial2.println("Data transmission complete.");
}

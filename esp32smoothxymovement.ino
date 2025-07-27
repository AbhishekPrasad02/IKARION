#include <Wire.h>
#include <AS5600.h>

// TCA9548A I2C Multiplexer
#define TCA_ADDRESS 0x70
#define ENCODER1_CHANNEL 7  // First joint
#define ENCODER2_CHANNEL 6  // Second joint

AS5600 encoder1, encoder2;

// Recording variables
float angleData1[1000];  // Store angle readings for motor 1
float angleData2[1000];  // Store angle readings for motor 2
int stepsData1[1000];    // Store converted steps for motor 1
int stepsData2[1000];    // Store converted steps for motor 2
int dataIndex = 0;
bool isRecording = false;

unsigned long lastRecordTime = 0;
const unsigned long RECORD_INTERVAL = 100; // 100ms for 10Hz sampling rate

// Steps per revolution for your stepper motors
const int STEPS_PER_REV = 4800; // Adjust based on your stepper motor

void setup() {
  Serial.begin(9600);   // For debugging/commands
  Serial2.begin(9600);  // For communication with Arduino Mega
  Wire.begin();
  
  // Initialize encoders
  selectTCAChannel(ENCODER1_CHANNEL);
  encoder1.begin();
  delay(10);
  
  selectTCAChannel(ENCODER2_CHANNEL);
  encoder2.begin();
  delay(10);
  
  Serial.println("SCARA Recording System Ready");
  Serial.println("Commands: rec, stop");
  Serial2.println("ESP32 SCARA System Connected");
}

void loop() {
  handleSerialCommands();
  handleRecording();
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Convert to lowercase for comparison
    String lowerCommand = command;
    lowerCommand.toLowerCase();
    
    if (lowerCommand == "rec") {
      startRecording();
    } else if (lowerCommand == "stop") {
      stopRecording();
    }
    else if(lowerCommand=="print")
    {
      Serial2.println("PRINT_DATA");
    }
     else {
      Serial.println("Unknown command. Available: rec, stop");
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
  Serial.println("Recording started...");
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
  Serial.print("Recording stopped. Recorded ");
  Serial.print(dataIndex);
  Serial.println(" data points.");
  
  Serial2.print("Recording stopped. Recorded ");
  Serial2.print(dataIndex);
  Serial2.println(" data points.");
  
  // Convert angles to steps and send to Arduino
  convertAndSendSteps();
}

void convertAndSendSteps() {
  Serial.println("Converting angles to steps...");
  Serial2.println("Converting angles to steps...");
  
  // Convert all recorded angles to steps and store in arrays
  for (int i = 0; i < dataIndex; i++) {
    stepsData1[i] = (int)(angleData1[i] * (STEPS_PER_REV / 360.0));
    stepsData2[i] = (int)(angleData2[i] * (STEPS_PER_REV / 360.0));
  }
  
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
  
  // Also print to local serial for debugging
  Serial.println("Steps Array 1 (Motor 1):");
  for (int i = 0; i < dataIndex; i++) {
    Serial.print(stepsData1[i]);
    if (i < dataIndex - 1) Serial.print(", ");
  }
  Serial.println();
  
  Serial.println("Steps Array 2 (Motor 2):");
  for (int i = 0; i < dataIndex; i++) {
    Serial.print(stepsData2[i]);
    if (i < dataIndex - 1) Serial.print(", ");
  }
  Serial.println();
  
  Serial.println("Data sent to Arduino. Recording data cleared.");
  Serial2.println("Data transmission complete.");
  dataIndex = 0; // Clear the recording data
  
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



#include <Wire.h>
#include <AS5600.h>
#include <HX711.h>

// TCA9548A I2C Multiplexer
#define TCA_ADDRESS 0x70
#define ENCODER1_CHANNEL 7  // First joint
#define ENCODER2_CHANNEL 6  // Second joint

// HX711 Load Cell pins
#define HX711_CLK 23
#define HX711_DT1 18
#define HX711_DT2 19
#define HX711_DT3 5

AS5600 encoder1, encoder2;
HX711 loadcell1, loadcell2, loadcell3;

// Force control variables
float baseForceValue1=0;
float baseForceValue2=0;
float baseForceValue3= 0;
float forceThreshold = 100.0;

unsigned long recordingEndTime = 0;

// Recording variables
struct DataPoint {
  unsigned long timestamp;
  float angle1;
  float angle2;
  long steps3;  // Added steps3 to the struct
};

DataPoint recordedData[1000];  // Store up to 1000 data points
int dataIndex = 0;
bool isRecording = false;
bool isPlayback = false;
int playbackRepetitions = 1;

// External playback variables
DataPoint externalPlaybackData[1000];  // Store external playback data
int externalDataIndex = 0;
bool isExternalPlayback = false;
bool isExternalPlaybackPaused = false;  // NEW: Pause state for external playback
bool waitingForExternalData = false;

// Previous angles for relative movement
float prevAngle1 = 0;
float prevAngle2 = 0;

unsigned long lastRecordTime = 0;
unsigned long recordingStartTime = 0;

// Steps per revolution for your stepper motors
const int STEPS_PER_REV = 4800; // Adjust based on your stepper motor

const unsigned long RECORD_INTERVAL = 100; // 100ms

static int currentRep = 0;
static int currentPoint = 0;
static unsigned long startTime = 0;
static unsigned long pauseStartTime = 0;  // NEW: Track when pause started
static unsigned long totalPausedTime = 0; // NEW: Track total paused time

// Add these variables at the top with your existing force control variables
//float forceThreshold = 4.0; // Force detection threshold
unsigned long lastForceCommandTime = 0;
const unsigned long FORCE_COMMAND_INTERVAL = 100; // Check interval in ms

// Speed mapping constants
const float MIN_FORCE = 100.0; // Minimum force threshold
const float MAX_FORCE = 200.0; // Maximum force for speed mapping
const int MIN_SPEED = 50; // Minimum motor speed
const int MAX_SPEED = 600; // Maximum motor speed

// Removed the separate receivedStepsArray since we're now storing in DataPoint struct
int receivedArraySize = 0;

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
  
  Wire.begin();
  
  // Initialize encoders
  selectTCAChannel(ENCODER1_CHANNEL);
  encoder1.begin();
  delay(10);
  
  selectTCAChannel(ENCODER2_CHANNEL);
  encoder2.begin();
  delay(10);
  
  // Initialize HX711 load cells
  loadcell1.begin(HX711_DT1, HX711_CLK);
  loadcell2.begin(HX711_DT2, HX711_CLK);
  loadcell3.begin(HX711_DT3, HX711_CLK);
  
  loadcell1.set_scale(700);
  loadcell2.set_scale(700);
  loadcell3.set_scale(700);
  
  Serial.println("Initializing load cells...");
  loadcell1.tare();
  loadcell2.tare();
  loadcell3.tare();
  
  calibrateBaseForce();
  
  // Get initial encoder positions
  prevAngle1 = readEncoder1();
  prevAngle2 = readEncoder2();
  
  Serial.println("SCARA Learning System Ready");
  Serial.println("Commands: rec, stop, delete, start <repetitions>, play{data}");
  Serial.println("External Playback Controls: pauseplay, resumeplay, stopplay");
}

int mapForceToSpeed(float forceValue) {
  // Clamp force value to our defined range
  if (forceValue < MIN_FORCE) forceValue = MIN_FORCE;
  if (forceValue > MAX_FORCE) forceValue = MAX_FORCE;
  // Map force range to speed range
  int speed = map(forceValue, MIN_FORCE, MAX_FORCE, MIN_SPEED, MAX_SPEED);
  Serial.print("force");
  Serial.println(forceValue);
  return speed;
}

void loop() {
  handleSerialCommands();
  handleRecording();
  handlePlayback();
  handleExternalPlayback();
  handleArduinoResponse();
  // if (loadcell2.is_ready())
  // {
  //   Serial.print("force");
  //   Serial.println(loadcell2.get_units());
  // }
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
    } else if (lowerCommand == "s") {
      Serial2.println("s");
    } else if (lowerCommand == "delete") {
      deleteRecording();
    } else if(lowerCommand == "PLAYBACK_STOP ") {
      Serial2.println("DISABLE");
    } 
    // NEW: External playback control commands
    else if (lowerCommand == "pauseplay") {
      pauseExternalPlayback();
    } else if (lowerCommand == "resumeplay") {
      resumeExternalPlayback();
    } else if (lowerCommand == "stopplay") {
      stopExternalPlayback();
    } else if (lowerCommand.startsWith("start")) {
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex != -1) {
        int reps = command.substring(spaceIndex + 1).toInt();
        startPlayback(reps);
      } else {
        startPlayback(1);
      }
    } 
    // UPDATED: Handle new format play<mode><resistance>data
    else if (command.startsWith("play<")) {
      Serial2.println("pm");
      handleExternalPlaybackCommand(command);
    } else {
      Serial.print(command);
      Serial.println("Unknown command. Available: rec, stop, delete, start <repetitions>, play<mode><resistance>data, pauseplay, resumeplay, stopplay");
    }
  }
}

void handleExternalPlaybackCommand(String command) {
  if (isRecording || isPlayback) {
    Serial.println("Cannot start external playback during recording or internal playback");
    return;
  }
  
  // Parse the external data with new format
  if (parseExternalPlaybackData(command)) {
    startExternalPlayback();
  } else {
    Serial.println("Error parsing external playback data");
  }
}

// Global variables to store parsed parameters
String playbackMode = "";
float level = 0;

bool parseExternalPlaybackData(String command) {
  // Check if command starts with "play<"
  if (!command.startsWith("play<")) {
    Serial.println("Invalid format: Expected play<mode><resistance>data");
    return false;
  }
  
  // Find the first closing bracket for playback mode
  int firstClose = command.indexOf('>', 5); // Start searching after "play<"
  if (firstClose == -1) {
    Serial.println("Invalid format: Missing closing bracket for playback mode");
    return false;
  }
  
  // Extract playback mode
  playbackMode = command.substring(5, firstClose); // Between "play<" and first ">"
  
  
  // Find the second opening bracket for resistance level
  int secondOpen = firstClose + 1;
  if (secondOpen >= command.length() || command.charAt(secondOpen) != '<') {
    Serial.println("Invalid format: Missing opening bracket for resistance level");
    return false;
  }
  
  // Find the second closing bracket for resistance level
  int secondClose = command.indexOf('>', secondOpen);
  if (secondClose == -1) {
    Serial.println("Invalid format: Missing closing bracket for resistance level");
    return false;
  }
  
  // Extract resistance level
  String resistanceStr = command.substring(secondOpen + 1, secondClose);
  level = resistanceStr.toFloat()*100;
  
  // Extract the data part (everything after the second closing bracket)
  String dataString = command.substring(secondClose + 1);
  
  // Print parsed values for debugging
  Serial.print("Parsed - Mode: ");
  Serial.print(playbackMode);
  Serial.print(", Resistance: ");
  Serial.print(level);
  Serial.print(", Data length: ");
  Serial.println(dataString.length());
  
  // Reset external data index
  externalDataIndex = 0;
  int startPos = 0;
  
  // Parse the data points (same logic as before)
  while (startPos < dataString.length() && externalDataIndex < 1000) {
    // Find the opening parenthesis
    int openParen = dataString.indexOf('(', startPos);
    if (openParen == -1) break;
    
    // Find the closing parenthesis
    int closeParen = dataString.indexOf(')', openParen);
    if (closeParen == -1) break;
    
    // Extract the data between parentheses
    String dataPoint = dataString.substring(openParen + 1, closeParen);
    
    // Parse angle1, angle2, steps3
    int comma1 = dataPoint.indexOf(',');
    int comma2 = dataPoint.indexOf(',', comma1 + 1);
    
    if (comma1 == -1 || comma2 == -1) {
      startPos = closeParen + 1;
      continue;
    }
    
    float angle1 = dataPoint.substring(0, comma1).toFloat();
    float angle2 = dataPoint.substring(comma1 + 1, comma2).toFloat();
    long steps3 = dataPoint.substring(comma2 + 1).toInt();
    
    // Store in external playback array
    externalPlaybackData[externalDataIndex].angle1 = angle1;
    externalPlaybackData[externalDataIndex].angle2 = angle2;
    externalPlaybackData[externalDataIndex].steps3 = steps3;
    externalPlaybackData[externalDataIndex].timestamp = 100; // Fixed interval
    
    externalDataIndex++;
    startPos = closeParen + 1;
  }
  
  Serial.print("Parsed ");
  Serial.print(externalDataIndex);
  Serial.println(" external data points");
  
  return externalDataIndex > 0;
}

// NEW: Pause external playback
void pauseExternalPlayback() {
  if (!isExternalPlayback) {
    Serial.println("No external playback is currently running");
    return;
  }
  
  if (isExternalPlaybackPaused) {
    Serial.println("External playback is already paused");
    return;
  }
  
  isExternalPlaybackPaused = true;
  pauseStartTime = millis();
  Serial2.println("s"); // Send stop command to Arduino to halt motors
  Serial.println("External playback paused");
}

// NEW: Resume external playback
void resumeExternalPlayback() {
  if (!isExternalPlayback) {
    Serial.println("No external playback is currently running");
    return;
  }
  
  if (!isExternalPlaybackPaused) {
    Serial.println("External playback is not paused");
    return;
  }
  
  // Calculate how long we were paused and add to total paused time
  totalPausedTime += (millis() - pauseStartTime);
  isExternalPlaybackPaused = false;
  
  Serial.println("External playback resumed");
}

// NEW: Stop external playback completely
void stopExternalPlayback() {
  if (!isExternalPlayback) {
    Serial.println("No external playback is currently running");
    return;
  }
  
  // Stop the playback
  isExternalPlayback = false;
  isExternalPlaybackPaused = false;
  currentPoint = 0;
  totalPausedTime = 0;
  
  // Send stop command to Arduino
  Serial2.println("s");
  Serial2.println("DISABLE");
  
  Serial.println("External playback stopped");
}


void startExternalPlayback() {
  if (externalDataIndex == 0) {
    Serial.println("No external data to playback");
    return;
  }
  
  isExternalPlayback = true;
  isExternalPlaybackPaused = false;  // NEW: Reset pause state
  currentPoint = 0;
  totalPausedTime = 0;  // NEW: Reset paused time
  Serial.print("Starting external playback with ");
  Serial.print(externalDataIndex);
  Serial.println(" data points");
}

void handleExternalPlayback() {
  if (!isExternalPlayback || externalDataIndex == 0) return;
  
  // NEW: Skip execution if paused
  if (isExternalPlaybackPaused) {
    return;
  }
  
  if (currentPoint == 0) {
    startTime = millis();
    sendExternalMovementCommand(currentPoint);
    currentPoint++;
    return;
  }
  
  if (currentPoint < (externalDataIndex-10)) 
  {
    // NEW: Adjust target time by subtracting total paused time
    unsigned long adjustedCurrentTime = millis() - totalPausedTime;
    unsigned long targetTime = adjustedCurrentTime - startTime;
    if((playbackMode=="Assistive" || playbackMode=="Resistive") && targetTime >= 100)
      {
      float targetangle1 = externalPlaybackData[currentPoint].angle1;
      float targetangle2 = externalPlaybackData[currentPoint].angle2;
      long steps3 = externalPlaybackData[currentPoint].steps3;
      float absVal1 = abs(targetangle1)*13.33;
      float absVal2 = abs(targetangle2)*13.33;
      long absVal3 = abs(steps3);
      float tcurrentForce;
      float tzcurrentForce;
      float netforce;
      float znetforce; 
      //Serial.println(level);
      tcurrentForce = loadcell3.get_units();
      tzcurrentForce = loadcell2.get_units();
      
      if (targetTime >= 100) 
      { 
        if((steps3<0 && tzcurrentForce>10) || (steps3>0 && tzcurrentForce<-10))
        {
          Serial.println("inside");
          startTime = adjustedCurrentTime;
          sendExternalMovementCommand(currentPoint);
          currentPoint++;
        }
        // else if(steps3==0)
        // {
        //   startTime = adjustedCurrentTime;
        //   //sendExternalMovementCommand(currentPoint);
        //   currentPoint++;
        // }
        // else if((targetangle1>0 && tcurrentForce>0) || (targetangle2>0 && tcurrentForce>0) || (targetangle1<0 && tcurrentForce<0) || (targetangle2<0 && tcurrentForce<0))
        // {
        //   startTime = adjustedCurrentTime;
        //   sendExternalMovementCommand(currentPoint);
        //   currentPoint++;  
        // }
        // else if((targetangle1<0 && tcurrentForce<0) || (targetangle2<0 && tcurrentForce<0))
        // {
        //   startTime = adjustedCurrentTime;
        //   sendExternalMovementCommand(currentPoint);
        //   currentPoint++;  
        // }
        // else if (targetangle1<0 &&targetangle1<0 &&tcurrentForce<0)
        // {
        //   startTime = adjustedCurrentTime;
        //   sendExternalMovementCommand(currentPoint);
        //   currentPoint++; 
        // }
        // else if (targetangle1>0 &&targetangle1>0 &&tcurrentForce>0)
        // {
        //   startTime = adjustedCurrentTime;
        //   sendExternalMovementCommand(currentPoint);
        //   currentPoint++; 
        // }
        else if (targetangle1>0 &&targetangle2<0 &&tcurrentForce<-10)
        {
          Serial.println("inside zero");
          startTime = adjustedCurrentTime;
          sendExternalMovementCommand(currentPoint);
          currentPoint++; 
        }
        else if(targetangle1<0 &&targetangle2>0 &&tcurrentForce>10)
        {
          Serial.println("inside one");
          startTime = adjustedCurrentTime;
          sendExternalMovementCommand(currentPoint);
          currentPoint++; 
        }
        else if (targetangle1>=0 && targetangle2>0 &&tcurrentForce<10)
        {
          Serial.println("inside second");
          startTime = adjustedCurrentTime;
          sendExternalMovementCommand(currentPoint);
          currentPoint++; 
        }
        else if(targetangle1<=-0 && targetangle2<0 &&tcurrentForce>-10)
        {
          Serial.println("inside third");
          startTime = adjustedCurrentTime;
          sendExternalMovementCommand(currentPoint);
          currentPoint++; 
        }
        else if (targetangle1>0 && targetangle2>=0 &&tcurrentForce<10)
        {
          Serial.println("inside second");
          startTime = adjustedCurrentTime;
          sendExternalMovementCommand(currentPoint);
          currentPoint++; 
        }
        else if(targetangle1<0 && targetangle2<=0 &&tcurrentForce>-10)
        {
          Serial.println("inside third");
          startTime = adjustedCurrentTime;
          sendExternalMovementCommand(currentPoint);
          currentPoint++; 
        }
        // else if (targetangle1>0 &&tcurrentForce>10)
        // {
        //   Serial.println("inside fourth");
        //   startTime = adjustedCurrentTime;
        //   sendExternalMovementCommand(currentPoint);
        //   currentPoint++; 
        // }
        // else if(targetangle1<0 &&tcurrentForce<-10)
        // {
        //   Serial.println("inside fifth");
        //   startTime = adjustedCurrentTime;
        //   sendExternalMovementCommand(currentPoint);
        //   currentPoint++; 
        // }
        else if((targetangle1<0.09&&targetangle1>-0.09)&&(targetangle2<0.09&&targetangle2>-0.09)&&steps3==0)
        { 
          Serial.println("inside sixth");
          startTime = adjustedCurrentTime;
          sendExternalMovementCommand(currentPoint);
          currentPoint++; 
        }

      }
      }
      else if (targetTime >= 100)
      {   
          startTime = adjustedCurrentTime;
          sendExternalMovementCommand(currentPoint);
          currentPoint++;  
      }
  } 
  else {
    // Finished external playback
    isExternalPlayback = false;
    isExternalPlaybackPaused = false;  // NEW: Reset pause state
    totalPausedTime = 0;  // NEW: Reset paused time
    
    currentPoint = 0;
    Serial.println("complete"); 
  }
}

void sendExternalMovementCommand(int pointIndex) {
  if (pointIndex == 0) {
    // First point: Move to absolute position stored at index 0
    float ttargetAngle1 = externalPlaybackData[pointIndex].angle1;
    float ttargetAngle2 = externalPlaybackData[pointIndex].angle2;
    long tsteps3 = externalPlaybackData[pointIndex].steps3;
    Serial.print(ttargetAngle1);
    // Read current encoder positions
    float tcurrentAngle1 = readEncoder1();
    float tcurrentAngle2 = readEncoder2();
    Serial.print(ttargetAngle1);
    Serial.print("  ");
    Serial.print(tcurrentAngle1);
    
    // Calculate difference to reach target position
    float deltaAngle1 = ttargetAngle1 - tcurrentAngle1;
    float deltaAngle2 = ttargetAngle2 - tcurrentAngle2;
    Serial.println(deltaAngle1);
    // Convert angle difference to steps
    long steps1 = (long)(deltaAngle1 * (STEPS_PER_REV / 360.0));
    long steps2 = (long)(deltaAngle2 * (STEPS_PER_REV / 360.0));
    
    // Calculate speeds
    int speed1 = 100;
    int speed2 = 100; 
    int speed3 = 100;
    
    // Send relative movement command for initial positioning
    Serial2.print("REL:");
    Serial2.print(steps1);
    Serial2.print(",");
    Serial2.print(steps2);
    Serial2.print(",");
    Serial2.print(tsteps3);
    Serial2.print(",");
    Serial2.print(speed1);
    Serial2.print(",");
    Serial2.print(speed2);
    Serial2.print(",");
    Serial2.print(speed3);
    Serial2.print("\n");

    long maxSteps = max(max(abs(steps1/4), abs(steps2/4)), abs(tsteps3/4));
    unsigned long completionTime = (maxSteps * 1000) / 100; // Convert to milliseconds (steps * 1000ms/s / 100steps/s)
    delay(completionTime);

  } else {
    // Subsequent points: Use stored deltas directly for relative movement
    float deltaAngle1 = externalPlaybackData[pointIndex].angle1;
    float deltaAngle2 = externalPlaybackData[pointIndex].angle2;
    long steps3 = externalPlaybackData[pointIndex].steps3;
    
    // Convert angle difference to steps
    long steps1 = (long)(deltaAngle1 * (STEPS_PER_REV / 360.0));
    long steps2 = (long)(deltaAngle2 * (STEPS_PER_REV / 360.0));
    
    // Calculate individual speeds with division by zero protection
    int speed1 = max(10L, abs(steps1) * 10);
    int speed2 = max(10L, abs(steps2) * 10);
    int speed3 = max(10L, abs(steps3) * 10);
    
    // Send relative movement command with individual speeds
    Serial2.print("REL:");
    Serial2.print(steps1);
    Serial2.print(",");
    Serial2.print(steps2);
    Serial2.print(",");
    Serial2.print(steps3);
    Serial2.print(",");
    Serial2.print(speed1);
    Serial2.print(",");
    Serial2.print(speed2);
    Serial2.print(",");
    Serial2.print(speed3);
    Serial2.print("\n");
  }
}

void startRecording() {
  if (isPlayback || isExternalPlayback) {
    Serial.println("Cannot record during playback");
    return;
  }
  
  isRecording = true;
  dataIndex = 0;
  recordingStartTime = millis();  // Record the start time
  lastRecordTime = recordingStartTime;
  prevAngle1 = readEncoder1();
  prevAngle2 = readEncoder2();
  Serial.println("Recording started...");
}

void handleRecording() {
  if (isRecording && dataIndex < 1000) {
    unsigned long currentTime = millis();
    if (currentTime - lastRecordTime >= RECORD_INTERVAL) {
      recordDataPoint();
    }
  }
}

void recordDataPoint() {
  if (dataIndex >= 1000) return;
  
  float currentAngle1 = readEncoder1();
  float currentAngle2 = readEncoder2();
  if (loadcell2.is_ready()) {
    
    float tcurrentForce = loadcell2.get_units();
    float tnetForce = abs(tcurrentForce - baseForceValue2);
    int speed;
    
    if (tnetForce > forceThreshold && tcurrentForce>0) 
    {
      speed = mapForceToSpeed(tnetForce);
      speed=(-1)*speed;
    }
    else if (((-1)*tnetForce)<((-1)*forceThreshold)&& tcurrentForce<0)
    {
      speed=mapForceToSpeed(tnetForce);
    }
    else
    {
      speed=0;
    } 
    Serial.println(speed); 
      // Send force-based speed command
      Serial2.print("FORCE_SPEED:");
      Serial2.print(speed);
      Serial2.print("\n");
  }
      
  recordedData[dataIndex].timestamp = 100;
  
  if (dataIndex == 0) {
    // First data point: Store absolute angles
    recordedData[dataIndex].angle1 = currentAngle1;
    recordedData[dataIndex].angle2 = currentAngle2;
    
    Serial.print("Recording start position - Absolute angles: (");
    Serial.print(currentAngle1, 2);
    Serial.print(", ");
    Serial.print(currentAngle2, 2);
    Serial.println(")");
  } else {
    // Subsequent data points: Store relative movement (delta from previous)
    float deltaAngle1 = currentAngle1 - prevAngle1;
    float deltaAngle2 = currentAngle2 - prevAngle2;
    
    // Store relative angles
    recordedData[dataIndex].angle1 = deltaAngle1;
    recordedData[dataIndex].angle2 = deltaAngle2;
  }
  
  // Initialize steps3 to 0 during recording (will be filled when Arduino data is received)
  recordedData[dataIndex].steps3 = 0;
  
  // Update previous angles for next recording
  prevAngle1 = currentAngle1;
  prevAngle2 = currentAngle2;
  
  dataIndex++;
  lastRecordTime = millis();
}

void stopRecording() {
  if (!isRecording) {
    Serial.println("Not recording");
    Serial2.println("sr");
    return;
  }
  
  isRecording = false;
  recordingEndTime = millis();  // Capture end time here
  Serial.print("Recording stopped. Recorded ");
  Serial.print(dataIndex);
  Serial.println(" data points.");
  
  // Send fetch command to Arduino after 100ms delay
  Serial2.println("FETCH");
}

void sendRecordedDataOverSerial() {
  Serial.print("{");
  for (int i = 0; i < dataIndex; i++) {
    Serial.print("(");
    Serial.print(recordedData[i].angle1, 6);
    Serial.print(",");
    Serial.print(recordedData[i].angle2, 6);
    Serial.print(",");
    Serial.print(recordedData[i].steps3);
    Serial.print(")");
    
    if (i < dataIndex - 1) {
      Serial.print(",");
    }
  }
  Serial.println("}");
}

void handleArduinoResponse() {
  if (Serial2.available()) {
    String response = Serial2.readStringUntil('\n');
    response.trim();
    //Serial.println(response);
    if (response.startsWith("STEPS_ARRAY:")) {
      parseStepsData(response);
    }
  }
}

void parseStepsData(String data) {
  // Remove "STEPS_ARRAY:" prefix
  data.remove(0, 12);
  
  // Find the separator between size and data
  int colonIndex = data.indexOf(':');
  if (colonIndex == -1) return;
  
  // Parse array size
  receivedArraySize = data.substring(0, colonIndex).toInt();
  
  // Parse steps data
  String stepsData = data.substring(colonIndex + 1);
  
  int index = 0;
  int startPos = 0;
  
  // Store steps data directly in the DataPoint struct
  while (index < receivedArraySize && index < dataIndex && startPos < stepsData.length()) {
    int commaPos = stepsData.indexOf(',', startPos);
    
    if (commaPos == -1) {
      // Last element
      recordedData[index].steps3 = stepsData.substring(startPos).toInt();
      break;
    } else {
      recordedData[index].steps3 = stepsData.substring(startPos, commaPos).toInt();
      startPos = commaPos + 1;
    }
    index++;
  }
  
  // Calculate and print recording information
  unsigned long totalRecordingTime = recordingStartTime > 0 ? (millis() - recordingStartTime) : 0;
  
  Serial.print("Recording completed. Total time: ");
  Serial.print(totalRecordingTime);
  Serial.print(" ms, DataPoints: ");
  Serial.print(dataIndex);
  Serial.print(", Steps data points filled: ");
  Serial.println(index);
  
  // Send the recorded data over Serial0 in the requested format
  sendRecordedDataOverSerial();
}

void startPlayback(int repetitions) {
  if (isRecording || isExternalPlayback) {
    Serial.println("Cannot playback during recording or external playback");
    return;
  }
  
  if (dataIndex == 0) {
    Serial.println("No recorded data to playback");
    return;
  }
  
  playbackRepetitions = repetitions;
  isPlayback = true;
}

void handlePlayback() {
  if (!isPlayback || dataIndex == 0) return;
  
  if (currentPoint == 0) {
    startTime = millis();
    sendMovementCommand(currentPoint);
    currentPoint++;
    return;
  }
  
  if (currentPoint < dataIndex) {
    unsigned long targetTime = millis()-startTime;
    
    if (targetTime>=100) {
      startTime = millis();
      sendMovementCommand(currentPoint);
      currentPoint++;
    }
  } else {
    // Finished one repetition
    currentRep++;
    if (currentRep < playbackRepetitions) {
      currentPoint = 0;
      Serial.print("Repetition ");
      Serial.print(currentRep + 1);
      Serial.print(" of ");
      Serial.println(playbackRepetitions);
    } else {
      // Finished all repetitions
      isPlayback = false;
      Serial2.println("DISABLE");
      currentRep = 0;
      currentPoint = 0;
      Serial.println("Playback completed");
    }
  }
}

void selectTCAChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

float readEncoder1() {
  selectTCAChannel(ENCODER1_CHANNEL);
  delay(2);
  return encoder1.rawAngle() * 0.087890625;
}

float readEncoder2() {
  selectTCAChannel(ENCODER2_CHANNEL);
  delay(2);
  return encoder2.rawAngle() * 0.087890625;
}

void calibrateBaseForce() {
  Serial.println("Calibrating base force value...");
  delay(1000);
  
  float totalForce1,totalForce2,totalForce3 = 0;
  int samples = 10;
  
  for (int i = 0; i < samples; i++) {
    if (loadcell1.is_ready() && loadcell2.is_ready() && loadcell3.is_ready()) 
    {
      totalForce1 += loadcell1.get_units();
      totalForce2 += loadcell2.get_units();
      totalForce3 += loadcell3.get_units();
    }
    else
    {
      --i;
    }
    delay(100);
  }
  
  baseForceValue1 = totalForce1 / samples;
  baseForceValue2 = totalForce2 / samples;
  baseForceValue3 = totalForce3 / samples;
  Serial.print("Base force value set to: ");
  Serial.print(baseForceValue3, 2);
  Serial.print("  ");
  Serial.print(baseForceValue2, 2);
  Serial.print("  ");
  Serial.println(baseForceValue1, 2);
}

void sendMovementCommand(int pointIndex) {
  if (pointIndex == 0) {
    // First point: Move to absolute position stored at index 0
    float targetAngle1 = recordedData[pointIndex].angle1;
    float targetAngle2 = recordedData[pointIndex].angle2;
    long steps3 = recordedData[pointIndex].steps3;
    
    // Read current encoder positions
    float currentAngle1 = readEncoder1();
    float currentAngle2 = readEncoder2();
    
    // Calculate difference to reach target position
    float deltaAngle1 = targetAngle1 - currentAngle1;
    float deltaAngle2 = targetAngle2 - currentAngle2;
    
    Serial.print("Moving to start position - Target: (");
    Serial.print(targetAngle1, 2);
    Serial.print(", ");
    Serial.print(targetAngle2, 2);
    Serial.print("), Current: (");
    Serial.print(currentAngle1, 2);
    Serial.print(", ");
    Serial.print(currentAngle2, 2);
    Serial.print("), Delta: (");
    Serial.print(deltaAngle1, 2);
    Serial.print(", ");
    Serial.print(deltaAngle2, 2);
    Serial.println(")");
    
    // Convert angle difference to steps
    long steps1 = (long)(deltaAngle1 * (STEPS_PER_REV / 360.0));
    long steps2 = (long)(deltaAngle2 * (STEPS_PER_REV / 360.0));
    
    // Calculate speeds
    int speed1 = max(10L, abs(steps1) * 10);
    int speed2 = max(10L, abs(steps2) * 10);
    int speed3 = max(10L, abs(steps3) * 10);
    
    // Send relative movement command for initial positioning (no completion confirmation expected)
    Serial2.print("REL:");
    Serial2.print(steps1);
    Serial2.print(",");
    Serial2.print(steps2);
    Serial2.print(",");
    Serial2.print(steps3);
    Serial2.print(",");
    Serial2.print(speed1);
    Serial2.print(",");
    Serial2.print(speed2);
    Serial2.print(",");
    Serial2.print(speed3);
    Serial2.print("\n");
  } else {
    // Subsequent points: Use stored deltas directly for relative movement
    float deltaAngle1 = recordedData[pointIndex].angle1;
    float deltaAngle2 = recordedData[pointIndex].angle2;
    long steps3 = recordedData[pointIndex].steps3;
    
    Serial.println(steps3);
    
    // Convert angle difference to steps
    long steps1 = (long)(deltaAngle1 * (STEPS_PER_REV / 360.0));
    long steps2 = (long)(deltaAngle2 * (STEPS_PER_REV / 360.0));
    
    // Calculate individual speeds with division by zero protection
    int speed1 = max(10L, abs(steps1) * 10);
    int speed2 = max(10L, abs(steps2) * 10);
    int speed3 = max(10L, abs(steps3) * 10);
    
    // Send relative movement command with individual speeds
    Serial2.print("REL:");
    Serial2.print(steps1);
    Serial2.print(",");
    Serial2.print(steps2);
    Serial2.print(",");
    Serial2.print(steps3);
    Serial2.print(",");
    Serial2.print(speed1);
    Serial2.print(",");
    Serial2.print(speed2);
    Serial2.print(",");
    Serial2.print(speed3);
    Serial2.print("\n");
  }
}

void deleteRecording() {
  if (isRecording || isPlayback || isExternalPlayback) {
    Serial.println("Cannot delete during recording or playback");
    return;
  }
  
  dataIndex = 0;
  externalDataIndex = 0;
  Serial.println("Recording deleted");
}
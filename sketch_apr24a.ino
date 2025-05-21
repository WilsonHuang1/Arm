/*
  Color-Position Memory Robot Arm
  This program allows you to:
  1. Manually position a robot arm using servo controls
  2. Associate specific colors with specific arm positions
  3. When a color is detected, the arm will move to the saved position
  Hardware:
  - TCS3200 Color Sensor
  - PCA9685 Servo Driver
  - 3 Servo Motors (Base, Shoulder, Elbow)
  - Arduino (Uno, Mega, etc.)
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>
void saveCurrentPosition(bool colorOnly);
void saveCurrentColor();
void printColorPositionsInfo();
void adjustSelectedServo(int adjustment);
void moveToSavedPosition(int colorIndex);
void resetServos();
void flashLEDs();
void saveToEEPROM();
void loadFromEEPROM();
void printStatus();

// Create servo driver instance
Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();

// Color sensor pins
const int S0 = 2;
const int S1 = 3;
const int S2 = 4;
const int S3 = 5;
const int sensorOut = 6;

// LED pins (optional for visual feedback)
const int redLED = 10;
const int greenLED = 9;
const int blueLED = 8;

// Servo indices
#define BASE_SERVO 0       // Base servo (horizontal rotation)
#define SHOULDER_SERVO 1   // Shoulder servo (around base)
#define ELBOW_SERVO 2      // Elbow servo (around shoulder)

// Servo limits and positions
int servoMins[3] = {150, 150, 150};       // Minimum pulse lengths
int servoMaxs[3] = {600, 600, 600};       // Maximum pulse lengths
int servoPositions[3] = {150, 325, 425};  // Current positions - with specific starting values

// Step sizes for manual control
#define LARGE_STEP 25    // For W/S keys
#define SMALL_STEP 5     // For A/D keys

// EEPROM addresses for storing data
#define EEPROM_VALID_FLAG 0
#define EEPROM_COLOR_DATA_START 10
#define EEPROM_COLOR_DATA_SIZE 20 // Each color takes 20 bytes of EEPROM

// Color storage
#define MAX_COLORS 5     // Store up to 5 color-position combinations
struct ColorPosition {
  unsigned long red;     // Red value
  unsigned long green;   // Green value
  unsigned long blue;    // Blue value
  int basePos;           // Base servo position
  int shoulderPos;       // Shoulder servo position
  int elbowPos;          // Elbow servo position
  bool calibrated;       // Whether this position is calibrated
  String colorName;      // Name of the color (e.g., "RED", "GREEN", etc.)
};
ColorPosition savedColors[MAX_COLORS];
int currentColor = 0;    // Currently selected color (0-4)
boolean detectionActive = false;

// Color readings
unsigned long redValue, greenValue, blueValue, clearValue;

// Selected servo for manual control
int selectedServo = BASE_SERVO;

// Color detection parameters
unsigned long colorMatchThreshold = 30;  // Maximum distance for color match
unsigned long lastDetectionTime = 0;     // For timing
#define DETECTION_INTERVAL 500           // Check for color every 500ms
unsigned long lastRGBPrintTime = 0;      // For timing RGB data printing when in detection mode
#define RGB_PRINT_INTERVAL 1000          // Print RGB data every 1000ms when in detection mode

// Return to home state after detection
boolean returningHome = false;
unsigned long colorDetectedTime = 0;
#define RETURN_DELAY 4500                // Return to home after 3 seconds in color zone

// Mode for only updating color for existing position
boolean colorOnlyMode = false;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Set up color sensor pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Set frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  
  // Set up LED pins
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  
  // Initialize servo driver
  servoDriver.begin();
  servoDriver.setPWMFreq(60);  // 60 Hz update frequency
  
  // Initialize saved color positions and names
  for (int i = 0; i < MAX_COLORS; i++) {
    savedColors[i].calibrated = false;
    
    // Set color names
    if (i == 0) savedColors[i].colorName = "RED";
    else if (i == 1) savedColors[i].colorName = "GREEN";
    else if (i == 2) savedColors[i].colorName = "BLUE";
    else if (i == 3) savedColors[i].colorName = "COLOR 4";
    else if (i == 4) savedColors[i].colorName = "COLOR 5";
  }
  
  // Load saved positions from EEPROM
  loadFromEEPROM();
  
  // Move servos to starting positions
  for (int i = 0; i < 3; i++) {
    servoDriver.setPWM(i, 0, servoPositions[i]);
  }
  
  // Print welcome message and instructions
  Serial.println("\nColor-Position Memory Robot Arm");
  Serial.println("-------------------------------");
  Serial.println("Manual Servo Control:");
  Serial.println("  B/S/E - Select Base, Shoulder, or Elbow servo");
  Serial.println("  W/Z - Large step increase/decrease");
  Serial.println("  A/D - Small step increase/decrease");
  Serial.println("  0 - Reset all servos to initial positions {150, 325, 425}");
  Serial.println("Color Commands:");
  Serial.println("  R/G/U - Select a color slot (Red, Green, blUe)");
  Serial.println("  4/5 - Select color slot 4 or 5");
  Serial.println("  M - Save current position and record color for selected slot");
  Serial.println("  C - Only update color data (keep saved position)");
  Serial.println("  T - Toggle color detection on/off");
  Serial.println("  I - Print info about all saved color positions");
  Serial.println("  P - Print current color values and saved positions");
  Serial.println("  F - Toggle color-only mode (only update colors, not positions)");
  
  // Flash all LEDs to indicate ready
  flashLEDs();
}

void loop() {
  // Read current color values
  readColorValues();
  
  // If detection is active, print RGB values periodically
  if (detectionActive && millis() - lastRGBPrintTime > RGB_PRINT_INTERVAL) {
    lastRGBPrintTime = millis();
    Serial.print("Current RGB - R:");
    Serial.print(redValue);
    Serial.print(" G:");
    Serial.print(greenValue);
    Serial.print(" B:");
    Serial.println(blueValue);
  }
  
  // Process any serial commands
  processSerialCommands();
  
  // If returning to home state and delay has passed
  if (returningHome && millis() - colorDetectedTime > RETURN_DELAY) {
    Serial.println("Returning to home position after 5 seconds");
    returningHome = false;
    resetServos();
  }
  
  // If detection is active, check for matching colors
  if (detectionActive && !returningHome && millis() - lastDetectionTime > DETECTION_INTERVAL) {
    lastDetectionTime = millis();
    // Check each calibrated color
    for (int i = 0; i < MAX_COLORS; i++) {
      if (savedColors[i].calibrated) {
        unsigned long distance = calculateColorDistance(
          redValue, greenValue, blueValue,
          savedColors[i].red, savedColors[i].green, savedColors[i].blue
        );
        // If color matches, move to the saved position
        if (distance < colorMatchThreshold) {
          // Show which color matched
          Serial.print("Detected ");
          Serial.print(savedColors[i].colorName);
          Serial.print(" (distance: ");
          Serial.print(distance);
          Serial.println(")");
          
          // Visual indicator based on which color matched
          if (i == 0) digitalWrite(redLED, HIGH);
          else if (i == 1) digitalWrite(greenLED, HIGH);
          else if (i == 2) digitalWrite(blueLED, HIGH);
          else flashLEDs();
          
          // Move servos to the saved position
          moveToSavedPosition(i);
          
          // Start timer for returning to home
          colorDetectedTime = millis();
          returningHome = true;
          
          break; // Stop checking after first match
        }
      }
    }
  }
  
  // Short delay
  delay(10);
}

// Read RGB values from the color sensor
void readColorValues() {
  // Read red values
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redValue = pulseIn(sensorOut, LOW);
  
  // Read green values
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenValue = pulseIn(sensorOut, LOW);
  
  // Read blue values
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueValue = pulseIn(sensorOut, LOW);
  
  // Read clear values (optional)
  digitalWrite(S2, HIGH);
  digitalWrite(S3, LOW);
  clearValue = pulseIn(sensorOut, LOW);
}

// Move all servos to a saved position
void moveToSavedPosition(int colorIndex) {
  if (savedColors[colorIndex].calibrated) {
    // Move each servo to its saved position
    servoPositions[BASE_SERVO] = savedColors[colorIndex].basePos;
    servoPositions[SHOULDER_SERVO] = savedColors[colorIndex].shoulderPos;
    servoPositions[ELBOW_SERVO] = savedColors[colorIndex].elbowPos;
    
    // Update servo positions
    servoDriver.setPWM(BASE_SERVO, 0, servoPositions[BASE_SERVO]);
    servoDriver.setPWM(SHOULDER_SERVO, 0, servoPositions[SHOULDER_SERVO]);
    servoDriver.setPWM(ELBOW_SERVO, 0, servoPositions[ELBOW_SERVO]);
    
    Serial.print("Moved to position for ");
    Serial.println(savedColors[colorIndex].colorName);
  }
}

// Calculate distance between two colors
unsigned long calculateColorDistance(unsigned long r1, unsigned long g1, unsigned long b1,
                                    unsigned long r2, unsigned long g2, unsigned long b2) {
  // Calculate the difference between two colors
  unsigned long rDiff = ((r1 > r2) ? (r1 - r2) : (r2 - r1));
  unsigned long gDiff = ((g1 > g2) ? (g1 - g2) : (g2 - g1));
  unsigned long bDiff = ((b1 > b2) ? (b1 - b2) : (b2 - b1));
  
  // Return weighted sum of differences
  return (rDiff * 2) + (gDiff * 3) + (bDiff * 1);
}

// Process commands from serial port
void processSerialCommands() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    switch (command) {
      // Servo selection
      case 'B': // Select base servo
      case 'b':
        selectedServo = BASE_SERVO;
        Serial.println("Selected: Base servo");
        break;
      case 'S': // Select shoulder servo
      case 's':
        selectedServo = SHOULDER_SERVO;
        Serial.println("Selected: Shoulder servo");
        break;
      case 'E': // Select elbow servo
      case 'e':
        selectedServo = ELBOW_SERVO;
        Serial.println("Selected: Elbow servo");
        break;
        
      // Large servo adjustments
      case 'W': // Increase servo position (large step)
      case 'w':
        adjustSelectedServo(LARGE_STEP);
        break;
      case 'Z': // Decrease servo position (large step)
      case 'z':
        adjustSelectedServo(-LARGE_STEP);
        break;
        
      // Small servo adjustments
      case 'A': // Increase servo position (small step)
      case 'a':
        adjustSelectedServo(SMALL_STEP);
        break;
      case 'D': // Decrease servo position (small step)
      case 'd':
        adjustSelectedServo(-SMALL_STEP);
        break;
        
      // Color selection
      case 'R': // Red - select color 1
      case 'r':
        currentColor = 0;
        Serial.println("Selected RED slot (1)");
        break;
      case 'G': // Green - select color 2
      case 'g':
        currentColor = 1;
        Serial.println("Selected GREEN slot (2)");
        break;
      case 'U': // blUe - select color 3 (changed from 'B' to avoid duplicate)
      case 'u':
        currentColor = 2;
        Serial.println("Selected BLUE slot (3)");
        break;
      case '4': // Select color 4
        currentColor = 3;
        Serial.println("Selected color slot 4");
        break;
      case '5': // Select color 5
        currentColor = 4;
        Serial.println("Selected color slot 5");
        break;
        
      // Save and detect
      case 'M': // Save current position and color for selected slot
      case 'm':
        saveCurrentPosition(false);
        break;
      case 'C': // Save only color data for selected slot
      case 'c':
        saveCurrentColor();
        break;
      case 'F': // Toggle color-only mode
      case 'f':
        colorOnlyMode = !colorOnlyMode;
        Serial.print("Color-only mode: ");
        Serial.println(colorOnlyMode ? "ON" : "OFF");
        if (colorOnlyMode) {
          Serial.println("Will only update colors, keeping saved positions");
        } else {
          Serial.println("Will update both positions and colors when saving");
        }
        break;
      case 'T': // Toggle detection
      case 't':
        detectionActive = !detectionActive;
        returningHome = false; // Cancel any pending return
        Serial.print("Color detection: ");
        Serial.println(detectionActive ? "ON" : "OFF");
        if (detectionActive) {
          // Print current values immediately when detection is turned on
          Serial.print("Current RGB - R:");
          Serial.print(redValue);
          Serial.print(" G:");
          Serial.print(greenValue);
          Serial.print(" B:");
          Serial.println(blueValue);
        }
        break;
        
      // Information and reset
      case 'I': // Info - list all color positions
      case 'i':
        printColorPositionsInfo();
        break;
      case 'P': // Print current values
      case 'p':
        printStatus();
        break;
      case '0': // Reset all servos
        resetServos();
        break;
    }
    
    // Clear any remaining input
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}

// Save only color data for current position
void saveCurrentColor() {
  // Make sure the position is already calibrated
  if (!savedColors[currentColor].calibrated) {
    Serial.println("Position not yet calibrated. Use 'M' to save position first.");
    return;
  }
  
  Serial.print("Updating color values for ");
  Serial.println(savedColors[currentColor].colorName);
  Serial.println("Hold color in front of sensor...");
  
  // Flash LED to indicate we're reading color
  if (currentColor == 0) digitalWrite(redLED, HIGH);
  else if (currentColor == 1) digitalWrite(greenLED, HIGH);
  else if (currentColor == 2) digitalWrite(blueLED, HIGH);
  else flashLEDs();
  
  // Take multiple color readings and average them
  long rSum = 0, gSum = 0, bSum = 0;
  const int numReadings = 5;
  for (int i = 0; i < numReadings; i++) {
    readColorValues();
    rSum += redValue;
    gSum += greenValue;
    bSum += blueValue;
    delay(200); // Longer delay for more stable readings
  }
  
  // Turn off LEDs
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);
  
  // Save color information
  savedColors[currentColor].red = rSum / numReadings;
  savedColors[currentColor].green = gSum / numReadings;
  savedColors[currentColor].blue = bSum / numReadings;
  
  // Save to EEPROM
  saveToEEPROM();
  
  // Confirmation
  Serial.println("Color values updated!");
  Serial.print(savedColors[currentColor].colorName);
  Serial.print(": ");
  Serial.print(savedColors[currentColor].basePos);
  Serial.print(",");
  Serial.print(savedColors[currentColor].shoulderPos);
  Serial.print(",");
  Serial.print(savedColors[currentColor].elbowPos);
  Serial.print(" R:");
  Serial.print(savedColors[currentColor].red);
  Serial.print(" G:");
  Serial.print(savedColors[currentColor].green);
  Serial.print(" B:");
  Serial.println(savedColors[currentColor].blue);
  
  // Visual feedback to confirm completion
  if (currentColor == 0) {
    digitalWrite(redLED, HIGH);
    delay(1000);
    digitalWrite(redLED, LOW);
  } else if (currentColor == 1) {
    digitalWrite(greenLED, HIGH);
    delay(1000);
    digitalWrite(greenLED, LOW);
  } else if (currentColor == 2) {
    digitalWrite(blueLED, HIGH);
    delay(1000);
    digitalWrite(blueLED, LOW);
  } else {
    flashLEDs();
  }
}

// Print all saved color positions in the requested format
void printColorPositionsInfo() {
  Serial.println("\n--- SAVED COLOR POSITIONS ---");
  for (int i = 0; i < MAX_COLORS; i++) {
    Serial.print(savedColors[i].colorName);
    Serial.print(": ");
    
    if (savedColors[i].calibrated) {
      // Print servo positions
      Serial.print(savedColors[i].basePos);
      Serial.print(",");
      Serial.print(savedColors[i].shoulderPos);
      Serial.print(",");
      Serial.print(savedColors[i].elbowPos);
      
      // Print RGB values
      Serial.print(" R:");
      Serial.print(savedColors[i].red);
      Serial.print(" G:");
      Serial.print(savedColors[i].green);
      Serial.print(" B:");
      Serial.println(savedColors[i].blue);
    } else {
      Serial.println("Not calibrated");
    }
  }
  Serial.println("---------------------------");
}

// Adjust the selected servo by the specified amount
void adjustSelectedServo(int adjustment) {
  // Apply the adjustment
  servoPositions[selectedServo] += adjustment;
  
  // Ensure it's within limits
  if (servoPositions[selectedServo] < servoMins[selectedServo]) {
    servoPositions[selectedServo] = servoMins[selectedServo];
  }
  if (servoPositions[selectedServo] > servoMaxs[selectedServo]) {
    servoPositions[selectedServo] = servoMaxs[selectedServo];
  }
  
  // Update the servo
  servoDriver.setPWM(selectedServo, 0, servoPositions[selectedServo]);
  
  // Show new position
  Serial.print("Servo ");
  if (selectedServo == BASE_SERVO) Serial.print("Base");
  else if (selectedServo == SHOULDER_SERVO) Serial.print("Shoulder");
  else if (selectedServo == ELBOW_SERVO) Serial.print("Elbow");
  Serial.print(" position: ");
  Serial.println(servoPositions[selectedServo]);
}

// Save current arm position and color for the selected color slot
void saveCurrentPosition(bool colorOnly = false) {
  // If in color-only mode, only update color
  if (colorOnlyMode || colorOnly) {
    saveCurrentColor();
    return;
  }
  
  // First save the position
  savedColors[currentColor].basePos = servoPositions[BASE_SERVO];
  savedColors[currentColor].shoulderPos = servoPositions[SHOULDER_SERVO];
  savedColors[currentColor].elbowPos = servoPositions[ELBOW_SERVO];
  
  // Message to user
  Serial.print("Position saved for ");
  Serial.println(savedColors[currentColor].colorName);
  Serial.println("Now reading color values - hold color in front of sensor...");
  
  // Flash LED to indicate we're reading color
  if (currentColor == 0) digitalWrite(redLED, HIGH);
  else if (currentColor == 1) digitalWrite(greenLED, HIGH);
  else if (currentColor == 2) digitalWrite(blueLED, HIGH);
  else flashLEDs();
  
  // Take multiple color readings and average them
  long rSum = 0, gSum = 0, bSum = 0;
  const int numReadings = 5;
  for (int i = 0; i < numReadings; i++) {
    readColorValues();
    rSum += redValue;
    gSum += greenValue;
    bSum += blueValue;
    delay(200); // Longer delay for more stable readings
  }
  
  // Turn off LEDs
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);
  
  // Save color information
  savedColors[currentColor].red = rSum / numReadings;
  savedColors[currentColor].green = gSum / numReadings;
  savedColors[currentColor].blue = bSum / numReadings;
  savedColors[currentColor].calibrated = true;
  
  // Save to EEPROM
  saveToEEPROM();
  
  // Confirmation
  Serial.println("Position and color values saved!");
  Serial.print(savedColors[currentColor].colorName);
  Serial.print(": ");
  Serial.print(savedColors[currentColor].basePos);
  Serial.print(",");
  Serial.print(savedColors[currentColor].shoulderPos);
  Serial.print(",");
  Serial.print(savedColors[currentColor].elbowPos);
  Serial.print(" R:");
  Serial.print(savedColors[currentColor].red);
  Serial.print(" G:");
  Serial.print(savedColors[currentColor].green);
  Serial.print(" B:");
  Serial.println(savedColors[currentColor].blue);
  
  // Visual feedback to confirm completion
  if (currentColor == 0) {
    digitalWrite(redLED, HIGH);
    delay(1000);
    digitalWrite(redLED, LOW);
  } else if (currentColor == 1) {
    digitalWrite(greenLED, HIGH);
    delay(1000);
    digitalWrite(greenLED, LOW);
  } else if (currentColor == 2) {
    digitalWrite(blueLED, HIGH);
    delay(1000);
    digitalWrite(blueLED, LOW);
  } else {
    flashLEDs();
  }
}

// Reset all servos to initial positions
void resetServos() {
  Serial.println("Resetting servos to initial positions");
  
  // Reset to initial positions
  servoPositions[BASE_SERVO] = 150;
  servoPositions[SHOULDER_SERVO] = 325;
  servoPositions[ELBOW_SERVO] = 425;
  
  // Update servos
  servoDriver.setPWM(BASE_SERVO, 0, servoPositions[BASE_SERVO]);
  servoDriver.setPWM(SHOULDER_SERVO, 0, servoPositions[SHOULDER_SERVO]);
  servoDriver.setPWM(ELBOW_SERVO, 0, servoPositions[ELBOW_SERVO]);
  
  // Turn off all LEDs
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);
}

// Flash all LEDs briefly
void flashLEDs() {
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, HIGH);
  digitalWrite(blueLED, HIGH);
  delay(200);
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);
  delay(200);
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, HIGH);
  digitalWrite(blueLED, HIGH);
  delay(200);
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);
}

// Save current settings to EEPROM
void saveToEEPROM() {
  // Write a valid flag to EEPROM to indicate data is saved
  EEPROM.write(EEPROM_VALID_FLAG, 0xAA);
  
  // Write each color position
  for (int i = 0; i < MAX_COLORS; i++) {
    int addr = EEPROM_COLOR_DATA_START + (i * EEPROM_COLOR_DATA_SIZE);
    
    // Write calibrated flag
    EEPROM.write(addr, savedColors[i].calibrated ? 1 : 0);
    addr++;
    
    // Write servo positions (2 bytes each)
    EEPROM.write(addr, savedColors[i].basePos >> 8);
    EEPROM.write(addr + 1, savedColors[i].basePos & 0xFF);
    addr += 2;
    
    EEPROM.write(addr, savedColors[i].shoulderPos >> 8);
    EEPROM.write(addr + 1, savedColors[i].shoulderPos & 0xFF);
    addr += 2;
    
    EEPROM.write(addr, savedColors[i].elbowPos >> 8);
    EEPROM.write(addr + 1, savedColors[i].elbowPos & 0xFF);
    addr += 2;
    
    // Write RGB values (2 bytes each)
    EEPROM.write(addr, savedColors[i].red >> 8);
    EEPROM.write(addr + 1, savedColors[i].red & 0xFF);
    addr += 2;
    
    EEPROM.write(addr, savedColors[i].green >> 8);
    EEPROM.write(addr + 1, savedColors[i].green & 0xFF);
    addr += 2;
    
    EEPROM.write(addr, savedColors[i].blue >> 8);
    EEPROM.write(addr + 1, savedColors[i].blue & 0xFF);
  }
  
  Serial.println("Data saved to EEPROM.");
}

// Load settings from EEPROM
void loadFromEEPROM() {
  // Check if EEPROM has valid data
  if (EEPROM.read(EEPROM_VALID_FLAG) != 0xAA) {
    Serial.println("No valid data in EEPROM. Using defaults.");
    return;
  }
  
  Serial.println("Loading data from EEPROM...");
  
  // Read each color position
  for (int i = 0; i < MAX_COLORS; i++) {
    int addr = EEPROM_COLOR_DATA_START + (i * EEPROM_COLOR_DATA_SIZE);
    
    // Read calibrated flag
    savedColors[i].calibrated = (EEPROM.read(addr) == 1);
    addr++;
    
    // Read servo positions (2 bytes each)
    savedColors[i].basePos = (EEPROM.read(addr) << 8) | EEPROM.read(addr + 1);
    addr += 2;
    
    savedColors[i].shoulderPos = (EEPROM.read(addr) << 8) | EEPROM.read(addr + 1);
    addr += 2;
    
    savedColors[i].elbowPos = (EEPROM.read(addr) << 8) | EEPROM.read(addr + 1);
    addr += 2;
    
    // Read RGB values (2 bytes each)
    savedColors[i].red = (EEPROM.read(addr) << 8) | EEPROM.read(addr + 1);
    addr += 2;
    
    savedColors[i].green = (EEPROM.read(addr) << 8) | EEPROM.read(addr + 1);
    addr += 2;
    
    savedColors[i].blue = (EEPROM.read(addr) << 8) | EEPROM.read(addr + 1);
  }
  
  Serial.println("EEPROM data loaded successfully.");
  printColorPositionsInfo();
}

// Print current status information
void printStatus() {
  Serial.println("\n--- Current Status ---");
  
  // Current color readings
  Serial.print("Current color - R:");
  Serial.print(redValue);
  Serial.print(" G:");
  Serial.print(greenValue);
  Serial.print(" B:");
  Serial.println(blueValue);
  
  // Current servo positions
  Serial.print("Base position: ");
  Serial.println(servoPositions[BASE_SERVO]);
  Serial.print("Shoulder position: ");
  Serial.println(servoPositions[SHOULDER_SERVO]);
  Serial.print("Elbow position: ");
  Serial.println(servoPositions[ELBOW_SERVO]);
  
  // Saved color positions
  Serial.println("\nSaved color positions:");
  for (int i = 0; i < MAX_COLORS; i++) {
    Serial.print(savedColors[i].colorName);
    Serial.print(": ");
    if (savedColors[i].calibrated) {
      Serial.print("R:");
      Serial.print(savedColors[i].red);
      Serial.print(" G:");
      Serial.print(savedColors[i].green);
      Serial.print(" B:");
      Serial.print(savedColors[i].blue);
      Serial.print(" - Base:");
      Serial.print(savedColors[i].basePos);
      Serial.print(" Shoulder:");
      Serial.print(savedColors[i].shoulderPos);
      Serial.print(" Elbow:");
      Serial.println(savedColors[i].elbowPos);
    } else {
      Serial.println("Not calibrated");
    }
  }
  
  Serial.print("Currently selected color: ");
  Serial.println(savedColors[currentColor].colorName);
  
  Serial.print("Selected servo: ");
  if (selectedServo == BASE_SERVO) Serial.println("Base");
  else if (selectedServo == SHOULDER_SERVO) Serial.println("Shoulder");
  else if (selectedServo == ELBOW_SERVO) Serial.println("Elbow");
  
  Serial.print("Detection active: ");
  Serial.println(detectionActive ? "Yes" : "No");
  Serial.print("Color-only mode: ");
  Serial.println(colorOnlyMode ? "ON" : "OFF");
  Serial.print("Match threshold: ");
  Serial.println(colorMatchThreshold);
  
  if (returningHome) {
    unsigned long remainingTime = (RETURN_DELAY - (millis() - colorDetectedTime)) / 1000;
    Serial.print("Returning to home in ");
    Serial.print(remainingTime);
    Serial.println(" seconds");
  }
  
  Serial.println("---------------------");
}
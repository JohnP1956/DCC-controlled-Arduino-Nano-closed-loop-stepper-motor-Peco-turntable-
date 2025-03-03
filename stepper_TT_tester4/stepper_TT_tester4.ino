/*
 * Stepper Motor Turntable Control with DCC Commands
 *
 * This software controls a stepper motor for a model railway turntable, using DCC commands
 * to move the turntable to various indexed positions. It supports an iterative setup process
 * for configuring index positions, storing configurations in EEPROM, and ensuring compliance
 * with NMRA DCC standards.
 *
 * Key Features:
 * - DCC control with address validation (addresses 1-511).
 * - Iterative setup process with EEPROM storage and `EEPROM.update()` for efficient writes.
 * - Configurable homing direction (clockwise or counter-clockwise, default is now counter-clockwise).
 * - Debug mode for serial communication during setup.
 * - It allows for iterative setup of each index position, viewing and modifying stored positions,
 *  -and uses EEPROM.update() to write only if a value changes.
 * - Supports 1.8-degree stepper motor with 32x microstepping (6400 steps per revolution).
 * - Configurable number of index positions (1-12)
 * - License: MIT License (see below).
 *
 * Libraries Used:
 * - AccelStepper: https://www.airspayce.com/mikem/arduino/AccelStepper/
 * - NmraDcc: https://github.com/mrrwa/NmraDcc
 *
 * Author: JP1 c/o RM Web
 * Date: 20/07/2024
 *
 * MIT License:
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <AccelStepper.h>
#include <NmraDcc.h>
#include <EEPROM.h>
#include "config.h"

// Create an instance of the AccelStepper class
AccelStepper stepper(MotorInterfaceType, stepPin, dirPin);

// Create an instance of the DCC decoder
NmraDcc Dcc;

// Position-related variables
long magnetEndPositions[MAX_INDEX_POSITIONS];    // Positions for magnet end, stored in EEPROM
long nonMagnetEndPositions[MAX_INDEX_POSITIONS]; // Positions for non-magnet end, stored in EEPROM
long currentPosition = 0;
bool homed = false;

// DCC addresses
int homePositionAddress;
int magnetEndAddresses[MAX_INDEX_POSITIONS];
int nonMagnetEndAddresses[MAX_INDEX_POSITIONS];
int numIndexPositions;  // Number of index positions, stored in EEPROM

// Last Address variables to prevent duplicate events
uint16_t lastAddr = 0xFFFF;
uint8_t lastDirection = 0xFF;

// Function prototypes
int getValidDccAddress();
int getValidIndexNumber();
void debugSetup();
String getUserInput();
long getUserInputLong();
bool getUserYesNo();
void loadConfigurations();
void homeMotor();
void moveToPosition(long position);
void moveRelative(long steps);
int calculateShortestStepDifference(long currentPosition, long nextPosition);

void setup() {
  Serial.begin(9600); // Start serial communication for debugging

  // Initialize pins
  pinMode(enPin, OUTPUT);
  pinMode(hallPin, INPUT_PULLUP);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(enPin, LOW); // Enable the motor

  // Set max speed and acceleration
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);

  Serial.println(F("Setup complete, loading configurations..."));

  // Load configurations from EEPROM
  loadConfigurations();

  // Initialize DCC decoder
  Dcc.pin(0,DCC_PIN, 1);
  Dcc.init(MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0);// Adjust manufacturer ID and version as needed
  Serial.println(F("DCC decoder initialized"));
  
  // Home the motor at startup
  homeMotor();

  // Call the debug setup function if debugging is enabled
  if (debug) {
    Serial.println(F("Debug mode enabled. Calling debug setup..."));
    debugSetup();
    Serial.println(F("Returned from debug setup..."));
  }
}

void loop() {
  // Process DCC commands
  Dcc.process();

  // Ensure the motor is running
  stepper.run();
}

// DCC command notification handler
void notifyDccAccTurnoutOutput(uint16_t Addr, uint8_t Direction, uint8_t OutputPower) {
  Serial.print(F("DCC Command Received - Address: "));
  Serial.print(Addr);
  Serial.print(F(" Direction: "));
  Serial.print(Direction);
  Serial.print(F(" Power: "));
  Serial.println(OutputPower);

  if (OutputPower) {
    if (Addr == homePositionAddress) {
      Serial.println(F("Moving to home position..."));
      moveRelative(-currentPosition);
      return;
    }

    for (int i = 0; i < numIndexPositions; i++) {
      if (Addr == magnetEndAddresses[i]) {
        Serial.print(F("Moving to magnet end position: "));
        Serial.println(magnetEndPositions[i]);
        long stepsToMove = calculateShortestStepDifference(currentPosition, magnetEndPositions[i]);
        moveRelative(stepsToMove);
        return;
      } else if (Addr == nonMagnetEndAddresses[i]) {
        Serial.print(F("Moving to non-magnet end position: "));
        Serial.println(nonMagnetEndPositions[i]);
        long stepsToMove = calculateShortestStepDifference(currentPosition, nonMagnetEndPositions[i]);
        moveRelative(stepsToMove);
        return;
      }
    }
  }
}

void homeMotor() {
    Serial.println(F("Starting homing process..."));

    // Initialize the counter for Hall sensor triggers
    int hallSensorTriggerCount = 0;

    // Set the speed and direction for the homing movement
    stepper.setSpeed(homingDirection ? maxSpeed : -maxSpeed);

    // Debugging print to confirm the direction
    Serial.print(F("Homing motor in direction: "));
    Serial.println(homingDirection ? F("Counter-Clockwise") : F("Clockwise"));

    // Move until the Hall sensor is triggered twice
    bool sensorPreviouslyTriggered = false;
    while (hallSensorTriggerCount < 2) {
        stepper.runSpeed();
        bool sensorCurrentlyTriggered = (digitalRead(hallPin) == LOW);
        if (sensorCurrentlyTriggered && !sensorPreviouslyTriggered) { // Rising edge detection
            hallSensorTriggerCount++;
            Serial.print(F("Hall sensor triggered "));
            Serial.print(hallSensorTriggerCount);
            Serial.println(F(" time(s)"));
        }
        sensorPreviouslyTriggered = sensorCurrentlyTriggered;
    }

    // Set the home position
    stepper.setCurrentPosition(0);
    currentPosition = 0;
    homed = true;

    Serial.println(F("Homing process completed."));
    Serial.println(F("Exiting homeMotor function"));

    // Response after homing
    Serial.println(F("Motor homed successfully."));

    // Small delay to allow hall sensor state to stabilize
    delay(500);
}

void moveToPosition(long position) {
  if (!homed) homeMotor();
  long stepsToMove = calculateShortestStepDifference(currentPosition, position);
  moveRelative(stepsToMove);
}

// Modified version of moveRelative to handle the special case
void moveRelative(long steps) {
  // Special case: moving to zero from a position more than half a revolution away
  if (steps == -currentPosition && abs(currentPosition) > stepsPerRevolution / 2) {
    Serial.println(F("Special case: Moving to zero position using homing-like approach"));
    
    // Set the speed for counterclockwise rotation (use the same direction as homing)
    stepper.setSpeed(homingDirection ? maxSpeed : -maxSpeed);
    
    // Initialize variables for hall sensor detection
    bool sensorPreviouslyTriggered = false;
    int hallSensorTriggerCount = 0;
    
    // Run until we trigger the hall sensor twice (just like in homing)
    while (hallSensorTriggerCount < 1) {
        stepper.runSpeed();
        bool sensorCurrentlyTriggered = (digitalRead(hallPin) == LOW);
        
        if (sensorCurrentlyTriggered && !sensorPreviouslyTriggered) { 
            hallSensorTriggerCount++;
            //Serial.print(F("Hall sensor triggered "));
            //Serial.print(hallSensorTriggerCount);
            //Serial.println(F(" time(s) during move to zero"));
        }
        sensorPreviouslyTriggered = sensorCurrentlyTriggered;
    }
    
    // Reset position tracking
    stepper.setCurrentPosition(0);
    currentPosition = 0;
    
    // Small delay to allow hall sensor state to stabilize
    delay(500);
    
    Serial.println(F("Special move to zero completed successfully"));
  } 
  // Normal case: use the existing movement code
  else {
    stepper.move(steps);
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }
    currentPosition += steps;
  }
}

// Define the loadConfigurations function
void loadConfigurations() {
  Serial.println(F("Loading configurations from EEPROM..."));

  // Load the home position address from EEPROM
  EEPROM.get(EEPROM_HOME_ADDRESS, homePositionAddress);
  Serial.print(F("Home Position Address       : "));
  Serial.println(homePositionAddress);

  // Load the number of index positions from EEPROM
  EEPROM.get(EEPROM_NUM_INDEX_POSITIONS, numIndexPositions);
  
  // Validate numIndexPositions
  if (numIndexPositions < 1 || numIndexPositions > MAX_INDEX_POSITIONS) {
    numIndexPositions = 1;  // Default to 1 if invalid
    EEPROM.put(EEPROM_NUM_INDEX_POSITIONS, numIndexPositions);
  }
  
  Serial.print(F("Number of Index Positions   : "));
  Serial.println(numIndexPositions);

  // Load the positions for magnet and non-magnet ends from EEPROM
  for (int i = 0; i < numIndexPositions; i++) {
    EEPROM.get(EEPROM_MAGNET_POSITIONS + i * sizeof(long), magnetEndPositions[i]);
    EEPROM.get(EEPROM_NON_MAGNET_POSITIONS + i * sizeof(long), nonMagnetEndPositions[i]);

    Serial.print(F("Magnet End Position     ["));
    Serial.print(i + 1);
    Serial.print(F("] : "));
    Serial.println(magnetEndPositions[i]);

    Serial.print(F("Non-Magnet End Position ["));
    Serial.print(i + 1);
    Serial.print(F("] : "));
    Serial.println(nonMagnetEndPositions[i]);
  }

  // Calculate index addresses based on home address
  for (int i = 0; i < numIndexPositions; i++) {
    magnetEndAddresses[i] = homePositionAddress + (2 * i) + 1;
    nonMagnetEndAddresses[i] = homePositionAddress + (2 * i) + 2;

    Serial.print(F("Magnet End Address      ["));
    Serial.print(i + 1);
    Serial.print(F("] : "));
    Serial.println(magnetEndAddresses[i]);

    Serial.print(F("Non-Magnet End Address  ["));
    Serial.print(i + 1);
    Serial.print(F("] : "));
    Serial.println(nonMagnetEndAddresses[i]);
  }
}

int getValidDccAddress() {
  int address = 0;
  while (true) {
    Serial.println(F("Enter a DCC address (1-511):"));
    address = getUserInputLong();
    if (address >= 1 && address <= 511) {
      return address;
    } else {
      Serial.println(F("Invalid address. Please enter a number between 1 and 511:"));
    }
  }
}

int getValidIndexNumber() {
  int number = 0;
  while (true) {
    Serial.print(F("Enter the number of index positions (1-"));
    Serial.print(MAX_INDEX_POSITIONS);
    Serial.println(F("):"));
    number = getUserInputLong();
    if (number >= 1 && number <= MAX_INDEX_POSITIONS) {
      return number;
    } else {
      Serial.print(F("Invalid number. Please enter a number between 1 and "));
      Serial.print(MAX_INDEX_POSITIONS);
      Serial.println(F(":"));
    }
  }
}

// Utility function to get user input as a String
String getUserInput() {
  while (Serial.available() == 0) {
    // Wait for user input
  }
  String input = Serial.readStringUntil('\n');
  input.trim(); // Remove any trailing newline or carriage return characters
  return input;
}

// Utility function to get user input as a long
long getUserInputLong() {
  String input = getUserInput();
  return input.toInt();
}

// Utility function to get user input as a yes/no response
bool getUserYesNo() {
  while (Serial.available() == 0) {
    // Wait for user input
  }
  String input = Serial.readStringUntil('\n');
  input.trim(); // Remove any trailing newline or carriage return characters
  return (input.equalsIgnoreCase("y"));
}

// Debugging Setup Code for Stepper Motor Turntable
// This code is included only when the 'debug' flag is true.

void debugSetup() {
    Serial.println(F("Starting setup in debug mode..."));

    // Get or set home DCC address
    Serial.println(F("First, let's configure the home position DCC address."));
    EEPROM.get(EEPROM_HOME_ADDRESS, homePositionAddress);
    
    Serial.print(F("Current home position address: "));
    Serial.println(homePositionAddress);
    Serial.println(F("Do you want to change this address? (y/n):"));
    
    if (getUserYesNo() || homePositionAddress < 1 || homePositionAddress > 511) {
        homePositionAddress = getValidDccAddress();
        EEPROM.put(EEPROM_HOME_ADDRESS, homePositionAddress);
        Serial.print(F("Home position address set to: "));
        Serial.println(homePositionAddress);
    }

    // Get or set number of index positions - ALWAYS ASK THE USER
    Serial.println(F("Now, let's configure the number of index positions."));
    EEPROM.get(EEPROM_NUM_INDEX_POSITIONS, numIndexPositions);
    
    Serial.print(F("Current number of index positions: "));
    Serial.println(numIndexPositions);
    Serial.println(F("Do you want to change the number of index positions? (y/n):"));
    
    if (getUserYesNo() || numIndexPositions < 1 || numIndexPositions > MAX_INDEX_POSITIONS) {
        numIndexPositions = getValidIndexNumber();
        EEPROM.put(EEPROM_NUM_INDEX_POSITIONS, numIndexPositions);
        Serial.print(F("Number of index positions set to: "));
        Serial.println(numIndexPositions);
    }

    // Calculate index addresses based on home address - addresses go in pairs
    Serial.println(F("Calculating DCC addresses for index positions..."));
    for (int i = 0; i < numIndexPositions; i++) {
        magnetEndAddresses[i] = homePositionAddress + (2 * i) + 1;
        nonMagnetEndAddresses[i] = homePositionAddress + (2 * i) + 2;
        
        Serial.print(F("Index "));
        Serial.print(i + 1);
        Serial.print(F(" - Magnet End Address: "));
        Serial.print(magnetEndAddresses[i]);
        Serial.print(F(", Non-Magnet End Address: "));
        Serial.println(nonMagnetEndAddresses[i]);
    }

    // Iteratively set positions for each index
    Serial.println(F("\nNow let's setup the physical positions for each index."));
    Serial.println(F("Setting up magnet end positions..."));
    
    for (int i = 0; i < numIndexPositions; i++) {
        bool positionAccepted = false;
        while (!positionAccepted) {
            // Load existing positions
            EEPROM.get(EEPROM_MAGNET_POSITIONS + i * sizeof(long), magnetEndPositions[i]);

            Serial.print(F("\nIndex "));
            Serial.print(i + 1);
            Serial.println(F(" Magnet End:"));

            Serial.print(F("Current Position (steps): "));
            Serial.println(magnetEndPositions[i]);
            Serial.println(F("Do you want to modify this position? (y/n): "));
            
            if (getUserYesNo()) {
                Serial.println(F("Enter new Magnet End Position (steps): "));
                long newPosition = getUserInputLong();
                EEPROM.put(EEPROM_MAGNET_POSITIONS + i * sizeof(long), newPosition);
                magnetEndPositions[i] = newPosition;
                Serial.println(F("Position updated."));
                moveRelative(magnetEndPositions[i] - currentPosition); // Move the stepper to the new position

                // Ask for user confirmation for the new position
                Serial.println(F("Do you accept this new position? (y/n): "));
                positionAccepted = getUserYesNo();
                if (!positionAccepted) {
                    Serial.println(F("Re-enter the position..."));
                }
            } else {
                Serial.println(F("Position accepted without changes."));
                positionAccepted = true;
            }
        }
    }

    Serial.println(F("\nSetting up non-magnet end positions..."));
    for (int i = 0; i < numIndexPositions; i++) {
        bool positionAccepted = false;
        while (!positionAccepted) {
            // Load existing positions
            EEPROM.get(EEPROM_NON_MAGNET_POSITIONS + i * sizeof(long), nonMagnetEndPositions[i]);

            Serial.print(F("\nIndex "));
            Serial.print(i + 1);
            Serial.println(F(" Non-Magnet End:"));

            Serial.print(F("Current Position (steps): "));
            Serial.println(nonMagnetEndPositions[i]);
            Serial.println(F("Do you want to modify this position? (y/n): "));
            
            if (getUserYesNo()) {
                Serial.println(F("Enter new Non-Magnet End Position (steps): "));
                long newPosition = getUserInputLong();
                EEPROM.put(EEPROM_NON_MAGNET_POSITIONS + i * sizeof(long), newPosition);
                nonMagnetEndPositions[i] = newPosition;
                Serial.println(F("Position updated."));
                moveRelative(nonMagnetEndPositions[i] - currentPosition); // Move the stepper to the new position

                // Ask for user confirmation for the new position
                Serial.println(F("Do you accept this new position? (y/n): "));
                positionAccepted = getUserYesNo();
                if (!positionAccepted) {
                    Serial.println(F("Re-enter the position..."));
                }
            } else {
                Serial.println(F("Position accepted without changes."));
                positionAccepted = true;
            }
        }
    }

    // Summary of configurations
    Serial.println(F("\n==== Configuration Summary ===="));
    Serial.print(F("Home Position Address: "));
    Serial.println(homePositionAddress);
    Serial.print(F("Number of Index Positions: "));
    Serial.println(numIndexPositions);
    
    for (int i = 0; i < numIndexPositions; i++) {
        Serial.print(F("\nIndex "));
        Serial.print(i + 1);
        Serial.println(F(":"));
        
        Serial.print(F("  Magnet End - Address: "));
        Serial.print(magnetEndAddresses[i]);
        Serial.print(F(", Position: "));
        Serial.println(magnetEndPositions[i]);
        
        Serial.print(F("  Non-Magnet End - Address: "));
        Serial.print(nonMagnetEndAddresses[i]);
        Serial.print(F(", Position: "));
        Serial.println(nonMagnetEndPositions[i]);
    }

    Serial.println(F("\nSetup completed successfully."));
}

// Function to calculate the shortest step difference between current and next positions
int calculateShortestStepDifference(long currentPosition, long nextPosition) {
  // For regular moves not involving home position
  if (nextPosition != 0) {
    long stepDifference = nextPosition - currentPosition;
    
    // If the absolute difference is greater than half a revolution,
    // then we should go the other way around
    if (abs(stepDifference) > stepsPerRevolution / 2) {
      if (stepDifference > 0) {
        return stepDifference - stepsPerRevolution;
      } else {
        return stepDifference + stepsPerRevolution;
      }
    }
    
    return stepDifference;
  }
  
  // For moves to home position (zero)
  else {
    return -currentPosition;
  }
}
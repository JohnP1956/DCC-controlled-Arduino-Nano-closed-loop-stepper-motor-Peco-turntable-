#ifndef CONFIG_H
#define CONFIG_H

// Define stepper motor interface type
#define MotorInterfaceType 1

// Pin connections
const int stepPin = 10;         // Step signal to stepper driver
const int dirPin = 9;           // Direction signal to stepper driver
const int enPin = 11;           // Enable signal to stepper driver
const int hallPin = 5;          // Hall effect sensor input
const int DCC_PIN = 2;          // DCC input via optocoupler

// Stepper motor settings
const int stepsPerRevolution = 6400; // 1.8-degree motor with 32x microstepping
const float maxSpeed = 142.22;       // Max speed (steps per second) for 45 seconds per revolution
const float acceleration = 16.0;    // Acceleration (steps per second squared)

// Maximum number of index positions
const int MAX_INDEX_POSITIONS = 12;

// EEPROM storage addresses
const int EEPROM_START_ADDRESS = 100;
const int EEPROM_HOME_ADDRESS = EEPROM_START_ADDRESS;
const int EEPROM_NUM_INDEX_POSITIONS = EEPROM_HOME_ADDRESS + sizeof(int);
const int EEPROM_MAGNET_POSITIONS = EEPROM_NUM_INDEX_POSITIONS + sizeof(int);
const int EEPROM_NON_MAGNET_POSITIONS = EEPROM_MAGNET_POSITIONS + sizeof(long) * MAX_INDEX_POSITIONS;

// Homing direction (true for counter-clockwise, false for clockwise)
const bool homingDirection = true; // True for CCW. Check motor default direction if movement is opposite.

// Debug flag
const bool debug = false; // Set to true for debugging

#endif // CONFIG_H
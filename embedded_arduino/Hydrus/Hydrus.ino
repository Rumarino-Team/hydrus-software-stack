// This will be the main arduino sketch to put in the Hydrus microcontroller.

#include <Arduino.h>
#include "devices.h" // Include devices.h for thrusterArr access

// For print timing
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 5000; // 5 seconds

// Serial communication variables
String inputString = "";
bool stringComplete = false;

void setup() 
{
  // Initialize Serial communication for debugging
  Serial.begin(115200);
  Serial.println("Hydrus Arduino starting up...");
  
  // Reserve memory for the input string
  inputString.reserve(50);  // Prevent dynamic allocation
  
  // Initialize thrusters
  initializeThrustersArduino();
  
  Serial.println("Initialization complete");
  lastPrintTime = millis(); // Initialize the timer
}

void loop() 
{
  // Process any available serial commands
  processSerialCommands();
  
  // Print pin values every 5 seconds
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= printInterval) {
    // Reset timer
    lastPrintTime = currentTime;
    
    // Print header
    Serial.println("\n=== THRUSTER PIN VALUES ===");
    Serial.println("Thruster | Pin | PWM Value | Direction");
    Serial.println("------------------------------------");
    
    // Loop through all thrusters and print their values
    for (int i = 0; i < MOTOR_NUM; i++) {
      Serial.print(i + 1);
      Serial.print(" | ");
      Serial.print(thrusterArr[i].pin);
      Serial.print(" | ");
      Serial.print(thrusterArr[i].motor.readMicroseconds());
      Serial.print(" | ");
      Serial.println(thrusterArr[i].forward ? "Forward" : "Backward");
    }
    
    Serial.println("============================\n");
  }
}

// Serial event handler - called when new serial data arrives
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

// Process commands received from serial
void processSerialCommands() {
  if (stringComplete) {
    // Serial.print("Received command: ");
    // Serial.println(inputString);
    
    // Command format: T1:3 (Thruster 1, value 3)
    // D:-2 (Depth motors, value -2)
    // P:1 (Torpedo, value 1)
    // C:45 (Camera motor, angle 45)
    
    if (inputString.length() >= 3) {
      char cmdType = inputString.charAt(0);
      int colonPos = inputString.indexOf(':');
      
      if (colonPos > 0) {
        int value = 0;
        
        // Parse the value part
        String valueStr = inputString.substring(colonPos + 1);
        value = valueStr.toInt();
        
        // Process command type
        switch (cmdType) {
          case 'T': // Thruster command
            {
              // Get thruster number
              int thrusterNum = inputString.substring(1, colonPos).toInt();
              
              // Call the appropriate thruster function
              if (thrusterNum == 1) {
                setThruster_1(value);
              } else if (thrusterNum == 2) {
                setThruster_2(value);
              } else if (thrusterNum == 3) {
                setThruster_3(value);
              } else if (thrusterNum == 4) {
                setThruster_4(value);
              } else {
                Serial.println("Invalid thruster number!");
              }
            }
            break;
          
          case 'D': // Depth command
            setDepth(value);
            break;
            
          case 'P': // Torpedo/propulsion command
            launchTorpedo(value);
            break;
            
          case 'C': // Camera command
            setCameraMotor(value);
            break;
            
          default:
            Serial.println("Unknown command type!");
            break;
        }
      } else {
        Serial.println("Invalid command format! Use format: T1:3, D:-2, etc.");
      }
    }
    
    // Reset for next command
    inputString = "";
    stringComplete = false;
  }
}

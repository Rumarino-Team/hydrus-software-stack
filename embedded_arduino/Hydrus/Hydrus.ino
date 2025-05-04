// This will be the main arduino sketch to put in the  Hydrus microcontroller.

# pragma once
#include <Arduino.h>

# include "ros_embedded_node.h"

void setup() 
{
  // Initialize Serial communication for debugging
  Serial.begin(115200);
  Serial.println("Hydrus Arduino starting up...");
  
  initRosNode();
  initializeThrustersArduino();
  
  Serial.println("Initialization complete");
}

void loop() 
{
  // publishCurrentDepth();
  runRosNode();
  
  // Periodic status message (uncomment if needed)
  // static unsigned long lastPrint = 0;
  // if (millis() - lastPrint > 5000) { // Print every 5 seconds
  //   Serial.println("Hydrus running...");
  //   lastPrint = millis();
  // }
}

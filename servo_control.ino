#include <Servo.h>

// Define variables
int angle;
Servo servo1;  // Create a servo object for servo motor 1
Servo servo2;  // Create a servo object for servo motor 2

void setup() {
  servo1.attach(11);  // Attach servo motor 1 to pin 11
  servo2.attach(9);   // Attach servo motor 2 to pin 9
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  // Check if there is data available to read from serial
  while (Serial.available() > 0) {
    // Read the incoming data until newline character ('\n')
    String incomingData = Serial.readStringUntil('\n');
    // Find the index of comma (',') in the incoming data
    int commaIndex = incomingData.indexOf(',');
    // If comma is found in the incoming data
    if (commaIndex != -1) {
      // Extract the first part of the data and convert it to integer
      int value1 = incomingData.substring(0, commaIndex).toInt();
      // Extract the second part of the data and convert it to integer
      int value2 = incomingData.substring(commaIndex + 1).toInt();
      
      // Write the first value to servo motor 1
      servo1.write(value1);
      // Write the second value to servo motor 2
      servo2.write(value2);
    }
  }
}

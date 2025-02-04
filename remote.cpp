#include <Arduino.h>
#include <BluetoothSerial.h>

// Create an instance of BluetoothSerial
BluetoothSerial ESP_BT;

// Define the Bluetooth commands (Example: S for speed, A for angle, B for braking)
#define CMD_SPEED 'S'    // Speed control command
#define CMD_ANGLE 'A'    // Steering angle control command
#define CMD_BRAKING 'B'  // Braking control command
#define CMD_MODE 'M'     // Mode switch command (RC or Cruise)

// Mode variables
bool isRemoteControlMode = true;  // Default to Remote Control Mode

// Variable to hold control values
int speed = 0;        // Speed (range: -100 to 100)
int angle = 90;       // Angle (range: 0 to 180)
float braking = 0.0f; // Braking force (range: 0.0 to 1.0)

// Setup Bluetooth and Serial communication
void setup() {
  Serial.begin(115200);          // For debugging
  ESP_BT.begin("ESP32_RC_Car");  // Start Bluetooth with a custom name
  Serial.println("Bluetooth Ready");
  Serial.println("Please connect to ESP32 Bluetooth with your app.");
}

// Read and process Bluetooth commands
void loop() {
  if (ESP_BT.available()) {
    char receivedChar = ESP_BT.read();  // Read incoming data from Bluetooth

    // Debugging: print received character
    Serial.print("Received: ");
    Serial.println(receivedChar);

    // Process commands
    if (receivedChar == CMD_SPEED) {
      String speedStr = ESP_BT.readStringUntil(';');  // Read speed value until ';'
      speed = speedStr.toInt();  // Convert string to integer
      Serial.print("Speed Set To: ");
      Serial.println(speed);
      
      // In Remote Control Mode, send the speed to STM32
      if (isRemoteControlMode) {
        // Send the speed value to STM32 (e.g., sendSpeedToSTM32(speed));
      }
    } 
    else if (receivedChar == CMD_ANGLE) {
      String angleStr = ESP_BT.readStringUntil(';');  // Read angle value until ';'
      angle = angleStr.toInt();  // Convert string to integer
      Serial.print("Angle Set To: ");
      Serial.println(angle);
      
      // In Remote Control Mode, send the angle to STM32
      if (isRemoteControlMode) {
        // Send the angle value to STM32 (e.g., sendAngleToSTM32(angle));
      }
    } 
    else if (receivedChar == CMD_BRAKING) {
      String brakingStr = ESP_BT.readStringUntil(';');  // Read braking value until ';'
      braking = brakingStr.toFloat();  // Convert string to float
      Serial.print("Braking Set To: ");
      Serial.println(braking);
      
      // In Remote Control Mode, send the braking value to STM32
      if (isRemoteControlMode) {
        // Send the braking value to STM32 (e.g., sendBrakingToSTM32(braking));
      }
    } 
    else if (receivedChar == CMD_MODE) {
      String modeStr = ESP_BT.readStringUntil(';');  // Read mode value until ';'
      if (modeStr == "RC") {
        isRemoteControlMode = true;
        Serial.println("Switched to Remote Control Mode");
      }
      else if (modeStr == "CRUISE") {
        isRemoteControlMode = false;
        Serial.println("Switched to Cruise Control Mode");
      }
    }
  }

  // In Cruise Control Mode, send pre-set speed and steering (example)
  if (!isRemoteControlMode) {
    // Set a fixed speed for cruise control (e.g., speed = 50)
    speed = 50;  // Example cruise control speed
    // Set a fixed angle for cruise control (e.g., angle = 90)
    angle = 90;  // Example cruise control steering angle

    // Send these values to STM32 (for motor and steering control)
    // e.g., sendSpeedToSTM32(speed);
    // e.g., sendAngleToSTM32(angle);

    Serial.print("Cruise Control - Speed: ");
    Serial.print(speed);
    Serial.print(" Angle: ");
    Serial.println(angle);
  }

  // Add any other logic for sending data to STM32, e.g., via Serial or SPI
}

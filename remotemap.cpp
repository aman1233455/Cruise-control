#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

int speed = 0;
int angle = 90;
float braking = 0.0f;
bool isRemoteControl = true;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Car_Controller");
  Serial.println("Car Controller Ready");
}

void loop() {
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil(';');
    
    if (command.startsWith("S")) {
      speed = command.substring(1).toInt();
      Serial.print("Speed set to: ");
      Serial.println(speed);
    } 
    
    else if (command.startsWith("A")) {
      angle = command.substring(1).toInt();
      Serial.print("Angle set to: ");
      Serial.println(angle);
    } 
    
    else if (command.startsWith("B")) {
      braking = command.substring(1).toFloat();
      Serial.print("Braking force set to: ");
      Serial.println(braking);
    } 
    
    else if (command.startsWith("M")) {
      String mode = command.substring(2);
      
      if (mode == "RC") {
        isRemoteControl = true;
        Serial.println("Switched to Remote Control Mode");
      } 
      else if (mode == "CRUISE") {
        isRemoteControl = false;
        Serial.println("Switched to Cruise Control Mode");
      }
    }
  }
  
  if (isRemoteControl) {
    // Add motor control for remote control mode
  } else {
    // Add autonomous control for cruise control mode
  }
  
  delay(100);
}

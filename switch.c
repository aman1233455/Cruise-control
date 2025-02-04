#include "main.h"
#include "stdio.h"

// Define the two modes
#define REMOTE_CONTROL_MODE 0
#define CRUISE_CONTROL_MODE 1

// Define UART handle
extern UART_HandleTypeDef huart1;  // Assuming UART1 for communication

// Control Mode variable (0 for Remote Control, 1 for Cruise Control)
uint8_t controlMode = REMOTE_CONTROL_MODE;  // Default to Remote Control Mode

// Control variables
int speed = 0;        // Speed (range: -100 to 100)
int angle = 90;       // Angle (range: 0 to 180)
float braking = 0.0f; // Braking (range: 0.0 to 1.0)

// Function to set the car's speed, angle, and braking (Placeholder)
void setCarControl(int speed, int angle, float braking) {
    // Code to adjust motors and actuators here
    // For example:
    // Set motor speed via PWM, adjust steering angle via servo
    // Implement PWM or motor driver control for speed and steering
}

// Function to handle the mode switch via UART
void handleModeSwitch(char* mode) {
    if (strcmp(mode, "RC") == 0) {
        controlMode = REMOTE_CONTROL_MODE; // Switch to Remote Control Mode
        HAL_UART_Transmit(&huart1, (uint8_t*)"Switched to Remote Control Mode\r\n", 33, 100);
    } else if (strcmp(mode, "CRUISE") == 0) {
        controlMode = CRUISE_CONTROL_MODE; // Switch to Cruise Control Mode
        HAL_UART_Transmit(&huart1, (uint8_t*)"Switched to Cruise Control Mode\r\n", 33, 100);
    }
}

// Function to parse incoming UART data for control commands
void parseUARTCommand(char* command) {
    // Check if the command is for switching modes
    if (command[0] == 'M') {
        handleModeSwitch(&command[2]); // Skip the 'M' and handle the rest (e.g., RC or CRUISE)
    } else if (command[0] == 'S') {
        speed = atoi(&command[1]);  // Update speed from the command
    } else if (command[0] == 'A') {
        angle = atoi(&command[1]);  // Update steering angle from the command
    } else if (command[0] == 'B') {
        braking = atof(&command[1]); // Update braking force from the command
    }
}

// UART receive callback for receiving data from ESP32
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    static char uartBuffer[100];  // Buffer to store incoming UART data
    static uint8_t uartIndex = 0;

    // Store received byte in buffer
    uartBuffer[uartIndex++] = huart->Instance->DR;

    // Check if end of command (e.g., newline or semicolon)
    if (uartBuffer[uartIndex - 1] == '\n' || uartBuffer[uartIndex - 1] == ';') {
        uartBuffer[uartIndex - 1] = '\0'; // Null-terminate the command string
        parseUARTCommand(uartBuffer);      // Parse the command
        uartIndex = 0;                    // Reset buffer index
    }

    // Re-enable UART receive interrupt
    HAL_UART_Receive_IT(&huart1, (uint8_t*)uartBuffer + uartIndex, 1);
}

// Main loop
int main(void) {
    HAL_Init();  // HAL initialization
    HAL_UART_Init(&huart1); // Initialize UART1

    // Enable UART receive interrupt
    HAL_UART_Receive_IT(&huart1, (uint8_t*)uartBuffer, 1);

    while (1) {
        if (controlMode == REMOTE_CONTROL_MODE) {
            // Process remote control commands
            // Send speed, angle, braking values to motor control
            setCarControl(speed, angle, braking);
        } else if (controlMode == CRUISE_CONTROL_MODE) {
            // Set fixed values for cruise control mode (or AI-based logic)
            // Example: Cruise control with speed = 50, angle = 90, no braking
            setCarControl(50, 90, 0.0f); // Autonomous cruise control with pre-set values
        }

        // Add delays or other operations as needed for control updates
        HAL_Delay(50);
    }
}

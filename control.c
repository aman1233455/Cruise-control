#include "main.h"
#include "cmsis_os.h"  // For FreeRTOS
#include "math.h"
#include "KalmanFilter.h"
#include "FuzzyLogic.h"

// Motor Control Pins
#define MOTOR_PWM_Pin GPIO_PIN_6
#define MOTOR_PWM_GPIO_Port GPIOA

// Sensor Pins
#define ULTRASONIC_TRIGGER_Pin GPIO_PIN_0
#define ULTRASONIC_ECHO_Pin GPIO_PIN_1
#define IR_SENSOR_Pin GPIO_PIN_2

// FreeRTOS Handles
osThreadId motorControlTaskHandle;
osThreadId aiProcessingTaskHandle;

// Global variables
volatile float currentSpeed = 0.0f;
volatile float distanceToObstacle = 100.0f;  // Default distance
volatile float desiredSpeed = 50.0f;  // Default desired speed

// Kalman Filter objects
KalmanFilter speedKalmanFilter;

// Fuzzy Logic object
FuzzyLogic brakingFuzzyLogic;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_NVIC_Init(void);

// Function prototypes
void StartMotorControlTask(void const * argument);
void StartAIProcessingTask(void const * argument);
void UltrasonicSensor_Read(void);
void IR_Sensor_Read(void);
void PWM_SetSpeed(float speed);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_USART2_UART_Init();
    MX_NVIC_Init();

    // Initialize Kalman filter and fuzzy logic
    KalmanFilter_Init(&speedKalmanFilter);
    FuzzyLogic_Init(&brakingFuzzyLogic);

    // Create FreeRTOS tasks
    osThreadDef(motorControlTask, StartMotorControlTask, osPriorityNormal, 0, 128);
    motorControlTaskHandle = osThreadCreate(osThread(motorControlTask), NULL);

    osThreadDef(aiProcessingTask, StartAIProcessingTask, osPriorityNormal, 0, 128);
    aiProcessingTaskHandle = osThreadCreate(osThread(aiProcessingTask), NULL);

    // Start scheduler
    osKernelStart();

    // Main loop (if scheduler fails)
    while (1)
    {
        osDelay(1000);
    }
}

// Motor Control Task (PWM Control)
void StartMotorControlTask(void const * argument)
{
    for(;;)
    {
        // Set PWM duty cycle based on desired speed
        PWM_SetSpeed(desiredSpeed);

        // Delay 100ms (control loop)
        osDelay(100);
    }
}

// AI Processing Task (AI-based Braking & Cruise Control)
void StartAIProcessingTask(void const * argument)
{
    for(;;)
    {
        // Read sensors (Ultrasonic and IR)
        UltrasonicSensor_Read();
        IR_Sensor_Read();

        // Kalman filter for speed estimation
        float estimatedSpeed = KalmanFilter_Update(&speedKalmanFilter, currentSpeed);

        // Apply Fuzzy Logic for braking decision
        float brakingForce = FuzzyLogic_CalculateBrakingForce(&brakingFuzzyLogic, distanceToObstacle, estimatedSpeed);

        // Adjust braking force
        if (distanceToObstacle < 20.0f)  // Critical distance threshold
        {
            // Apply maximum braking if too close
            desiredSpeed = 0.0f;  // Full stop
        }
        else if (distanceToObstacle < 50.0f)  // Apply gradual braking if obstacle is near
        {
            desiredSpeed -= brakingForce;  // Gradual decrease in speed
        }
        else
        {
            // Cruise control (Maintain desired speed)
            desiredSpeed = 50.0f;  // Maintain cruise control speed
        }

        // Delay 200ms (AI decision loop)
        osDelay(200);
    }
}

// Ultrasonic Sensor Reading (Dummy for now, implement actual)
void UltrasonicSensor_Read(void)
{
    // For demonstration: Replace with actual sensor readings
    distanceToObstacle = 30.0f;  // Dummy value (change based on actual reading)
}

// IR Sensor Reading (Dummy for now, implement actual)
void IR_Sensor_Read(void)
{
    // For demonstration: Replace with actual IR sensor logic
    // Example: If IR sensor detects an obstacle, trigger braking
    if (HAL_GPIO_ReadPin(GPIOB, IR_SENSOR_Pin) == GPIO_PIN_SET)
    {
        distanceToObstacle = 10.0f;  // Simulate obstacle detection
    }
    else
    {
        distanceToObstacle = 100.0f;  // No obstacle
    }
}

// PWM Control for Motor Speed (Dummy for now, implement actual)
void PWM_SetSpeed(float speed)
{
    // Convert speed to PWM duty cycle (0-100%)
    uint16_t pwmValue = (uint16_t)(speed * 10);  // Assuming 0-100 scale
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwmValue);  // Set PWM for motor control
}

// System Clock Configuration (Example for STM32F4)
void SystemClock_Config(void)
{
    // System Clock Configuration goes here
}

// GPIO Initialization (Example for STM32F4)
static void MX_GPIO_Init(void)
{
    // GPIO Initialization for Ultrasonic and IR sensors
}

// TIM2 PWM Initialization (For motor control)
static void MX_TIM2_Init(void)
{
    // PWM Timer setup for motor control
}

// UART Initialization (For Debugging)
static void MX_USART2_UART_Init(void)
{
    // UART configuration for serial debugging (optional)
}

// NVIC Initialization (Interrupts)
static void MX_NVIC_Init(void)
{
    // NVIC interrupt setup for real-time sensor readings (optional)
}


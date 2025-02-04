#include "main.h"

// PWM configuration for speed motor (DC motor)
extern TIM_HandleTypeDef htim2;   // Timer for speed motor control (TIM2)
extern TIM_HandleTypeDef htim1;   // Timer for servo motor control (TIM1)

// Constants for servo control
#define MIN_SERVO_PULSE_WIDTH 500    // Minimum PWM pulse width (for 0 degrees)
#define MAX_SERVO_PULSE_WIDTH 2500   // Maximum PWM pulse width (for 180 degrees)
#define SERVO_PWM_FREQ 50           // Frequency for servo PWM (50Hz)

// Constants for speed control (adjust based on your motor)
#define MAX_SPEED_PWM 1000  // Maximum PWM for full speed (adjust accordingly)
#define MIN_SPEED_PWM 100   // Minimum PWM for slow speed

// Function to initialize PWM for motor and servo control
void PWM_Init(void)
{
    // Initialize the PWM for motor control (speed motor)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Assuming TIM2 is connected to the speed motor

    // Initialize the PWM for steering (servo motor)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Assuming TIM1 is connected to the servo motor
}

// Function to set the speed of the motor (forward or backward)
void Set_Speed(float speed)
{
    // Ensure the speed is within the valid range
    if (speed > 100)
        speed = 100;
    else if (speed < -100)
        speed = -100;

    // Convert speed (-100 to 100) into a PWM value (MIN_SPEED_PWM to MAX_SPEED_PWM)
    uint16_t pwm_value = (uint16_t)((speed + 100) * (MAX_SPEED_PWM - MIN_SPEED_PWM) / 200 + MIN_SPEED_PWM);
    
    // Set the PWM duty cycle for the speed motor
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_value);
}

// Function to set the servo position (steering angle)
void Set_Servo_Angle(float angle)
{
    // Ensure the angle is within the valid range (0 to 180 degrees)
    if (angle < 0)
        angle = 0;
    else if (angle > 180)
        angle = 180;

    // Convert angle (0 to 180 degrees) into PWM pulse width (MIN_SERVO_PULSE_WIDTH to MAX_SERVO_PULSE_WIDTH)
    uint16_t pwm_value = (uint16_t)((angle / 180.0f) * (MAX_SERVO_PULSE_WIDTH - MIN_SERVO_PULSE_WIDTH) + MIN_SERVO_PULSE_WIDTH);

    // Set the PWM pulse width for the servo motor
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);
}

// Function to apply braking force by reducing motor speed
void Apply_Braking(float braking_force)
{
    // Ensure the braking force is within the valid range (0 to 1)
    if (braking_force < 0)
        braking_force = 0;
    else if (braking_force > 1)
        braking_force = 1;

    // Convert braking force (0 to 1) into a PWM value (MIN_SPEED_PWM to MAX_SPEED_PWM)
    uint16_t pwm_value = (uint16_t)((1 - braking_force) * (MAX_SPEED_PWM - MIN_SPEED_PWM) + MIN_SPEED_PWM);

    // Apply the braking by reducing the PWM duty cycle for the speed motor
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_value);
}

// Main function that controls the RC car
int main(void)
{
    // HAL Initialization
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();  // Initialize TIM1 for servo control
    MX_TIM2_Init();  // Initialize TIM2 for speed motor control

    // Initialize PWM
    PWM_Init();

    // Example of controlling the car based on AI decision-making
    while (1)
    {
        // Assume the AI model provides these values (braking force, speed adjustment, and steering angle)
        float speed = 50.0f;  // Example: Speed adjustment by AI (from -100 to 100)
        float steering_angle = 30.0f;  // Example: Steering angle by AI (from 0 to 180 degrees)
        float braking_force = 0.5f;  // Example: Braking force by AI (from 0 to 1)

        // Control the speed of the car
        Set_Speed(speed);

        // Control the steering (servo motor)
        Set_Servo_Angle(steering_angle);

        // Apply braking based on AI decision
        Apply_Braking(braking_force);

        // Add a delay or wait for new data
        HAL_Delay(100);
    }
}

// PWM Timer Initialization for Servo (TIM1)
void MX_TIM1_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 20000 - 1;  // 20ms period (50Hz)
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim1);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1000;  // Default servo pulse width (1.5ms = 90 degrees)
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
}

// PWM Timer Initialization for Speed Motor (TIM2)
void MX_TIM2_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 20000 - 1;  // 20ms period for motor control
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim2);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1500;  // Default PWM value for 0 speed
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
}

/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ========== ENUMERATIONS ==========
typedef enum {
  HOMING_IDLE,
  HOMING_IN_PROGRESS,
  HOMING_COMPLETE,
  HOMING_ERROR
} HomingState_t;

typedef enum {
  MOTOR_SPARK_MAX_PWM,
  MOTOR_H_BRIDGE,
  MOTOR_STEPPER
} MotorType_t;

typedef enum {
  MOTOR_MODE_IDLE,
  MOTOR_MODE_VELOCITY,
  MOTOR_MODE_POSITION,
  MOTOR_MODE_MANUAL
} MotorMode_t;

// ========== STRUCTURES ==========
typedef struct {
  int32_t velocity;
  int64_t position;
  int64_t home_position;
  uint32_t last_counter;
  TIM_HandleTypeDef *htim;

  // Index pulse homing parameters (ENC I)
  GPIO_TypeDef *index_port;
  uint16_t index_pin;
  HomingState_t homing_state;
  uint32_t homing_start_time;
  uint32_t homing_timeout_ms;

  // Index pulse detection
  uint8_t is_homed;
  uint8_t index_detected;
  uint8_t last_index_state;
  uint32_t index_pulse_count;

  char name[16];
  uint8_t enabled;
} Encoder_t;

typedef struct {
  MotorType_t type;
  MotorMode_t mode;

  // PWM Control
  TIM_HandleTypeDef *pwm_tim;
  uint32_t pwm_channel;

  // Direction Control (H-Bridge & Stepper)
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;

  // Enable Control (Stepper only)
  GPIO_TypeDef *enable_port;
  uint16_t enable_pin;

  // Control Parameters
  int32_t target_position;
  int16_t target_velocity;    // -1000 to +1000 (scaled)
  int16_t current_output;     // Current motor output

  // Position Control (PID)
  float kp, ki, kd;
  float error_sum;
  int32_t last_error;

  // Safety Limits
  int32_t min_position;
  int32_t max_position;
  int16_t max_velocity;

  // Stepper specific
  uint32_t step_count;
  uint32_t target_steps;
  uint16_t step_frequency;
  uint8_t is_moving;

  uint8_t enabled;
  char name[16];
} Motor_t;

// ========== GLOBAL ARRAYS ==========
Motor_t motors[6] = {
  // Base - NEO via Spark Max (TIM5_CH1, PA0)
  {MOTOR_SPARK_MAX_PWM, MOTOR_MODE_IDLE, &htim5, TIM_CHANNEL_1, NULL, 0, NULL, 0,
   0, 0, 0, 2.0f, 0.1f, 0.5f, 0, 0, -100000, 100000, 800, 0, 0, 1000, 0, 1, "Base"},

  // Shoulder - NEO via Spark Max (TIM5_CH3, PA2)
  {MOTOR_SPARK_MAX_PWM, MOTOR_MODE_IDLE, &htim5, TIM_CHANNEL_3, NULL, 0, NULL, 0,
   0, 0, 0, 2.0f, 0.1f, 0.5f, 0, 0, -80000, 80000, 800, 0, 0, 1000, 0, 1, "Shoulder"},

  // Elbow - NEO via Spark Max (TIM5_CH4, PA3)
  {MOTOR_SPARK_MAX_PWM, MOTOR_MODE_IDLE, &htim5, TIM_CHANNEL_4, NULL, 0, NULL, 0,
   0, 0, 0, 2.0f, 0.1f, 0.5f, 0, 0, -90000, 90000, 800, 0, 0, 1000, 0, 1, "Elbow"},

  // Wrist1 - Grimson via H-Bridge (TIM9_CH1, PE5 + PC5 direction)
  {MOTOR_H_BRIDGE, MOTOR_MODE_IDLE, &htim9, TIM_CHANNEL_1, GPIOC, GPIO_PIN_5, NULL, 0,
   0, 0, 0, 1.5f, 0.05f, 0.3f, 0, 0, -50000, 50000, 600, 0, 0, 1000, 0, 1, "Wrist1"},

  // Wrist2 - Grimson via H-Bridge (TIM9_CH2, PE6 + PB0 direction)
  {MOTOR_H_BRIDGE, MOTOR_MODE_IDLE, &htim9, TIM_CHANNEL_2, GPIOB, GPIO_PIN_0, NULL, 0,
   0, 0, 0, 1.5f, 0.05f, 0.3f, 0, 0, -50000, 50000, 600, 0, 0, 1000, 0, 1, "Wrist2"},

  // End Effector - Stepper (TIM11_CH1, PB9 + PB2 direction + PB1 enable)
  {MOTOR_STEPPER, MOTOR_MODE_IDLE, &htim11, TIM_CHANNEL_1, GPIOB, GPIO_PIN_2, GPIOB, GPIO_PIN_1,
   0, 0, 0, 0, 0, 0, 0, 0, -10000, 10000, 200, 0, 0, 1000, 0, 1, "Stepper"}
};

Encoder_t encoders[5] = {
  {0, 0, 0, 0, &htim1, GPIOC, GPIO_PIN_0, HOMING_IDLE, 0, 10000, 0, 0, 0, 0, "Base", 1},
  {0, 0, 0, 0, &htim2, GPIOC, GPIO_PIN_1, HOMING_IDLE, 0, 10000, 0, 0, 0, 0, "Shoulder", 1},
  {0, 0, 0, 0, &htim3, GPIOC, GPIO_PIN_2, HOMING_IDLE, 0, 10000, 0, 0, 0, 0, "Elbow", 1},
  {0, 0, 0, 0, &htim4, NULL, 0, HOMING_IDLE, 0, 10000, 0, 0, 0, 0, "Wrist1", 1},
  {0, 0, 0, 0, &htim8, NULL, 0, HOMING_IDLE, 0, 10000, 0, 0, 0, 0, "Wrist2", 1}
};

// ========== GLOBAL VARIABLES ==========
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

uint32_t last_encoder_send = 0;
uint32_t last_status_send = 0;
uint32_t last_debug_send = 0;
uint32_t last_motor_update = 0;
uint32_t last_motor_status_send = 0;

// Add missing global variables
uint32_t last_watchdog_reset = 0;
uint8_t system_initialized = 0;
uint8_t can_error_count = 0;
uint8_t emergency_stop_active = 0;

// ========== FUNCTION PROTOTYPES ==========
void SystemInit_RobotArm(void);
void MainControlLoop(void);
void InitializeMotors(void);
void InitializeEncoders(void);
void InitializeCAN(void);
void SetMotorOutput(uint8_t motor_id, int16_t output);
int32_t CalculatePID(Motor_t *motor, int32_t current_position);
void UpdateMotor(uint8_t motor_id);
void SetMotorVelocity(uint8_t motor_id, int16_t velocity);
void SetMotorPosition(uint8_t motor_id, int32_t position);
void SetStepperFrequency(uint16_t frequency_hz);
void MoveStepperSteps(int32_t steps, uint16_t frequency);
void EmergencyStopAllMotors(void);
void ClearEmergencyStop(void);
void SendMotorStatus(void);
void UpdateEncoder(Encoder_t *enc);
void UpdateIndexPulse(Encoder_t *enc);
void StartHoming(uint8_t encoder_id);
void ProcessHoming(uint8_t encoder_id);
void SendEncoderData(uint8_t encoder_id, Encoder_t *enc);
void SendSystemStatus(void);
void SendDebugInfo(void);

// New debug functions for Motor 0
void TestMotor0PWM(void);
void DebugMotor0(void);
void VerifyTIM5_Config(void);
void Force_TIM5_Init(void);

// ========== MOTOR CONTROL FUNCTIONS ==========
void InitializeMotors(void) {
    // Force proper TIM5 initialization first
    Force_TIM5_Init();

    // Start PWM timers for motor control
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);  // Base NEO
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);  // Shoulder NEO
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);  // Elbow NEO
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);  // Wrist1 Grimson
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);  // Wrist2 Grimson
    HAL_TIM_PWM_Start_IT(&htim11, TIM_CHANNEL_1); // Stepper with interrupt for step counting

    // Initialize all motors to safe states
    for(int i = 0; i < 6; i++) {
        SetMotorOutput(i, 0);
        motors[i].error_sum = 0;
        motors[i].last_error = 0;
    }

    // Set all NEO motors to neutral position
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 1500);  // Motor 0
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 1500);  // Motor 1
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 1500);  // Motor 2
}

void SetMotorOutput(uint8_t motor_id, int16_t output) {
    if (motor_id >= 6 || !motors[motor_id].enabled || emergency_stop_active) return;

    Motor_t *motor = &motors[motor_id];

    // Apply velocity limits
    if (output > motor->max_velocity) output = motor->max_velocity;
    if (output < -motor->max_velocity) output = -motor->max_velocity;

    motor->current_output = output;

    switch(motor->type) {
        case MOTOR_SPARK_MAX_PWM:
            {
                // NEO Motor: 50Hz PWM servo signal (1000-2000μs)
                // Map -1000:+1000 input to 1000-2000μs pulse width
                int32_t pulse_width = 1500 + (output * 500 / 1000);

                // Clamp to safe range
                if (pulse_width < 1000) pulse_width = 1000;
                if (pulse_width > 2000) pulse_width = 2000;

                // Set compare value (TIM5 ARR should be 19999 for 50Hz)
                __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel, pulse_width);
            }
            break;

        case MOTOR_H_BRIDGE:
            {
                // H-Bridge: PWM + Direction (20kHz)
                uint16_t pwm_value = (output < 0) ? -output : output;
                pwm_value = (pwm_value > 1000) ? 1000 : pwm_value;

                // Set direction pin
                HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, (output >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

                // Set PWM duty cycle (TIM9 ARR = 1049 for 20kHz)
                uint32_t compare_value = (pwm_value * 1049) / 1000;
                __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel, compare_value);
            }
            break;

        case MOTOR_STEPPER:
            {
                // Stepper: Set direction and enable/disable
                if (motor->dir_port != NULL) {
                    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, (output >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                }

                if (output != 0 && motor->enable_port != NULL) {
                    // Enable stepper (active LOW)
                    HAL_GPIO_WritePin(motor->enable_port, motor->enable_pin, GPIO_PIN_RESET);

                    // Set PWM for step generation
                    uint16_t arr = __HAL_TIM_GET_AUTORELOAD(motor->pwm_tim);
                    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel, arr / 2);
                } else if (motor->enable_port != NULL) {
                    // Disable stepper and stop PWM
                    HAL_GPIO_WritePin(motor->enable_port, motor->enable_pin, GPIO_PIN_SET);
                    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel, 0);
                }
            }
            break;
    }
}

int32_t CalculatePID(Motor_t *motor, int32_t current_position) {
    int32_t error = motor->target_position - current_position;
    motor->error_sum += error;

    // Anti-windup
    if (motor->error_sum > 50000) motor->error_sum = 50000;
    if (motor->error_sum < -50000) motor->error_sum = -50000;

    int32_t derivative = error - motor->last_error;
    motor->last_error = error;

    float output = motor->kp * error + motor->ki * motor->error_sum + motor->kd * derivative;

    // Limit output
    if (output > 1000) output = 1000;
    if (output < -1000) output = -1000;

    return (int32_t)output;
}

void UpdateMotor(uint8_t motor_id) {
    if (motor_id >= 6 || !motors[motor_id].enabled) return;

    Motor_t *motor = &motors[motor_id];
    int32_t current_position = 0;

    // Get encoder feedback (if available)
    if (motor_id < 5) {
        current_position = encoders[motor_id].position;

        // Check position limits
        if (current_position < motor->min_position || current_position > motor->max_position) {
            // Position limit reached - stop motor
            motor->mode = MOTOR_MODE_IDLE;
            SetMotorOutput(motor_id, 0);
            return;
        }
    }

    switch(motor->mode) {
        case MOTOR_MODE_IDLE:
            SetMotorOutput(motor_id, 0);
            break;

        case MOTOR_MODE_VELOCITY:
            SetMotorOutput(motor_id, motor->target_velocity);
            break;

        case MOTOR_MODE_POSITION:
            {
                int32_t pid_output = CalculatePID(motor, current_position);
                SetMotorOutput(motor_id, (int16_t)pid_output);
            }
            break;

        case MOTOR_MODE_MANUAL:
            // Manual mode - output set directly via CAN commands
            break;
    }

    // Update stepper step counting
    if (motor_id == 5 && motor->type == MOTOR_STEPPER && motor->is_moving) {
        if (motor->step_count >= motor->target_steps) {
            motor->is_moving = 0;
            SetMotorOutput(5, 0);
        }
    }
}

void SetMotorVelocity(uint8_t motor_id, int16_t velocity) {
    if (motor_id >= 6) return;
    motors[motor_id].mode = MOTOR_MODE_VELOCITY;
    motors[motor_id].target_velocity = velocity;
}

void SetMotorPosition(uint8_t motor_id, int32_t position) {
    if (motor_id >= 6) return;
    motors[motor_id].mode = MOTOR_MODE_POSITION;
    motors[motor_id].target_position = position;
    motors[motor_id].error_sum = 0;  // Reset integral term
}

void SetStepperFrequency(uint16_t frequency_hz) {
    if (frequency_hz < 10) frequency_hz = 10;
    if (frequency_hz > 10000) frequency_hz = 10000;

    // Calculate ARR for desired frequency (1MHz timer clock)
    uint16_t arr_value = (1000000 / frequency_hz) - 1;

    __HAL_TIM_SET_AUTORELOAD(&htim11, arr_value);
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, arr_value / 2);

    motors[5].step_frequency = frequency_hz;
}

void MoveStepperSteps(int32_t steps, uint16_t frequency) {
    if (steps == 0) return;

    Motor_t *stepper = &motors[5];

    // Set frequency and direction
    SetStepperFrequency(frequency);
    stepper->target_steps = (steps > 0) ? steps : -steps;
    stepper->step_count = 0;
    stepper->is_moving = 1;

    // Set direction and enable
    SetMotorOutput(5, (steps > 0) ? 500 : -500);
}

void EmergencyStopAllMotors(void) {
    emergency_stop_active = 1;
    for(int i = 0; i < 6; i++) {
        motors[i].mode = MOTOR_MODE_IDLE;
        SetMotorOutput(i, 0);
    }
}

void ClearEmergencyStop(void) {
    emergency_stop_active = 0;
}

void SendMotorStatus(void) {
    TxHeader.StdId = 0x210;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    for (int i = 0; i < 6; i++) {
        TxData[i] = (motors[i].mode << 4) |
                    (motors[i].enabled << 3) |
                    ((motors[i].current_output > 0) << 2) |
                    ((motors[i].current_output < 0) << 1) |
                    (motors[i].is_moving);
    }
    TxData[6] = emergency_stop_active ? 0xFF : 0xBB;  // Motor status marker
    TxData[7] = 0xCC;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

// ========== ENCODER FUNCTIONS ==========
void InitializeEncoders(void) {
    // Start encoder timers in encoder mode
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);  // Base
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  // Shoulder
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  // Elbow
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  // Wrist1
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);  // Wrist2

    // Initialize encoder positions
    for (int i = 0; i < 5; i++) {
        encoders[i].last_counter = __HAL_TIM_GET_COUNTER(encoders[i].htim);
        encoders[i].position = 0;
        encoders[i].velocity = 0;
    }
}

void UpdateEncoder(Encoder_t *enc) {
  if (!enc->enabled) return;

  uint32_t current = __HAL_TIM_GET_COUNTER(enc->htim);

  int32_t delta;
  if (enc->htim->Instance == TIM2) {
    // TIM2 is 32-bit timer
    delta = (int32_t)(current - enc->last_counter);
  } else {
    // 16-bit timers
    delta = (int16_t)(current - enc->last_counter);
  }

  enc->last_counter = current;
  enc->velocity = delta;
  enc->position += delta;
}

void UpdateIndexPulse(Encoder_t *enc) {
  if (!enc->enabled || enc->index_port == NULL) return;

  GPIO_PinState pin_state = HAL_GPIO_ReadPin(enc->index_port, enc->index_pin);
  uint8_t current_index = (pin_state == GPIO_PIN_SET) ? 1 : 0;

  if (current_index && !enc->last_index_state) {
    enc->index_detected = 1;
    enc->index_pulse_count++;

    if (enc->homing_state == HOMING_IN_PROGRESS) {
      enc->home_position = enc->position;
      enc->position = 0;
      enc->homing_state = HOMING_COMPLETE;
      enc->is_homed = 1;
    }
  } else {
    enc->index_detected = 0;
  }

  enc->last_index_state = current_index;
}

void StartHoming(uint8_t encoder_id) {
  if (encoder_id >= 5) return;

  Encoder_t *enc = &encoders[encoder_id];
  if (!enc->enabled || enc->index_port == NULL || enc->is_homed) return;

  enc->homing_state = HOMING_IN_PROGRESS;
  enc->homing_start_time = HAL_GetTick();
  enc->is_homed = 0;
  enc->index_pulse_count = 0;

  // Set motor to slow velocity mode for homing
  SetMotorVelocity(encoder_id, 100);  // Slow positive velocity
}

void ProcessHoming(uint8_t encoder_id) {
  if (encoder_id >= 5) return;

  Encoder_t *enc = &encoders[encoder_id];
  if (!enc->enabled || enc->homing_state != HOMING_IN_PROGRESS) return;

  if (HAL_GetTick() - enc->homing_start_time > enc->homing_timeout_ms) {
    enc->homing_state = HOMING_ERROR;
    motors[encoder_id].mode = MOTOR_MODE_IDLE;
    SetMotorOutput(encoder_id, 0);
  }
}

void SendEncoderData(uint8_t encoder_id, Encoder_t *enc) {
  if (!enc->enabled) return;

  int32_t pos = (int32_t)(enc->position & 0xFFFFFFFF);
  int32_t vel = enc->velocity;

  TxHeader.StdId = 0x100 + encoder_id;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;

  TxData[0] = (pos >> 24) & 0xFF;
  TxData[1] = (pos >> 16) & 0xFF;
  TxData[2] = (pos >> 8) & 0xFF;
  TxData[3] = pos & 0xFF;

  TxData[4] = (vel >> 24) & 0xFF;
  TxData[5] = (vel >> 16) & 0xFF;
  TxData[6] = (vel >> 8) & 0xFF;
  TxData[7] = vel & 0xFF;

  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

void SendSystemStatus(void) {
  TxHeader.StdId = 0x200;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;

  for (int i = 0; i < 5; i++) {
    TxData[i] = (encoders[i].homing_state << 4) |
                (encoders[i].is_homed << 3) |
                (encoders[i].index_detected << 2) |
                (encoders[i].enabled << 1);
  }
  TxData[5] = 0xAA;  // Heartbeat
  TxData[6] = encoders[0].index_pulse_count & 0xFF;
  TxData[7] = 0x55;  // Status marker

  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

void SendDebugInfo(void) {
  Encoder_t *enc = &encoders[0];
  if (!enc->enabled) return;

  TxHeader.StdId = 0x201;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;

  uint32_t raw_timer = __HAL_TIM_GET_COUNTER(enc->htim);
  uint8_t index_pin_state = HAL_GPIO_ReadPin(enc->index_port, enc->index_pin);

  TxData[0] = (raw_timer >> 8) & 0xFF;
  TxData[1] = raw_timer & 0xFF;
  TxData[2] = index_pin_state;
  TxData[3] = enc->last_index_state;
  TxData[4] = enc->homing_state;
  TxData[5] = (enc->index_pulse_count >> 8) & 0xFF;
  TxData[6] = enc->index_pulse_count & 0xFF;
  TxData[7] = enc->enabled;

  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

// ========== CAN FUNCTIONS ==========
void InitializeCAN(void) {
    CAN_FilterTypeDef sFilterConfig;

    // Configure CAN filter
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    // Start CAN
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }

    // Activate CAN RX notification
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
}

// ========== DEBUG FUNCTIONS FOR MOTOR 0 ==========
void TestMotor0PWM(void) {
    // Test 1: Direct GPIO toggle
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure PA0 as GPIO output first
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Toggle PA0 manually
    for(int i = 0; i < 10; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_Delay(100);
    }

    // Reconfigure for PWM
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Restart PWM
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
}

void DebugMotor0(void) {
    // Send comprehensive motor 0 status
    TxHeader.StdId = 0x380;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    // Get timer values
    uint16_t tim5_cnt = __HAL_TIM_GET_COUNTER(&htim5);
    uint16_t tim5_arr = __HAL_TIM_GET_AUTORELOAD(&htim5);
    uint16_t tim5_ccr1 = __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_1);

    TxData[0] = motors[0].enabled;
    TxData[1] = motors[0].mode;
    TxData[2] = (motors[0].current_output >> 8) & 0xFF;
    TxData[3] = motors[0].current_output & 0xFF;
    TxData[4] = (tim5_ccr1 >> 8) & 0xFF;
    TxData[5] = tim5_ccr1 & 0xFF;
    TxData[6] = (tim5_arr >> 8) & 0xFF;
    TxData[7] = tim5_arr & 0xFF;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

void VerifyTIM5_Config(void) {
    // Check if TIM5 is running
    if (__HAL_TIM_GET_COUNTER(&htim5) == 0) {
        // Timer not running - restart it
        HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
        HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);

        // Reinitialize
        MX_TIM5_Init();

        // Start all channels
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
    }

    // Set neutral position on all NEO motors
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 1500);  // Motor 0
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 1500);  // Motor 1
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 1500);  // Motor 2
}

void Force_TIM5_Init(void) {
    // Manually initialize TIM5 for PWM with correct settings

    // Stop the timer first
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);

    // Reconfigure with correct period
    htim5.Init.Prescaler = 83;          // 84MHz/84 = 1MHz
    htim5.Init.Period = 19999;          // 1MHz/20000 = 50Hz (THIS IS THE FIX!)
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
        Error_Handler();
    }

    // Configure PWM mode
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1500;  // Initial neutral position
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    // Channel 1 (PA0)
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    // Channel 3 (PA2)
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }

    // Channel 4 (PA3)
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
}

// ========== CAN CALLBACK FUNCTION ==========
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
    switch (RxHeader.StdId) {
      case 0x301:  // Start individual homing
        if (RxData[0] < 5) {
          StartHoming(RxData[0]);
        }
        break;

      case 0x302:  // Reset encoder position
        if (RxData[0] < 5 && encoders[RxData[0]].enabled) {
          encoders[RxData[0]].position = 0;
          encoders[RxData[0]].home_position = 0;
        }
        break;

      case 0x303:  // Enable/disable encoder
        if (RxData[0] < 5) {
          encoders[RxData[0]].enabled = RxData[1] ? 1 : 0;
        }
        break;

      case 0x310:  // Set motor velocity [motor_id][velocity_high][velocity_low][reserved...]
        if (RxData[0] < 6) {
          int16_t velocity = (int16_t)((RxData[1] << 8) | RxData[2]);
          SetMotorVelocity(RxData[0], velocity);
        }
        break;

      case 0x311:  // Set motor position [motor_id][pos_byte3][pos_byte2][pos_byte1][pos_byte0][reserved...]
        if (RxData[0] < 6) {
          int32_t position = (int32_t)((RxData[1] << 24) | (RxData[2] << 16) | (RxData[3] << 8) | RxData[4]);
          SetMotorPosition(RxData[0], position);
        }
        break;

      case 0x312:  // Emergency stop all motors
        EmergencyStopAllMotors();
        break;

      case 0x313:  // Enable/disable motor [motor_id][enable][reserved...]
        if (RxData[0] < 6) {
          motors[RxData[0]].enabled = RxData[1] ? 1 : 0;
          if (!motors[RxData[0]].enabled) {
            motors[RxData[0]].mode = MOTOR_MODE_IDLE;
            SetMotorOutput(RxData[0], 0);
          }
        }
        break;

      case 0x314:  // Set PID gains [motor_id][kp*10][ki*100][kd*10][reserved...]
        if (RxData[0] < 6) {
          motors[RxData[0]].kp = RxData[1] / 10.0f;
          motors[RxData[0]].ki = RxData[2] / 100.0f;
          motors[RxData[0]].kd = RxData[3] / 10.0f;
        }
        break;

      case 0x315:  // Coordinated joint movement [j0_high][j0_low][j1_high][j1_low][j2_high][j2_low][reserved...]
        {
          int16_t joint_pos[3];
          joint_pos[0] = (int16_t)((RxData[0] << 8) | RxData[1]); // Base
          joint_pos[1] = (int16_t)((RxData[2] << 8) | RxData[3]); // Shoulder
          joint_pos[2] = (int16_t)((RxData[4] << 8) | RxData[5]); // Elbow

          for(int i = 0; i < 3; i++) {
            SetMotorPosition(i, joint_pos[i] * 100);
          }
        }
        break;

      case 0x316:  // Stepper move [steps_high][steps_low][freq_high][freq_low][reserved...]
        {
          int16_t steps = (int16_t)((RxData[0] << 8) | RxData[1]);
          uint16_t freq = (uint16_t)((RxData[2] << 8) | RxData[3]);
          MoveStepperSteps(steps, freq);
        }
        break;

      case 0x317:  // Clear emergency stop
        ClearEmergencyStop();
        break;

      case 0x318:  // Manual motor control [motor_id][output_high][output_low][reserved...]
        if (RxData[0] < 6) {
          int16_t output = (int16_t)((RxData[1] << 8) | RxData[2]);
          motors[RxData[0]].mode = MOTOR_MODE_MANUAL;
          SetMotorOutput(RxData[0], output);
        }
        break;

      case 0x319:  // Set position limits [motor_id][min_high][min_low][max_high][max_low][reserved...]
        if (RxData[0] < 6) {
          int16_t min_limit = (int16_t)((RxData[1] << 8) | RxData[2]);
          int16_t max_limit = (int16_t)((RxData[3] << 8) | RxData[4]);
          motors[RxData[0]].min_position = min_limit * 100;
          motors[RxData[0]].max_position = max_limit * 100;
        }
        break;

      case 0x320:  // Home all axes
        for (int i = 0; i < 3; i++) {  // Only home Base, Shoulder, Elbow
          if (encoders[i].index_port != NULL) {
            StartHoming(i);
          }
        }
        break;

      case 0x360:  // Motor 0 debug commands
        switch(RxData[0]) {
            case 0x01:  // Test GPIO toggle
                TestMotor0PWM();
                break;

            case 0x02:  // Send motor 0 debug info
                DebugMotor0();
                break;

            case 0x03:  // Verify TIM5
                VerifyTIM5_Config();
                break;

            case 0x04:  // Force PWM output
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, RxData[1] << 8 | RxData[2]);
                break;

            case 0x05:  // Force neutral (1500us)
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 1500);
                break;
        }
        break;
    }
  }
}

// ========== SYSTEM INITIALIZATION ==========
void SystemInit_RobotArm(void) {
    // Initialize all subsystems
    InitializeCAN();
    InitializeEncoders();
    InitializeMotors();

    // System is ready
    system_initialized = 1;

    // Blink LED to indicate system ready
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);  // Using board LED
        HAL_Delay(200);
    }
}

// ========== MAIN CONTROL LOOP ==========
void MainControlLoop(void) {
    uint32_t current_time = HAL_GetTick();

    // 1kHz Motor Update (1ms)
    if (current_time - last_motor_update >= 1) {
        last_motor_update = current_time;

        // Update all encoders
        for (int i = 0; i < 5; i++) {
            UpdateEncoder(&encoders[i]);
            UpdateIndexPulse(&encoders[i]);
            ProcessHoming(i);
        }

        // Update all motors
        for (int i = 0; i < 6; i++) {
            UpdateMotor(i);
        }
    }

    // 50Hz Encoder Data Send (20ms)
    if (current_time - last_encoder_send >= 20) {
        last_encoder_send = current_time;

        for (int i = 0; i < 5; i++) {
            SendEncoderData(i, &encoders[i]);
        }
    }

    // 10Hz Motor Status Send (100ms)
    if (current_time - last_motor_status_send >= 100) {
        last_motor_status_send = current_time;
        SendMotorStatus();
    }

    // 2Hz System Status (500ms)
    if (current_time - last_status_send >= 500) {
        last_status_send = current_time;
        SendSystemStatus();

        // Toggle heartbeat LED
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }

    // 1Hz Debug Info (1000ms)
    if (current_time - last_debug_send >= 1000) {
        last_debug_send = current_time;
        SendDebugInfo();
    }

    // Watchdog reset (100ms)
    if (current_time - last_watchdog_reset >= 100) {
        last_watchdog_reset = current_time;
        // HAL_IWDG_Refresh(&hiwdg);  // Uncomment if using watchdog
    }
}

// ========== INTERRUPT HANDLERS ==========
// Stepper step counting interrupt
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim11) {
        // Count stepper steps
        if (motors[5].is_moving) {
            motors[5].step_count++;
            if (motors[5].step_count >= motors[5].target_steps) {
                motors[5].is_moving = 0;
                SetMotorOutput(5, 0);
            }
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  // Initialize robot arm system
  SystemInit_RobotArm();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    MainControlLoop();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */
  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */
  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */
  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 19999;  // FIXED: Changed from 4294967295 to 19999 for 50Hz
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;  // Default to neutral position
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */
  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */
  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */
  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */
  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */
  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 3;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1049;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */
  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */
  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */
  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 83;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */
  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GRIMSON_LEFT_DIR_GPIO_O_GPIO_Port, GRIMSON_LEFT_DIR_GPIO_O_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GRIMSON_RIGHT_DIR_GPIO_O_Pin|NEMA_DIR_GPIO_O_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NEMA_ENABLE_GPIO_O_GPIO_Port, NEMA_ENABLE_GPIO_O_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD3_Pin|LD5_Pin|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BASE_HOMING_GPIO_INP_Pin SHOULDER_HOMING_GPIO_INP_Pin ELBOW_HOMING_GPIO_INP_Pin */
  GPIO_InitStruct.Pin = BASE_HOMING_GPIO_INP_Pin|SHOULDER_HOMING_GPIO_INP_Pin|ELBOW_HOMING_GPIO_INP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GRIMSON_LEFT_DIR_GPIO_O_Pin */
  GPIO_InitStruct.Pin = GRIMSON_LEFT_DIR_GPIO_O_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GRIMSON_LEFT_DIR_GPIO_O_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GRIMSON_RIGHT_DIR_GPIO_O_Pin NEMA_ENABLE_GPIO_O_Pin NEMA_DIR_GPIO_O_Pin */
  GPIO_InitStruct.Pin = GRIMSON_RIGHT_DIR_GPIO_O_Pin|NEMA_ENABLE_GPIO_O_Pin|NEMA_DIR_GPIO_O_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD5_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD5_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_SCL_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(Audio_SCL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  // Disable all motors
  EmergencyStopAllMotors();

  // Disable interrupts
  __disable_irq();

  // Flash error LED rapidly
  while (1) {
    HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
    HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

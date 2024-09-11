// Universidade Federal da Bahia (UFBA)
// Departamento de Engenharia Elétrica e da Computação (DEEC)
// Equipe: Gabriel, Márcio e Hugo
// Matéria: Programação em Tempo Real para Sist. Embarcados
// Professor: Jess Fiais
// CONTROLE DE VELOCIDADE DOS MOTORES DE UM ROBÔ OMNIDIRECIONAL
// Importação de Bibliotecas
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"
#include "iwdg.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

// Definition of Queues
osMessageQueueId_t QueueCorrenteHandle;
osMessageQueueId_t QueueVelocidadeHandle;
osMessageQueueId_t QueuePosicaoHandle;

// Definition of PID Controllers
PID_Controller pid_traction;
PID_Controller pid_velocity;
PID_Controller pid_position;

// Definition of PID Controllers
PID_TypeDef PID_Motor1, PID_Motor2, PID_Motor3;

// Definition of global variables
volatile int32_t encoderReading1 = 0;
volatile int32_t encoderReading2 = 0;
volatile int32_t encoderReading3 = 0;
double Roll, Pitch, Yaw;
float speed, traction;
double Motor1_Setpoint, Motor2_Setpoint, Motor3_Setpoint;
double Motor1_Input, Motor2_Input, Motor3_Input;
double Motor1_Output, Motor2_Output, Motor3_Output;

// Prototyping of Task Functions
void TaskControleCorrente(void *argument);
void TaskControleVelocidade(void *argument);
void TaskControlePosicao(void *argument);
void TaskControlePID(void *argument);
void TaskEnvioDadosUART(void *argument);

// Function prototypes
void PID_Compute(PID_TypeDef *pid);
void PID_Init(PID_TypeDef *pid, double kp, double ki, double kd);
double Read_Angle_Roll(void);
double Read_Angle_Pitch(void);
double Read_Angle_Yaw(void);
void Error_Handler(void);

// Peripheral Initialization Function
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART2_UART_Init(void);

int main(void) {
    // Initialization of hardware, system, and peripherals
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_IWDG_Init();
    MX_USART2_UART_Init();

    // Initialization of global variables
    Roll = 0.0f;
    Pitch = 0.0f;
    Yaw = 0.0f;
    speed = 0.0f;
    traction = 0.0f;
    Motor1_Setpoint = 0.0f;
    Motor2_Setpoint = 0.0f;
    Motor3_Setpoint = 0.0f;

    // Initialization of Queues
    QueueCorrenteHandle = osMessageQueueNew(3, sizeof(float), NULL);
    // Adjustable queue size of 3
    QueueVelocidadeHandle = osMessageQueueNew(3, sizeof(float), NULL);
    // Adjustable queue size of 3
    QueuePosicaoHandle = osMessageQueueNew(3, sizeof(float), NULL);
    // Adjustable queue size of 3

    // Initialization of PID Controllers
    PID_Init(&pid_traction, KP_TRACTION, KI_TRACTION, KD_TRACTION);
    PID_Init(&pid_velocity, KP_VELOCITY, KI_VELOCITY, KD_VELOCITY);
    PID_Init(&pid_position, KP_POSITION, KI_POSITION, KD_POSITION);

    // Creation of Tasks
    const osThreadAttr_t highPriorityTask_attr = {.priority = (osPriority_t) osPriorityHigh}; // Set high priority for control tasks
    osThreadNew(TaskControleCorrente, NULL, &highPriorityTask_attr);  // Current Control Task - reading current sensors in ADC
    osThreadNew(TaskControleVelocidade, NULL, &highPriorityTask_attr); // Speed Control Task - reading Encoder
    osThreadNew(TaskControlePosicao, NULL, &highPriorityTask_attr);    // Position Control Task - reading GPS
    osThreadNew(TaskControlePID, NULL, NULL);                          // PID Task with default priority
    osThreadNew(TaskEnvioDadosUART, NULL, NULL);                       // UART data transmission task with default priority

    // Start the FreeRTOS Scheduler
    osKernelStart();

    // In case the FreeRTOS Scheduler fails
    while (1) {}
}

// Task for Current Control (Sensor Reading)
void TaskControleCorrente(void *argument) {
    float correnteMotor[3] = {0.0f, 0.0f, 0.0f};
    const float referenciaADC = 3.3f;         // ADC reference voltage
    const float resolucaoADC = 4096.0f;       // 12-bit ADC resolution
    const float vccDiv2 = referenciaADC / 2.0f; // Vcc/2 = 1.65V for ACS712
    const float sensibilidade = 0.185f;       // Sensitivity of 185 mV/A for ACS712-05B

    for (;;) {
        // Reading current sensors (3 data with a period of 1ms)
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        float valorADC = HAL_ADC_GetValue(&hadc1);
        float vout = (valorADC / resolucaoADC) * referenciaADC;
        correnteMotor[0] = (vout - vccDiv2) / sensibilidade;

        HAL_ADC_Start(&hadc2);
        HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
        valorADC = HAL_ADC_GetValue(&hadc2);
        vout = (valorADC / resolucaoADC) * referenciaADC;
        correnteMotor[1] = (vout - vccDiv2) / sensibilidade;

        HAL_ADC_Start(&hadc3);
        HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
        valorADC = HAL_ADC_GetValue(&hadc3);
        vout = (valorADC / resolucaoADC) * referenciaADC;
        correnteMotor[2] = (vout - vccDiv2) / sensibilidade;

        // Checking for queue space before sending data and sending data to Queue
        if (osMessageQueueGetSpace(QueueCorrenteHandle) > 0) {
            osMessageQueuePut(QueueCorrenteHandle, &correnteMotor[0], 0, 0);
        }

        // Wait 1ms before executing again
        osDelay(1);
    }
}

#define WINDOW_SIZE 5

// Task for Speed Control (Encoder Reading)
void TaskControleVelocidade(void *argument) {
    static float leituraBuffer[3][WINDOW_SIZE] = {0};
    static uint8_t bufferIndex = 0;
    static uint32_t lastSampleTime = 0;
    float velocidadeMotor[3] = {0.0f};
    int32_t encoderReading[3] = {0};
    int32_t lastEncoderReading[3] = {0};

    for (;;) {
        uint32_t currentTime = osKernelGetTickCount();
        uint32_t elapsedTime = currentTime - lastSampleTime;

        if (elapsedTime >= SAMPLE_PERIOD_MS) {
            // Reading current encoder values
            encoderReading[0] = __HAL_TIM_GET_COUNTER(&htim1);
            encoderReading[1] = __HAL_TIM_GET_COUNTER(&htim2);
            encoderReading[2] = __HAL_TIM_GET_COUNTER(&htim3);

            // Calculate the raw speed
            float velocidadeBruta[3];
            for (int i = 0; i < 3; i++) {
                velocidadeBruta[i] = (float)(encoderReading[i] - lastEncoderReading[i]) / 36.0f;
                leituraBuffer[i][bufferIndex] = velocidadeBruta[i];
                lastEncoderReading[i] = encoderReading[i];
            }

            // Update buffer index
            bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;
            lastSampleTime = currentTime;
        }

        // Calculate moving average using the latest readings
        for (int i = 0; i < 3; i++) {
            float soma = 0.0f;
            for (uint8_t j = 0; j < WINDOW_SIZE; j++) {
                soma += leituraBuffer[i][j];
            }
            velocidadeMotor[i] = soma / WINDOW_SIZE;
        }

        // Check for space in the queue before sending data
        if (osMessageQueueGetSpace(QueueVelocidadeHandle) > 0) {
            osMessageQueuePut(QueueVelocidadeHandle, &velocidadeMotor[0], 0, 0);
        }

        // Wait 10ms before executing again
        osDelay(10);
    }
}

// Task for Position Control (Reading derived from GPS)
void TaskControlePosicao(void *argument) {
    float posicaoBase[3] = {0.0f, 0.0f, 0.0f};

    for (;;) {
        // Reading angles
        Roll = Read_Angle_Roll();
        Pitch = Read_Angle_Pitch();
        Yaw = Read_Angle_Yaw();

        // Reading the base rotation angles (3 data points with 100ms period)
        posicaoBase[0] = Roll;   // Function to read the Roll angle
        posicaoBase[1] = Pitch;  // Function to read the Pitch angle
        posicaoBase[2] = Yaw;    // Function to read the Yaw angle

        // Check for space in the queue before sending data
        if (osMessageQueueGetSpace(QueuePosicaoHandle) > 0) {
            osMessageQueuePut(QueuePosicaoHandle, &posicaoBase[0], 0, 0);
        }

        // Wait 100ms before executing again
        osDelay(100);
    }
}

// Add a structure to store dynamic PID parameters
typedef struct {
    double kp;
    double ki;
    double kd;
} PID_Params;

PID_Params params_traction = {KP_TRACTION, KI_TRACTION, KD_TRACTION};
PID_Params params_velocity = {KP_VELOCITY, KI_VELOCITY, KD_VELOCITY};
PID_Params params_position = {KP_POSITION, KI_POSITION, KD_POSITION};

// Mutex for synchronization
osMutexId_t mutexPIDParamsHandle;
osMutexId_t mutexSetpointsHandle;

// Semaphores for synchronization
osSemaphoreId_t semPIDParamsUpdateHandle;
osSemaphoreId_t semSetpointsUpdateHandle;

// Task for UART processing and setpoints update
void TaskEnvioDadosUART(void *argument) {
    char buffer[256];
    int len;
    float new_kp, new_ki, new_kd;
    float new_traction_setpoint[3];
    float new_velocity_setpoint[3];
    float new_position_setpoint[3];
    char command[64];

    for (;;) {
        // Send data via UART
        len = snprintf(buffer, sizeof(buffer), "Roll: %.2f, Pitch: %.2f, Yaw: %.2f, Speed: %.2f, Traction: %.2f\n",
                       Roll, Pitch, Yaw, speed, traction);
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

        // Receive command via UART
        HAL_UART_Receive(&huart2, (uint8_t*)command, sizeof(command) - 1, HAL_MAX_DELAY);

        // Process the command to update PID parameters
        if (sscanf(command, "KP_TRACTION:%f KI_TRACTION:%f KD_TRACTION:%f KP_VELOCITY:%f KI_VELOCITY:%f KD_VELOCITY:%f KP_POSITION:%f KI_POSITION:%f KD_POSITION:%f",
                   &new_kp, &new_ki, &new_kd, &params_velocity.kp, &params_velocity.ki, &params_velocity.kd,
                   &params_position.kp, &params_position.ki, &params_position.kd) == 9) {
            osMutexWait(mutexPIDParamsHandle, osWaitForever);
            params_traction.kp = new_kp;
            params_traction.ki = new_ki;
            params_traction.kd = new_kd;
            osMutexRelease(mutexPIDParamsHandle);

            // Signal that the PID parameters have been updated
            osSemaphoreRelease(semPIDParamsUpdateHandle);
        }

        // Update setpoints via UART
        if (sscanf(command, "TRACTION_SETPOINT:%f %f %f VELOCITY_SETPOINT:%f %f %f POSITION_SETPOINT:%f %f %f",
                   &new_traction_setpoint[0], &new_traction_setpoint[1], &new_traction_setpoint[2],
                   &new_velocity_setpoint[0], &new_velocity_setpoint[1], &new_velocity_setpoint[2],
                   &new_position_setpoint[0], &new_position_setpoint[1], &new_position_setpoint[2]) == 9) {
            osMutexWait(mutexSetpointsHandle, osWaitForever);
            memcpy(traction_setpoint, new_traction_setpoint, sizeof(new_traction_setpoint));
            memcpy(velocity_setpoint, new_velocity_setpoint, sizeof(new_velocity_setpoint));
            memcpy(position_setpoint, new_position_setpoint, sizeof(new_position_setpoint));
            osMutexRelease(mutexSetpointsHandle);

            // Signal that setpoints have been updated
            osSemaphoreRelease(semPIDParamsUpdateHandle);
        }

        // Wait 500ms before sending again
        osDelay(500);
    }
}

// Task for motor PID control
void TaskControlePID(void *argument) {
    float correnteMotor[3];
    float velocidadeMotor[3];
    float posicaoBase[3];
    float traction_setpoint[3];
    float velocity_setpoint[3];
    float position_setpoint[3];

    for (;;) {
        // Wait for PID parameters and setpoints to be updated
        osSemaphoreAcquire(semPIDParamsUpdateHandle, osWaitForever);
        osSemaphoreAcquire(semSetpointsUpdateHandle, osWaitForever);

        // Read data from queues
        osMessageQueueGet(QueueCorrenteHandle, &correnteMotor[0], NULL, osWaitForever);
        osMessageQueueGet(QueueVelocidadeHandle, &velocidadeMotor[0], NULL, osWaitForever);
        osMessageQueueGet(QueuePosicaoHandle, &posicaoBase[0], NULL, osWaitForever);

        // Copy the current setpoints
        osMutexWait(mutexSetpointsHandle, osWaitForever);
        memcpy(traction_setpoint, global_traction_setpoint, sizeof(traction_setpoint));
        memcpy(velocity_setpoint, global_velocity_setpoint, sizeof(velocity_setpoint));
        memcpy(position_setpoint, global_position_setpoint, sizeof(position_setpoint));
        osMutexRelease(mutexSetpointsHandle);

        // PID control for each motor
        float dt = 0.01; // Sampling interval in seconds

        // Traction control
        float traction_command[3];
        for (int i = 0; i < 3; i++) {
            traction_command[i] = PID_Compute(&pid_traction, traction_setpoint[i], correnteMotor[i], dt);
        }

        // Speed control
        float velocity_command[3];
        for (int i = 0; i < 3; i++) {
            velocity_command[i] = PID_Compute(&pid_velocity, velocity_setpoint[i], velocidadeMotor[i], dt);
        }

        // Position control
        float position_command[3];
        for (int i = 0; i < 3; i++) {
            position_command[i] = PID_Compute(&pid_position, position_setpoint[i], posicaoBase[i], dt);
        }

        // Apply commands to the motors
        Set_Motor_Commands(traction_command, velocity_command, position_command);

        // Send data via UART
        for (int i = 0; i < 3; i++) {
            Send_Motor_Data(i, traction_command[i], velocity_command[i], position_command[i]);
        }

        // Wait 10ms before next iteration
        osDelay(10);
    }
}

// Function to apply commands to the motors
void Set_Motor_Commands(float traction_command[3], float velocity_command[3], float position_command[3]) {
    // Define maximum and minimum limits for motor commands
    const float MAX_PWM = 255.0f; // Maximum PWM value (adjust as needed)
    const float MIN_PWM = 0.0f;   // Minimum PWM value (adjust as needed)
    
    // Variables for the final motor commands
    float motor_command[3][2]; // [0] for forward, [1] for backward

    // Adjust final motor commands based on the combinations of traction and velocity commands
    for (int i = 0; i < 3; i++) {
        float total_command = traction_command[i] + velocity_command[i] + position_command[i];
        
        // Saturate the command to ensure it is within the range [-1, 1]
        if (total_command > 1.0f) total_command = 1.0f;
        if (total_command < -1.0f) total_command = -1.0f;
        
        // Determine PWM for forward and backward
        if (total_command > 0) {
            motor_command[i][0] = total_command * MAX_PWM;
            motor_command[i][1] = 0;
        } else {
            motor_command[i][0] = 0;
            motor_command[i][1] = -total_command * MAX_PWM;
        }
    }

    // Apply the PWM commands to the motors (adjust as needed)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)motor_command[0][0]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)motor_command[1][0]);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)motor_command[2][0]);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)motor_command[0][1]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)motor_command[1][1]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)motor_command[2][1]);
}

// Function to send data via serial communication (APPLICATION WITHOUT FREERTOS)
// void Send_Motor_Data_UART(float speed, float traction, float position) {
//     char uart_buffer[100];
//     int len = snprintf(uart_buffer, sizeof(uart_buffer), "Speed: %.2f, Traction: %.2f, Position: %.2f\r\n", speed, traction, position);
//     
//     if (len > 0) {
//         HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, len, HAL_MAX_DELAY);
//         if (status != HAL_OK) {
//             // Handle transmission error if needed
//             Error_Handler(); // Or some other appropriate error handling
//         }
//     }
// }

// Hardware Configuration Functions
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

// GPIO Initialization
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Configure motor pins and other peripherals
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// ADC1 Initialization
static void MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

// ADC2 Initialization
static void MX_ADC2_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.ScanConvMode = ENABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 1;
    hadc2.Init.DMAContinuousRequests = DISABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc2) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_7;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

// ADC3 Initialization
static void MX_ADC3_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc3.Instance = ADC3;
    hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc3.Init.Resolution = ADC_RESOLUTION_12B;
    hadc3.Init.ScanConvMode = ENABLE;
    hadc3.Init.ContinuousConvMode = DISABLE;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc3.Init.NbrOfConversion = 1;
    hadc3.Init.DMAContinuousRequests = DISABLE;
    hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc3) != HAL_OK) {
        Error_Handler();
    }
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

// Timer 1 Initialization
static void MX_TIM1_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 0xFFFF;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim1);
}

// Timer 2 Initialization
static void MX_TIM2_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFF;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim2);
}

// Timer 3 Initialization
static void MX_TIM3_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFF;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim3);
}

// Serial Communication Initialization
void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

// Independent Watchdog Timer (IWDG) Initialization
static void MX_IWDG_Init(void) {
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 4095;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }
}

// Error handling function
void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}

// -> Angle Reading Functions (I am not sure exactly how to implement)
float Read_Angle_Roll(void) {
    // Placeholder function for reading Roll angle
    return 0.0f;
}

float Read_Angle_Pitch(void) {
    // Placeholder function for reading Pitch angle
    return 0.0f;
}

float Read_Angle_Yaw(void) {
    // Placeholder function for reading Yaw angle
    return 0.0f;
}

// PID_Compute function implementation
void PID_Compute(PID_TypeDef *pid) {
    double error = pid->Setpoint - pid->Input;
    pid->ITerm += (pid->Ki * error);
    if (pid->ITerm > pid->outMax) pid->ITerm = pid->outMax;
    else if (pid->ITerm < pid->outMin) pid->ITerm = pid->outMin;
    double dInput = pid->Input - pid->lastInput;
    pid->Output = pid->Kp * error + pid->ITerm - pid->Kd * dInput;
    if (pid->Output > pid->outMax) pid->Output = pid->outMax;
    else if (pid->Output < pid->outMin) pid->Output = pid->outMin;
    pid->lastInput = pid->Input;
}

// PID_Init function implementation
void PID_Init(PID_TypeDef *pid, double kp, double ki, double kd) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Integral = 0;
    pid->LastError = 0;
}

// Structure for PID control
typedef struct {
    double Setpoint;
    double Input;
    double Output;
    double Kp;
    double Ki;
    double Kd;
    double Integral;
    double LastError;
} PID_TypeDef;

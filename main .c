/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <string.h>
#include "my_lcd_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_RED = 0,
    STATE_BLUE,
    STATE_GREEN,

} LedState_t;
LedState_t LED_State;

typedef enum{
	Condtion_RED=0,
	Condion_BLUE,
	Contion_GREEN,

} Condion_t;
Condion_t Contion_State;


typedef struct {
    uint16_t r, g, b; // PWM compare
} Color_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT_PORT GPIOA
#define DHT_PIN GPIO_PIN_1

#define MAX_PWM       999
#define LED_CYCLE_MS  2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for WaterLevel_Task */
osThreadId_t WaterLevel_TaskHandle;
const osThreadAttr_t WaterLevel_Task_attributes = {
  .name = "WaterLevel_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ServoBuzzer_Tas */
osThreadId_t ServoBuzzer_TasHandle;
const osThreadAttr_t ServoBuzzer_Tas_attributes = {
  .name = "ServoBuzzer_Tas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DHT_Task */
osThreadId_t DHT_TaskHandle;
const osThreadAttr_t DHT_Task_attributes = {
  .name = "DHT_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LCD_task */
osThreadId_t LCD_taskHandle;
const osThreadAttr_t LCD_task_attributes = {
  .name = "LCD_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for StepMoter_Task */
osThreadId_t StepMoter_TaskHandle;
const osThreadAttr_t StepMoter_Task_attributes = {
  .name = "StepMoter_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RGB_Task */
osThreadId_t RGB_TaskHandle;
const osThreadAttr_t RGB_Task_attributes = {
  .name = "RGB_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
uint32_t adc_value = 0;

int adc_read_flag=0;
char msg[64];

volatile uint8_t  adc_it_done = 0;
static   uint32_t adc_it_buf[2];
static   uint8_t  adc_it_idx = 0;

volatile uint8_t g_temperature = 0;
volatile uint8_t g_humidity    = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void waterlevel_task(void *argument);
void servobuzzer_control_task(void *argument);
void dht_task(void *argument);
void lcd_Task(void *argument);
void stepmoter_task(void *argument);
void rgb_task(void *argument);

/* USER CODE BEGIN PFP */
//WaterLevel Sensor
void send_water_level_uart(uint16_t value) //수위센서 uart
{
  char msg[32];
  int len = sprintf(msg, "Water Level: %u\r\n", value);
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
}

//ServoMoter
void Servo_Set_Angle(uint8_t angle) //0도->0ms, 90도->1.5ms, 180도->2.4ms
{
  uint16_t pulse = 500 + ((uint32_t)angle * (2400 - 500)) / 180;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

void Servo_Pump_Cycle(uint8_t times)
{
  for (uint8_t i = 0; i < times; i++)
  {
    Servo_Set_Angle(0);
    osDelay(500);  // 0.5초
    Servo_Set_Angle(180);
    osDelay(500);
  }
  Servo_Set_Angle(0);  // 기본 위치로 정지
}


//StepMoter

const uint8_t stepSeq[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

void stepMotor(int step) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, stepSeq[step][0]);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, stepSeq[step][1]);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, stepSeq[step][2]);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, stepSeq[step][3]);
}

//RGB
static const Color_t stateColors[] = {
    [STATE_RED   ] = {999, 0,   0  },
    [STATE_BLUE  ] = {0,   0,   999},
    [STATE_GREEN ] = {0,   999, 0  },
    //[STATE_WHITE ] = {999, 999, 999},
   // [STATE_PURPLE] = {999, 0,   999}
};

extern TIM_HandleTypeDef htim3;  // PB4 묬H1, PB5 묬H2
extern TIM_HandleTypeDef htim2;  // PB10 묬H3

static inline void SetColor(uint16_t r, uint16_t g, uint16_t b)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, r);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, g);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, b);
}

static inline void SetLedState(LedState_t s)
{
    Color_t c = stateColors[s];
    SetColor(c.r, c.g, c.b);
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


volatile uint16_t water_level = 0;


uint16_t water_flag=0;


// 마이크로초 단위(us)의 지연 시간 생성
void delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim3, 0);            // 타이머            1 카운터 0으로 초기화
    while (__HAL_TIM_GET_COUNTER(&htim3) < us);  // 타이머 값이 us(마이크로초)만큼 올라갈 때까지 대기
}
void set_pin_output(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT_PIN;                // DHT 센서가 연결된 핀 지정
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;   // 출력 오픈드레인 모드 설정
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // 저속으로 설정 (전송속도 낮음)
    HAL_GPIO_Init(DHT_PORT, &GPIO_InitStruct);
}

void set_pin_input(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;       // 입력 모드 설정
    GPIO_InitStruct.Pull = GPIO_NOPULL;           // 내부 풀업이나 풀다운 저항 비활성화
    HAL_GPIO_Init(DHT_PORT, &GPIO_InitStruct);
}

uint8_t DHT_Read(uint8_t *temp, uint8_t *hum) {
    uint8_t bits[5] = {0};
    uint32_t time;

    // MCU -> 센서 신호 송출
    set_pin_output();
    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_RESET);
    osDelay(20);  // 18~20ms
    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET);
    delay_us(40);   // 20~40us

    set_pin_input();

    // 센서 -> MCU 응답 대기
    time = 0;
    while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET)
    {
        if (++time > 200) return 1; // Timeout
        delay_us(1);
    }

    // LOW(80us) + HIGH(80us)
    time = 0;
    while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_RESET)
    {
        if (++time > 200)
           return 2;
        delay_us(1);
    }

    time = 0;
    while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET) {
        if (++time > 200) return 3;
        delay_us(1);
    }

    // 40bit 데이터 수신
    for (int i = 0; i < 40; i++) {
        // 1. LOW 시작 비트 감지(50us), LOW가 끝날때까지 대기
        while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_RESET);

        // 2. 중간지점 샘플링 (HIGH: 0이면 약 26~28us, 1이면 70us)
        delay_us(40);
        if (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN)) {
            bits[i / 8] |= (1 << (7 - (i % 8)));
        }
            // 3. HIGH 끝날 때까지 대기
        while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET);
    }
    // 체크섬 확인
    uint8_t sum = bits[0] + bits[1] + bits[2] + bits[3];
    if (sum != bits[4]) return 4;

    *hum = bits[0];
    *temp = bits[2];
    return 0; // 성공
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);

  lcd_init(&hi2c1);
  HAL_TIM_Base_Start(&htim2);
   HAL_TIM_Base_Start(&htim3);  // Delay 타이머 시작
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);



   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of WaterLevel_Task */
  WaterLevel_TaskHandle = osThreadNew(waterlevel_task, NULL, &WaterLevel_Task_attributes);

  /* creation of ServoBuzzer_Tas */
  ServoBuzzer_TasHandle = osThreadNew(servobuzzer_control_task, NULL, &ServoBuzzer_Tas_attributes);

  /* creation of DHT_Task */
  DHT_TaskHandle = osThreadNew(dht_task, NULL, &DHT_Task_attributes);

  /* creation of LCD_task */
  LCD_taskHandle = osThreadNew(lcd_Task, NULL, &LCD_task_attributes);

  /* creation of StepMoter_Task */
  StepMoter_TaskHandle = osThreadNew(stepmoter_task, NULL, &StepMoter_Task_attributes);

  /* creation of RGB_Task */
  RGB_TaskHandle = osThreadNew(rgb_task, NULL, &RGB_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */



  HAL_NVIC_SetPriority(ADC_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 95;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LD2_Pin|StepMoter_IN4_Pin|StepMoter_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Buzzer_Pin|StepMoter_IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(StepMoter_IN2_GPIO_Port, StepMoter_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 LD2_Pin StepMoter_IN4_Pin StepMoter_IN1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD2_Pin|StepMoter_IN4_Pin|StepMoter_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Pin StepMoter_IN3_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|StepMoter_IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : StepMoter_IN2_Pin */
  GPIO_InitStruct.Pin = StepMoter_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(StepMoter_IN2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_waterlevel_task */
/**
* @brief Function implementing the WaterLevel_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_waterlevel_task */
void waterlevel_task(void *argument)
{
  /* USER CODE BEGIN waterlevel_task */
  /* Infinite loop */
  for(;;)
  {

     for (;;)
     {
        HAL_ADC_Start(&hadc1);  // 蹂     쒖옉

            if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
            {
              water_level = HAL_ADC_GetValue(&hadc1);  //  섏쐞媛   쎄린
            }

            HAL_ADC_Stop(&hadc1);  // Optional

            send_water_level_uart(water_level);  // UART 異쒕젰


            if(water_level<1000)
            	Contion_State=STATE_BLUE;

            else if(water_level>1800 &&water_level<2229)
            	Contion_State=STATE_GREEN;

            osDelay(2000);  //

     }
  }
  /* USER CODE END waterlevel_task */
}

/* USER CODE BEGIN Header_servobuzzer_control_task */
/**
* @brief Function implementing the ServoBuzzer_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servobuzzer_control_task */
void servobuzzer_control_task(void *argument)
{
    /* USER CODE BEGIN servobuzzer_control_task */
    /* Infinite loop */

    static uint8_t pump_activated = 0;
    uint8_t cycle = 0;

    for (;;)
    {
        // 1) 수위가 너무 높으면 부저 ON, LED는 초록
        if (water_level > 1950) {
            HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
            pump_activated = 0;            // 펌프 리셋
            Contion_State=STATE_BLUE;
        }
        else
        {
            // 부저 끄기
            HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);


            if (water_level <= 1500 && pump_activated == 0) {

                if (water_level < 1200)
                	{
                		cycle = 3;

                	}
                else if (water_level < 1500)   cycle = 2;
                else                            cycle = 1;

                Servo_Pump_Cycle(cycle);    // 서보 펌프 작동
                pump_activated = 1;         // 한 번만 실행
            }

            // 3) 수위가 회복되면(>1700) 다시 초록불로 전환
            if (water_level > 1700) {
                pump_activated = 0;         // 다음 저수 시 다시 작동 위해 리셋
            }
        }

        osDelay(1000);
    }
    /* USER CODE END servobuzzer_control_task */
}


/* USER CODE BEGIN Header_dht_task */
/**
* @brief Function implementing the DHT_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dht_task */
void dht_task(void *argument)
{
  /* USER CODE BEGIN dht_task */
  /* Infinite loop */
     for(;;)
     {

        uint8_t temp, hum;
                if (DHT_Read(&temp, &hum) == 0)
                {
                    g_temperature = temp;
                    g_humidity = hum;

                    char msg[64];
                       sprintf(msg, "Temp: %d C, Hum: %d %%\r\n", temp, hum);
                       HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                }
                else{
                   //char err[] = "DHT read failed\r\n";
                       //HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
                }
                //vTaskDelay(pdMS_TO_TICKS(500));  // 2초마다 측정

                if(g_temperature>=30)
                	Contion_State=STATE_RED;
                else
                	Contion_State=STATE_GREEN;


       osDelay(500);
     }
  /* USER CODE END dht_task */
}

/* USER CODE BEGIN Header_lcd_Task */
/**
* @brief Function implementing the LCD_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lcd_Task */
void lcd_Task(void *argument)
{
  /* USER CODE BEGIN lcd_Task */
  /* Infinite loop */
   lcd_clear();  // 초기 1회만 호출

   char buf[20];
          for (;;) {

              lcd_set_cursor(0, 0);
              snprintf(buf, sizeof(buf), "Tempe: %dC", g_temperature);
              lcd_send_string(buf);

              lcd_set_cursor(1, 0);
              snprintf(buf, sizeof(buf), "Humid: %d", g_humidity);
              lcd_send_string(buf);

              //vTaskDelay(pdMS_TO_TICKS(500));  // 2초마다 출력
              osDelay(1000);
          }
  /* USER CODE END lcd_Task */
}

/* USER CODE BEGIN Header_stepmoter_task */
/**
* @brief Function implementing the StepMoter_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_stepmoter_task */
void stepmoter_task(void *argument)
{
    /* USER CODE BEGIN stepmoter_task */
    /* Infinite loop */
    for (;;)
    {
        if (g_temperature > 30)   // 온도 높을 때
        {
            for (int i = 0; i < 4096; i++)
            {
                Contion_State = STATE_RED;
                stepMotor(i % 8);
                delay_us(900);

                if (g_temperature < 30)
                    break;
            }
        }
        else   // 온도 낮을 때
        {
            for (int i = 0; i < 4096; i++)
            {
                Contion_State = STATE_GREEN;
                stepMotor(i % 8);
                osDelay(10);

                if (g_temperature > 30)
                    break;
            }
        }

        osDelay(1);
    }
    /* USER CODE END stepmoter_task */
}


/* USER CODE BEGIN Header_rgb_task */
/**
* @brief Function implementing the RGB_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rgb_task */
void rgb_task(void *argument)
{
  /* USER CODE BEGIN rgb_task */

	//SetLedState(STATE_GREEN);

  /* Infinite loop */
  for(;;)
  {
	  switch(Contion_State)
	  {
	  	 case Condtion_RED :
	  		 SetLedState(STATE_RED);
	  		 break;

	  	 case Condion_BLUE:
	  		  SetLedState(STATE_BLUE);
	  		   break;

	  	 case Contion_GREEN:
	  		 SetLedState(STATE_GREEN);
	  		 break;

	  }
	  osDelay(50);
  }
  /* USER CODE END rgb_task */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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

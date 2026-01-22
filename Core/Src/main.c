/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdbool.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ADC_CHANNELS 3
#define ADC_FULL_SCALE 4096
#define DIGITAL_INPUTS 1
#define PWM_OUTPUTS 3



#define BUTTON_Pin GPIO_PIN_13

#define NUM_LEDS 8

// Button thresholds (in ms)
#define PRESS_SHORT_TIME_MS 300
#define PRESS_LONG_TIME_MS 500 
#define PRESS_DOUBLE_TIME_MS 500

// Random
#define RAND_BUFFER_SIZE 16

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

osThreadId Handle_ButtonHandle;
osThreadId defaultTaskHandle;
osThreadId LED_PC15Handle;
osThreadId LED_PC13Handle;
/* USER CODE BEGIN PV */

// Buffer to store ADC readings from 3 channels 
uint16_t adc_buffer[ADC_CHANNELS];

// Which sensor (0, 1, 2) we'are currently displaying
uint8_t current_channel = 0;

// There're two options (0 - bar graph | 1 - binary)
uint8_t display_mode = 1;

// Running light animation (0 - normal display | 1 - one LED moves back and forth)
bool running_mode = false;

// Button handling variables
typedef struct {
  bool is_button_pressed;
  uint32_t press_time; // in seconds
  uint32_t last_release; // in seconds
  uint32_t click_count;
} ButtonInput;

ButtonInput button_input = {
  .is_button_pressed = false,
  .press_time = 0, // in seconds
  .last_release = 0, // in seconds
  .click_count = 0, // for double clicks
};

// Running LED animation state
typedef struct{
  bool direction;
  uint8_t position;
  uint32_t last_update_time;
} RunningLED;

RunningLED running_led = {
  .position = 0,
  .last_update_time = 0,
  .direction = true,
};

uint32_t random_buffer[RAND_BUFFER_SIZE] = {0};
uint8_t random_buffer_index = 0;

// Mutex for thread-safe random number access
osMutexId randomMutexHandle;

// Set LED (GPIO) pins 
GPIO_TypeDef *led_ports[NUM_LEDS] = {
  LED_PB0_GPIO_Port, LED_PB1_GPIO_Port, LED_PB2_GPIO_Port, LED_PB3_GPIO_Port,
  LED_PB4_GPIO_Port, LED_PB5_GPIO_Port, LED_PB6_GPIO_Port, LED_PB7_GPIO_Port
};
  
uint16_t led_pins[NUM_LEDS] = {
  LED_PB0_Pin, LED_PB1_Pin, LED_PB2_Pin, LED_PB3_Pin,
  LED_PB4_Pin, LED_PB5_Pin, LED_PB6_Pin, LED_PB7_Pin
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
void Handle_Button_Task(void const * argument);
void default_Task(void const * argument);
void LED_PC15_Task(void const * argument);
void LED_PC04_Task(void const * argument);

/* USER CODE BEGIN PFP */
void set_NUM_LEDS(uint8_t pattern);
void Display_Scale(uint16_t adc_buffer);
void Display_Bits(uint16_t adc_value);
void Indicate_Channels(uint8_t channel);
void Update_Running_LED(void);
void Update_PWM_Outputs(void);
uint32_t ADC_Random_Byte(void);
uint32_t ADC_Random_Word(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Set LED GPIO pins 
void set_NUM_LEDS(uint8_t pattern)
{
  for (uint16_t i = 0; i < NUM_LEDS; i++)
  {
    HAL_GPIO_WritePin(led_ports[i], led_pins[i],
    (pattern & (1 << i)) ?  GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}

// The bar graph scale mode for 8 NUM_LEDS
void Display_Scale(uint16_t adc_value)
{
  // Convert ADC_value (0-4095) to the LED count (0-8)
  // It results which led is lighting - (0-8)
  uint8_t led_count = (adc_value * 9) / ADC_FULL_SCALE;
  uint8_t pattern = 0;

  // It does bit shifting manipulation (shift it left by i pos)
  for (uint16_t i = 0; i < led_count; i++)
  {
    pattern = pattern | (1 << i);    
  }

  set_NUM_LEDS(pattern);
}

// Display upper 8 bits of ADC value in binary
void Display_Bits(uint16_t adc_value)
{
  uint8_t upper_bits = (adc_value >> 4) & 0xFF;
  set_NUM_LEDS(upper_bits);
}

// Blink all NUM_LEDS to indicate channel number
void Indicate_Channel(uint8_t channel)
{
  // Blink (channel + 1) times 
  for (uint8_t i = 0; i < (channel + 1); i++)
  {
    set_NUM_LEDS(0xFF); // All NUM_LEDS ON
    HAL_Delay(200);
    set_NUM_LEDS(0x00); // All NUM_LEDS OFF
    HAL_Delay(200);
  }
}

// Update running LED animation
void Update_Running_LED(void)
{
  uint16_t adc_value = adc_buffer[current_channel];

  // Calculate delay based on Button thresholds and ADC value
  uint32_t delay = PRESS_LONG_TIME_MS - ((adc_value * (PRESS_LONG_TIME_MS - PRESS_SHORT_TIME_MS)) / ADC_FULL_SCALE);
  uint32_t current_time = HAL_GetTick();

  if (current_time - running_led.last_update_time >= delay)
  {
    running_led.last_update_time = current_time;

    // Light only the current LED
    set_NUM_LEDS(1 << running_led.position);
    
    if (running_led.direction)
    {
      running_led.position++;
      if (running_led.position >= (NUM_LEDS - 1))
      {
        running_led.position = NUM_LEDS - 1;
        running_led.direction = false; // Reverse direction
      }
    } else {
      // Moving backward
      if (running_led.position == 0)
      {
        running_led.direction = true;
      } else {
        running_led.position--;
      }
    }
  }
}

void Update_PWM_Outputs(void)
{
  uint32_t pwm_period = htim1.Init.Period; 

  // Set PWM duty cycle proportional to ADC value
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (adc_buffer[0] * pwm_period) / ADC_FULL_SCALE);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (adc_buffer[1] * pwm_period) / ADC_FULL_SCALE);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (adc_buffer[2] * pwm_period) / ADC_FULL_SCALE);
}

// Uses least significant bits which contain thermal noise   
uint32_t ADC_Random_Byte(void)
{
  uint8_t random = 0;

  // Collect Least Significnt Bits multiple ADC samples
  for (int i = 0; i < 8; i++)
  {
    uint32_t bit = adc_buffer[i % ADC_CHANNELS] & 0x01;
    random = (random << 1) | bit;

    for (volatile int j = 0; j < 100; j++);
  }

  return random;
}

// Get 32-bit random from ADC
uint32_t ADC_Random_Word(void)
{
  uint32_t random = 0;

  random |= ((uint32_t)ADC_Random_Byte()) << 13;
  random |= ((uint32_t)ADC_Random_Byte() << 17);
  random |= ((uint32_t)ADC_Random_Byte() << 5);
  random |= ((uint32_t)ADC_Random_Byte());
  
  return random;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // Start ADC with DMA
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, sizeof(adc_buffer) / sizeof(adc_buffer[0])) != HAL_OK)
  {
    while (1)
    {
      set_NUM_LEDS(0xFF); // All NUM_LEDS ON
      HAL_Delay(100);
      set_NUM_LEDS(0x00); // All NUM_LEDS OFF 
      HAL_Delay(200);
    }
  }

  // Start PWM on all 3 channels
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  // Initial indication: show channel 0 selected
  Indicate_Channel(current_channel);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
    // Create mutex for random number access
  
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
  /* definition and creation of Handle_Button */
  osThreadDef(Handle_Button, Handle_Button_Task, osPriorityAboveNormal, 0, 128);
  Handle_ButtonHandle = osThreadCreate(osThread(Handle_Button), NULL);

  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, default_Task, osPriorityAboveNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LED_PC15 */
  osThreadDef(LED_PC15, LED_PC15_Task, osPriorityNormal, 0, 128);
  LED_PC15Handle = osThreadCreate(osThread(LED_PC15), NULL);

  /* definition and creation of LED_PC13 */
  osThreadDef(LED_PC13, LED_PC04_Task, osPriorityNormal, 0, 128);
  LED_PC13Handle = osThreadCreate(osThread(LED_PC13), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


    // Set small delay for button debouncing to prevent CPU overflow
    HAL_Delay(10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Period = 4000;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_PB0_Pin|LED_PB1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_PB2_Pin|LED_PB3_Pin|LED_PB5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_PB6_Pin|LED_PB7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_PB0_Pin LED_PB1_Pin */
  GPIO_InitStruct.Pin = LED_PB0_Pin|LED_PB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_PB2_Pin LED_PB3_Pin LED_PB5_Pin */
  GPIO_InitStruct.Pin = LED_PB2_Pin|LED_PB3_Pin|LED_PB5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_PB4_Pin */
  GPIO_InitStruct.Pin = LED_PB4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LED_PB4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_PB6_Pin */
  GPIO_InitStruct.Pin = LED_PB6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PB6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_PB7_Pin */
  GPIO_InitStruct.Pin = LED_PB7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PB7_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Handle_Button_Task */
/**
  * @brief  Function implementing the Handle_Button thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Handle_Button_Task */
void Handle_Button_Task(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  
  for(;;)
  {
    bool button_state = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET);
    uint32_t current_time = HAL_GetTick();

    // Button PRESS detection
    if (button_state && !button_input.is_button_pressed)
    {
      button_input.is_button_pressed = true;
      button_input.press_time = current_time; 
    }

    // Button RELEASE detection
    if (!button_state && button_input.is_button_pressed)
    {
      button_input.is_button_pressed = false;
      uint32_t press_duration = current_time - button_input.press_time;

      // SHORT PRESS (<300ms)
      if (press_duration < PRESS_SHORT_TIME_MS)
      {
        // Check for double click
        if (current_time - button_input.last_release < PRESS_DOUBLE_TIME_MS)
        {
          button_input.click_count++;
          
          if (button_input.click_count == 2)
          {
            // DOUBLE CLICK - Toggle running mode
            running_mode = !running_mode;
            
            if (running_mode)
            {
              running_led.position = 0;
              running_led.direction = true;
              running_led.last_update_time = current_time;
            }
            
            button_input.click_count = 0;
          }
        } else {
          // SINGLE SHORT PRESS - Toggle display mode
          if (!running_mode)
          {
            display_mode = !display_mode;
          }
          button_input.click_count = 1;
        }
        
        button_input.last_release = current_time;
      }
      // LONG PRESS (>=500ms)
      else if (press_duration >= PRESS_LONG_TIME_MS)
      {
        // Change channel
        running_mode = false;
        current_channel = (current_channel + 1) % ADC_CHANNELS;
        Indicate_Channel(current_channel);
        button_input.click_count = 0;
      }
    }
    
    // Reset click counter if timeout
    if (current_time - button_input.last_release > PRESS_DOUBLE_TIME_MS && 
        button_input.click_count == 1)
    {
      button_input.click_count = 0;
    }

   
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_default_Task */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_default_Task */
void default_Task(void const * argument)
{
  /* USER CODE BEGIN default_Task */
  /* Infinite loop */
  for(;;)
  {
    Update_PWM_Outputs();

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, sizeof(adc_buffer) / sizeof(adc_buffer[0]));

    if (running_mode == true)
    {
      Update_Running_LED();
    } else {
      uint16_t current_adc = adc_buffer[current_channel];
      
      if (display_mode == 0)
      {
        Display_Scale(current_adc);
      } else {
        Display_Bits(current_adc);
      }
    }

    osDelay(1);
  }
  /* USER CODE END default_Task */
}

/* USER CODE BEGIN Header_LED_PC15_Task */
/**
* @brief Function implementing the LED_PC15 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED_PC15_Task */
void LED_PC15_Task(void const * argument)
{
  /* USER CODE BEGIN LED_PC15_Task */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
    osDelay(500);
  }
  /* USER CODE END LED_PC15_Task */
}

/* USER CODE BEGIN Header_LED_PC04_Task */
/**
* @brief Function implementing the LED_PC13 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED_PC04_Task */
void LED_PC04_Task(void const * argument)
{
  /* USER CODE BEGIN LED_PC04_Task */
  /* Infinite loop */
  for(;;)
  {
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay(500);
  }
  /* USER CODE END LED_PC04_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

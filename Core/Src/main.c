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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

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
#define ADC_FULL_SCALE 4095
#define DIGITAL_INPUTS 1
#define PWM_OUTPUTS 3

#define LEDS 8

// Button thresholds (in ms)
#define PRESS_SHORT_TIME_MS 300
#define PRESS_LONG_TIME_MS 500 
#define PRESS_DOUBLE_TIME_MS 500

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

// Buffer to store ADC readings from 3 channels 
uint16_t adc_buffer[ADC_CHANNELS];

// Which sensor (0, 1, 2) we'are currently displaying
uint8_t current_channel = 0;

// There're two options (0 - bar graph | 1 - binary)
uint8_t display_mode = 0;

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


// Set LED (GPIO) pins 
GPIO_TypeDef *led_ports[LEDS] = {
  GPIOB, GPIOB, GPIOB, GPIOB,
  GPIOB, GPIOB, GPIOB, GPIOB
};
  
uint16_t led_pins[LEDS] = {
  GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3,
  GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
void set_LEDs(uint8_t pattern);
void Display_Scale(uint16_t adc_buffer);
void Display_Bits(uint16_t adc_value);
void Indicate_Channels(uint8_t channel);
void Update_Running_LED(void);
void Update_PWM_Outputs(void);
void Handle_Button(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Set LED GPIO pins 
void set_LEDs(uint8_t pattern)
{
  for (uint16_t i = 0; i < LEDS; i++)
  {
    HAL_GPIO_WritePin(led_ports[i], led_pins[i],
    (pattern & (1 << i)) ?  GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}

// The bar graph scale mode for 8 LEDs
void Display_Scale(uint16_t adc_value)
{
  // Convert ADC_value (0-4095) to the LED count (0-8)
  // It results which led is lighting - (0-8)
  uint8_t led_count = (adc_value * 9) / 4096;
  uint8_t pattern = 0;

  // It does bit shifting manipulation (shift it left by i pos)
  for (uint16_t i = 0; i < led_count; i++)
  {
    pattern = pattern | (1 << i);    
  }

  set_LEDs(pattern);
}

// Display upper 8 bits of ADC value in binary
void Display_Bits(uint16_t adc_value)
{
  uint8_t upper_bits = (adc_value >> 4) & 0xFF;
  set_LEDs(upper_bits);
}

// Blink all LEDs to indicate channel number
void Indicate_Channel(uint8_t channel)
{
  // Blink (channel + 1) times 
  for (uint8_t i = 0; i < (channel + 1); i++)
  {
    set_LEDs(0xFF); // All LEDs ON
    HAL_Delay(200);
    set_LEDs(0x00); // All LEDs OFF
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
    set_LEDs(1 << running_led.position);
    
    if (running_led.direction)
    {
      running_led.position++;
      if (running_led.position >= (LEDS - 1))
      {
        running_led.position = LEDS - 1;
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
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (adc_buffer[ADC_CHANNEL_0] * pwm_period) / ADC_FULL_SCALE);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (adc_buffer[ADC_CHANNEL_1] * pwm_period) / ADC_FULL_SCALE);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (adc_buffer[ADC_CHANNEL_2] * pwm_period) / ADC_FULL_SCALE);
}

void Handle_Button(void)
{
  bool button_state = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_RESET);
  uint32_t current_time = HAL_GetTick();

  // Button press detection
  if (button_state && !button_input.is_button_pressed)
  {
    // button state is pressed
    button_input.is_button_pressed = true;
    button_input.press_time = current_time; 
  }

  // Button release detection
  if (!button_state && button_input.is_button_pressed)
  {
    button_input.is_button_pressed = false;
    
    // Initialize Button Press Duration
    uint32_t press_duration = current_time - button_input.press_time;

    // Detect SHORT Press
    if (press_duration < PRESS_SHORT_TIME_MS)
    {
      // Check for double click
      if (current_time - button_input.last_release < PRESS_DOUBLE_TIME_MS)
      {
        button_input.click_count++;
        if (button_input.click_count == 2)
        {
          running_mode = false;
        } else {
          running_mode = true;
         
          running_led.direction = true;
          running_led.position = 0;
          running_led.last_update_time = current_time;
        }
      }
      button_input.click_count = 0;
    }
 } else {
  if (running_mode == false)
  {
    // Opposite value
    display_mode = !display_mode;
  }
  button_input.click_count = 1;
 }

  button_input.last_release = current_time;
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
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer,ADC_CHANNELS) != HAL_OK)
  {
    while (1)
    {
      set_LEDs(0xFF); // All LEDs ON
      HAL_Delay(100);
      set_LEDs(0x00); // All LEDs OFF 
      HAL_Delay(200);
    }
  }

  // Start PWM on all 3 channels
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  // Initial indication: show channel 0 selected
  Indicate_Channel(current_channel);
  
  // Small delay before entering main loop
  HAL_Delay(500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    // Handling button input
    Handle_Button();

    Update_PWM_Outputs();

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

    // Set small delay for button debouncing to prevent CPU overflow
    HAL_Delay(10);

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
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
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
  htim1.Init.Period = 999;
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
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

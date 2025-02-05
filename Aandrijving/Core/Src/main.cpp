/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <main.hpp> // Main header file
#include "CAN_Bus.hpp" // Custom CAN Bus library
#include "stm32f1xx_hal.h" //STM32 HAL library
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
CAN_HandleTypeDef hcan; // CAN handle structure


/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef RxHeader;  // CAN receive header structure
uint8_t RxData[8]; // Array to hold received CAN data
uint32_t can_id = 0; // Variable for received CAN ID
uint32_t received_can_data = 0;  // Variable for received CAN data payload
uint16_t ADC_Read(void); // Function prototype for ADC reading
uint16_t adc_value; // Variable to store ADC value from the potentiometer
uint16_t Deadzone = 75; // Deadzone threshold for the potentiometer, can be adjusted

// Structure to hold CAN messages
typedef struct {
    uint32_t id;       // CAN ID
    uint8_t data[8];   // CAN data payload
} CAN_Message;

#define MAX_UNIQUE_IDS 50  // Maximum unique CAN IDs to store, can be adjusted
CAN_Message message_buffer[MAX_UNIQUE_IDS]; // Buffer to store CAN messages
uint8_t message_count = 0; // Counter for stored unique CAN messages


// Structure for storing CAN error flags
typedef struct {
    uint8_t error_warning_flag;
    uint8_t error_passive_flag;
    uint8_t bus_off_flag;
    uint8_t stuff_error;
    uint8_t form_error;
    uint8_t acknowledgment_error;
    uint8_t bit_recessive_error;
    uint8_t bit_dominant_error;
    uint8_t crc_error;
} CAN_ErrorFlags;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void); // Function to configure system clock
void process_can_message(uint32_t can_id, uint8_t *data); // Process received CAN message
static void MX_GPIO_Init(void); // Initialize GPIO
static void MX_CAN_Init(void); // Initialize CAN
void ADC_Init(void); // Initialize ADC
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Callback function for processing received CAN messages
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
    	// Extract ADC value from the first two bytes of received data
        received_can_data = (RxData[0] << 8) | RxData[1];

        // Clamp value to valid ADC range (0-4095)
        if (received_can_data > 4095) {
            received_can_data = 4095;
        }
    }
}

// Check for CAN errors and update error flags
void check_CAN_errors(CAN_HandleTypeDef *hcan, CAN_ErrorFlags *error_flags) {
    uint32_t error = HAL_CAN_GetError(hcan);

    // Reset all error flags
    error_flags->error_warning_flag = 0;
    error_flags->error_passive_flag = 0;
    error_flags->bus_off_flag = 0;
    error_flags->stuff_error = 0;
    error_flags->form_error = 0;
    error_flags->acknowledgment_error = 0;
    error_flags->bit_recessive_error = 0;
    error_flags->bit_dominant_error = 0;
    error_flags->crc_error = 0;

    // Update error flags based on HAL error codes
    if (error != HAL_CAN_ERROR_NONE) {
        if (error & HAL_CAN_ERROR_EWG) {
            error_flags->error_warning_flag = 1;
        }
        if (error & HAL_CAN_ERROR_EPV) {
            error_flags->error_passive_flag = 1;
        }
        if (error & HAL_CAN_ERROR_BOF) {
            error_flags->bus_off_flag = 1;
        }
        if (error & HAL_CAN_ERROR_STF) {
            error_flags->stuff_error = 1;
        }
        if (error & HAL_CAN_ERROR_FOR) {
            error_flags->form_error = 1;
        }
        if (error & HAL_CAN_ERROR_ACK) {
            error_flags->acknowledgment_error = 1;
        }
        if (error & HAL_CAN_ERROR_BR) {
            error_flags->bit_recessive_error = 1;
        }
        if (error & HAL_CAN_ERROR_BD) {
            error_flags->bit_dominant_error = 1;
        }
        if (error & HAL_CAN_ERROR_CRC) {
            error_flags->crc_error = 1;
        }
    }
}

// Initialize ADC
void ADC_Init(void) {
    // Enable GPIOA and ADC1 clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;

    // Configure PA1 as analog mode
    GPIOA->CRL &= ~GPIO_CRL_MODE1;  // Input mode
    GPIOA->CRL &= ~GPIO_CRL_CNF1;   // Analog mode

    // Configure ADC1
    ADC1->SQR1 = 0;  // 1 conversion in the regular sequence
    ADC1->SQR3 = 1;  // Channel 1 (PA1) is the first in the sequence
    ADC1->CR2 |= ADC_CR2_ADON;  // Enable ADC

    // Start ADC calibration
    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL);  // Wait for calibration to complete
}

// Read ADC value
uint16_t ADC_Read(void) {
    ADC1->CR2 |= ADC_CR2_ADON; // Start ADC conversion
    while (!(ADC1->SR & ADC_SR_EOC)); // Wait for conversion to complete
    return ADC1->DR; // Return ADC result
}

// Initialize PWM using TIM2
void PWM_Init(void) {
    // Enable clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // GPIOA clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // TIM2 clock

    // Configure PA0 (TIM2 CH1) as alternate function push-pull
    GPIOA->CRL &= ~GPIO_CRL_CNF0;         // Clear CNF bits
    GPIOA->CRL |= GPIO_CRL_CNF0_1;       // Set CNF0 to Alternate function output Push-pull
    GPIOA->CRL |= GPIO_CRL_MODE0;        // Set MODE0 to Output mode, max speed 50 MHz

    // Configure TIM2 for PWM
    TIM2->PSC = 71;                      // Prescaler (72 MHz / (PSC+1) = 1 MHz)
    TIM2->ARR = 999;                     // Auto-reload register (1 MHz / 1000 = 1 kHz PWM frequency)
    TIM2->CCR1 = 0;                    // Capture/Compare register (500 = 50% duty cycle)

    // Configure PWM mode
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM mode 1
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;      // Enable preload register on TIM2_CCR1
    TIM2->CCER |= TIM_CCER_CC1E;         // Enable capture/compare for channel 1

    // Enable TIM2
    TIM2->CR1 |= TIM_CR1_ARPE;           // Enable auto-reload preload
    TIM2->CR1 |= TIM_CR1_CEN;            // Enable TIM2
}

// Transmit potentiometer data over CAN
void TransmitPotentiometerData(CANBus &canBus, CAN_HandleTypeDef *hcan) {
    uint8_t TxData[8] = {0}; // Buffer for CAN data
     adc_value = ADC_Read(); // Read ADC value

     // Pack ADC value into CAN message
    TxData[0] = (adc_value >> 8) & 0xFF; // High byte
    TxData[1] = adc_value & 0xFF;        // Low byte

    // Transmit CAN message with ID 0x123
    can_id = 0x123;
    canBus.transmit(hcan, can_id, TxData, 2);
}


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	CANBus canBus;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init(); // Initialize HAL library
  PWM_Init(); // Initialize PWM
  ADC_Init(); // Initialize ADC

  SystemClock_Config(); // Initialize CANBus object
  MX_GPIO_Init(); // Initialize GPIO
  MX_CAN_Init(); // Initialize CAN
  /* USER CODE BEGIN 2 */

  canBus.start(&hcan, CAN_ID_STD); // Start CAN communication
  HAL_CAN_Start(&hcan); // Enable CAN peripheral

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
          // Read the potentiometer value
      TransmitPotentiometerData(canBus, &hcan);
         	if (received_can_data < Deadzone) {
         	  TIM2->CCR1 = 0;  // Set duty cycle to 0 if within deadzone
         	        }
         	else {
         	  TIM2->CCR1 = (received_can_data * TIM2->ARR) / 4095; // Map ADC value (DEADZONE_THRESHOLD to 4095) to PWM duty cycle

          HAL_Delay(100); // Small delay to allow smooth adjustments
//
//
      }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 36;
  hcan.Init.Mode = CAN_MODE_LOOPBACK; // Canbus normal mode = CAN_MODE_NORMAL, canbus loopback mode = CAN_MODE_LOOPBACK
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CANBus canBus;

  canBus.configureFilter(&hcan, 0b00100010000, 0b00110110000, 10, 0);
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
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

#ifdef  USE_FULL_ASSERT
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

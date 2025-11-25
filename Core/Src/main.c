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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cc2500.h"
#include "usbd_cdc_if.h"
#include "cli_handler.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */
CC2500CTX cc2500_ctx;

// Глобальный флаг отладки
volatile uint8_t debug_mode = 1;

// Режим работы (определяется автоматически)
volatile uint8_t is_tx_mode = 0;

// Переменные для режима приема
volatile uint8_t rx_packet_received = 0;
uint8_t rx_buffer[64];
uint8_t rx_length = 0;
uint32_t rx_packet_count = 0;
uint32_t rx_error_count = 0;

// Переменные для режима передачи
uint32_t tx_packet_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// обработчик из cli_handler
void CDC_On_Receive_FS(uint8_t* Buf, uint32_t Len)
{
    cli_process_input(Buf, Len);
}

// Обработчик прерываний GDO (для режима RX)
void cc2500_GDO_IRQHandler(uint16_t GPIO_Pin)
{
    if (!is_tx_mode) {  // Только в режиме RX
        if (GPIO_Pin == GD00_Pin || GPIO_Pin == GD02_Pin) {
            rx_packet_received = 1;
        }
    }
}

// Функция обработки принятого пакета
void process_rx_packet(void)
{
    uint8_t rxbytes;
    uint8_t marcstate;
    uint8_t pktstatus;

    // Проверка состояния чипа
    marcstate = cc2500_getState(&cc2500_ctx);

    // Проверка наличия данных в FIFO
    cc2500_readStatusRegister(&cc2500_ctx, CC2500_3B_RXBYTES, &rxbytes);
    cc2500_readStatusRegister(&cc2500_ctx, CC2500_38_PKTSTATUS, &pktstatus);

    uint8_t fifo_bytes = rxbytes & 0x7F;
    
    if (fifo_bytes >= 1) {
        // Читаем первый байт - длина пакета
        uint8_t pkt_length;
        cc2500_readRegister(&cc2500_ctx, CC2500_3F_RXFIFO, &pkt_length);
        
        // Проверка корректности длины (макс 61 байт + 2 байта статуса)
        if (pkt_length <= 61 && (fifo_bytes >= pkt_length + 2)) {
            // Читаем данные пакета
            rx_length = pkt_length;
            cc2500_readRegisterBurst(&cc2500_ctx, CC2500_3F_RXFIFO, rx_buffer, rx_length);
            
            // Читаем 2 байта статуса (RSSI и LQI|CRC_OK)
            uint8_t status_bytes[2];
            cc2500_readRegisterBurst(&cc2500_ctx, CC2500_3F_RXFIFO, status_bytes, 2);
            
            // Парсинг статуса
            int8_t rssi_raw = (int8_t)status_bytes[0];
            int8_t rssi_dbm = (rssi_raw / 2) - 74;
            uint8_t lqi = status_bytes[1] & 0x7F;
            uint8_t crc_ok = (status_bytes[1] & 0x80) ? 1 : 0;

            // Переключение LED при получении пакета
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

            // Увеличение счетчика пакетов
            rx_packet_count++;

            // Форматирование и отправка через USB CDC
            char usb_buffer[400];
            int pos = 0;

            // Временная метка
            pos += sprintf(usb_buffer + pos, "[%lu] RX: ", HAL_GetTick());

            // Данные в HEX
            for (int i = 0; i < rx_length; i++) {
                pos += sprintf(usb_buffer + pos, "%02X ", rx_buffer[i]);
            }

            // ASCII (если печатные символы)
            pos += sprintf(usb_buffer + pos, "| ASCII: ");
            for (int i = 0; i < rx_length; i++) {
                if (rx_buffer[i] >= 32 && rx_buffer[i] <= 126) {
                    usb_buffer[pos++] = rx_buffer[i];
                } else {
                    usb_buffer[pos++] = '.';
                }
            }

            // RSSI, LQI, CRC и статистика
            pos += sprintf(usb_buffer + pos, " | RSSI: %d dBm, LQI: %u, CRC: %s, PKT: %lu",
                          rssi_dbm, lqi, crc_ok ? "OK" : "FAIL", rx_packet_count);

            // Отладочная информация
            if (debug_mode) {
                pos += sprintf(usb_buffer + pos, "\r\n  DEBUG: LEN=%u, RXBYTES=0x%02X, MARCSTATE=0x%02X",
                              pkt_length, rxbytes, marcstate);
            }

            pos += sprintf(usb_buffer + pos, "\r\n");

            // Отправка через USB CDC
            CDC_Transmit_FS((uint8_t*)usb_buffer, pos);

            // Очистка RX FIFO
            cc2500_strobe(&cc2500_ctx, CC2500_SFRX);

            // Возврат в режим приема
            cc2500_setRxMode(&cc2500_ctx);
        } else {
            // Некорректная длина или недостаточно данных
            if (debug_mode) {
                char debug_msg[100];
                sprintf(debug_msg, "DEBUG: Invalid packet, LEN=%u, RXBYTES=0x%02X\r\n",
                        pkt_length, rxbytes);
                CDC_Transmit_FS((uint8_t*)debug_msg, strlen(debug_msg));
            }
            rx_error_count++;
            cc2500_strobe(&cc2500_ctx, CC2500_SFRX);
            cc2500_setRxMode(&cc2500_ctx);
        }
    } else {
        // Нет данных в FIFO
        if (debug_mode) {
            char debug_msg[100];
            sprintf(debug_msg, "DEBUG: Spurious interrupt, RXBYTES=0x%02X, MARCSTATE=0x%02X\r\n",
                    rxbytes, marcstate);
            CDC_Transmit_FS((uint8_t*)debug_msg, strlen(debug_msg));
        }
        rx_error_count++;
        cc2500_strobe(&cc2500_ctx, CC2500_SFRX);
        cc2500_setRxMode(&cc2500_ctx);
    }
}

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
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  
  // Определение режима работы через пин TRX_SEL (PA0)
  // HIGH = TX mode, LOW = RX mode
  is_tx_mode = (HAL_GPIO_ReadPin(TRX_SEL_GPIO_Port, TRX_SEL_Pin) == GPIO_PIN_SET) ? 1 : 0;
  
  // Инициализация обработчика команд
  cli_init(&cc2500_ctx);

  // Инициализация CC2500
  cc2500_init_full(&cc2500_ctx,
                  CSN_GPIO_Port, CSN_Pin,
                  GD00_GPIO_Port, GD00_Pin,
                  GD02_GPIO_Port, GD02_Pin,
                  PA_EN_GPIO_Port, PA_EN_Pin,
                  RX_EN_GPIO_Port, RX_EN_Pin,
                  &hspi1);

  // Сброс и конфигурация CC2500
  cc2500_reset(&cc2500_ctx);
  HAL_Delay(100);

  cc2500_configure(&cc2500_ctx);
  HAL_Delay(10);

  // Установка мощности передатчика
  cc2500_writeRegister(&cc2500_ctx, CC2500_3E_PATABLE, 0x50);

  // Ожидание инициализации USB
  HAL_Delay(500);

  if (is_tx_mode) {
    // ========== РЕЖИМ ПЕРЕДАЧИ ==========
    char init_msg[] = "CC2500 TX Mode (auto-detected via PA0=HIGH)\r\n";
    CDC_Transmit_FS((uint8_t*)init_msg, strlen(init_msg));
  } else {
    // ========== РЕЖИМ ПРИЕМА ==========
    // Включение прерываний для GDO пинов
    HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    // Переход в режим приема
    cc2500_setRxMode(&cc2500_ctx);

    char init_msg[] = "CC2500 RX Mode (auto-detected via PA0=LOW)\r\n";
    CDC_Transmit_FS((uint8_t*)init_msg, strlen(init_msg));
  }
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t counter = 0;
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    if (is_tx_mode) {
      // ========== ЦИКЛ РЕЖИМА ПЕРЕДАЧИ ==========
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

      uint8_t tx_buffer[8];
      int message_len = sprintf((char*)tx_buffer, "PING%d", counter++);

      for (int i = message_len; i < 8; i++) {
          tx_buffer[i] = 0;
      }

      cc2500_transmit(&cc2500_ctx, tx_buffer, 8);
      tx_packet_count++;

      if (tx_packet_count % 10 == 0) {
          char stat_msg[64];
          sprintf(stat_msg, "TX: %lu packets sent\r\n", tx_packet_count);
          CDC_Transmit_FS((uint8_t*)stat_msg, strlen(stat_msg));
      }

      HAL_Delay(500);
    } else {
      // ========== ЦИКЛ РЕЖИМА ПРИЕМА ==========
      if (rx_packet_received) {
          rx_packet_received = 0;
          process_rx_packet();
      }

      HAL_Delay(1);
    }
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
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PA_EN_Pin|RX_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRX_SEL_Pin */
  GPIO_InitStruct.Pin = TRX_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TRX_SEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CSN_Pin */
  GPIO_InitStruct.Pin = CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GD00_Pin GD02_Pin */
  GPIO_InitStruct.Pin = GD00_Pin|GD02_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA_EN_Pin RX_EN_Pin */
  GPIO_InitStruct.Pin = PA_EN_Pin|RX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

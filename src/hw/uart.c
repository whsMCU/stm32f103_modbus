/*
 * uart.c
 *
 *  Created on: 2020. 12. 8.
 *      Author: baram
 */


#include "uart.h"
#include "ring_buffer.h"


static bool is_open[UART_MAX_CH];
#define MAX_SIZE 128

static qbuffer_t ring_buffer[UART_MAX_CH];
static volatile uint8_t rx_buf[UART_MAX_CH-1][MAX_SIZE];
static volatile uint8_t rx_buf1[MAX_SIZE];

static volatile uint8_t rx_data[UART_MAX_CH];

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;


const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000,
        400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000}; // see baudRate_e

#define BAUD_RATE_COUNT (sizeof(baudRates) / sizeof(baudRates[0]))

bool uartInit(void)
{
  for (int i=0; i<UART_MAX_CH; i++)
  {
    is_open[i] = false;
  }

  return true;
}

bool uartOpen(uint8_t ch, uint32_t baud)
{
  bool ret = false;

  switch(ch)
  {
    case _DEF_UART1:
	  /* DMA controller clock enable */
	  __HAL_RCC_DMA1_CLK_ENABLE();

	  /* DMA interrupt init */
	  /* DMA1_Channel4_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	  /* DMA1_Channel5_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

      huart1.Instance = USART1;
      huart1.Init.BaudRate = baud;
      huart1.Init.WordLength = UART_WORDLENGTH_8B;
      huart1.Init.StopBits = UART_STOPBITS_1;
      huart1.Init.Parity = UART_PARITY_NONE;
      huart1.Init.Mode = UART_MODE_TX_RX;
      huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      huart1.Init.OverSampling = UART_OVERSAMPLING_16;

      qbufferCreate(&ring_buffer[ch], (uint8_t *)&rx_buf1, MAX_SIZE);

      if (HAL_UART_Init(&huart1) != HAL_OK)
      {
        ret = false;
      }
      else
      {
        ret = true;
        is_open[ch] = true;
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_data[_DEF_UART1], 1);
      }
      break;
  }

  return ret;
}

bool uartIsConnected(uint8_t ch)
{
  return is_open[ch];
}

uint32_t uartAvailable(uint8_t ch)
{
  uint32_t ret = 0;

  switch(ch)
  {
    case _DEF_UART1:
      //ring_buffer[ch].in = (ring_buffer[ch].len - hdma_usart2_rx.Instance->NDTR);
      ret = qbufferAvailable(&ring_buffer[ch]);
      break;
  }
  return ret;
}

bool uartTxBufEmpty(uint8_t ch)
{
  bool ret = 0;

  switch(ch)
  {
    case _DEF_UART1:
      ret = qbufferTxEmpty(&ring_buffer[ch]);
      break;
  }
  return ret;
}

uint32_t uartTotalTxBytesFree(uint8_t ch)
{
  uint32_t ret = 0;

  switch(ch)
  {
    case _DEF_UART1:
      ret = qbufferTxBytesFree(&ring_buffer[ch]);
      break;
  }
  return ret;
}

void waitForSerialPortToFinishTransmitting(uint8_t ch)
{
    while (!uartTxBufEmpty(ch)) {
        delay(10);
    };
}

uint8_t uartRead(uint8_t ch)
{
  uint8_t ret = 0;

  switch(ch)
  {
    case _DEF_UART1:
      qbufferRead(&ring_buffer[ch], &ret, 1);
      break;
  }

  return ret;
}

uint8_t uartFlush(uint8_t ch)
{
	  uint8_t ret = 0;

	  switch(ch)
	  {
	    case _DEF_UART1:
	    	qbufferFlush(&ring_buffer[ch]);
	      break;
	  }

	  return ret;
}

uint32_t uartWrite(uint8_t ch, uint8_t *p_data, uint32_t length)
{
  uint32_t ret = 0;
  HAL_StatusTypeDef status;

  switch(ch)
  {
    case _DEF_UART1:
      status = HAL_UART_Transmit(&huart1, p_data, length, 100);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;
  }

  return ret;
}

uint32_t uartWriteIT(uint8_t ch, uint8_t *p_data, uint32_t length)
{
  uint32_t ret = 0;
  HAL_StatusTypeDef status;

  switch(ch)
  {
    case _DEF_UART1:
      status = HAL_UART_Transmit_IT(&huart1, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;
  }

  return ret;
}

uint32_t uartWriteDMA(uint8_t ch, uint8_t *p_data, uint32_t length)
{
  uint32_t ret = 0;
  HAL_StatusTypeDef status;

  switch(ch)
  {
    case _DEF_UART1:
      status = HAL_UART_Transmit_DMA(&huart1, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;
  }

  return ret;
}

void serialPrint(uint8_t channel, const char *str)
{
    uint8_t ch;
    while ((ch = *(str++)) != 0) {
      uartWrite(channel, &ch, 1);
    }
}

uint32_t uartPrintf(uint8_t ch, char *fmt, ...)
{
  char buf[MAX_SIZE];
  va_list args;
  int len;
  uint32_t ret;

  va_start(args, fmt);
  len = vsnprintf(buf, MAX_SIZE, fmt, args);

  ret = uartWrite(ch, (uint8_t *)buf, len);

  va_end(args);


  return ret;
}

uint32_t uartPrintf_IT(uint8_t ch, char *fmt, ...)
{
  char buf[MAX_SIZE];
  va_list args;
  int len;
  uint32_t ret;

  va_start(args, fmt);
  len = vsnprintf(buf, MAX_SIZE, fmt, args);

  ret = uartWriteIT(ch, (uint8_t *)buf, len);

  va_end(args);


  return ret;
}

uint32_t uartGetBaud(uint8_t ch)
{
  uint32_t ret = 0;

  switch(ch)
  {
    case _DEF_UART1:
      ret = huart1.Init.BaudRate;
      break;
  }

  return ret;
}

bool uartSetBaud(uint8_t ch, uint32_t baud)
{
	bool ret = false;

	switch(ch)
	{
		case _DEF_UART1:
		  huart1.Init.BaudRate = baud;
		  if (HAL_UART_Init(&huart1) != HAL_OK)
		  {
			ret = false;
		  }else
		  {
			ret = true;
		  }
		  break;
	}

	return ret;
}

baudRate_e lookupBaudRateIndex(uint32_t baudRate)
{
    uint8_t index;

    for (index = 0; index < BAUD_RATE_COUNT; index++) {
        if (baudRates[index] == baudRate) {
            return index;
        }
    }
    return BAUD_AUTO;
}

uint8_t uart1_rx_data = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_data[_DEF_UART1], 1);
    qbufferWrite(&ring_buffer[_DEF_UART1], (uint8_t *)&rx_data[_DEF_UART1], 1);
    uart1_rx_data = uartRead(_DEF_UART1);
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

 GPIO_InitTypeDef GPIO_InitStruct = {0};
 if(uartHandle->Instance==USART1)
 {
 /* USER CODE BEGIN USART1_MspInit 0 */

 /* USER CODE END USART1_MspInit 0 */
   /* USART1 clock enable */
   __HAL_RCC_USART1_CLK_ENABLE();

   __HAL_RCC_GPIOA_CLK_ENABLE();
   /**USART1 GPIO Configuration
   PA9     ------> USART1_TX
   PA10     ------> USART1_RX
   */
   GPIO_InitStruct.Pin = GPIO_PIN_9;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   GPIO_InitStruct.Pin = GPIO_PIN_10;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   /* USART1 DMA Init */
   /* USART1_RX Init */
   hdma_usart1_rx.Instance = DMA1_Channel5;
   hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
   hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
   hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
   hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
   hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
   hdma_usart1_rx.Init.Mode = DMA_NORMAL;
   hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
   HAL_DMA_Init(&hdma_usart1_rx);


   __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

   /* USART1_TX Init */
   hdma_usart1_tx.Instance = DMA1_Channel4;
   hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
   hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
   hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
   hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
   hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
   hdma_usart1_tx.Init.Mode = DMA_NORMAL;
   hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
   HAL_DMA_Init(&hdma_usart1_tx);

   __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

   /* USART1 interrupt Init */
   HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(USART1_IRQn);
 /* USER CODE BEGIN USART1_MspInit 1 */

 /* USER CODE END USART1_MspInit 1 */
 }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
	/* Peripheral clock disable */
	__HAL_RCC_USART1_CLK_DISABLE();

	/**USART1 GPIO Configuration
	PA9     ------> USART1_TX
	PA10     ------> USART1_RX
	*/
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

	/* USART1 DMA DeInit */
	HAL_DMA_DeInit(uartHandle->hdmarx);
	HAL_DMA_DeInit(uartHandle->hdmatx);

	/* USART1 interrupt Deinit */
	HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

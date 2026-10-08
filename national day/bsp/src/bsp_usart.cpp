#include "bsp_usart.hpp"
#include "usart.h" 

/* 接收缓冲区，以及"这一帧收了多少字节" */
uint8_t  usart1_rx_buf[64];
uint8_t  usart6_rx_buf[64];
volatile uint16_t usart1_rx_len = 0;
volatile uint16_t usart6_rx_len = 0;


void USART_Init(void)
{
  USART1_Init();
  USART6_Init();
}

void USART6_Init()
{
  // TODO: 配置 USART6 的 DMA 接收、发送及空闲中断，并启动接收。
  HAL_UART_Receive_DMA(&huart6, usart6_rx_buf, sizeof(usart6_rx_buf));
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
};

void USART1_Init()
{
  // TODO: 配置 USART1 的 DMA 接收、发送及空闲中断，并启动接收。
  HAL_UART_Receive_DMA(&huart1, usart1_rx_buf, sizeof(usart1_rx_buf));
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
};

void USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, enum USART_Mode mode)
{
  if (huart == nullptr || pData == nullptr || Size == 0U) return;

  switch (mode)
  {
    case USART_MODE_BLOCK:
        HAL_UART_Transmit(huart, pData, Size, 100);
        break;
    case USART_MODE_DMA:
        HAL_UART_Transmit_DMA(huart, pData, Size);
        break;
    case USART_MODE_IT:
        HAL_UART_Transmit_IT(huart, pData, Size);
        break;
    default:
        break;
  }
  // TODO: 检查参数，并按 mode 选择阻塞、DMA 或中断发送。
  
}

void USART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  // TODO: 选择接收方式并启动 HAL UART 接收。
  
  HAL_UART_Receive_DMA(huart, pData, Size);
}

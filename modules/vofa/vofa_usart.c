#include "stdio.h"
#include "usart.h"
#include "bsp_usart.h"

// Printf 重定向 - 覆盖标准库的 fputc
// 使用 __attribute__((used)) 确保链接器不会优化掉这个函数
int fputc(int ch, FILE *f) __attribute__((used));

int fputc(int ch, FILE *f) {
  HAL_UART_Transmit(&huart8, (uint8_t *)&ch, 1, 100);
  return ch;
}

// 如果需要非阻塞方式，可以使用下面的 DMA 版本（注释掉上面的阻塞版本）
/*
#define TX_BUF_SIZE 256
static uint8_t tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_index = 0;
static volatile uint8_t is_sending = 0;

int fputc(int ch, FILE *f) {
  // 等待之前的发送完成
  while (is_sending);
  
  tx_buf[tx_index++] = ch;
  
  // 当缓冲区满或遇到换行符时发送
  if (tx_index >= TX_BUF_SIZE || ch == '\n') {
    is_sending = 1;
    HAL_UART_Transmit_DMA(&huart8, tx_buf, tx_index);
    tx_index = 0;
  }
  
  return ch;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == UART8) {
    is_sending = 0;
  }
}
*/

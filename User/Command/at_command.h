/**
  ******************************************************************************
  * @file    at_command.h
  * @brief   AT命令处理头文件
  ******************************************************************************
  */

#ifndef AT_COMMAND_H
#define AT_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/
// 缓冲区大小定义
#define AT_RX_BUFFER_SIZE     1024  // 接收缓冲区大小
#define AT_CMD_MAX_LEN        1024  // 最大命令长度
#define AT_TX_BUFFER_SIZE     256  // 发送缓冲区大小
#define AT_TX_QUEUE_SIZE      32   // 发送队列大小

/* Exported types ------------------------------------------------------------*/
// AT命令处理状态
typedef enum {
    AT_OK = 0,              // 成功
    AT_ERROR,               // 一般错误
    AT_PARAM_ERROR,         // 参数错误
    AT_UNKNOWN_CMD,         // 未知命令
    AT_EXECUTION_ERROR      // 执行错误
} AtCmdStatus_t;

// 发送缓冲区结构
//typedef struct {
//    uint8_t data[AT_TX_BUFFER_SIZE];
//    uint16_t length;
//} TxBuffer_t;

/* Exported functions --------------------------------------------------------*/
// 初始化AT命令处理模块
void AT_Init(UART_HandleTypeDef *huart);

// AT命令主循环处理函数
void AT_MainLoopHandler(void);

// 处理AT命令
AtCmdStatus_t AT_ProcessCommand(char *cmd);

// 发送AT响应
void AT_SendResponse(const char *format, ...);

// 发送错误响应
void AT_SendErrorResponse(AtCmdStatus_t status);

// UART中断回调函数
void AT_UART_IdleCallback(UART_HandleTypeDef *huart);
//void AT_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void AT_UART_RxCpltCallback(UART_HandleTypeDef *huart);

// 获取队列状态
uint8_t AT_GetTxQueueCount(void);
uint8_t AT_GetTxQueueSize(void);

#ifdef __cplusplus
}
#endif

#endif /* AT_COMMAND_H */
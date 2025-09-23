#ifndef USER_LOG_H
#define USER_LOG_H

#include "stm32f3xx_hal.h"
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

// 日志级别定义
typedef enum {
  LOG_LEVEL_DEBUG = 0,
  LOG_LEVEL_INFO,
  LOG_LEVEL_WARNING,
  LOG_LEVEL_ERROR
} LogLevel_t;

// 设置最小日志级别，低于此级别的不会输出
#ifndef MIN_LOG_LEVEL
#define MIN_LOG_LEVEL LOG_LEVEL_DEBUG
#endif

// 日志宏
#define LOG_DEBUG(fmt, ...)   Log_Print(LOG_LEVEL_DEBUG, "[DEBUG] " fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)    Log_Print(LOG_LEVEL_INFO, "[INFO] " fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)    Log_Print(LOG_LEVEL_WARNING, "[WARN] " fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)   Log_Print(LOG_LEVEL_ERROR, "[ERROR] " fmt, ##__VA_ARGS__)

// 日志初始化与输出函数
void Log_Init(UART_HandleTypeDef* huart);
void Log_Print(LogLevel_t level, const char* fmt, ...);

#ifdef __cplusplus
}
#endif

#endif // USER_LOG_H
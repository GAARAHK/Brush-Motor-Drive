#include "log.h"
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef* log_huart = NULL;
static LogLevel_t current_log_level = MIN_LOG_LEVEL;

void Log_Init(UART_HandleTypeDef* huart)
{
  log_huart = huart;
}

void Log_Print(LogLevel_t level, const char* fmt, ...)
{
  if (!log_huart || level < current_log_level)
    return;
    
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  
  if (n > 0) {
    HAL_UART_Transmit(log_huart, (uint8_t*)buf, 
                     (uint16_t)((n < (int)sizeof(buf)) ? n : (int)sizeof(buf)), 100);
  }
}
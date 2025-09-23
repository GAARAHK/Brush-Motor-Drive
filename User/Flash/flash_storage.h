#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include "stm32f3xx_hal.h"
#include <stdint.h>

// 配置存储结构
typedef struct {
    uint32_t magic;       // 魔术数字，用于验证配置有效性
    uint16_t device_id;   // 设备ID
    uint16_t crc;         // 简单的校验和
} FlashConfig_t;

// 函数声明
void Flash_Init(void);
uint16_t Flash_ReadDeviceID(void);
HAL_StatusTypeDef Flash_WriteDeviceID(uint16_t device_id);

#endif // FLASH_STORAGE_H
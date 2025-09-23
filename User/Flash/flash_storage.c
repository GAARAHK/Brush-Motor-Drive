#include "flash_storage.h"
#include "log.h"

// 常量定义
#define FLASH_CONFIG_PAGE_ADDR  0x0800F800  // 使用最后一页 (64KB Flash的最后2KB)
#define FLASH_MAGIC_NUMBER      0xA5B6C7D8  // 魔术数字

// 静态函数声明
static uint16_t CalculateCRC(FlashConfig_t *config);
static HAL_StatusTypeDef Flash_ErasePage(uint32_t page_address);
static HAL_StatusTypeDef Flash_WriteData(uint32_t address, uint32_t *data, uint32_t data_length);

// 初始化Flash存储
void Flash_Init(void)
{
    FlashConfig_t *flash_config = (FlashConfig_t *)FLASH_CONFIG_PAGE_ADDR;
    
    // 检查Flash中是否已有有效配置
    if (flash_config->magic != FLASH_MAGIC_NUMBER) {
        // 没有有效配置，初始化默认值
        LOG_INFO("No valid configuration found in Flash, initializing defaults\r\n");
        
        // 默认设备ID为1
        Flash_WriteDeviceID(1);
    } else {
        // 校验CRC
        uint16_t calc_crc = CalculateCRC(flash_config);
        if (calc_crc != flash_config->crc) {
            LOG_WARN("Flash configuration CRC error, initializing defaults\r\n");
            Flash_WriteDeviceID(1);
        } else {
            LOG_INFO("Valid configuration found in Flash, Device ID: %d\r\n", flash_config->device_id);
        }
    }
}

// 读取设备ID
uint16_t Flash_ReadDeviceID(void)
{
    FlashConfig_t *flash_config = (FlashConfig_t *)FLASH_CONFIG_PAGE_ADDR;
    
    // 检查是否有有效配置
    if (flash_config->magic != FLASH_MAGIC_NUMBER) {
        return 1;  // 默认ID
    }
    
    // 校验CRC
    uint16_t calc_crc = CalculateCRC(flash_config);
    if (calc_crc != flash_config->crc) {
        return 1;  // 默认ID
    }
    
    return flash_config->device_id;
}

// 写入设备ID
HAL_StatusTypeDef Flash_WriteDeviceID(uint16_t device_id)
{
    FlashConfig_t new_config;
    new_config.magic = FLASH_MAGIC_NUMBER;
    new_config.device_id = device_id;
    new_config.crc = 0; // 临时值
    new_config.crc = CalculateCRC(&new_config);
    
    // 解锁Flash
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        LOG_ERROR("Failed to unlock Flash: %d\r\n", status);
        return status;
    }
    
    // 擦除页
    status = Flash_ErasePage(FLASH_CONFIG_PAGE_ADDR);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        LOG_ERROR("Failed to erase Flash page: %d\r\n", status);
        return status;
    }
    
    // 写入配置
    status = Flash_WriteData(FLASH_CONFIG_PAGE_ADDR, (uint32_t *)&new_config, sizeof(new_config) / sizeof(uint32_t));
    
    // 锁定Flash
    HAL_FLASH_Lock();
    
    if (status != HAL_OK) {
        LOG_ERROR("Failed to write configuration to Flash: %d\r\n", status);
    } else {
        LOG_INFO("Device ID %d successfully written to Flash\r\n", device_id);
    }
    
    return status;
}

// 计算简单的CRC校验和
//static uint16_t CalculateCRC(FlashConfig_t *config)
//{
//    uint16_t crc = 0;
//    uint16_t *data = (uint16_t *)config;
//    uint32_t length = sizeof(FlashConfig_t) / sizeof(uint16_t);
//    
//    // 保存原始CRC值
//    uint16_t original_crc = config->crc;
//    config->crc = 0;
//    
//    // 计算XOR校验和
//    for (uint32_t i = 0; i < (length-1); i++) {
//        crc ^= data[i];
//    }
//    
//    // 恢复原始CRC值
//    config->crc = original_crc;
//    
//    return crc;
//	
//}


// 修正的CRC计算函数
static uint16_t CalculateCRC(FlashConfig_t *config)
{
    uint16_t crc = 0;
    
    // 创建临时拷贝，避免修改原始数据
    FlashConfig_t temp_config = *config;
    
    // 计算前将CRC字段置零
    temp_config.crc = 0;
    
    // 按16位字计算XOR校验和
    uint16_t *data = (uint16_t *)&temp_config;
    uint32_t length = sizeof(FlashConfig_t) / sizeof(uint16_t);
    
    // 计算所有字段（包括已置零的CRC字段）的XOR
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
    }
    
    // 确保CRC不为零（如果计算结果为零，使用一个特定值）
    if (crc == 0) {
        crc = 0xFFFF;  // 避免CRC为零
    }
    
    return crc;
}

// 擦除Flash页
static HAL_StatusTypeDef Flash_ErasePage(uint32_t page_address)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error = 0;
    
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = page_address;
    erase_init.NbPages = 1;
    
    return HAL_FLASHEx_Erase(&erase_init, &page_error);
}

// 写入数据到Flash
static HAL_StatusTypeDef Flash_WriteData(uint32_t address, uint32_t *data, uint32_t data_length)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    for (uint32_t i = 0; i < data_length; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + (i * 4), data[i]);
        if (status != HAL_OK) {
            break;
        }
    }
    
    return status;
}
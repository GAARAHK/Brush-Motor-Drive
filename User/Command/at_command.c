#include "at_command.h"
#include "cmd_motor.h"
#include "cmd_parser.h"
#include "log.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// 接收缓冲区
static uint8_t at_rx_buffer[AT_RX_BUFFER_SIZE];
static char at_cmd_buffer[AT_CMD_MAX_LEN];
static bool at_cmd_ready = false;
static uint16_t at_cmd_len = 0;

// 发送缓冲区和队列
//typedef struct {
//    uint8_t data[AT_TX_BUFFER_SIZE];
//    uint16_t length;
//} TxBuffer_t;

//static TxBuffer_t at_tx_queue[AT_TX_QUEUE_SIZE];
//static uint8_t at_tx_queue_head = 0;
//static uint8_t at_tx_queue_tail = 0;
//static uint8_t at_tx_queue_count = 0;
//static bool at_tx_busy = false;

// 用于通信的 UART 句柄
static UART_HandleTypeDef *at_uart;

// 初始化 AT 命令处理器
void AT_Init(UART_HandleTypeDef *huart) {
    at_uart = huart;
    at_cmd_ready = false;
    at_cmd_len = 0;
    
    // 初始化发送队列
//    at_tx_queue_head = 0;
//    at_tx_queue_tail = 0;
//    at_tx_queue_count = 0;
//    at_tx_busy = false;
    
    // 清空接收缓冲区
    memset(at_rx_buffer, 0, AT_RX_BUFFER_SIZE);
    
    // 使能UART空闲中断
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    
    // 启动DMA接收
    HAL_UART_Receive_DMA(huart, at_rx_buffer, AT_RX_BUFFER_SIZE);
    
    //LOG_INFO("AT Command processor initialized with DMA\r\n");
}

/**
  * @brief  AT命令主循环处理函数，应在主循环中调用
  * @param  无
  * @retval 无
  */
void AT_MainLoopHandler(void) {
    // 处理命令
    if (at_cmd_ready) {
        at_cmd_ready = false;
        AT_ProcessCommand(at_cmd_buffer);
    }
    
    // 执行其他周期性任务
    MotorCmd_PeriodicHandler();
}

/**
  * @brief  UART空闲中断回调函数
  * @param  huart UART句柄
  * @retval 无
  */
void AT_UART_IdleCallback(UART_HandleTypeDef *huart) {
    if (huart == at_uart) {
        // 空闲中断表示一帧数据接收完成
        
        // 计算接收到的数据长度
        uint16_t dma_counter = __HAL_DMA_GET_COUNTER(huart->hdmarx);
        at_cmd_len = AT_RX_BUFFER_SIZE - dma_counter;
        
        // 暂停DMA接收
        HAL_StatusTypeDef status = HAL_UART_DMAStop(huart);
        if (status != HAL_OK) {
            LOG_WARN("Failed to stop DMA, status: %d\r\n", status);
        }
        
        // 检查接收的数据是否合法
        if (at_cmd_len > 0 && at_cmd_len < AT_CMD_MAX_LEN) {
            // 复制到命令缓冲区并移除可能的\r\n
            memcpy(at_cmd_buffer, at_rx_buffer, at_cmd_len);
            
            // 查找并移除结尾的\r\n
            if (at_cmd_len >= 2 && 
                at_rx_buffer[at_cmd_len-2] == '\r' && 
                at_rx_buffer[at_cmd_len-1] == '\n') {
                at_cmd_len -= 2;
            }
            
            at_cmd_buffer[at_cmd_len] = '\0';
            
            LOG_DEBUG("Command received: %s\r\n", at_cmd_buffer);
            
            // 设置命令就绪标志
            at_cmd_ready = true;
            
            // 优先处理关键命令
            if (strstr(at_cmd_buffer, "AT+MotorStop") != NULL ||
                strstr(at_cmd_buffer, "AT+MotorStopCustom") != NULL ||
                strstr(at_cmd_buffer, "AT+MotorStatusUpLoadStop") != NULL) {
                
                // 立即处理停止类命令
                AT_ProcessCommand(at_cmd_buffer);
                at_cmd_ready = false;
                
                LOG_INFO("Priority command processed immediately\r\n");
            }
        }
        
        // 清空缓冲区
        memset(at_rx_buffer, 0, AT_RX_BUFFER_SIZE);
        
       
        
        // 重新启动DMA接收
        status = HAL_UART_Receive_DMA(huart, at_rx_buffer, AT_RX_BUFFER_SIZE);
        if (status != HAL_OK) {
            LOG_ERROR("Failed to restart DMA reception, status: %d\r\n", status);
        }
        
        // 确保空闲中断保持启用
//        __HAL_UART_CLEAR_IDLEFLAG(huart);
//        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    }
}


/**
  * @brief  UART DMA 接收完成回调
  * @param  huart UART句柄
  * @retval 无
  */
void AT_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == at_uart) {
       
       
    }
}


/**
  * @brief  启动下一个DMA发送
  * @param  无
  * @retval 无
  */
//static void AT_StartNextTransmit(void) {
//    if (at_tx_queue_count > 0 && !at_tx_busy) {
//        // 获取队列头部数据
//        TxBuffer_t *buffer = &at_tx_queue[at_tx_queue_head];
//        
//        // 标记为忙状态
//        at_tx_busy = true;
//        
//        // 开始发送
//        HAL_UART_Transmit(at_uart, buffer->data, buffer->length,1000);
//        
//        // 更新队列头
//        at_tx_queue_head = (at_tx_queue_head + 1) % AT_TX_QUEUE_SIZE;
//        
//        // 原子操作减少队列计数
//        __disable_irq();
//        at_tx_queue_count--;
//        __enable_irq();
//    }
//}



/**
  * @brief  UART 发送完成回调函数
  * @param  huart UART句柄
  * @retval 无
  */
//void AT_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
//    if (huart == at_uart) {
//        // 标记发送完成
//        at_tx_busy = false;
//        
//        // 检查是否有更多数据需要发送
//        AT_StartNextTransmit();
//    }
//}


/**
  * @brief  获取当前发送队列中的消息数量
  * @param  无
  * @retval 当前队列中的消息数量
  */
//uint8_t AT_GetTxQueueCount(void) {
//    return at_tx_queue_count;
//}

///**
//  * @brief  获取发送队列的总容量
//  * @param  无
//  * @retval 队列总容量
//  */
//uint8_t AT_GetTxQueueSize(void) {
//    return AT_TX_QUEUE_SIZE;
//}

/**
  * @brief  向上位机发送响应
  * @param  format 格式化字符串
  * @param  ... 可变参数
  * @retval 无
  */
void AT_SendResponse(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len > 0 && len < sizeof(buffer)) {
        // 添加结束符 \r\n
        buffer[len++] = '\r';
        buffer[len++] = '\n';
        
        // 通过 UART 发送
        HAL_UART_Transmit(at_uart, (uint8_t*)buffer, len, 100);
    }
} 

/**
  * @brief  处理完整的AT命令
  * @param  cmd 命令字符串
  * @retval 命令处理状态
  */
AtCmdStatus_t AT_ProcessCommand(char *cmd) {
    //LOG_DEBUG("Processing AT command: %s\r\n", cmd);
    
    // 检查是否是AT命令
    if (strncmp(cmd, "AT+", 3) != 0) {
        LOG_WARN("Invalid AT command format\r\n");
        return AT_ERROR;
    }
    
    // 提取命令名称
    char *cmd_name = cmd + 3;  // 跳过 "AT+"
    
    // 检查是否是查询命令（带有问号）
    char *question_mark = strchr(cmd_name, '?');
    if (question_mark != NULL) {
        *question_mark = '\0';  // 移除问号
        
        // 处理查询命令
        if (strcmp(cmd_name, "MotorSetDevID") == 0) {
            return MotorCmd_QueryDevID();
        }
        else if (strcmp(cmd_name, "QueryVersion") == 0) {
            return MotorCmd_QueryVersion();
        }
        // 可以添加更多查询命令...
        
        LOG_WARN("Unknown query command: %s?\r\n", cmd_name);
        return AT_UNKNOWN_CMD;
    }
    
    // 处理带参数的命令
    char *param_start = strchr(cmd_name, '=');
    if (param_start != NULL) {
        *param_start = '\0';  // 分隔命令名和参数
        param_start++;        // 参数开始位置
        
        // 处理各种命令...
        if (strcmp(cmd_name, "MotorRun") == 0) {
            return MotorCmd_Run(param_start);
        }
        else if (strcmp(cmd_name, "MotorStop") == 0) {
            return MotorCmd_Stop(param_start);
        }
        else if (strcmp(cmd_name, "MotorStatusQuery") == 0) {
            return MotorCmd_StatusQuery(param_start);
        }
        else if (strcmp(cmd_name, "MotorStatusUpLoad") == 0) {
            return MotorCmd_StatusUpLoad(param_start);
        }
        else if (strcmp(cmd_name, "MotorStatusUpLoadStop") == 0) {
            return MotorCmd_StatusUpLoadStop(param_start);
        }
        else if (strcmp(cmd_name, "MotorSetDevID") == 0) {
            return MotorCmd_SetDevID(param_start);
        }
        else if (strcmp(cmd_name, "MotorSetType") == 0) {
            return MotorCmd_SetType(param_start);
        }
        else if (strcmp(cmd_name, "MotorAlarmValue") == 0) {
            return MotorCmd_AlarmValue(param_start);
        }
        else if (strcmp(cmd_name, "MotorRunRepeat") == 0) {
            return MotorCmd_RunRepeat(param_start);
        }
        else if (strcmp(cmd_name, "MotorStopRepeat") == 0) {
            return MotorCmd_StopRepeat(param_start);
        }
        else if (strcmp(cmd_name, "MotorRunCustom") == 0) {
            return MotorCmd_RunCustom(param_start);
        }
        else if (strcmp(cmd_name, "MotorStopCustom") == 0) {
            return MotorCmd_StopCustom(param_start);
        }
		
		
		else if (strcmp(cmd_name, "MotorPairRunRepeat") == 0) {
            return MotorCmd_PairRunRepeat(param_start);
        }
        else if (strcmp(cmd_name, "MotorPairStopRepeat") == 0) {
            return MotorCmd_PairStopRepeat(param_start);
        }
        else if (strcmp(cmd_name, "MotorPairRunCustom") == 0) {
            return MotorCmd_PairRunCustom(param_start);
        }
        else if (strcmp(cmd_name, "MotorPairStopCustom") == 0) {
            return MotorCmd_PairStopCustom(param_start);
        }
		
		
		
		
		
		
		//换气阀开始命令
		else if (strcmp(cmd_name, "HuanQi_Start") == 0) {
            return HuanQi_Start(param_start);
        }
		
		// 在AT_ProcessCommand函数中添加新命令支持.电流环，速度环控制
		if (strcmp(cmd_name, "MotorControlMode") == 0) {
			return MotorCmd_SetControlMode(param_start);
		}
		else if (strcmp(cmd_name, "MotorSetTorque") == 0) {
			return MotorCmd_SetTorque(param_start);
		}
		else if (strcmp(cmd_name, "MotorSetSpeed") == 0) {
			return MotorCmd_SetTargetSpeed(param_start);
		}
		else if (strcmp(cmd_name, "MotorCurrentPID") == 0) {
			return MotorCmd_SetCurrentPID(param_start);
		}
		else if (strcmp(cmd_name, "MotorSpeedPID") == 0) {
			return MotorCmd_SetSpeedPID(param_start);
		}
		else if (strcmp(cmd_name, "MotorControlQuery") == 0) {
			return MotorCmd_QueryControlStatus(param_start);
		}
		else if (strcmp(cmd_name, "MotorEnable") == 0) {
			return MotorCmd_SetEnable(param_start);
		}
				
		
		
    }
    else {
        // 处理不带参数的命令
        if (strcmp(cmd_name, "QueryVersion") == 0) {
            return MotorCmd_QueryVersion();
        }
		
		//换气阀开始命令
		else if (strcmp(cmd_name, "HuanQi_Stop") == 0) {
            return HuanQi_Stop(param_start);
        }
        // 其他不带参数的命令...
    }
    
    LOG_WARN("Unknown AT command: %s\r\n", cmd_name);
    return AT_UNKNOWN_CMD;
}

/**
  * @brief  发送AT错误响应
  * @param  status 错误状态
  * @retval 无
  */
void AT_SendErrorResponse(AtCmdStatus_t status) {
    switch (status) {
        case AT_OK:
            AT_SendResponse("OK");
            break;
        case AT_ERROR:
            AT_SendResponse("ERROR");
            break;
        case AT_PARAM_ERROR:
            AT_SendResponse("ERROR:PARAM");
            break;
        case AT_UNKNOWN_CMD:
            AT_SendResponse("ERROR:UNKNOWN_CMD");
            break;
        case AT_EXECUTION_ERROR:
            AT_SendResponse("ERROR:EXECUTION");
            break;
        default:
            AT_SendResponse("ERROR:UNKNOWN");
            break;
    }
}
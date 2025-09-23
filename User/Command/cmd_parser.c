#include "cmd_parser.h"
#include <string.h>
#include <stdlib.h>

// 从字符串中解析整数
int CmdParser_ParseInt(const char *str, int *value) {
    if (str == NULL || value == NULL) {
        return 0;
    }
    
    char *end;
    *value = (int)strtol(str, &end, 10);
    
    // 检查是否成功解析
    if (end == str) {
        return 0;
    }
    
    return 1;
}

// 从字符串中解析指定位置的参数
int CmdParser_GetParam(const char *params, int index, char *param, int param_size) {
    if (params == NULL || param == NULL || param_size <= 0) {
        return 0;
    }
    
    // 初始化输出参数
    param[0] = '\0';
    
    // 查找第 index 个参数
    const char *start = params;
    int current = 0;
    
    while (current < index) {
        start = strchr(start, ',');
        if (start == NULL) {
            return 0;  // 没有足够的参数
        }
        start++;  // 跳过逗号
        current++;
    }
    
    // 找到参数结束位置
    const char *end = strchr(start, ',');
    if (end == NULL) {
        end = start + strlen(start);
    }
    
    // 计算参数长度
    int len = end - start;
    if (len >= param_size) {
        len = param_size - 1;
    }
    
    // 复制参数
    strncpy(param, start, len);
    param[len] = '\0';
    
    return 1;
}

// 计算参数数量
int CmdParser_CountParams(const char *params) {
    if (params == NULL || *params == '\0') {
        return 0;
    }
    
    int count = 1;  // 至少有一个参数
    const char *p = params;
    
    while ((p = strchr(p, ',')) != NULL) {
        count++;
        p++;  // 跳过逗号
    }
    
    return count;
}
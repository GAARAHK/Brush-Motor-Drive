#ifndef CMD_PARSER_H
#define CMD_PARSER_H

#include <stdint.h>

// 从字符串中解析整数
int CmdParser_ParseInt(const char *str, int *value);

// 从字符串中解析指定位置的参数
int CmdParser_GetParam(const char *params, int index, char *param, int param_size);

// 计算参数数量
int CmdParser_CountParams(const char *params);

#endif // CMD_PARSER_H
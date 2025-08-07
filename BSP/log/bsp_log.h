/********************************************************************************
 * @file        : log.h
 * @author      : Zhang jia ming
 * @brief       : ���ļ�ʵ������־��¼���ܣ�������־��ʼ������ͬ������־����ȹ��ܡ�
 * @version     : V1.0
 * @date        : 2025 - 1 - 11
 *
 * @details:
 *  - ���ļ���������־��¼��صĺ�����ͨ�� `LogInit` ������ʼ����־ϵͳ��ʹ�����ض��Ĵ���ʵ���������
 *  - `LOG_PROTO` �����Ǻ��ĵ���־��������������ݴ������־������ `LOG_DEBUG`��`LOG_INFO`��`LOG_WARNING`��`LOG_ERROR`����
 *    ��ʽ����־��Ϣ����ͨ��������DMAģʽ���ͳ�ȥ��
 *  - ͬʱ��ͨ���궨�� `LOGDEBUG`��`LOGINFO`��`LOGWARNING` �� `LOGERROR` Ϊ��ͬ�������־����ṩ�˱�ݵĵ��÷�ʽ��
 *
 * @note:
 *  - ��ʹ�ñ���־ģ��ǰ����ȷ����ش�����������������ȷ��ʼ�����ر����� `log_usart_instance` ��صĴ������á�
 *  - ��־��������С�� `LOG_BUFFER_SIZE` �궨�壬��ǰ����Ϊ1024�ֽڡ�����־��Ϣ���������ܻᵼ�����ݽضϣ�ʹ��ʱ��ע�⡣
 *  - ���� `USARTSend` ����������������ȷʵ������DMAģʽ���ܿɿ��ط������ݡ����ú������ش��󣬵�ǰ�����Ԥ���˴�����λ�ã�
 *    ʵ��Ӧ����������������ƴ������߼���
 *
 * @history:
 *  V1.0:
 *    - �޸�ע�ͣ����ƹ��ܡ��Ժ������ܡ�ʹ��ע������Ƚ�������ϸ˵������ߴ���Ŀɶ��ԺͿ�ά���ԡ�
 *
 * Copyright (c) 2025 Zhang jia ming. All rights reserved.
 ********************************************************************************/
#ifndef BSP_LOG_H
#define BSP_LOG_H

#include <stdarg.h>
#include <stdbool.h>
#include "bsp_usart.h"

#define LOG_BUFFER_SIZE 256 // ��󻺳���

/**
 * @brief ��־����
 */
typedef enum
{
  LOG_DEBUG = 0,
  LOG_INFO,
  LOG_WARNING,
  LOG_ERROR,
} LOG_LEVEL;

/**
 * @brief ��־��ʼ��
 */
void LogInit(UART_HandleTypeDef *log_config);

/**
 * @brief ��־�������
 */
void LOG_PROTO(const char *fmt, LOG_LEVEL level, const char *file,
               int line, const char *func, ...);

// debug level
#define LOGDEBUG(fmt, ...) LOG_PROTO(fmt, LOG_DEBUG, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
// information level
#define LOGINFO(fmt, ...) LOG_PROTO(fmt, LOG_INFO, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
// warning level
#define LOGWARNING(fmt, ...) LOG_PROTO(fmt, LOG_WARNING, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
// error level
#define LOGERROR(fmt, ...) LOG_PROTO(fmt, LOG_ERROR, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)

#endif

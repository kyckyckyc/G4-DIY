/********************************************************************************
 * @file        : log.c
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

#include "bsp_log.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static USARTInstance *log_usart_instance;

/**
 * @brief          ע��LOGʵ��
 * @param[in]      log_config: LOGʵ������
 * @retval         LOGʵ��
 */
void LogInit(UART_HandleTypeDef *log_config)
{
  USART_Init_Config_s config;
  config.usart_handle = log_config;
  log_usart_instance = USARTRegister(&config);
}

/**
 * @brief          ��־�������
 *
 * @param[in]      fmt:          �����͵����ݸ�ʽ
 * @param[in]      level:        �����͵���־����
 * @param[in]      file:         �����͵������ļ���
 * @param[in]      line:         �����͵������к�
 * @param[in]      func:         �����͵����ݺ�����
 *
 * @retval         note
 */
void LOG_PROTO(const char *fmt, LOG_LEVEL level, const char *file,
               int line, const char *func, ...)
{
  char tmp[LOG_BUFFER_SIZE];
  char buf[LOG_BUFFER_SIZE];
  memset(tmp, 0, sizeof(tmp));
  memset(buf, 0, sizeof(buf));

  va_list args;
  va_start(args, func);
  vsnprintf(tmp, sizeof(tmp) - 1, fmt, args);
  va_end(args);

  switch (level)
  {
  case LOG_DEBUG:
    snprintf(buf, sizeof(buf), "[DEBUG] <%s> | <%d> | <%s>: %s\r\n", file, line, func, tmp);
    break;
  case LOG_INFO:
    snprintf(buf, sizeof(buf), "[INFO] <%s> | <%d> | <%s>: %s\r\n", file, line, func, tmp);
    break;
  case LOG_WARNING:
    snprintf(buf, sizeof(buf), "[WARN] <%s> | <%d> | <%s>: %s\r\n", file, line, func, tmp);
    break;
  case LOG_ERROR:
    snprintf(buf, sizeof(buf), "[ERROR] <%s> | <%d> | <%s>: %s\r\n", file, line, func, tmp);
    break;
  default:
    return;
  }

  USARTSend(log_usart_instance, (uint8_t *)buf, strlen(buf), USART_TRANSFER_IT);
}

#ifndef LED_H
#define LED_H

#include "general_def.h"
#include "tim.h"

/*�����Ǽ�������CCR�ĺ궨��*/
#define CODE_1 (140) // 1�붨ʱ����������
#define CODE_0 (70)  // 0�붨ʱ����������

#define LED_MAX_NUM 1 // LED�����궨�壬������ʹ��һ��LED

/*����һ�����嵥��LED��ԭɫֵ��С�Ľṹ��*/
typedef struct
{
    uint8_t R;
    uint8_t G;
    uint8_t B;
} RGB_Color_TypeDef;

void RGB_SetColor(uint8_t LedId, RGB_Color_TypeDef Color); // ��һ��LEDװ��24����ɫ�����루0���1�룩
void Reset_Load(void);                                     // �ú������ڽ��������24�����ݱ�Ϊ0������RESET_code
void RGB_SendArray(void);                                  // ����LED����
void RGB_DisplayColor(RGB_Color_TypeDef color);            // ��ʾָ����ɫ
void RGB_DisplayColorById(uint8_t color_id);               // ������ɫ�����ʾ��ɫ
void RGB_DMA_CompleteCallback(void);                       // DMA������ɺ�Ļص�����

#endif

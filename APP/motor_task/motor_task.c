#include "motor_task.h"

#define ADJUST_EN 0 // ����ʹ��

/**
 * @brief ADC1 ����������ɻص�����
 *
 * ��������Ƶ��20khz
 *
 * @param hadc ����� ADC1
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // �����¼�������
    UNUSED(motor_data.components.current->hadc);
    // ������ת���� ADC �Ƿ�Ϊ ADC1
    if (hadc == &ADC_HSPI)
    {
#if ADJUST_EN
        GetMotorADC1PhaseCurrent(&motor_data);
        GetMotor_Angle(motor_data.components.encoder);
#else
        GetMotorADC1PhaseCurrent(&motor_data);
        GetMotor_Angle(motor_data.components.encoder);
        MotorStateTask(&motor_data);
#endif
    }
}

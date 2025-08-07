#include "bsp_adc.h" // ���� ADC ��ص� BSP���弶֧�ְ���ͷ�ļ�

/* adc_dma ���ݴ洢����
	 ���ڴ洢 ADC1 �� DMA ת�������
	 adc1_samples: ��������
	 adc1_channel: ͨ����
*/
/* adc_dma ���ݴ洢����*/
uint16_t adc1_dma_value[adc1_samples][adc1_channel];
uint16_t adc2_dma_value[adc2_samples][adc2_channel];
/**
 * @brief ��ʼ�� ADC BSP
 *
 * �˺������ڳ�ʼ�� ADC ��������ã��������� DMA ��ע��ģʽ��
 */
void adc_bsp_init(void)
{
#if ADC_INJECTED_ENABLE
	/**ADC��У��**/
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	/**����ADCע���ж�**/
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_dma_value, adc2_length);

#else // ���������ע��ģʽ

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_dma_value, adc1_length);
#endif
}

#ifndef BLDCMotor_H
#define BLDCMotor_H


#define _ARM_COMPAT_USAT // ����ģʽ
#include "general_def.h"
#include "tim.h"
#include "arm_math.h"
#include "mt6816_encoder.h"

// �������
#define MOTOR_PM3510 1
#define MOTOR_DJI2312 0

#define lf 0
#define rf 0
#define lb 0
#define rb 1

#define N_BASE 3.0f               // 3S���
#define BATVEL (4.0f * N_BASE)    // ��ص�ѹ
#define INVBATVEL (1.0f / BATVEL) // ��ص�ѹ�ĵ���

#define PRE_CALIBRATED 1 // ��һ������ʱ������0���ϵ��У׼�������ȡ������Ĳ�����д�����棻����Ϊ1����ֱ��ʹ������Ĳ���

#if MOTOR_DJI2312 // �������
#if PRE_CALIBRATED
#define MPTOR_P 7u              // ���������
#define MOTOR_RS 0.135551453f   // �����
#define MOTOR_LS 2.4615214e-05f // ����
#define MOTOR_FLUX 0.0f         // ����
#define MOTOR_OFFSET 3997       // ���ƫ����
#define MOTOR_DIRECTION CCW     // �������
#else
#define MPTOR_P 0u              // ���������
#define MOTOR_RS 0.0f           // �����
#define MOTOR_LS 0.0f           // ����
#define MOTOR_FLUX 0.0f         // ����
#define MOTOR_OFFSET 0.0f       // ���ƫ����
#define MOTOR_DIRECTION UNKNOWN // �������
#endif

// ���������ض��� ���ݶȼӼ��ٲ���̫�󣬲�Ȼ�����ڼ���ʱ����ֶ���ʽ�µ���λ�Ʋ����̿������������������ԣ�������
#define MOTOR_INERTIA 0.0001f               // ת������ [A/(turn/s^2)]������Ϊ�������1תÿ��ļ��ٶ���תʱ��Ҫ�ṩ�����ڼ������ĵĵ�������Ҫ���ݵ�����������������������е��ԡ�
#define MOTOR_CURRENT_RAMP_RATE 0.001f      // ������������ [Nm/s]
#define MOTOR_VEL_RAMP_RATE 160.0f          // ת�������ٶ� [(turn/s)/s]
#define MOTOR_TRAJ_VEL 160.0f               // ���ι켣����ģʽ�����ת�� [turn/s] (�趨ֵӦ��С�ڻ���� MOTOR_VEL_LIMIT)
#define MOTOR_TRAJ_ACCEL 40.0f              // ���ι켣����ģʽ�¼��ٶ� [(turn/s)/s]
#define MOTOR_TRAJ_DECEL 40.0f              // ���ι켣����ģʽ�¼��ٶ� [(turn/s)/s]
#define MOTOR_TORQUE_CONST 0.0f             // ת�س��� [Nm/A]
#define MOTOR_TORQUE_LIMIT 0.0f             // ת������ֵ [Nm] (��ֵ���Ƶ���ʵ�������е����ת�� torque) һ������Ϊ����������ת��
#define MOTOR_VEL_LIMIT 168.0f              // ת������ֵ [turn/s] (turn/s = ���ת��rpm / 60) һ������Ϊ����������ת��
#define MOTOR_VOLTAGE_LIMIT 12.0f           // �����ѹ����ֵ [V]
#define MOTOR_CURRENT_LIMIT 6.0f            // �����������ֵ [A] (��ֵ���Ƶ���ʵ�������е��������� foc->i_q_filt) һ������Ϊ�������������
#define MOTOR_CURRENT_CTRL_P_GAIN 0.0f      // ���������棬��У׼�������Զ�����ó���Ҳ���������� (Auto)
#define MOTOR_CURRENT_CTRL_I_GAIN 0.0f      // �������������棬��У׼�������Զ�����ó���Ҳ���������� (Auto)
#define MOTOR_CURRENT_CTRL_BANDWIDTH 230.0f // ����������[rad/s]����Χ (2~60000)
#define MOTOR_INPUT_CURRENT 5.0f            // Ŀ�����
#define MOTOR_INPUT_TORQUE 0.0f             // Ŀ������
#define MOTOR_INPUT_VELOCITY 0.0f           // Ŀ��ת��
#define MOTOR_INPUT_POSITION 0.0f           // Ŀ��λ��

// PID����
#define MOTOR_IQ_PID_KP 0.0f
#define MOTOR_IQ_PID_KI 0.0f
#define MOTOR_IQ_PID_KD 0.0f
#define MOTOR_ID_PID_KP 0.0f
#define MOTOR_ID_PID_KI 0.0f
#define MOTOR_ID_PID_KD 0.0f
#define MOTOR_VEL_PID_KP 0.40f
#define MOTOR_VEL_PID_KI 0.0040f // ����̫�󣬷���ᵼ�µ�����Ҷ������Ժ������޸�ʱ�л��Ͳ�˳����
#define MOTOR_VEL_PID_KD 0.0f
#define MOTOR_POS_PID_KP 165.0f // ���Կ�����ʱ�ٶ��Ƿ��׼��
#define MOTOR_POS_PID_KI 0.0f
#define MOTOR_POS_PID_KD 0.0f

/*��������ʱ��      �޷�ֵû��ʲôԼ�����ǲ��ܴ���ĸ�ߵ�ѹ/����3������ᵼ�µ������������
**�����ٶȻ�ʱ��    ��ΪҪ�Ⱦ����ٶȻ���Ȼ���ٰ��ٶȻ���ֵ���������������Ե��������޷�Ϊĸ�ߵ�ѹ/����3���ٶȻ����޷�Ϊ������ֵ
**�����ٶ�λ�û�ʱ����λ�û����޷���Ϊ������ת�ټ��ɣ����������޷�Ϊĸ�ߵ�ѹ/����3���ٶȻ����޷�Ϊ������ֵ
**
**λ�û����޷����ɹ��󣬲�Ȼ���кܴ�Ĺ���һ���Ӿͱ������ˣ�����
*/
#define CURRENT_PID_MAX_OUT ((BATVEL) / _SQRT3)
#define IQ_PID_MAX_OUT CURRENT_PID_MAX_OUT
#define IQ_PID_MAX_IOUT CURRENT_PID_MAX_OUT
#define ID_PID_MAX_OUT CURRENT_PID_MAX_OUT
#define ID_PID_MAX_IOUT CURRENT_PID_MAX_OUT
#define VEL_PID_MAX_OUT MOTOR_CURRENT_LIMIT
#define VEL_PID_MAX_IOUT MOTOR_CURRENT_LIMIT
#define POS_PID_MAX_OUT (MOTOR_VEL_LIMIT / 1.3f)
#define POS_PID_MAX_IOUT (MOTOR_VEL_LIMIT / 1.3f)
#endif

#if MOTOR_PM3510 // �������
#if PRE_CALIBRATED
#if lf
#define MPTOR_P 11u              // ���������
#define MOTOR_RS 1.73649645f     // �����
#define MOTOR_LS 0.000637838093f // ����
#define MOTOR_FLUX 0.0f          // ����
#define MOTOR_OFFSET 2552        // ���ƫ����
#define MOTOR_DIRECTION CW       // �������
#endif
#if rf
#define MPTOR_P 11u              // ���������
#define MOTOR_RS 1.74725103f     // �����
#define MOTOR_LS 0.000649235852f // ����
#define MOTOR_FLUX 0.0f          // ����
#define MOTOR_OFFSET 14868       // ���ƫ����
#define MOTOR_DIRECTION CCW      // �������
#endif
#if lb
#define MPTOR_P 11u              // ���������
#define MOTOR_RS 1.72242999f     // �����
#define MOTOR_LS 0.000622660271f // ����
#define MOTOR_FLUX 0.0f          // ����
#define MOTOR_OFFSET 15772       // ���ƫ����
#define MOTOR_DIRECTION CW       // �������
#endif
#if rb
#define MPTOR_P 11u              // ���������
#define MOTOR_RS 1.73619509f     // �����
#define MOTOR_LS 0.000621680229f // ����
#define MOTOR_FLUX 0.0f          // ����
#define MOTOR_OFFSET 8001        // ���ƫ����
#define MOTOR_DIRECTION CCW      // �������
#endif
#else
#define MPTOR_P 0u              // ���������
#define MOTOR_RS 0.0f           // �����
#define MOTOR_LS 0.0f           // ����
#define MOTOR_FLUX 0.0f         // ����
#define MOTOR_OFFSET 0.0f       // ���ƫ����
#define MOTOR_DIRECTION UNKNOWN // �������
#endif

// ���������ض���
#define MOTOR_INERTIA 0.0001f                                         // ת������ [A/(turn/s^2)]������Ϊ�������1תÿ��ļ��ٶ���תʱ��Ҫ�ṩ�����ڼ������ĵĵ�������Ҫ���ݵ�����������������������е��ԡ�
#define MOTOR_CURRENT_RAMP_RATE 0.001f                                // ������������ [Nm/s]
#define MOTOR_VEL_RAMP_RATE 15.0f                                     // ת�������ٶ� [(turn/s)/s]
#define MOTOR_TRAJ_VEL 15.0f                                          // ���ι켣����ģʽ�����ת�� [turn/s] (�趨ֵӦ��С�ڻ���� MOTOR_VEL_LIMIT)
#define MOTOR_TRAJ_ACCEL 5.0f                                         // ���ι켣����ģʽ�¼��ٶ� [(turn/s)/s]
#define MOTOR_TRAJ_DECEL 5.0f                                         // ���ι켣����ģʽ�¼��ٶ� [(turn/s)/s]
#define MOTOR_TORQUE_CONST 0.2f                                       // ת�س��� [Nm/A]
#define MOTOR_TORQUE_LIMIT 0.11f                                      // ת������ֵ [Nm] (��ֵ���Ƶ���ʵ�������е����ת�� torque) һ������Ϊ����������ת��
#define MOTOR_VEL_LIMIT 15.75f                                        // ת������ֵ [turn/s] (turn/s = ���ת��rpm / 60) һ������Ϊ����������ת��
#define MOTOR_VOLTAGE_LIMIT 12.0f                                     // �����ѹ����ֵ [V]
#define MOTOR_CURRENT_LIMIT (MOTOR_TORQUE_CONST / MOTOR_TORQUE_LIMIT) // �����������ֵ [A] (��ֵ���Ƶ���ʵ�������е��������� foc->i_q_filt) һ������Ϊ�������������
#define MOTOR_CURRENT_CTRL_P_GAIN 0.0f                                // ���������棬��У׼�������Զ�����ó���Ҳ���������� (Auto)
#define MOTOR_CURRENT_CTRL_I_GAIN 0.0f                                // �������������棬��У׼�������Զ�����ó���Ҳ���������� (Auto)
#define MOTOR_CURRENT_CTRL_BANDWIDTH 230.0f                           // ����������[rad/s]����Χ (2~60000)
#define MOTOR_INPUT_CURRENT 0.4f                                      // Ŀ�����
#define MOTOR_INPUT_TORQUE 0.08f                                      // Ŀ������
#define MOTOR_INPUT_VELOCITY 0.0f                                     // Ŀ��ת��
#define MOTOR_INPUT_POSITION 0.0f                                     // Ŀ��λ��

// PID����
#define MOTOR_IQ_PID_KP 0.0f
#define MOTOR_IQ_PID_KI 0.0f
#define MOTOR_IQ_PID_KD 0.0f
#define MOTOR_ID_PID_KP 0.0f
#define MOTOR_ID_PID_KI 0.0f
#define MOTOR_ID_PID_KD 0.0f
#define MOTOR_VEL_PID_KP 0.12f
#define MOTOR_VEL_PID_KI 0.0001f // ����̫�󣬷���ᵼ�µ�����Ҷ������Ժ������޸�ʱ�л��Ͳ�˳����
#define MOTOR_VEL_PID_KD 0.0f
#define MOTOR_POS_PID_KP 120.0f // ���Կ�����ʱ�ٶ��Ƿ��׼��
#define MOTOR_POS_PID_KI 0.0f
#define MOTOR_POS_PID_KD 0.0f

#define CURRENT_PID_MAX_OUT ((BATVEL) / _SQRT3)
#define IQ_PID_MAX_OUT CURRENT_PID_MAX_OUT
#define IQ_PID_MAX_IOUT CURRENT_PID_MAX_OUT
#define ID_PID_MAX_OUT CURRENT_PID_MAX_OUT
#define ID_PID_MAX_IOUT CURRENT_PID_MAX_OUT
#define VEL_PID_MAX_OUT MOTOR_CURRENT_LIMIT
#define VEL_PID_MAX_IOUT MOTOR_CURRENT_LIMIT
#define POS_PID_MAX_OUT (MOTOR_VEL_LIMIT - 3.0f)
#define POS_PID_MAX_IOUT (MOTOR_VEL_LIMIT - 3.0f)

#endif

// Calib
#define CURRENT_MAX_CALIB 0.5f // У׼���������� (������У׼��ʱ��ٴ�����ػ��߶�һ��Ͳ����ˣ�����Ҫ�������ֵ)
#define VOLTAGE_MAX_CALIB 3.0f // У׼����ѹ����(������У׼��ʱ�򣬵��û�з������١�������(������ͨ�������������һ��)������Ҫ�������ֵ)

#define OFFSET_LUT_NUM 128               // ƫ����LUT��
#define COGGING_MAP_NUM 3000             // ������
#define CALB_SPEED M_2PI                 // ���ת��
#define MOTOR_POLE_PAIRS_MAX 20          // ����⼫�������������ӵ���Ҫע��ѵĴ�С���䣩
#define SAMPLES_PER_PPAIR OFFSET_LUT_NUM // ������������

// Control
#define BATVEL_MAX_LIMIT (BATVEL * 1.2f)         // ��ع�ѹ������ֵ
#define BATVEL_MIN_LIMIT (BATVEL / 1.5f)         // ��ص�ѹ������ֵ
#define SPEED_MAX_LIMIT (MOTOR_VEL_LIMIT * 1.2f) // ��������
#define TEMP_MAX_LIMIT 65.0f                     // ��������

// ��������
#define TIMER1_CLK_MHz 168                                                                                         // ��ʱ��ʱ��Ƶ��
#define PWM_FREQUENCY 20000                                                                                        // PWMƵ��20KHz
#define PWM_MEASURE_PERIOD (float)(1.0f / (float)PWM_FREQUENCY)                                                    // PWM����
#define CURRENT_MEASURE_HZ PWM_FREQUENCY                                                                           // ����Ƶ��
#define CURRENT_MEASURE_PERIOD (float)(1.0f / (float)CURRENT_MEASURE_HZ)                                           // ��������
#define PWM_PERIOD_CYCLES (uint16_t)((TIMER1_CLK_MHz * (uint32_t)1000000u / ((uint32_t)(PWM_FREQUENCY))) & 0xFFFE) // ��ʱ��1��������
#define PWM_ARR (uint16_t)(PWM_PERIOD_CYCLES / 2u)                                                                 // 3500
#define VEL_POS_HZ 5000                                                                                            // �ٶ�λ�û�Ƶ��
#define VEL_POS_PERIOD (float)(1.0f / (float)VEL_POS_HZ)                                                           // �ٶ�λ�û�����
#define CURRENT_CALIBRATION_DURATION 1000u                                                                         // ����У׼ʱ��
#define DEADTIME_COMP 20u                                                                                          // ����ʱ�䲹��
#define TS 1.0f                                                                                                    // �������� (��ռ�ձ�������0-1֮��)

// Ӳ������
#define V_REG 1.65f                                                             // ADC�ο���ѹ
#define CURRENT_SHUNT_RES 0.02f                                                 // ������������
#define CURRENT_AMP_GAIN 50.0f                                                  // �����Ŵ���
#define VIN_R1 1000.0f                                                          // ��ѹ������ѹ����
#define VIN_R2 10000.0f                                                         // ��ѹ������ѹ����
#define FAC_CURRENT ((3.3f / 4095.0f) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN)) // �����Ŵ���
#define VOLTAGE_TO_ADC_FACTOR (((VIN_R2 + VIN_R1) / VIN_R1) * (3.3f / 4095.0f)) // ��ѹ�Ŵ���

// ����PWMռ�ձȺ궨��
#define set_dtc_a(value) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, value) // ����A��PWMռ�ձ�
#define set_dtc_b(value) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, value) // ����B��PWMռ�ձ�
#define set_dtc_c(value) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, value) // ����C��PWMռ�ձ�

#pragma pack(1)
// ����ö��
typedef enum
{
    SECTOR_1 = 1,
    SECTOR_2,
    SECTOR_3,
    SECTOR_4,
    SECTOR_5,
    SECTOR_6
} svpwm_sector_t;

typedef struct
{
    float vbus;     // ĸ�ߵ�ѹ
    float inv_vbus; // ĸ�ߵ�ѹ���������ڼ�������͵�ѹ
    float theta;    // �Ƕ�
    float i_q_filt, i_d_filt, i_bus, i_bus_filt, power_filt;

    float current_ctrl_integral_d, current_ctrl_integral_q; // Current error integrals
    float current_ctrl_p_gain;                              // (Auto)
    float current_ctrl_i_gain;                              // (Auto)
    // ������Ϣ������SVPWM��
    int sector; // ��ǰ����������SVPWM����

    // ����ϵ��ѹ����������
    float i_a; // A �����
    float i_b; // B �����
    float i_c; // C �����

    float v_a; // A ���ѹ
    float v_b; // B ���ѹ
    float v_c; // C ���ѹ

    float i_d; // D ����ϵ������FOC�е�ֱ�������
    float i_q; // Q ����ϵ������FOC�еĽ��������

    float v_d; // D ����ϵ��ѹ��FOC�е�ֱ���ѹ�� ����Ť��
    float v_q; // Q ����ϵ��ѹ��FOC�еĽ����ѹ�� ���ſ���

    float i_alpha; // Alpha ����ϵ������Clarke�任��ĵ�����
    float i_beta;  // Beta  ����ϵ������Clarke�任��ĵ�����

    float v_alpha; // Alpha ����ϵ��ѹ����Clarke�任��ĵ�ѹ��
    float v_beta;  // Beta  ����ϵ��ѹ����Clarke�任��ĵ�ѹ��

    float dtc_a; // A �� PWM ռ�ձ�
    float dtc_b; // B �� PWM ռ�ձ�
    float dtc_c; // C �� PWM ռ�ձ�

    float vd_set; // d ���ѹ����
    float vq_set; // q ���ѹ����
    float id_set; // d ���������
    float iq_set; // q ���������

    float sin_val; // ��Ƕȵ�����ֵ������FOC����
    float cos_val; // ��Ƕȵ�����ֵ������FOC����
} FOC_DATA;
#pragma pack()

extern FOC_DATA foc_data;

// ������������
void FOC_reset(FOC_DATA *foc);                                                             // ����FOC����
void Foc_Pwm_LowSides(void);                                                               // ����FOC ��ռ�ձ�Ϊ��
void Foc_Pwm_Start(void);                                                                  // ����FOC PWM
void Foc_Pwm_Stop(void);                                                                   // ֹͣFOC PWM
void SetPwm(FOC_DATA *foc);                                                                // ����PWMռ�ձ�
void Sin_Cos_Val(FOC_DATA *foc);                                                           // �������Һ�����ֵ
void Clarke(FOC_DATA *foc);                                                                // Clarke�任
void Park(FOC_DATA *foc);                                                                  // Park�任
void Inv_clarke(FOC_DATA *foc);                                                            // ��Clarke�任
void Inv_Park(FOC_DATA *foc);                                                              // ��Park�任
void Svpwm_Midpoint(FOC_DATA *foc);                                                        // SVPWM����
void Svpwm_Sector(FOC_DATA *foc);                                                          // SVPWM����ѡ��
void commonFOCOperations(FOC_DATA *foc);                                                   // ͨ��FOC����
void commonInverseFOCOperations(FOC_DATA *foc);                                            // ͨ����FOC����
void setPhaseVoltage(FOC_DATA *foc, float Vd_set, float Vq_set, float phase);              // �������ѹ�������
void FOC_voltage(FOC_DATA *foc, float Vd_set, float Vq_set, float phase);                  // ��ѹ����
void FOC_current(FOC_DATA *foc, float Id_set, float Iq_set, float phase, float phase_vel); // ��������
#endif                                                                                     // BLDCMotor_H

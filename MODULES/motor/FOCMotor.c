#include "FOCMotor.h"
#include "bsp_dwt.h"
#include "led.h"
#include "trapTraj.h"

MOTOR_DATA motor_data           = {
    .components                 = {
        .foc                    = &foc_data,
        .encoder                = &encoder_data,
        .current                = &current_data,
    },
    .state = {
        .State_Mode             = STATE_MODE_IDLE,              // �������״̬
        .Control_Mode           = CONTROL_MODE_VELOCITY_RAMP,   // �������ģʽ
        .Sub_State              = SUB_STATE_IDLE,               // �����У׼״̬
        .Cs_State               = CS_STATE_IDLE,                // �����У׼״̬
        .Fault_State            = FAULT_STATE_NORMAL,           // �������״̬
    },
    .parameters = {
        .Rs                     = MOTOR_RS,     // �����
        .Ls                     = MOTOR_LS,     // ����
        .flux                   = MOTOR_FLUX, // ����
    },
    .Controller                 = {
        .inertia                = MOTOR_INERTIA,                // ת������ [A/(turn/s^2)]������Ϊ�������1תÿ��ļ��ٶ���תʱ��Ҫ�ṩ�����ڼ������ĵĵ�������Ҫ���ݵ�����������������������е��ԡ�
        .torque_ramp_rate       = MOTOR_CURRENT_RAMP_RATE,      // ������������[Nm/s]
        .vel_ramp_rate          = MOTOR_VEL_RAMP_RATE,          // ת�������ٶ� [(turn/s)/s]
        .traj_vel               = MOTOR_TRAJ_VEL,               // ���ι켣����ģʽ�����ת�� [turn/s]
        .traj_accel             = MOTOR_TRAJ_ACCEL,             // ���ι켣����ģʽ�¼��ٶ� [(turn/s)/s]
        .traj_decel             = MOTOR_TRAJ_DECEL,             // ���ι켣����ģʽ�¼��ٶ� [(turn/s)/s]
        .torque_const           = MOTOR_TORQUE_CONST,           // ���ת�س��� [Nm/A]
        .torque_limit           = MOTOR_TORQUE_LIMIT,           // ������ת�� [Nm]
        .vel_limit              = MOTOR_VEL_LIMIT,              // ת������ֵ [turn/s]
        .voltage_limit          = MOTOR_VOLTAGE_LIMIT,          // �����ѹ����ֵ [V]
        .current_limit          = MOTOR_CURRENT_LIMIT,          // �����������ֵ [A]
        .current_ctrl_p_gain    = MOTOR_CURRENT_CTRL_P_GAIN,    // ���������棬��У׼�������Զ�����ó���Ҳ���������� (Auto)
        .current_ctrl_i_gain    = MOTOR_CURRENT_CTRL_I_GAIN,    // �������������棬��У׼�������Զ�����ó���Ҳ���������� (Auto)
        .current_ctrl_bandwidth = MOTOR_CURRENT_CTRL_BANDWIDTH, // ����������[rad/s]����Χ 100~2000
        .input_current          = MOTOR_INPUT_CURRENT,          // Ŀ�����
        .input_torque           = MOTOR_INPUT_TORQUE,           // Ŀ������
        .input_velocity         = MOTOR_INPUT_VELOCITY,         // Ŀ��ת��
        .input_position         = MOTOR_INPUT_POSITION,         // Ŀ��λ��
        .input_updated          = true,                         // λ�û��趨ֵ�Ƿ��Ѹ���
    },
    .IqPID                      = {
        // ������IQ����
        .mode                   = PID_POSITION,
        .Kp                     = MOTOR_IQ_PID_KP,
        .Ki                     = MOTOR_IQ_PID_KI,
        .Kd                     = MOTOR_IQ_PID_KD,
        .max_out                = IQ_PID_MAX_OUT,   
        .max_iout               = IQ_PID_MAX_IOUT,
    },
    .IdPID                      = {
        // ������ID����
        .mode                   = PID_POSITION,
        .Kp                     = MOTOR_ID_PID_KP,
        .Ki                     = MOTOR_ID_PID_KI,
        .Kd                     = MOTOR_ID_PID_KD,
        .max_out                = ID_PID_MAX_OUT,
        .max_iout               = ID_PID_MAX_IOUT,
    },
    .VelPID                     = {
        // ת�ٻ�PID����
        .mode                   = PID_POSITION,
        .Kp                     = MOTOR_VEL_PID_KP,
        .Ki                     = MOTOR_VEL_PID_KI,
        .Kd                     = MOTOR_VEL_PID_KD,
        .max_out                = VEL_PID_MAX_OUT,
        .max_iout               = VEL_PID_MAX_IOUT,
    },
    .PosPID                     = {
        // λ�û�PID����
        .mode                   = PID_POSITION,
        .Kp                     = MOTOR_POS_PID_KP,
        .Ki                     = MOTOR_POS_PID_KI,
        .Kd                     = MOTOR_POS_PID_KD,
        .max_out                = POS_PID_MAX_OUT,
        .max_iout               = POS_PID_MAX_IOUT,
    },
};

static int *p_error_arr         = NULL;

/**
 * @brief ��ȡNTC�¶�����
 *
 * @param motor ָ�������ݽṹ��ָ��
 */
void TempResultTask(MOTOR_DATA *motor)
{
    GetTempNtc(adc2_median_filter(adc2_ch12), &motor->components.current->Temp_Result);
}

/**
 * @brief �޸�PID�޷�ֵ
 *
 * @param motor ָ�������ݽṹ��ָ��
 */
static void SetPIDLimit(MOTOR_DATA *motor,
                        float current_max_out,
                        float current_max_iout,
                        float vel_max_out,
                        float vel_max_iout,
                        float pos_limit)
{
    motor->IdPID.max_out   = CURRENT_PID_MAX_OUT;
    motor->IdPID.max_iout  = CURRENT_PID_MAX_OUT;
    motor->IqPID.max_out   = current_max_out;
    motor->IqPID.max_iout  = current_max_iout;
    motor->VelPID.max_out  = vel_max_out;
    motor->VelPID.max_iout = vel_max_iout;
    motor->PosPID.max_out  = pos_limit;
    motor->PosPID.max_iout = pos_limit;
}

/**
 * @brief �ٶȿ�����������
 *
 * �˺�������ִ�п����������񣬸���Ŀ���ٶȼ������ĵ�Ƕȣ������õ���ֵ��
 *
 * @param motor ָ�������ݽṹ��ָ��
 */
static void OpenControlMode(MOTOR_DATA *motor, float target_velocity)
{
    float Ts = 0.001f;
    if (Ts <= 0 || Ts > 0.5f)
        Ts = 1e-3f;

    motor->components.foc->theta = normalize_angle(motor->components.foc->theta + target_velocity * Ts); // �̶��Ƕȡ�����

    FOC_voltage(motor->components.foc, 0.0f, 0.5f, motor->components.foc->theta); // ��ѹ����
}

/**
 * @brief ��ȡ��������
 *
 * �˺����� ADC �ж�ȡ����������ֵ��������洢�� foc_data �ṹ���С�
 * �ú���ͨ���ڿ���ѭ���е��ã��Ի�ȡʵʱ�������ݡ�
 *
 * @param motor ָ�� foc_data �ṹ���ָ�룬�ýṹ��������״̬�Ϳ������ݡ�
 */
void GetMotorADC1PhaseCurrent(MOTOR_DATA *motor)
{
    // �������ƫ�á�ĸ�ߵ�ѹ����
    motor->components.foc->i_a  = ((float)motor->components.current->hadc->Instance->JDR3 - motor->components.current->Ia_offset) * FAC_CURRENT;
    motor->components.foc->i_b  = ((float)motor->components.current->hadc->Instance->JDR2 - motor->components.current->Ib_offset) * FAC_CURRENT;
    motor->components.foc->i_c  = ((float)motor->components.current->hadc->Instance->JDR1 - motor->components.current->Ic_offset) * FAC_CURRENT;
    motor->components.foc->vbus = ((float)motor->components.current->hadc->Instance->JDR4 * VOLTAGE_TO_ADC_FACTOR);
}

/**
 * @brief ���µ������������������
 *
 * �������������
 * P��Ls * �����ת��/60 * ��������* 2 * 3.14
 * I��Rs * �����ת��/60 * ��������* 2 * 3.14 * ��������Ƶ��
 *
 * @param bandwidth ��������
 */
#define CURRENT_AUTO_CALIBRATION 1 // �ֶ��������滹���Զ�����
static void FOC_update_current_gain(MOTOR_DATA *motor)
{
#if CURRENT_AUTO_CALIBRATION
    motor->Controller.current_ctrl_p_gain = motor->parameters.Ls * motor->Controller.vel_limit * motor->components.encoder->pole_pairs * M_2PI;
    motor->Controller.current_ctrl_i_gain = motor->parameters.Rs * motor->Controller.vel_limit * motor->components.encoder->pole_pairs * M_2PI * CURRENT_MEASURE_PERIOD;
#else
    motor->Controller.current_ctrl_p_gain = motor->parameters.Ls * motor->Controller.current_ctrl_bandwidth * 1.0f;
    motor->Controller.current_ctrl_i_gain = motor->parameters.Rs * motor->Controller.current_ctrl_bandwidth * CURRENT_MEASURE_PERIOD;
#endif
    motor->IdPID.Kp = motor->Controller.current_ctrl_p_gain;
    motor->IdPID.Ki = motor->Controller.current_ctrl_i_gain;
    motor->IqPID.Kp = motor->Controller.current_ctrl_p_gain;
    motor->IqPID.Ki = motor->Controller.current_ctrl_i_gain;
}

void Init_Motor_No_Calib(MOTOR_DATA *motor)
{
    motor->state.Sub_State = SUB_STATE_IDLE;
    motor->state.Cs_State = CS_STATE_IDLE;
    motor->state.State_Mode = STATE_MODE_RUNNING;
    FOC_update_current_gain(motor);
}

void Init_Motor_Calib(MOTOR_DATA *motor)
{
    if (p_error_arr == NULL)
    {
        p_error_arr = malloc(SAMPLES_PER_PPAIR * MOTOR_POLE_PAIRS_MAX * sizeof(int));
    }

    motor->components.encoder->calib_valid = false;
    motor->state.Sub_State                 = RSLS_CALIBRATING; // �ı���״̬
    motor->state.Cs_State                  = CS_MOTOR_R_START; // �ı���״̬
}

/**
 * @brief ��������
 *
 * @param motor ָ��������״̬�Ϳ������ݵĽṹ��ָ��
 */
static void CurrentCalibration(MOTOR_DATA *motor)
{
    static uint16_t loop_count = 0;
    loop_count++;

    // �ۼӵ���ƫ��ֵ
    motor->components.current->current_offset_sum_a += (float)(ADC1->JDR3);
    motor->components.current->current_offset_sum_b += (float)(ADC1->JDR2);
    motor->components.current->current_offset_sum_c += (float)(ADC1->JDR1);

    if (loop_count >= CURRENT_CALIBRATION_DURATION)
    {
        loop_count = 0;

#if PRE_CALIBRATED // �Ƿ���ҪУ׼
        Init_Motor_No_Calib(motor);
#else
        Init_Motor_Calib(motor);
#endif
        // У׼��ɣ����µ���ƫ��ֵ
        motor->components.current->Ia_offset = motor->components.current->current_offset_sum_a / (float)CURRENT_CALIBRATION_DURATION;
        motor->components.current->Ib_offset = motor->components.current->current_offset_sum_b / (float)CURRENT_CALIBRATION_DURATION;
        motor->components.current->Ic_offset = motor->components.current->current_offset_sum_c / (float)CURRENT_CALIBRATION_DURATION;

        // ����ƫ�����ۼ���
        motor->components.current->current_offset_sum_a = 0;
        motor->components.current->current_offset_sum_b = 0;
        motor->components.current->current_offset_sum_c = 0;
    }
}
/**
 * @brief У׼������衢���
 *
 * @param motor ָ��������״̬�Ϳ������ݵĽṹ��ָ��
 */
static void RSLSCalibration(MOTOR_DATA *motor)
{
    static uint32_t loop_count;

    // R
    static const float kI              = 2.0f;
    static const uint32_t num_R_cycles = CURRENT_MEASURE_HZ * 2;

    // L
    static float Ialphas[2];
    static float voltages[2];
    static const uint32_t num_L_cycles = CURRENT_MEASURE_HZ / 2;

    static const float calib_phase_vel = M_PI;

    static float phase_set;
    static float start_count;
    static int16_t sample_count;
    static float next_sample_time;

    float time                         = (float)loop_count * CURRENT_MEASURE_PERIOD;
    const float voltage                = CURRENT_MAX_CALIB * motor->parameters.Rs * 3.0f / 2.0f;

    switch (motor->state.Cs_State)
    {
    case CS_STATE_IDLE: // ����״̬
        break;

    case CS_MOTOR_R_START: // �趨��ղ���
        loop_count                     = 0;
        voltages[0]                    = 0.0f;
        motor->components.encoder->dir = CW;
        motor->state.Cs_State          = CS_MOTOR_R_LOOP;
        break;

    case CS_MOTOR_R_LOOP: // У׼����
        voltages[0] += kI * CURRENT_MEASURE_PERIOD * (CURRENT_MAX_CALIB - motor->components.foc->i_a);

        // Test voltage along phase A
        FOC_voltage(motor->components.foc, voltages[0], 0, 0);

        if (loop_count >= num_R_cycles)
        {
            Foc_Pwm_LowSides();
            motor->state.Cs_State = CS_MOTOR_R_END;
        }

        break;

    case CS_MOTOR_R_END: // У׼�������
        motor->parameters.Rs  = (voltages[0] / CURRENT_MAX_CALIB) * (2.0f / 3.0f);
        motor->state.Cs_State = CS_MOTOR_L_START;

        break;

    case CS_MOTOR_L_START: // �趨��ղ���
        loop_count  = 0;
        Ialphas[0]  = 0.0f;
        Ialphas[1]  = 0.0f;
        voltages[0] = -VOLTAGE_MAX_CALIB;
        voltages[1] = +VOLTAGE_MAX_CALIB;
        FOC_voltage(motor->components.foc, voltages[0], 0.0f, 0.0f);
        motor->state.Cs_State = CS_MOTOR_L_LOOP;
        break;

    case CS_MOTOR_L_LOOP: // У׼���
    {
        int i       = loop_count & 1;
        Ialphas[i] += motor->components.foc->i_a;

        // Test voltage along phase A
        FOC_voltage(motor->components.foc, voltages[i], 0.0f, 0.0f);

        if (loop_count >= (num_L_cycles << 1))
        {
            Foc_Pwm_LowSides();
            motor->state.Cs_State = CS_MOTOR_L_END;
        }
    }
    break;

    case CS_MOTOR_L_END: // У׼������
    {
        float dI_by_dt        = (Ialphas[1] - Ialphas[0]) / (float)(CURRENT_MEASURE_PERIOD * num_L_cycles);
        float L               = VOLTAGE_MAX_CALIB / dI_by_dt;
        motor->parameters.Ls  = fabsf(L * 2.0f / 3.0f); // FIXME ����ָ��������
        FOC_update_current_gain(motor);

        phase_set             = 0;
        loop_count            = 0;
        motor->state.Cs_State = CS_DIR_PP_START;
    }
    break;

    case CS_DIR_PP_START: // �趨��ղ���
        FOC_voltage(motor->components.foc, (voltage * time / 2.0f), 0.0f, phase_set);
        if (time >= 2.0f)
        {
            start_count           = (float)motor->components.encoder->shadow_count_;
            motor->state.Cs_State = CS_DIR_PP_LOOP;
            break;
        }
        break;

    case CS_DIR_PP_LOOP: // У׼����͵��������

        phase_set += calib_phase_vel * CURRENT_MEASURE_PERIOD;
        FOC_voltage(motor->components.foc, voltage, 0.0f, phase_set);
        if (phase_set >= 4.0f * M_2PI)
        {
            motor->state.Cs_State = CS_DIR_PP_END;
        }
        break;

    case CS_DIR_PP_END: // У׼����͵�����������
    {
        int32_t diff = motor->components.encoder->shadow_count_ - start_count;

        // Check direction
        if (diff > 0)
        {
            motor->components.encoder->dir = CW;
        }
        else
        {
            motor->components.encoder->dir = CCW;
        }

        // Motor pole pairs
        motor->components.encoder->pole_pairs = round(4.0f / ABS(diff / ENCODER_CPR_F));

        if (motor->components.encoder->pole_pairs > MOTOR_POLE_PAIRS_MAX)
        {
            break;
        }

        motor->state.Cs_State = CS_ENCODER_START;
    }
    break;

    case CS_ENCODER_START: // �趨��ղ���
        phase_set             = 0;
        loop_count            = 0;
        sample_count          = 0;
        next_sample_time      = 0;
        motor->state.Cs_State = CS_ENCODER_CW_LOOP;
        break;

    case CS_ENCODER_CW_LOOP: // У׼����������ת��
        // Test voltage along phase A
        //        FOC_voltage(motor->components.foc, 0, voltage, M_3PI_2);

        //        if (loop_count >= num_R_cycles/1.5)
        //        {
        //			motor->components.encoder->encoder_offset = motor->components.encoder->cnt;
        //			FOC_voltage(motor->components.foc, 0, 0, M_3PI_2);
        //
        //			free(p_error_arr);
        //            p_error_arr = NULL;
        //            motor->components.encoder->calib_valid = true;
        //            motor->state.Cs_State = CS_STATE_IDLE;
        //            motor->state.Sub_State = SUB_STATE_IDLE;      // �ı���״̬
        //            motor->state.State_Mode = STATE_MODE_RUNNING; // �ı���״̬
        //        }

        if (sample_count < (motor->components.encoder->pole_pairs * SAMPLES_PER_PPAIR))
        {
            if (time > next_sample_time)
            {
                next_sample_time         += M_2PI / ((float)SAMPLES_PER_PPAIR * calib_phase_vel);

                int count_ref             = (phase_set * ENCODER_CPR_F) / (M_2PI * (float)motor->components.encoder->pole_pairs);
                int error                 = motor->components.encoder->cnt - count_ref;
                error                    += ENCODER_CPR * (error < 0);
                p_error_arr[sample_count] = error;

                sample_count++;
            }

            phase_set += calib_phase_vel * CURRENT_MEASURE_PERIOD;
        }
        else
        {
            phase_set            -= calib_phase_vel * CURRENT_MEASURE_PERIOD;
            loop_count            = 0;
            sample_count--;
            next_sample_time      = 0;
            motor->state.Cs_State = CS_ENCODER_CCW_LOOP;
            break;
        }
        FOC_voltage(motor->components.foc, voltage, 0, phase_set);
        break;

    case CS_ENCODER_CCW_LOOP: // У׼����������ת��
        if (sample_count >= 0)
        {
            if (time > next_sample_time)
            {
                next_sample_time         += M_2PI / ((float)SAMPLES_PER_PPAIR * calib_phase_vel);

                int count_ref             = (phase_set * ENCODER_CPR_F) / (M_2PI * (float)motor->components.encoder->pole_pairs);
                int error                 = motor->components.encoder->cnt - count_ref;
                error                    += ENCODER_CPR * (error < 0);
                p_error_arr[sample_count] = (p_error_arr[sample_count] + error) / 2;

                sample_count--;
            }

            phase_set -= calib_phase_vel * CURRENT_MEASURE_PERIOD;
        }
        else
        {
            Foc_Pwm_LowSides();
            motor->state.Cs_State = CS_ENCODER_END;
            break;
        }
        FOC_voltage(motor->components.foc, voltage, 0, phase_set);
        break;

    case CS_ENCODER_END: // У׼���������
    {
        // Calculate average offset
        int64_t moving_avg = 0;
        for (int i = 0; i < (motor->components.encoder->pole_pairs * SAMPLES_PER_PPAIR); i++)
        {
            moving_avg += p_error_arr[i];
        }
        motor->components.encoder->encoder_offset = moving_avg / (motor->components.encoder->pole_pairs * SAMPLES_PER_PPAIR);

        // FIR and map measurements to lut
        int window     = SAMPLES_PER_PPAIR;
        int lut_offset = p_error_arr[0] * OFFSET_LUT_NUM / ENCODER_CPR;
        for (int i = 0; i < OFFSET_LUT_NUM; i++)
        {
            moving_avg = 0;
            for (int j = (-window) / 2; j < (window) / 2; j++)
            {
                int index   = i * motor->components.encoder->pole_pairs * SAMPLES_PER_PPAIR / OFFSET_LUT_NUM + j;
                if (index < 0)
                {
                    index  += (SAMPLES_PER_PPAIR * motor->components.encoder->pole_pairs);
                }
                else if (index > (SAMPLES_PER_PPAIR * motor->components.encoder->pole_pairs - 1))
                {
                    index  -= (SAMPLES_PER_PPAIR * motor->components.encoder->pole_pairs);
                }
                moving_avg += p_error_arr[index];
            }
            moving_avg      = moving_avg / window;
            int lut_index   = lut_offset + i;
            if (lut_index > (OFFSET_LUT_NUM - 1))
            {
                lut_index  -= OFFSET_LUT_NUM;
            }
            motor->components.encoder->offset_lut[lut_index] = moving_avg - motor->components.encoder->encoder_offset;
        }

        loop_count            = 0;
        sample_count          = 0;
        next_sample_time      = 0;
        motor->state.Cs_State = CS_REPORT_OFFSET_LUT;
    }
    break;

    case CS_REPORT_OFFSET_LUT: // ���У׼���
        if (sample_count < OFFSET_LUT_NUM)
        {
            if (time > next_sample_time)
            {
                next_sample_time += 0.001f;
                sample_count++;
            }
        }
        else
        {
            free(p_error_arr);
            p_error_arr                            = NULL;
            motor->state.Cs_State                  = CS_STATE_IDLE;
            motor->state.Sub_State                 = SUB_STATE_IDLE;    
            motor->state.State_Mode                = STATE_MODE_RUNNING; 
            motor->components.encoder->calib_valid = true;
            PID_clear(&motor->IqPID);
            PID_clear(&motor->IdPID);
            PID_clear(&motor->VelPID);
            PID_clear(&motor->PosPID);
        }
        break;

    default:
        break;
    }
    loop_count++;
}

static void MotorInitializeTask(MOTOR_DATA *motor)
{
    switch (motor->state.Sub_State)
    {
    case SUB_STATE_IDLE:      // ����״̬

        break;
    case CURRENT_CALIBRATING: // ��������
        CurrentCalibration(motor);
        break;
    case RSLS_CALIBRATING:    // �������н���
        RSLSCalibration(motor);
        break;
    case FLUX_CALIBRATING:    // ��������
        // ������
        break;

    default:
        break;
    }
}

/**
 * @brief FOC��������
 *
 * �˺�������ִ��FOC��Field Oriented Control���������񡣸��ݿ���ģʽ��ѡ����Ӧ�Ŀ�������
 *
 * @param motor ָ�������ݽṹ��ָ��
 *
 * ���رջ��ĵ��ڣ�
 * 1.����Q���趨ֵΪ0����D���趨ֵΪһ��ֵ��Ȼ������pid֮��鿴id���Σ�����id����һ����Ծ��Ӧ���۲��Ƿ�Ϊ���ǵ��趨ֵ
 * 2.�ٽ�Q������Ϊ����ֵ��D������Ϊ0��������ס������Ӧ���
 *
 * �ٶȱջ��ĵ��ڣ�
 * 1.����kp��ki��Ҫ���̫������Ȼ����ĵ�����Ծ�ͻ᲻�ȶ�����������ض���
 * 2.kp����Ҫʱ��������ֶ����������ٶ���ӦҪ������kiҪ��kpС��������
 *
 * λ�ñջ��ĵ��ڣ�
 * 1.����Ӧ�ٶ��Լ��Ƿ��й���
 */
#define TORQUE_ADJUST      0 // ���رջ�D�����
#define TORQUE_AND_CURRENT 0 // ���رջ�������ջ�
static void MotorControlTask(MOTOR_DATA *motor)
{
    static uint8_t loop_count;
    static float vel_set;

    switch (motor->state.Control_Mode)
    {
    case CONTROL_MODE_OPEN: // �ٶȵ�ѹ����
    {
        OpenControlMode(motor, 5);
    }
    break;

    case CONTROL_MODE_TORQUE: // ���رջ�������ʱ����С��������ס��תʱ�����ﵽĿ�������
    {
        SetPIDLimit(motor, 2.0f, CURRENT_PID_MAX_OUT, 0.0f, 0.0f, 0.0f);

#if TORQUE_AND_CURRENT
#if TORQUE_ADJUST
        motor->components.foc->iq_set  = 0;
        motor->components.foc->id_set  = 0.5f;
#else
		motor->components.foc->theta   = motor->components.encoder->phase_;
        motor->Controller.input_current= CLAMP(motor->Controller.input_current, -motor->Controller.current_limit, motor->Controller.current_limit);
        float max_step_size            = fabs(CURRENT_MEASURE_PERIOD * motor->Controller.torque_ramp_rate);
        float full_step                = motor->Controller.input_current - motor->components.foc->iq_set;
        float step                     = CLAMP(full_step, -max_step_size, max_step_size);
        motor->components.foc->iq_set += step;
        motor->components.foc->id_set  = 0.0f;
#endif
#else
        motor->components.foc->theta   = motor->components.encoder->phase_;
        motor->Controller.input_torque = CLAMP(motor->Controller.input_torque, -motor->Controller.torque_limit, motor->Controller.torque_limit);
        motor->Controller.input_current= CLAMP(motor->Controller.input_torque / motor->Controller.torque_const, -motor->Controller.current_limit, +motor->Controller.current_limit);
        float max_step_size            = fabs(CURRENT_MEASURE_PERIOD * motor->Controller.torque_ramp_rate);
        float full_step                = motor->Controller.input_current - motor->components.foc->iq_set;
        float step                     = CLAMP(full_step, -max_step_size, max_step_size);
        motor->components.foc->iq_set += step;
        motor->components.foc->id_set  = 0.0f;
#endif
    }
    break;

    case CONTROL_MODE_VELOCITY: // �ٶ�->���رջ�
    {
        SetPIDLimit(motor, CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT, VEL_PID_MAX_OUT, VEL_PID_MAX_OUT, 0.0f);
        motor->components.foc->theta = motor->components.encoder->phase_;
    }
    break;

    case CONTROL_MODE_POSITION: // λ��->�ٶ�->���رջ�
    {
        SetPIDLimit(motor, CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT, VEL_PID_MAX_OUT, VEL_PID_MAX_OUT, POS_PID_MAX_OUT);
        motor->components.foc->theta = motor->components.encoder->phase_;
    }
    break;

    case CONTROL_MODE_VELOCITY_RAMP: // �ٶ�->�����ݶȱջ�
    {
        SetPIDLimit(motor, CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT, VEL_PID_MAX_OUT, VEL_PID_MAX_OUT, POS_PID_MAX_OUT);
        motor->components.foc->theta      = motor->components.encoder->phase_;
        float max_step_size               = fabs(VEL_POS_PERIOD * motor->Controller.vel_ramp_rate);
        float full_step                   = motor->Controller.input_velocity - motor->Controller.vel_setpoint;
        float step                        = CLAMP(full_step, -max_step_size, max_step_size);
        motor->Controller.vel_setpoint   += step;
        motor->Controller.torque_setpoint = (step / VEL_POS_PERIOD) * motor->Controller.inertia;
    }
    break;

    case CONTROL_MODE_POSITION_RAMP: // λ��->�ٶ�->�����ݶȱջ�
    {
        SetPIDLimit(motor, CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT, VEL_PID_MAX_OUT, VEL_PID_MAX_OUT, POS_PID_MAX_OUT);
        motor->components.foc->theta = motor->components.encoder->phase_;
        if (motor->Controller.input_updated)
        {
            TRAJ_plan(motor->Controller.input_position,
                      motor->components.encoder->pos_estimate_,
                      motor->components.encoder->vel_estimate_,
                      motor->Controller.traj_vel,    // Velocity
                      motor->Controller.traj_accel,  // Acceleration
                      motor->Controller.traj_decel); // Deceleration

            Traj.t = 0.0f;
            Traj.trajectory_done = false;
            motor->Controller.input_updated = false;
        }

        // Avoid updating uninitialized trajectory
        if (Traj.trajectory_done)
        {
            break;
        }

        if (Traj.t > Traj.Tf_)
        {
            Traj.trajectory_done              = true;
            motor->Controller.pos_setpoint    = motor->Controller.input_position;
            motor->Controller.vel_setpoint    = 0.0f;
            motor->Controller.torque_setpoint = 0.0f;
        }
        else
        {
            TRAJ_eval(Traj.t);
            motor->Controller.pos_setpoint    = Traj.Y;
            motor->Controller.vel_setpoint    = Traj.Yd;
            motor->Controller.torque_setpoint = Traj.Ydd * motor->Controller.inertia;
            if (fabs(motor->Controller.pos_setpoint - motor->components.encoder->pos_estimate_) < 1.0f)
            {
                Traj.t += VEL_POS_PERIOD;
            }
        }
    }
    break;

    default:
        break;
    }

    // ������20khz���ٶ���λ��5khz���ٶ���λ�ñջ���Ŀ�������ͨ��λ�ú��ٶȵĵ����Զ��趨�ģ�
    if (motor->state.Control_Mode > CONTROL_MODE_OPEN)
    {
        commonFOCOperations(motor->components.foc);

        if (motor->state.Control_Mode >= CONTROL_MODE_VELOCITY && motor->state.Control_Mode <= CONTROL_MODE_POSITION)
        {
            if (++loop_count >= 4)
            {
                loop_count  = 0;

                if (motor->state.Control_Mode == CONTROL_MODE_POSITION)
                {
                    vel_set = PID_Calc(&motor->PosPID, motor->components.encoder->pos_estimate_, motor->Controller.input_position);
					vel_set = CLAMP(vel_set, -motor->Controller.input_velocity, +motor->Controller.input_velocity);
                }
                else if (motor->state.Control_Mode == CONTROL_MODE_VELOCITY)
                {
                    vel_set = motor->Controller.input_velocity;
					vel_set = CLAMP(vel_set, -motor->Controller.vel_limit, +motor->Controller.vel_limit);
                }
                motor->components.foc->iq_set            = PID_Calc(&motor->VelPID, motor->components.encoder->vel_estimate_, vel_set);
                motor->components.foc->id_set            = 0.0f;
            }
        }
        else if (motor->state.Control_Mode >= CONTROL_MODE_VELOCITY_RAMP)
        {
            if (++loop_count >= 4)
            {
                loop_count  = 0;

                if (motor->state.Control_Mode == CONTROL_MODE_POSITION_RAMP)
                {
                    vel_set = PID_Calc(&motor->PosPID, motor->components.encoder->pos_estimate_, motor->Controller.pos_setpoint) + motor->Controller.vel_setpoint;
                    vel_set = CLAMP(vel_set, -motor->Controller.input_velocity, +motor->Controller.input_velocity);
                }
                else if (motor->state.Control_Mode == CONTROL_MODE_VELOCITY_RAMP)
                {
                    vel_set = motor->Controller.vel_setpoint;
					vel_set = CLAMP(vel_set, -motor->Controller.vel_limit, +motor->Controller.vel_limit);
                }
                motor->components.encoder->vel_estimate_ = CLAMP(motor->components.encoder->vel_estimate_, -motor->Controller.vel_limit, +motor->Controller.vel_limit);
                motor->components.foc->iq_set            = PID_Calc(&motor->VelPID, motor->components.encoder->vel_estimate_, vel_set) + motor->Controller.torque_setpoint;
                motor->components.foc->id_set            = 0.0f;
            }
        }

        motor->components.foc->iq_set = CLAMP(motor->components.foc->iq_set, -motor->Controller.current_limit, +motor->Controller.current_limit);
        motor->components.foc->i_q    = CLAMP(motor->components.foc->i_q, -motor->Controller.current_limit, +motor->Controller.current_limit);

        motor->components.foc->v_q    = PID_Calc(&motor->IqPID, motor->components.foc->i_q, motor->components.foc->iq_set);
        motor->components.foc->v_d    = PID_Calc(&motor->IdPID, motor->components.foc->i_d, motor->components.foc->id_set);
        commonInverseFOCOperations(motor->components.foc);
    }
}

/**
 * @brief ���״̬��������
 *
 * ������20khz���ٶ���λ��5khz
 *
 * @param motor ָ��������״̬�Ϳ������ݵĽṹ��ָ��
 */
void MotorStateTask(MOTOR_DATA *motor)
{
    switch (motor->state.State_Mode)
    {
    case STATE_MODE_IDLE: // ����ģʽ
        PID_clear(&motor->IqPID);
        PID_clear(&motor->IdPID);
        PID_clear(&motor->VelPID);
        PID_clear(&motor->PosPID);
        FOC_reset(motor->components.foc);
        Foc_Pwm_LowSides();
        motor->state.State_Mode = STATE_MODE_DETECTING;
        motor->state.Sub_State  = CURRENT_CALIBRATING;
        break;
    case STATE_MODE_DETECTING: // �������ģʽ
        MotorInitializeTask(motor);
        break;
    case STATE_MODE_RUNNING: // ����ģʽ
        MotorControlTask(motor);
        break;
    case STATE_MODE_GUARD: // �ػ�ģʽ
        PID_clear(&motor->IqPID);
        PID_clear(&motor->IdPID);
        PID_clear(&motor->VelPID);
        PID_clear(&motor->PosPID);
        FOC_reset(motor->components.foc);
        Foc_Pwm_LowSides();
        break;
    }
}

/**
 * @brief ����ػ�����
 *
 * �˺�������ִ�е�����ػ�����
 *
 * @param motor ָ��������״̬�Ϳ������ݵĽṹ��ָ��
 */
void MotorGuardTask(MOTOR_DATA *motor)
{
    if (motor->state.State_Mode == STATE_MODE_IDLE || motor->state.State_Mode == STATE_MODE_DETECTING)
    {
        RGB_DisplayColorById(9); // ���ϵ���ʾ����У׼
    }
    else if (motor->state.State_Mode == STATE_MODE_RUNNING)
    {
        RGB_DisplayColorById(3); // ��������ʾ��������

        // ����ص�ѹ�Ƿ��ѹ��Ƿѹ
        if (motor->components.foc->vbus <= BATVEL_MIN_LIMIT)
        {
            motor->state.Fault_State = FAULT_STATE_UNDER_VOLTAGE;
            motor->state.State_Mode  = STATE_MODE_GUARD;
        }
        else if (motor->components.foc->vbus >= BATVEL_MAX_LIMIT)
        {
            motor->state.Fault_State = FAULT_STATE_OVER_VOLTAGE;
            motor->state.State_Mode  = STATE_MODE_GUARD;
        }

        // ����������Ƿ����
        else if ((motor->components.foc->i_q >= MOTOR_CURRENT_LIMIT) || (motor->components.foc->i_q <= -MOTOR_CURRENT_LIMIT))
        {
            motor->state.Fault_State = FAULT_STATE_OVER_CURRENT;
            motor->state.State_Mode  = STATE_MODE_GUARD;
        }

        // ������ٶ��Ƿ���
        else if (fabs(motor->components.encoder->vel_estimate_) >= SPEED_MAX_LIMIT)
        {
            motor->state.Fault_State = FAULT_STATE_SPEEDING;
            motor->state.State_Mode  = STATE_MODE_GUARD;
        }

        // ���mos�¶��Ƿ����
        else if (motor->components.current->Temp_Result >= TEMP_MAX_LIMIT)
        {
            motor->state.Fault_State = FAULT_STATE_OVER_TEMPERATURE;
            motor->state.State_Mode  = STATE_MODE_GUARD;
        }
    }
    else if (motor->state.State_Mode == STATE_MODE_GUARD)
    {
        RGB_DisplayColorById(0); // �������ʾ����
    }
}

#include "BLDCMotor.h"

// ȫ�ֱ�������������������
FOC_DATA foc_data = {
    .inv_vbus                = INVBATVEL,                 // ������ѹ
    .vd_set                  = 0.0f,                      // d ���ѹ����
    .vq_set                  = 0.0f,                      // q ���ѹ����
    .id_set                  = 0.0f,                      // d ���������
    .iq_set                  = MOTOR_INPUT_CURRENT,       // q ���������
    .i_d_filt                = 0.0f,                      // ��ͨ�˲���� d �����
    .i_q_filt                = 0.0f,                      // ��ͨ�˲���� q �����
    .i_bus                   = 0.0f,                      // �ܵ���
    .i_bus_filt              = 0.0f,                      // ��ͨ�˲�����ܵ���
    .power_filt              = 0.0f,                      // ��ͨ�˲�����ܹ���
    .current_ctrl_integral_d = 0.0f,                      // d �����������ֵ
    .current_ctrl_integral_q = 0.0f,                      // q �����������ֵ
    .current_ctrl_p_gain     = MOTOR_CURRENT_CTRL_P_GAIN, // ���������棬��У׼�������Զ�����ó���Ҳ���������� (Auto)
    .current_ctrl_i_gain     = MOTOR_CURRENT_CTRL_I_GAIN, // �������������棬��У׼�������Զ�����ó���Ҳ���������� (Auto)
};

void FOC_reset(FOC_DATA *foc)
{
    foc->i_d                 = 0.0f;
    foc->i_q                 = 0.0f;
    foc->current_ctrl_p_gain = 0.0f;
    foc->current_ctrl_i_gain = 0.0f;
}

/**
 * @brief ����PWM���
 *
 * ������ʱ��ͨ��1��2��3��PWM��������ڿ��Ƶ�����������
 *
 * @param
 */
void Foc_Pwm_Start(void)
{
    set_dtc_a(PWM_ARR >> 1); // ����ͨ��1��ռ�ձ�
    set_dtc_b(PWM_ARR >> 1); // ����ͨ��2��ռ�ձ�
    set_dtc_c(PWM_ARR >> 1); // ����ͨ��3��ռ�ձ�

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief ֹͣPWM���
 *
 * ֹͣ��ʱ��ͨ��1��2��3��PWM��������ڿ��Ƶ�����������
 *
 * @param
 */
void Foc_Pwm_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

void Foc_Pwm_LowSides(void)
{
    /* Set all duty to 0% */
    set_dtc_a(0); // ����ͨ��1��ռ�ձ�
    set_dtc_b(0); // ����ͨ��2��ռ�ձ�
    set_dtc_c(0); // ����ͨ��3��ռ�ձ�

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief ����PWMռ�ձ�
 *
 * ��������ֵ��������ͨ����PWMռ�ձȣ����ڿ��Ƶ��ת�ٺ�ת��
 *
 * @param dtc_a ͨ��1��ռ�ձ�ֵ��0-100%��
 * @param dtc_b ͨ��2��ռ�ձ�ֵ��0-100%��
 * @param dtc_c ͨ��3��ռ�ձ�ֵ��0-100%��
 */
void SetPwm(FOC_DATA *foc)
{
    // ���ø���ͨ����Ӧ�Ĵ����Ƚ�ֵ��ʵ��PWM�������
    set_dtc_a((uint16_t)(foc->dtc_a * PWM_ARR)); // ����ͨ��1��ռ�ձ�
    set_dtc_b((uint16_t)(foc->dtc_b * PWM_ARR)); // ����ͨ��2��ռ�ձ�
    set_dtc_c((uint16_t)(foc->dtc_c * PWM_ARR)); // ����ͨ��3��ռ�ձ�
}

/**
 * @brief �������Һ�����ֵ
 *
 * �����Ƕ� theta ��Ӧ�� sin �� cos ֵ
 *
 * @param foc ָ�������ݽṹ��ָ��
 */
void Sin_Cos_Val(FOC_DATA *foc)
{
    foc->sin_val = arm_sin_f32(foc->theta); // ��������ֵ
    foc->cos_val = arm_cos_f32(foc->theta); // ��������ֵ
}

/**
 * @brief Clarke�任
 *
 * Clarke�任���������ת��Ϊ Alpha-Beta ����ϵ�µĵ���
 *
 * @param foc ָ�������ݽṹ��ָ��
 */
void Clarke(FOC_DATA *foc)
{
    foc->i_alpha = foc->i_a;                            // �����������a�����
    foc->i_beta  = (foc->i_b - foc->i_c) * ONE_BY_SQRT3; // ����������㣬ʹ�á�3�ĵ������й�һ��
}

/**
 * @brief ��Clarke�任
 *
 * ��Clarke�任�� Alpha-Beta ����ϵ�µĵ�ѹת��Ϊ�����ѹ
 *
 * @param foc ָ�������ݽṹ��ָ��
 */
void Inv_clarke(FOC_DATA *foc)
{
    foc->v_a = foc->v_alpha;                                  // a���ѹ���ڦ����ѹ
    foc->v_b = -0.5f * foc->v_alpha + _SQRT3_2 * foc->v_beta; // b���ѹ����
    foc->v_c = -0.5f * foc->v_alpha - _SQRT3_2 * foc->v_beta; // c���ѹ����
}

/**
 * @brief Park�任
 *
 * Park�任�� Alpha-Beta ����ϵ�µĵ���ת��Ϊ dq ����ϵ�µĵ�����ת�����������㶨��ֱ������
 *
 * @param foc ָ�������ݽṹ��ָ��
 */
void Park(FOC_DATA *foc)
{
    foc->i_d = foc->i_alpha * foc->cos_val + foc->i_beta * foc->sin_val; // d���������
    foc->i_q = foc->i_beta * foc->cos_val - foc->i_alpha * foc->sin_val; // q���������
}

/**
 * @brief ��Park�任
 *
 * ��Park�任�� dq ����ϵ�µĵ�ѹת��Ϊ Alpha-Beta ����ϵ�µĵ�ѹ
 *
 * @param foc ָ�������ݽṹ��ָ��
 */
void Inv_Park(FOC_DATA *foc)
{
    foc->v_alpha = (foc->v_d * foc->cos_val - foc->v_q * foc->sin_val); // �����ѹ����
    foc->v_beta  = (foc->v_d * foc->sin_val + foc->v_q * foc->cos_val);  // �����ѹ����
}

/**
 * @brief ��ֵ�������ע��
 *
 * �Ԧ�-������ϵ�ĵ�ѹ���о�ֵ�������ע�룬�Ա����SVPWM����
 * ��ѹ�Ƕ��Ǵ�ֱ��ת�ӽǶȵģ�n������Ǻϳɵ�ѹ������������ת�����ڵ�����
 *
 * @param foc ָ�������ݽṹ��ָ��
 */
void Svpwm_Midpoint(FOC_DATA *foc)
{
    // ����ĸ�ߵ�ѹ�������ͦ����ѹ
    foc->v_alpha = foc->inv_vbus * foc->v_alpha;
    foc->v_beta  = foc->inv_vbus * foc->v_beta;

    float va     = foc->v_alpha;                                  // ��������ѹ
    float vb     = -0.5f * foc->v_alpha + _SQRT3_2 * foc->v_beta; // ����b���ѹ
    float vc     = -0.5f * foc->v_alpha - _SQRT3_2 * foc->v_beta; // ����c���ѹ

    float vmax   = max(max(va, vb), vc); // �ҵ�����ѹֵ
    float vmin   = min(min(va, vb), vc); // �ҵ���С��ѹֵ

    float vcom   = (vmax + vmin) * 0.5f; // �����������

    // ��������ռ�ձȵ���ֵ��ȷ����0��1֮��
    foc->dtc_a   = ((vcom - va) + 0.5f), 0.0f, 1.0f;
    foc->dtc_b   = ((vcom - vb) + 0.5f), 0.0f, 1.0f;
    foc->dtc_c   = ((vcom - vc) + 0.5f), 0.0f, 1.0f;
}

/**
 * @brief �����ж�
 *
 * ���ݦ�-������ϵ�еĵ�ѹֵ�жϵ�ǰ��������������������ռ�ձ�
 *
 * @param foc ָ�������ݽṹ��ָ��
 */
void Svpwm_Sector(FOC_DATA *foc)
{
    float ta    = 0.0f, tb = 0.0f, tc = 0.0f;                    // ����ռ�ձ�ʱ���ʼ��
    float k     = (TS * _SQRT3) * foc->inv_vbus;                 // ����ĸ�ߵ�ѹ����kֵ
    float va    = foc->v_beta;                                   // �����ѹ
    float vb    = (_SQRT3 * foc->v_alpha - foc->v_beta) * 0.5f;  // b���ѹ����
    float vc    = (-_SQRT3 * foc->v_alpha - foc->v_beta) * 0.5f; // c���ѹ����
    int a       = (va > 0.0f) ? 1 : 0;                           // �ж�a���Ƿ�����㣬���Ϊ1��0
    int b       = (vb > 0.0f) ? 1 : 0;                           // �ж�b���Ƿ�����㣬���Ϊ1��0
    int c       = (vc > 0.0f) ? 1 : 0;                           // �ж�c���Ƿ�����㣬���Ϊ1��0
    int sextant = (c << 2) + (b << 1) + a;                       // ����a��b��c���״̬ȷ������

    switch (sextant)
    {
    case SECTOR_3: // ����3
    {
        float t4 = k * vb;
        float t6 = k * va;
        float t0 = (TS - t4 - t6) * 0.5f; // ��ʸ��������ʱ��

        ta       = t4 + t6 + t0; // ����a��ռ�ձ�ʱ��
        tb       = t6 + t0;      // ����b��ռ�ձ�ʱ��
        tc       = t0;           // c��ռ�ձ�ʱ��Ϊt0
    }
    break;

    case SECTOR_1: // ����1
    {
        float t6 = -k * vc;
        float t2 = -k * vb;
        float t0 = (TS - t2 - t6) * 0.5f;

        ta       = t6 + t0;      // a��ռ�ձ�ʱ�����
        tb       = t2 + t6 + t0; // b��ռ�ձ�ʱ�����
        tc       = t0;           // c��ռ�ձ�ʱ��Ϊt0
    }
    break;

    case SECTOR_5: // ����5
    {
        float t2 = k * va;
        float t3 = k * vc;
        float t0 = (TS - t2 - t3) * 0.5f;

        ta       = t0;           // a��ռ�ձ�ʱ��Ϊt0
        tb       = t2 + t3 + t0; // b��ռ�ձ�ʱ�����
        tc       = t3 + t0;      // c��ռ�ձ�ʱ�����
    }
    break;

    case SECTOR_4: // ����4
    {
        float t1 = -k * va;
        float t3 = -k * vb;
        float t0 = (TS - t1 - t3) * 0.5f;

        ta       = t0;           // a��ռ�ձ�ʱ��Ϊt0
        tb       = t3 + t0;      // b��ռ�ձ�ʱ�����
        tc       = t1 + t3 + t0; // c��ռ�ձ�ʱ�����
    }
    break;

    case SECTOR_6: // ����6
    {
        float t1 = k * vc;
        float t5 = k * vb;
        float t0 = (TS - t1 - t5) * 0.5f;

        ta       = t5 + t0;      // a��ռ�ձ�ʱ�����
        tb       = t0;           // b��ռ�ձ�ʱ��Ϊt0
        tc       = t1 + t5 + t0; // c��ռ�ձ�ʱ�����
    }
    break;

    case SECTOR_2: // ����2
    {
        float t4 = -k * vc;
        float t5 = -k * va;
        float t0 = (TS - t4 - t5) * 0.5f;

        ta       = t4 + t5 + t0; // a��ռ�ձ�ʱ�����
        tb       = t0;           // b��ռ�ձ�ʱ��Ϊt0
        tc       = t5 + t0;      // c��ռ�ձ�ʱ�����
    }
    break;

    default:
        break; // Ĭ���������������
    }

    // ���¸����ռ�ձȵ���ֵ����ת����ӦSVPWM���Ʒ�ʽ��ʹ����[0,1]��Χ�ڡ�
    foc->dtc_a   = (1.0f - ta), 0.0f, 1.0f;
    foc->dtc_b   = (1.0f - tb), 0.0f, 1.0f;
    foc->dtc_c   = (1.0f - tc), 0.0f, 1.0f;
}

/**
 * @brief FOC�û�����
 *
 * �˺������ڰ�װFOC��һЩ��������
 *
 * @param motor ָ�������ݽṹ��ָ��
 */
void commonFOCOperations(FOC_DATA *foc)
{
    Sin_Cos_Val(foc);
    Clarke(foc);
    Park(foc);
}

void commonInverseFOCOperations(FOC_DATA *foc)
{
    Inv_Park(foc);
    Inv_clarke(foc);
    Svpwm_Sector(foc);
    SetPwm(foc);
}
void setPhaseVoltage(FOC_DATA *foc, float Vd_set, float Vq_set, float phase)
{
    foc->v_q   = Vq_set;
    foc->v_d   = Vd_set;
    foc->theta = phase;
    Sin_Cos_Val(foc);
    commonInverseFOCOperations(foc);
}

void FOC_voltage(FOC_DATA *foc, float Vd_set, float Vq_set, float phase)
{
    // Clarke transform
    float i_alpha, i_beta;
    clarke_transform(foc->i_a, foc->i_b, foc->i_c, &i_alpha, &i_beta);

    // Park transform
    float i_d, i_q;
    park_transform(i_alpha, i_beta, phase, &i_d, &i_q);

    // Used for report
    foc->i_q = i_q;
    UTILS_LP_FAST(foc->i_q_filt, foc->i_q, 0.01f);
    foc->i_d = i_d;
    UTILS_LP_FAST(foc->i_d_filt, foc->i_d, 0.01f);

    // Modulation
    float V_to_mod = 1.0f / (foc->vbus * 2.0f / 3.0f);
    float mod_d    = V_to_mod * Vd_set;
    float mod_q    = V_to_mod * Vq_set;

    // Vector modulation saturation, lock integrator if saturated
    float mod_scalefactor = 0.9f * _SQRT3_2 / sqrtf(SQ(mod_d) + SQ(mod_q));
    if (mod_scalefactor < 1.0f)
    {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
    }

    // Inverse park transform
    float mod_alpha;
    float mod_beta;
    inverse_park(mod_d, mod_q, phase, &mod_alpha, &mod_beta);

    // SVM
    if (0 == svm(mod_alpha, mod_beta, &foc->dtc_a, &foc->dtc_b, &foc->dtc_c))
    {
        SetPwm(foc);
    }
}

void FOC_current(FOC_DATA *foc, float Id_set, float Iq_set, float phase, float phase_vel)
{
    // Clarke transform
    float i_alpha, i_beta;
    clarke_transform(foc->i_a, foc->i_b, foc->i_c, &i_alpha, &i_beta);

    // Park transform
    float i_d, i_q;
    park_transform(i_alpha, i_beta, phase, &i_d, &i_q);

    float mod_to_V = foc->vbus * 2.0f / 3.0f;
    float V_to_mod = 1.0f / mod_to_V;

    // Apply PI control
    float Ierr_d = Id_set - i_d;
    float Ierr_q = Iq_set - i_q;
    float mod_d  = V_to_mod * (foc->current_ctrl_integral_d + Ierr_d * foc->current_ctrl_p_gain);
    float mod_q  = V_to_mod * (foc->current_ctrl_integral_q + Ierr_q * foc->current_ctrl_p_gain);

    // Vector modulation saturation, lock integrator if saturated
    float mod_scalefactor = 0.9f * _SQRT3_2 / sqrtf(SQ(mod_d) + SQ(mod_q));
    if (mod_scalefactor < 1.0f)
    {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
        foc->current_ctrl_integral_d *= 0.99f;
        foc->current_ctrl_integral_q *= 0.99f;
    }
    else
    {
        foc->current_ctrl_integral_d += Ierr_d * (foc->current_ctrl_i_gain * CURRENT_MEASURE_PERIOD);
        foc->current_ctrl_integral_q += Ierr_q * (foc->current_ctrl_i_gain * CURRENT_MEASURE_PERIOD);
    }

    // Inverse park transform
    float mod_alpha, mod_beta;
    float pwm_phase = phase + phase_vel * CURRENT_MEASURE_PERIOD;
    inverse_park(mod_d, mod_q, pwm_phase, &mod_alpha, &mod_beta);

    // Used for reportZ
    foc->i_q        = i_q;
    UTILS_LP_FAST(foc->i_q_filt, foc->i_q, 0.01f);
    foc->i_d        = i_d;
    UTILS_LP_FAST(foc->i_d_filt, foc->i_d, 0.01f);
    foc->i_bus      = (mod_d * i_d + mod_q * i_q);
    UTILS_LP_FAST(foc->i_bus_filt, foc->i_bus, 0.01f);
    foc->power_filt = foc->vbus * foc->i_bus_filt;

    // SVM
    if (0 == svm(mod_alpha, mod_beta, &foc->dtc_a, &foc->dtc_b, &foc->dtc_c))
    {
        SetPwm(foc);
    }
}

#include "mt6816_encoder.h"

/**
 * @brief ��������ʼ������
 *
 * �˺������ڳ�ʼ����������ز����� SPI ���衣
 *
 * @param None
 */
ENCODER_DATA encoder_data = {
    .hspi                 = &MT6816_SPI_Get_HSPI, // ָ�� SPI �����ָ��
    .CS_Port              = SPI1_CS_GPIO_Port,    // Ƭѡ���ŵĶ˿�
    .CS_Pin               = SPI1_CS_Pin,          // Ƭѡ���ŵ����ź�
    .pole_pairs           = MPTOR_P,              // ���ü�����Ϊ�������
    .encoder_offset       = MOTOR_OFFSET,         // ���õ�Ƕ�ƫ����
    .dir                  = MOTOR_DIRECTION,      // ���ñ�������ת����Ϊ˳ʱ��
    .raw                  = 0,                    // ��ʼ��ԭʼ����ֵΪ0
    .cnt                  = 0,                    // ������
    .theta_acc            = 0.01f,                // ���ýǶ��ۼӲ���Ϊ0.01����
    .count_in_cpr_        = 0,                    // ԭʼֵ�������ͣ�
    .shadow_count_        = 0,                    // ԭʼֵ�ۼӼ����������ͣ�
    .pos_estimate_counts_ = 0.0f,                 // ԭʼֵ�ۼӼ����������ͣ�
    .vel_estimate_counts_ = 0.0f,                 // ԭʼֵת��
    .pos_cpr_counts_      = 0.0f,                 // ԭʼֵ�������ͣ�
    .pos_estimate_        = 0.0f,                 // Ȧ��
    .vel_estimate_        = 0.0f,                 // ���ٶ�rad/s
    .pos_cpr_             = 0.0f,                 // 0-1
    .phase_               = 0.0f,                 // ��Ƕ�
    .interpolation_       = 0.0f,                 // ��ֵϵ��
    .elec_angle           = 0.0f,                 // ��Ƕ�
    .mec_angle            = 0.0f,                 // ��е�Ƕ�
    .calib_valid          = false,                // ��ʾУ׼�������Ƿ���Ч
};

/**
 * @brief ���Ƕȹ�һ���� [0, 2��] ��Χ��
 *
 * �˺����������ĽǶ�ֵ��һ���� [0, 2��] ��Χ�ڡ�
 *
 * @param angle Ҫ��һ���ĽǶ�
 */
float normalize_angle(float angle)
{
    float a = fmod(angle, M_2PI);    // ʹ��ȡ��������й�һ��
    return a >= 0 ? a : (a + M_2PI); // ȷ������ֵ�� [0, 2��] ��Χ��
}

/**
 * @brief �������������
 *
 * �˺���ͨ�� SPI �ӿڷ��ͺͽ���һ���ֽڵ����ݡ�
 *
 * @param txdata ָ��Ҫ���͵����ݵ�ָ��
 * @param rxdata ָ��������ݵ�ָ��
 */
bool MT6816_read_write_byte(uint8_t *txdata, uint8_t *rxdata)
{
    MT6816_SPI_CS_L();
    bool transmitResult = (HAL_OK == HAL_SPI_Transmit(&MT6816_SPI_Get_HSPI, txdata, 1, MT6816_MAX_DELAY));
    bool receiveResult  = (HAL_OK == HAL_SPI_Receive(&MT6816_SPI_Get_HSPI, rxdata, 1, MT6816_MAX_DELAY));
    MT6816_SPI_CS_H();
    return transmitResult && receiveResult;
}

/**
 * @brief �� MT6816 ��������ȡԭʼ�Ƕ�����
 *
 * �˺�����ȡ MT6816 ��������ԭʼ�Ƕ�ֵ
 *
 * @param encoder ָ��������״̬�Ϳ������ݵĽṹ��ָ��
 */
bool mt6816_read_raw(ENCODER_DATA *encoder)
{
    const uint8_t tx[] = {MT6816_Angle_Reg, MT6816_Warning_Reg};
    uint8_t rx[2];
    uint8_t h_count;
    uint16_t rawAngle;

    bool Result_One = MT6816_read_write_byte((uint8_t *)&tx[0], (uint8_t *)&rx[0]);
    bool Result_Two = MT6816_read_write_byte((uint8_t *)&tx[1], (uint8_t *)&rx[1]);

    if (!Result_One || !Result_Two)
    {
        goto TIMEOUT;
    }

    if (encoder->rx_err_count)
    {
        encoder->rx_err_count--;
    }

    // ����ԭʼ�Ƕ�
    rawAngle = ((rx[0] & 0xFF) << 8) | (rx[1] & 0xFF);

    // ��żУ��
    h_count = 0;
    for (uint8_t j = 0; j < 16; j++)
    {
        if (rawAngle & (0x01 << j))
        {
            h_count++;
        }
    }
    if (h_count & 0x01)
    {
        goto CHECK_ERR;
    }

    encoder->angle = rawAngle >> 2;

    if (encoder->check_err_count)
    {
        encoder->check_err_count--;
    }

    return true;

CHECK_ERR:

    return false;

TIMEOUT:

    MT6816_SPI_CS_H();

    if (encoder->rx_err_count < 0xFF)
    {
        encoder->rx_err_count++;
    }

    return false;
}

/**
 * @brief ����ȡ��ԭʼֵת����ʵ�ʽǶȺ͵�Ƕ�
 *
 * �˺�������ȡ��ԭʼֵת����ʵ�ʽǶȺ͵�Ƕȡ�
 *
 * @param encoder ָ��������״̬�Ϳ������ݵĽṹ��ָ��
 */
void GetMotor_Angle(ENCODER_DATA *encoder)
{
    static const float pll_kp_ = 2.0f * ENCODER_PLL_BANDWIDTH;
    static const float pll_ki_ = 0.25f * SQ(pll_kp_);
    static const float snap_threshold = 0.5f * CURRENT_MEASURE_PERIOD * pll_ki_;
    if (mt6816_read_raw(encoder))
    {
        if (encoder->dir == CW)
        {
            encoder->raw = encoder->angle;
        }
        else
        {
            encoder->raw = (ENCODER_CPR - encoder->angle);
        }
    }

    // offset compensation
    int off_1      = encoder->offset_lut[encoder->raw >> 7];
    int off_2      = encoder->offset_lut[((encoder->raw >> 7) + 1) % 128];
    int off_interp = off_1 + ((off_2 - off_1) * (encoder->raw - ((encoder->raw >> 7) << 7)) >> 7); // Interpolate between lookup table entries
    int cnt        = encoder->raw - off_interp;                                                           // Correct for nonlinearity with lookup table from calibration
    if (cnt > ENCODER_CPR)
    {
        cnt -= ENCODER_CPR;
    }
    else if (cnt < 0)
    {
        cnt += ENCODER_CPR;
    }
    encoder->cnt   = cnt;

    int delta_enc  = encoder->cnt - encoder->count_in_cpr_;
    delta_enc      = mod(delta_enc, ENCODER_CPR);
    if (delta_enc > ENCODER_CPR_DIV)
    {
        delta_enc -= ENCODER_CPR;
    }

    encoder->shadow_count_ += delta_enc;
    encoder->count_in_cpr_ += delta_enc;
    encoder->count_in_cpr_  = mod(encoder->count_in_cpr_, ENCODER_CPR);

    encoder->count_in_cpr_  = encoder->cnt;

    //// run pll (for now pll is in units of encoder counts)
    // Predict current pos
    encoder->pos_estimate_counts_ += CURRENT_MEASURE_PERIOD * encoder->vel_estimate_counts_;
    encoder->pos_cpr_counts_      += CURRENT_MEASURE_PERIOD * encoder->vel_estimate_counts_;
    // discrete phase detector
    float delta_pos_counts         = (float)(encoder->shadow_count_ - (int32_t)floor(encoder->pos_estimate_counts_));
    float delta_pos_cpr_counts     = (float)(encoder->count_in_cpr_ - (int32_t)floor(encoder->pos_cpr_counts_));
    delta_pos_cpr_counts           = wrap_pm(delta_pos_cpr_counts, ENCODER_CPR_DIV);
    // pll feedback
    encoder->pos_estimate_counts_ += CURRENT_MEASURE_PERIOD * pll_kp_ * delta_pos_counts;
    encoder->pos_cpr_counts_      += CURRENT_MEASURE_PERIOD * pll_kp_ * delta_pos_cpr_counts;
    encoder->pos_cpr_counts_       = fmodf_pos(encoder->pos_cpr_counts_, ENCODER_CPR_F);
    encoder->vel_estimate_counts_ += CURRENT_MEASURE_PERIOD * pll_ki_ * delta_pos_cpr_counts;
    bool snap_to_zero_vel          = false;
    if (ABS(encoder->vel_estimate_counts_) < snap_threshold) // 100
    {
        encoder->vel_estimate_counts_ = 0.0f; // align delta-sigma on zero to prevent jitter
        snap_to_zero_vel = true;
    }

    // Outputs from Encoder for Controller
    float pos_cpr_last     = encoder->pos_cpr_;
    encoder->pos_estimate_ = encoder->pos_estimate_counts_ / ENCODER_CPR_F;
    encoder->vel_estimate_ = encoder->vel_estimate_counts_ / ENCODER_CPR_F;

    encoder->pos_estimate_ = encoder->pos_estimate_counts_ / ENCODER_CPR_F;
    encoder->vel_estimate_ = encoder->vel_estimate_counts_ / ENCODER_CPR_F;
    encoder->pos_cpr_      = encoder->pos_cpr_counts_ / ENCODER_CPR_F;
    float delta_pos_cpr    = wrap_pm(encoder->pos_cpr_ - pos_cpr_last, 0.5f);

    //// run encoder count interpolation
    int32_t corrected_enc  = encoder->count_in_cpr_ - encoder->encoder_offset;
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel)
    {
        encoder->interpolation_ = 0.5f;
        // reset interpolation if encoder edge comes
        // TODO: This isn't correct. At high velocities the first phase in this count may very well not be at the edge.
    }
    else if (delta_enc > 0)
    {
        encoder->interpolation_ = 0.0f;
    }
    else if (delta_enc < 0)
    {
        encoder->interpolation_ = 1.0f;
    }
    else
    {
        // Interpolate (predict) between encoder counts using vel_estimate,
        encoder->interpolation_ += CURRENT_MEASURE_PERIOD * encoder->vel_estimate_counts_;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (encoder->interpolation_ > 1.0f)
            encoder->interpolation_ = 1.0f;
        if (encoder->interpolation_ < 0.0f)
            encoder->interpolation_ = 0.0f;
    }
    float interpolated_enc = corrected_enc + encoder->interpolation_;

    //// compute electrical phase
    float elec_rad_per_enc = encoder->pole_pairs * M_2PI * (1.0f / ENCODER_CPR_F);
    float ph               = elec_rad_per_enc * interpolated_enc;
    encoder->phase_        = wrap_pm_pi(ph); // ��Ƕ�
    encoder->mec_angle     = normalize_angle(corrected_enc * (360.0f / ENCODER_CPR_F) * (M_PI / 180.0f));
    encoder->elec_angle    = normalize_angle(encoder->pole_pairs * encoder->mec_angle); // ��Ƕ�
}

/**
 * @brief �Ƕ���������
 *
 * ���ݵ������ת������ٶȣ����㲢���µ���ĵ�Ƕȡ�
 *
 * @param encoder ָ�������ݽṹ��ָ��
 */
void Theta_ADD(ENCODER_DATA *encoder)
{
    encoder->elec_angle    += encoder->theta_acc;
    if (encoder->elec_angle > M_2PI)
        encoder->elec_angle = 0.0f;
    else if (encoder->elec_angle < 0)
        encoder->elec_angle = M_2PI;
}

#include "led.h" // ���� LED ������ص�ͷ�ļ�

/* ��̬��ɫ�������� */
const RGB_Color_TypeDef RED = {255, 0, 0};        // ��ɫ
const RGB_Color_TypeDef GREEN = {0, 255, 0};      // ��ɫ
const RGB_Color_TypeDef BLUE = {0, 0, 255};       // ��ɫ
const RGB_Color_TypeDef SKY = {0, 255, 255};      // ����ɫ
const RGB_Color_TypeDef MAGENTA = {255, 0, 220};  // Ʒ��ɫ
const RGB_Color_TypeDef YELLOW = {127, 216, 0};   // ��ɫ
const RGB_Color_TypeDef ORANGE = {127, 106, 0};   // ��ɫ
const RGB_Color_TypeDef BLACK = {0, 0, 0};        // ��ɫ
const RGB_Color_TypeDef WHITE = {255, 255, 255};  // ��ɫ
const RGB_Color_TypeDef PURPLE = {128, 0, 128};   // ��ɫ
const RGB_Color_TypeDef BROWN = {165, 42, 42};    // ��ɫ
const RGB_Color_TypeDef GRAY = {128, 128, 128};   // ��ɫ
const RGB_Color_TypeDef PINK = {255, 192, 203};   // ��ɫ
const RGB_Color_TypeDef GOLD = {255, 215, 0};     // ��ɫ
const RGB_Color_TypeDef SILVER = {192, 192, 192}; // ��ɫ

/* LED���ݻ����� */
uint32_t led_buf[LED_MAX_NUM + 1][24]; // LED_MAX_NUM + 1 ��Ϊ�˴洢���һ�е� RESET �ź�

/**
 * @brief �趨���� RGB LED ����ɫ
 *
 * �˺������ݸ����� LED ��ź���ɫ���ö�Ӧ LED ����ɫ���ݡ�
 *
 * @param LedId LED ��ţ���Χ�� 0 �� LED_MAX_NUM
 * @param Color ��ɫ�ṹ�壬���� RGB ��������
 */
void RGB_SetColor(uint8_t LedId, RGB_Color_TypeDef Color)
{
    if (LedId > LED_MAX_NUM) // ��� LED ����Ƿ�Խ��
        return;              // ��ֹ����Խ��

    // ����ɫ����ת��Ϊ PWM �źŸ�ʽ���洢�� led_buf ��
    for (uint8_t i = 0; i < 8; i++)
    {
        // ���� G����ɫ������
        led_buf[LedId][i]      = (Color.G & (1 << (7 - i))) ? CODE_1 : CODE_0;
        // ���� R����ɫ������
        led_buf[LedId][8 + i]  = (Color.R & (1 << (7 - i))) ? CODE_1 : CODE_0;
        // ���� B����ɫ������
        led_buf[LedId][16 + i] = (Color.B & (1 << (7 - i))) ? CODE_1 : CODE_0;
    }
}

/**
 * @brief �趨���һ�е� RESET ��
 *
 * �˺������ڽ� LED ���ݻ����������һ�����㣬�Է��� RESET �źš�
 */
void Reset_Load(void)
{
    memset(led_buf[LED_MAX_NUM], 0, sizeof(led_buf[LED_MAX_NUM])); // ������һ������
}

/**
 * @brief ���� LED ����
 *
 * �˺���ͨ�� DMA ���� PWM ������� LED ���ݷ��͵�Ӳ����
 */
void RGB_SendArray(void)
{
    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, (uint32_t *)led_buf, (LED_MAX_NUM + 1) * 24); // ���� PWM ���
}

/**
 * @brief ��ʾָ����ɫ
 *
 * �˺�����ָ����ɫӦ�õ����� LED��
 *
 * @param color ��ɫ�ṹ�壬���� RGB ��������
 */
void RGB_DisplayColor(RGB_Color_TypeDef color)
{
    for (uint16_t i = 0; i < LED_MAX_NUM; i++) // �������� LED
    {
        RGB_SetColor(i, color); // ����ÿ�� LED ����ɫ
    }
    Reset_Load();    // ���� RESET �ź�
    RGB_SendArray(); // �������ݵ� LED
}

/**
 * @brief ������ɫ�����ʾ��ɫ
 *
 * �˺������ݸ�������ɫ�����ʾ��Ӧ����ɫ��
 *
 * @param color_id ��ɫ��ţ���Χ�� 0 �� 14���������µ���ɫ��
 */
void RGB_DisplayColorById(uint8_t color_id)
{
    RGB_Color_TypeDef color; // ����һ����ɫ�ṹ��

    // ������ɫ���ѡ���Ӧ����ɫ
    switch (color_id)
    {
    case 0:
        color = RED; // ��ɫ
        break;
    case 1:
        color = GREEN; // ��ɫ
        break;
    case 2:
        color = BLUE; // ��ɫ
        break;
    case 3:
        color = SKY; // ����ɫ
        break;
    case 4:
        color = MAGENTA; // Ʒ��ɫ
        break;
    case 5:
        color = YELLOW; // ��ɫ
        break;
    case 6:
        color = ORANGE; // ��ɫ
        break;
    case 7:
        color = BLACK; // ��ɫ
        break;
    case 8:
        color = WHITE; // ��ɫ
        break;
    case 9:
        color = PURPLE; // ��ɫ
        break;
    case 10:
        color = BROWN; // ��ɫ
        break;
    case 11:
        color = GRAY; // ��ɫ
        break;
    case 12:
        color = PINK; // ��ɫ
        break;
    case 13:
        color = GOLD; // ��ɫ
        break;
    case 14:
        color = SILVER; // ��ɫ
        break;
    default:
        color = BLACK; // Ĭ����ɫΪ��ɫ
        break;
    }

    RGB_DisplayColor(color); // ������ʾ��ɫ����
}

/**
 * @brief DMA������ɺ�Ļص�����
 *
 * �˺����� DMA ���ݷ�����ɺ���ã���ȷ�� PWM ��������ܵ����š�
 */
void RGB_DMA_CompleteCallback(void)
{
    HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_2); // ֹͣ DMA PWM ���
}

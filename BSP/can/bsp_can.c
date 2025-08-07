#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"

/* can instance ptrs storage, used for recv callback */
// ��CAN���������жϻ��������,ѡ��hcan��rxid�뷢���жϵ�ʵ����ͬ���Ǹ�,������ص�����
// @todo: ����Ϊÿ��CAN���ߵ������һ��can_instanceָ������,��߻ص����ҵ�����
static FDCANInstance *fdcan_instance[FDCAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx; // ȫ��CANʵ������,ÿ�����µ�ģ��ע�������

/* ----------------two static function called by CANRegister()-------------------- */

/**
 * @brief CAN�˲�����ʼ��
 *
 * @param _instance can instance owned by specific module
 */
static bool FDCANAddFilter(FDCANInstance *_instance)
{
  HAL_StatusTypeDef result;
  FDCAN_FilterTypeDef fdcan_filter_conf;

  fdcan_filter_conf.IdType       = FDCAN_STANDARD_ID;       // ��׼ID
  fdcan_filter_conf.FilterIndex  = 0;                       // �˲�������
  fdcan_filter_conf.FilterType   = FDCAN_FILTER_RANGE;      // �˲�������
  fdcan_filter_conf.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // ������0������FIFO0
  fdcan_filter_conf.FilterID1    = 0x0000;                  // Ϊ0�Ͳ�������κ�id
  fdcan_filter_conf.FilterID2    = 0x0000;                  // ��������

  result = HAL_FDCAN_ConfigFilter(_instance->fdcan_handle, &fdcan_filter_conf);
  if (result != HAL_OK) // �˲�����ʼ��
  {
    return result == HAL_ERROR;
  }
  return result == HAL_OK;
}

/**
 * @brief �ڵ�һ��FDCANʵ����ʼ����ʱ����Զ����ô˺���,����CAN����
 *
 * @note �˺���������FDCAN1,����FDCAN1��FIFO0 & FIFO1���֪ͨ
 *
 */
static bool FDCANServiceInit(void)
{
  HAL_StatusTypeDef result;
  result = HAL_FDCAN_Start(&hfdcan1);
  result |= HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  result |= HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
  return result == HAL_OK;
}

/* ----------------------- two extern callable function -----------------------*/

FDCANInstance *FDCANRegister(FDCAN_Init_Config_s *config)
{
  if (!idx)
  {
    FDCANServiceInit(); // ��һ��ע��,�Ƚ���Ӳ����ʼ��
//    LOGINFO("[bsp_can] CAN Service Init");
  }
  if (idx >= FDCAN_MX_REGISTER_CNT) // �������ʵ����
  {
    while (1)
      LOGERROR("[bsp_can] CAN instance exceeded MAX num, consider balance the load of CAN bus");
  }
  for (size_t i = 0; i < idx; i++)
  { // �ظ�ע�� | id�ظ�
    if (fdcan_instance[i]->rx_id == config->rx_id && fdcan_instance[i]->fdcan_handle == config->fdcan_handle)
    {
      while (1)
        LOGERROR("[}bsp_can] CAN id crash ,tx [%d] or rx [%d] already registered", &config->tx_id, &config->rx_id);
    }
  }

  FDCANInstance *instance = (FDCANInstance *)malloc(sizeof(FDCANInstance)); // ����ռ�
  memset(instance, 0, sizeof(FDCANInstance));                               // ����Ŀռ�δ����0,����Ҫ�����
  // ���з��ͱ��ĵ�����
  instance->txconf.Identifier          = config->tx_id;      // ����id
  instance->txconf.IdType              = FDCAN_STANDARD_ID;  // ʹ�ñ�׼id,��չid��ʹ��CAN_ID_EXT(Ŀǰû������)
  instance->txconf.TxFrameType         = FDCAN_DATA_FRAME;   // ��������֡
  instance->txconf.DataLength          = 0x08;               // Ĭ�Ϸ��ͳ���Ϊ8
  instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;   // ����״ָ̬ʾ����
  instance->txconf.BitRateSwitch       = FDCAN_BRS_OFF;      // �������л��رգ��������ھ���CAN
  instance->txconf.FDFormat            = FDCAN_CLASSIC_CAN;  // ʹ�þ���CAN
  instance->txconf.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; // �����淢���¼�
  instance->txconf.MessageMarker       = 0x0;                // ��Ϣ���0
  // ���ûص������ͽ��շ���id
  instance->fdcan_handle               = config->fdcan_handle;
  instance->tx_id                      = config->tx_id;
  instance->rx_id                      = config->rx_id;
  instance->fdcan_module_callback      = config->fdcan_module_callback;
  instance->id                         = config->id;

  FDCANAddFilter(instance);         // ���CAN����������
  fdcan_instance[idx++] = instance; // ��ʵ�����浽can_instance��

  return instance; // ����canʵ��ָ��
}

/* @todo Ŀǰ�ƺ���װ����,Ӧ�����һ��ָ��tx_buff��ָ��,tx_buff��Ӧ����CAN instance���� */
/* �����CANinstance����txbuff,������һ�θ��ƵĿ��� */
uint8_t FDCANTransmit(FDCANInstance *_instance, float timeout)
{
  static uint32_t busy_count;
  static volatile float wait_time __attribute__((unused)); // for cancel warning
  float dwt_start = DWT_GetTimeline_ms();
  while (HAL_FDCAN_GetTxFifoFreeLevel(_instance->fdcan_handle) == 0) // �ȴ��������
  {
    if (DWT_GetTimeline_ms() - dwt_start > timeout) // ��ʱ
    {
      LOGWARNING("[bsp_can] CAN MAILbox full! failed to add msg to mailbox. Cnt [%d]", busy_count);
      busy_count++;
      return 0;
    }
  }
  wait_time = DWT_GetTimeline_ms() - dwt_start;
  // tx_mailbox�ᱣ��ʵ����������һ֡��Ϣ������,����֪�����ĸ����䷢���ƺ�Ҳûɶ��
  if (HAL_FDCAN_AddMessageToTxFifoQ(_instance->fdcan_handle, &_instance->txconf, _instance->tx_buff) != HAL_OK)
  {
    LOGWARNING("[bsp_can] CAN bus BUS! cnt:%d", busy_count);
    busy_count++;
    return 0;
  }
  return 1; // ���ͳɹ�
}

void FDCANSetDLC(FDCANInstance *_instance, uint8_t length)
{
  // ���ͳ��ȴ���!�����ò����Ƿ����,�����Ұָ��/Խ�����
  if (length > 8 || length == 0) // ��ȫ���
  {
    while (1)
      LOGERROR("[bsp_can] CAN DLC error! check your code or wild pointer");
  }
  _instance->txconf.DataLength = length;
}

/* -----------------------belows are callback definitions--------------------------*/

/**
 * @brief �˺����ᱻ����������������,���ڴ���FIFO0��FIFO1����ж�(˵���յ����µ�����)
 *        ���е�ʵ�����ᱻ����,�ҵ�can_handle��rx_id��ȵ�ʵ��ʱ,���ø�ʵ���Ļص�����
 *
 * @param _hcan
 * @param fifox passed to HAL_FDCAN_GetRxMessage() to get mesg from a specific fifo
 */
static void FDCANFIFOxCallback(FDCAN_HandleTypeDef *_hcan, uint32_t fifox)
{
  static FDCAN_RxHeaderTypeDef rxconf; // ͬ��
  uint8_t can_rx_buff[8];
  while (HAL_FDCAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO��Ϊ��,�п����������ж�ʱ�ж�֡���ݽ���
  {
    HAL_FDCAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff); // ��FIFO�л�ȡ����
    for (size_t i = 0; i < idx; ++i)
    { // �������˵������Ҫ�ҵ�ʵ��
      if (_hcan == fdcan_instance[i]->fdcan_handle && rxconf.Identifier == fdcan_instance[i]->rx_id)
      {
        if (fdcan_instance[i]->fdcan_module_callback != NULL) // �ص�������Ϊ�վ͵���
        {
          fdcan_instance[i]->rx_len = rxconf.DataLength;                      // ������յ������ݳ���
          memcpy(fdcan_instance[i]->rx_buff, can_rx_buff, rxconf.DataLength); // ��Ϣ��������Ӧʵ��
          fdcan_instance[i]->fdcan_module_callback(fdcan_instance[i]);        // �����ص��������ݽ����ʹ���
        }
        return;
      }
    }
  }
}

/**
 * @brief ע��,STM32������CAN�豸��������FIFO
 * ��������������HAL���еĻص�����,���Ǳ�HAL����Ϊ__weak,��������ǽ�������(��д)
 * ��FIFO0��FIFO1���ʱ���������������
 */
// ����ĺ��������CANFIFOxCallback()����һ�����������ض�CAN�豸����Ϣ

/**
 * @brief rx fifo callback. Once FIFO_0 is full,this func would be called
 *
 * @param hfdcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO0); // ���������Լ�д�ĺ�����������Ϣ
  }
}

/**
 * @brief rx fifo callback. Once FIFO_1 is full,this func would be called
 *
 * @param hfdcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
  {
    FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO1); // ���������Լ�д�ĺ�����������Ϣ
  }
}

#include "can_driver.h" // ���� CAN ������ͷ�ļ�
#include "string.h"     // �����ַ�����������ͷ�ļ�
#include "stdlib.h"
#include "crc8.h"

/**
 * @brief ����CAN comm�Ľ���״̬��buffer
 *
 * @param ins ��Ҫ���õ�ʵ��
 */
static void FDCANCommResetRx(FDCANCommInstance *ins)
{
  // ��ǰ�Ѿ��յ���buffer����
  memset(ins->raw_recvbuf, 0, ins->cur_recv_len);
  ins->recv_state   = 0; // ����״̬����
  ins->cur_recv_len = 0; // ��ǰ�Ѿ��յ��ĳ�������
}

/**
 * @brief fdcancomm�Ľ��ջص�����
 *
 * @param _instance
 */
static void FDCANCommRxCallback(FDCANInstance *_instance)
{
  // id���Խ� FDCANCommInstance �����ݴ��ݸ� FDCANInstance �Ļص�������
  FDCANCommInstance *comm = (FDCANCommInstance *)_instance->id; // ע��д��,��fdcan instance��idǿ��ת��ΪFDCANCommInstance*����

  /* ��ǰ����״̬�ж� */
  if (_instance->rx_buff[0] == FDCAN_COMM_HEADER && comm->recv_state == 0) // ֮ǰ��δ��ʼ�����Ҵ˴ΰ����һ��λ����֡ͷ
  {
    if (_instance->rx_buff[1] == comm->recv_data_len) // �����һ�����datalenҲ���������趨���ճ���(������Ϊ��ʱ��֧�ֶ�̬����)
    {
      comm->recv_state = 1; // ���ý���״̬Ϊ1,˵���Ѿ���ʼ����
    }
    else
      return; // ֱ����������
  }

  if (comm->recv_state) // �Ѿ��յ���֡ͷ
  {
    // ����Ѿ����յ��ĳ��ȼ��ϵ�ǰһ���ĳ��ȴ�����buf len,˵�����մ���
    if (comm->cur_recv_len + _instance->rx_len > comm->recv_buf_len)
    {
      FDCANCommResetRx(comm);
      return; // ����״̬Ȼ�󷵻�
    }

    // ֱ�Ӱѵ�ǰ���յ������ݽӵ�buffer����
    memcpy(comm->raw_recvbuf + comm->cur_recv_len, _instance->rx_buff, _instance->rx_len);
    comm->cur_recv_len += _instance->rx_len;

    // ������һ���Ժ�պõ�����buf len,˵���Ѿ�������
    if (comm->cur_recv_len == comm->recv_buf_len)
    {
      // ���buff�ﱾtail��λ�õ���CAN_COMM_TAIL
      if (comm->raw_recvbuf[comm->recv_buf_len - 1] == FDCAN_COMM_TAIL)
      { // ͨ��У��,�������ݵ�unpack_data��
        if (comm->raw_recvbuf[comm->recv_buf_len - 2] == crc_8(comm->raw_recvbuf + 2, comm->recv_data_len))
        { // ��������Ļ�����ʹ��DMA
          memcpy(comm->unpacked_recv_data, comm->raw_recvbuf + 2, comm->recv_data_len);
          comm->update_flag = 1;           // ���ݸ���flag��Ϊ1
          DaemonReload(comm->comm_daemon); // ����daemon,�������ݸ��º�һֱ������ȡ���������ݸ��²���ʱ
        }
      }
      FDCANCommResetRx(comm);
      return; // ����״̬Ȼ�󷵻�
    }
  }
}

static void FDCANCommLostCallback(void *cancomm)
{
  FDCANCommInstance *comm = (FDCANCommInstance *)cancomm;
  FDCANCommResetRx(comm);
}

FDCANCommInstance *FDCANCommInit(FDCANComm_Init_Config_s *comm_config)
{
  FDCANCommInstance *ins = (FDCANCommInstance *)malloc(sizeof(FDCANCommInstance));
  memset(ins, 0, sizeof(FDCANCommInstance));

  ins->recv_data_len  = comm_config->recv_data_len;
  ins->recv_buf_len   = comm_config->recv_data_len + FDCAN_COMM_OFFSET_BYTES; // head + datalen + crc8 + tail
  ins->send_data_len  = comm_config->send_data_len;
  ins->send_buf_len   = comm_config->send_data_len + FDCAN_COMM_OFFSET_BYTES;
  ins->raw_sendbuf[0] = FDCAN_COMM_HEADER;          // head,ֱ�����ñ���ÿ�η��Ͷ�Ҫ���¸�ֵ,�����tailͬ��
  ins->raw_sendbuf[1] = comm_config->send_data_len; // datalen
  ins->raw_sendbuf[comm_config->send_data_len + FDCAN_COMM_OFFSET_BYTES - 1] = FDCAN_COMM_TAIL;
  // can instance������
  comm_config->can_config.id = ins;                                    // FDCANComm��ʵ��ָ����ΪFDCANInstance��id,�ص������л��õ�
  comm_config->can_config.fdcan_module_callback = FDCANCommRxCallback; // ���ջص�����
  ins->can_ins = FDCANRegister(&comm_config->can_config);              // ע��CANʵ��

  Daemon_Init_Config_s daemon_config = {
      .callback     = FDCANCommLostCallback,
      .owner_id     = (void *)ins,
      .reload_count = comm_config->daemon_count,
  };
  ins->comm_daemon  = DaemonRegister(&daemon_config);
  return ins;
}

void FDCANCommSend(FDCANCommInstance *instance, uint8_t *data)
{
  static uint8_t crc8;
  static uint8_t send_len;
  // ��data copy��raw_sendbuf��,����crc8
  memcpy(instance->raw_sendbuf + 2, data, instance->send_data_len);
  crc8 = crc_8(data, instance->send_data_len);
  instance->raw_sendbuf[2 + instance->send_data_len] = crc8;

  // CAN���η������Ϊ8�ֽ�,�������8�ֽ�,��Ҫ�ְ�����
  for (size_t i = 0; i < instance->send_buf_len; i += 8)
  { // ��������һ��,send len����С��8,Ҫ�޸�CAN��txconf�е�DLCλ,����bsp_can�ṩ�Ľӿڼ���
    send_len = instance->send_buf_len - i >= 8 ? 8 : instance->send_buf_len - i;
    FDCANSetDLC(instance->can_ins, send_len);
    memcpy(instance->can_ins->tx_buff, instance->raw_sendbuf + i, send_len);
    FDCANTransmit(instance->can_ins, 2);
  }
}

void *FDCANCommGet(FDCANCommInstance *instance)
{
  instance->update_flag = 0; // ��ȡ�󽫸���flag��Ϊ0
  return instance->unpacked_recv_data;
}

uint8_t FDCANCommIsOnline(FDCANCommInstance *instance)
{
  return DaemonIsOnline(instance->comm_daemon);
}

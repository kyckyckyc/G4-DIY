#ifndef __CAN_DRIVER_H__ // ��ֹ�ظ�����ͷ�ļ�
#define __CAN_DRIVER_H__

#include "daemon.h"
#include "bsp_can.h" // ������ BSP���弶֧�ְ�����ص� CAN ����ͷ�ļ�

#define MX_FDCAN_COMM_COUNT 4 // ע����⸺��,һ�������ϲ�Ҫ���ع��������

#define FDCAN_COMM_MAX_BUFFSIZE 200 // �����/�����ֽ���,��������������Ӵ���ֵ
#define FDCAN_COMM_HEADER 's'       // ֡ͷ
#define FDCAN_COMM_TAIL 'e'         // ֡β
#define FDCAN_COMM_OFFSET_BYTES 4   // 's'+ datalen + crc8 + 'e'
#pragma pack(1)
/* CAN comm �ṹ��, ӵ��CAN comm��appӦ�ð���һ��CAN commָ�� */
typedef struct
{
  FDCANInstance *can_ins;
  /* ���Ͳ��� */
  uint8_t send_data_len;                                                  // �������ݳ���
  uint8_t send_buf_len;                                                   // ���ͻ���������,Ϊ�������ݳ���+֡ͷ�������ݳ���֡β�Լ�У���(4)
  uint8_t raw_sendbuf[FDCAN_COMM_MAX_BUFFSIZE + FDCAN_COMM_OFFSET_BYTES]; // ����4��bytes����֡ͷ֡βdatalen��У���
  /* ���ղ��� */
  uint8_t recv_data_len;                                                  // �������ݳ���
  uint8_t recv_buf_len;                                                   // ���ջ���������,Ϊ�������ݳ���+֡ͷ�������ݳ���֡β�Լ�У���(4)
  uint8_t raw_recvbuf[FDCAN_COMM_MAX_BUFFSIZE + FDCAN_COMM_OFFSET_BYTES]; // ����4��bytes����֡ͷ֡βdatalen��У���
  uint8_t unpacked_recv_data[FDCAN_COMM_MAX_BUFFSIZE];                    // ����������,����FDCANCommGet()��cast�ɶ�Ӧ������ͨ��ָ���ȡ����
  /* ���պ͸��±�־λ*/
  uint8_t recv_state;   // ����״̬,
  uint8_t cur_recv_len; // ��ǰ�Ѿ����յ������ݳ���(����֡ͷ֡βdatalen��У���)
  uint8_t update_flag;  // ���ݸ��±�־λ,�����յ�������ʱ,�Ὣ�˱�־λ��1,����FDCANCommGet()��Ὣ�˱�־λ��0

  DaemonInstance *comm_daemon;
} FDCANCommInstance;
#pragma pack()

/* CAN comm ��ʼ���ṹ�� */
typedef struct
{
  FDCAN_Init_Config_s can_config; // CAN��ʼ���ṹ��
  uint8_t send_data_len;          // �������ݳ���
  uint8_t recv_data_len;          // �������ݳ���
  uint16_t daemon_count;          // �ػ����̼���,���ڳ�ʼ���ػ�����
} FDCANComm_Init_Config_s;

/**
 * @brief ��ʼ��FDCANComm
 *
 * @param config FDCANComm��ʼ���ṹ��
 * @return FDCANCommInstance*
 */
FDCANCommInstance *FDCANCommInit(FDCANComm_Init_Config_s *comm_config);

/**
 * @brief ͨ��FDCANComm��������
 *
 * @param instance FDCANCommʵ��
 * @param data ע��˵�ַ����Ч���ݳ�����Ҫ�ͳ�ʼ��ʱ�����datalen��ͬ
 */
void FDCANCommSend(FDCANCommInstance *instance, uint8_t *data);

/**
 * @brief ��ȡCANComm���յ�����,��Ҫ�Լ�ʹ��ǿ������ת�������ص�voidָ��ת����ָ������
 *
 * @return void* ���ص�����ָ��
 * @attention ע�����ϣ��ֱ��ͨ��ת��ָ���������,���������union��struct,Ҫ����Ƿ�ʹ����pack(n)
 *            CANComm���յ������ݿ��Կ�����pack(1)֮���,��������ŵ�.
 *            ���ʹ����pack(n)���ܻᵼ�����ݴ���,�����޷�ʹ��ǿ������ת��ͨ��memcpyֱ�ӷ���,ת����Ҫ�ֶ����.
 *            ǿ�ҽ���ͨ��CANComm���������ʹ��pack(1)
 */
void *FDCANCommGet(FDCANCommInstance *instance);

/**
 * @brief ���CANComm�Ƿ�����
 *
 * @param instance
 * @return uint8_t
 */
uint8_t FDCANCommIsOnline(FDCANCommInstance *instance);

#endif // __CAN_DRIVER_H__

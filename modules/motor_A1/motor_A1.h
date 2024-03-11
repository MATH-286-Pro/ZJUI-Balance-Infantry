#ifndef __MOTOR_A1_H__
#define __MOTOR_A1_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
void Chassis_UART_TX(void);
/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */
//typedef int16_t q15_t;
//static int hex_len_send = 34;
//static int hex_len_rc = 78;
//static int correct=0;

//typedef enum {
//    A1,      // 4.8M baudrate, K_W x1024
//    B1          // 6.0M baudrate, K_W x512
//} MotorType;

//// �����õ����������ݽṹ
//typedef union{
//        int32_t           L;
//        uint8_t       u8[4];
//       uint16_t      u16[2];
//       uint32_t         u32;
//          float           F;
//}COMData32;

//typedef struct {
//	// ���� ���ݰ�ͷ
//    unsigned char  start[2];     // ��ͷ
//	unsigned char  motorID;      // ���ID  0,1,2,3 ...   0xBB ��ʾ�����е���㲥����ʱ�޷��أ�
//	unsigned char  reserved;
//} COMHead;

//typedef struct { 
//	
//	   uint8_t  fan_d;       // �ؽ��ϵ�ɢ�ȷ���ת��
//	   uint8_t  Fmusic;      // �������Ƶ��   /64*1000   15.625f Ƶ�ʷֶ�
//	   uint8_t  Hmusic;      // �������ǿ��   �Ƽ�ֵ4  ����ǿ�� 0.1 �ֶ�  
//	   uint8_t  reserved4;
//	
//	   uint8_t  FRGB[4];     // ���LED 
//	
//}LowHzMotorCmd;

//typedef struct {  // �� 4���ֽ�һ������ ����Ȼ�����������
//	// ���� ����
//    uint8_t  mode;        // �ؽ�ģʽѡ��
//    uint8_t  ModifyBit;   // ������Ʋ����޸�λ
//    uint8_t  ReadBit;     // ������Ʋ�������λ
//    uint8_t  reserved;

//    COMData32  Modify;     // ��������޸� ������ 
//    //ʵ�ʸ�FOC��ָ������Ϊ��
//    //K_P*delta_Pos + K_W*delta_W + T
//    q15_t     T;      // �����ؽڵ�������أ�������������أ�x256, 7 + 8 ����
//    q15_t     W;      // �����ؽ��ٶ� ������������ٶȣ� x128,       8 + 7����	
//    int32_t   Pos;      // �����ؽ�λ�� x 16384/6.2832, 14λ������������0������������ؽڻ����Ա�����0��Ϊ׼��

//    q15_t    K_P;      // �ؽڸն�ϵ�� x2048  4+11 ����
//    q15_t    K_W;      // �ؽ��ٶ�ϵ�� x1024  5+10 ����

//    uint8_t LowHzMotorCmdIndex;     // �����Ƶ�ʿ������������, 0-7, �ֱ����LowHzMotorCmd�е�8���ֽ�
//    uint8_t LowHzMotorCmdByte;      // �����Ƶ�ʿ���������ֽ�
//	
//     COMData32  Res[1];    // ͨѶ �����ֽ�  ����ʵ�ֱ��һЩͨѶ����
//	
//}MasterComdV3;   // �������ݰ��İ�ͷ ��CRC 34�ֽ�

//typedef struct {
//	// ���� ��������������ݰ�	
//    COMHead head;    
//    MasterComdV3 Mdata;
//    COMData32 CRCdata;
//}MasterComdDataV3;//��������

//typedef struct {  // �� 4���ֽ�һ������ ����Ȼ�����������
//    // ���� ����
//    uint8_t  mode;        // ��ǰ�ؽ�ģʽ
//    uint8_t  ReadBit;     // ������Ʋ����޸�     �Ƿ�ɹ�λ
//    int8_t  Temp;        // �����ǰƽ���¶�   
//    uint8_t  MError;      // ������� ��ʶ
// 
//    COMData32  Read;     // ��ȡ�ĵ�ǰ ��� �Ŀ������� 
//    int16_t     T;      // ��ǰʵ�ʵ���������       7 + 8 ����

//    int16_t     W;      // ��ǰʵ�ʵ���ٶȣ����٣�   8 + 7 ����
//    float      LW;      // ��ǰʵ�ʵ���ٶȣ����٣�   

//    int16_t     W2;      // ��ǰʵ�ʹؽ��ٶȣ����٣�   8 + 7 ����
//    float      LW2;      // ��ǰʵ�ʹؽ��ٶȣ����٣�   

//    int16_t    Acc;           // ���ת�Ӽ��ٶ�       15+0 ����  ������С
//    int16_t    OutAcc;        // �������ٶ�         12+3 ����  �����ϴ�
//		 
//    int32_t   Pos;      // ��ǰ���λ�ã�����0������������ؽڻ����Ա�����0��Ϊ׼��
//    int32_t   Pos2;     // �ؽڱ�����λ��(���������)

//    int16_t     gyro[3];  // ���������6�ᴫ��������
//    int16_t     acc[3];   

//    // ��������������   
//    int16_t     Fgyro[3];  //  
//    int16_t     Facc[3];
//    int16_t     Fmag[3];
//    uint8_t     Ftemp;     // 8λ��ʾ���¶�  7λ��-28~100�ȣ�  1λ0.5�ȷֱ���
//    
//    int16_t     Force16;   // ����������16λ����
//    int8_t      Force8;    // ����������8λ����
//		
//    uint8_t     FError;    //  ��˴����������ʶ
//		
//    int8_t      Res[1];    // ͨѶ �����ֽ�
//	
//}ServoComdV3;  // �������ݰ��İ�ͷ ��CRC 78�ֽڣ�4+70+4��

//typedef struct {
//    // ���� ��������������ݰ�	
//    COMHead        head;
//    ServoComdV3      Mdata;

//    COMData32    CRCdata;

//}ServoComdDataV3;	//��������


//struct MOTOR_send{
//	// ���� ���͸�ʽ������
//    MasterComdDataV3  motor_send_data;  //����������ݽṹ�壬���motor_msg.h
//    MotorType A1;
//	  //int hex_len = 34;                    //���͵�16�����������鳤��, 34
//    // long long send_time;            //���͸������ʱ��, ΢��(us)
//    // �����͵ĸ�������
//    unsigned short id;              //���ID��0xBB����ȫ�����
//    unsigned short mode;            //0:����, 5:����ת��, 10:�ջ�FOC����
//    //ʵ�ʸ�FOC��ָ������Ϊ��
//    //K_P*delta_Pos + K_W*delta_W + T
//    float T;                        //�����ؽڵ�������أ�������������أ���Nm��
//    float W;                        //�����ؽ��ٶȣ�����������ٶȣ�(rad/s)
//    float Pos;                      //�����ؽ�λ�ã�rad��
//    float K_P;                      //�ؽڸն�ϵ��
//    float K_W;                      //�ؽ��ٶ�ϵ��
//    COMData32 Res;                  // ͨѶ �����ֽ�  ����ʵ�ֱ��һЩͨѶ����
//};

//struct MOTOR_recv{
//    // ���� ��������
//    ServoComdDataV3 motor_recv_data;     //����������ݽṹ�壬���motor_msg.h
//    MotorType A1;
//    //int hex_len;                    //���յ�16�����������鳤��, 78
//	  //hex_len = 78;
//    // long long resv_time;            //���ո������ʱ��, ΢��(us)
//    //int correct;      // 0 ��ʾ false������ֵ��ʾ true                   //���������Ƿ�������true������false��������
//	  //correct = 0; 
//    //����ó��ĵ������
//    unsigned char motor_id;         //���ID
//    unsigned char mode;             //0:����, 5:����ת��, 10:�ջ�FOC����
//    int Temp;                       //�¶�
//    unsigned char MError;           //������

//    float T;                        // ��ǰʵ�ʵ���������
//    float W;                        // ��ǰʵ�ʵ���ٶȣ����٣�
//    float LW;                       // ��ǰʵ�ʵ���ٶȣ����٣�
//    int Acc;                      // ���ת�Ӽ��ٶ�
//    float Pos;                      // ��ǰ���λ�ã�����0������������ؽڻ����Ա�����0��Ϊ׼��

//    float gyro[3];                  // ���������6�ᴫ��������
//    float acc[3];
//};

typedef struct{
		uint8_t start[2];
		uint8_t Motor_ID;
		uint8_t reserved_a;
		uint8_t mode;
		uint8_t ModifyBit;
		uint8_t ReadBit;
		uint8_t reserved_b;
		uint32_t Modify;
		uint16_t T;
		uint16_t W;
		uint32_t Pos;
		uint16_t kp;
		uint16_t kw;
}Send_Data;

typedef struct{
		uint8_t mode;
		uint8_t Temp;
		uint16_t T;
		uint16_t W;
		uint16_t Acc;
		uint32_t Pos;
}Rx_Data;

typedef struct{
		uint8_t Mode;
		uint8_t Temp;
		float Torque;
		float Omega;
		uint16_t Acc;
		float Position;
}Motor_State;

uint32_t crc32_core(uint32_t *ptr, uint32_t len);
void Control_Message_Send(int ID);
void Mode_Control(int ID,int Mode);
void A1_Motor_Multiple_Control(int ID,int mode,float Torque,float W,float Position);
void A1_Motor_Position_Control(int ID,float Position);
void Received_Data_Dealer(const uint8_t *sbus_buf);
void A1_Motor_Speed_Control(int ID,float W);
/* USER CODE END Private defines */



/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */

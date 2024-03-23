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
//    A1,         // 4.8M baudrate, K_W x1024 // 这个x1024是什么东西
//    B1          // 6.0M baudrate, K_W x512
//} MotorType;

//// 发送用单个数据数据结构
//typedef union{
//        int32_t           L;
//        uint8_t       u8[4];
//       uint16_t      u16[2];
//       uint32_t         u32;
//          float           F;
//}COMData32;

//typedef struct {
//	// 定义 数据包头
//  unsigned char  start[2];     // 包头
//	unsigned char  motorID;      // 电机ID  0,1,2,3 ...   0xBB 表示向所有电机广播（此时无返回）
//	unsigned char  reserved;
//} COMHead;

//typedef struct { 
//	
//	   uint8_t  fan_d;       // 关节上的散热风扇转速
//	   uint8_t  Fmusic;      // 电机发声频率   /64*1000   15.625f 频率分度
//	   uint8_t  Hmusic;      // 电机发声强度   推荐值4  声音强度 0.1 分度  
//	   uint8_t  reserved4;
//	
//	   uint8_t  FRGB[4];     // 足端LED 
//	
//}LowHzMotorCmd;

//typedef struct {  // 以 4个字节一组排列 ，不然编译器会凑整
//	// 定义 数据
//    uint8_t  mode;        // 关节模式选择
//    uint8_t  ModifyBit;   // 电机控制参数修改位
//    uint8_t  ReadBit;     // 电机控制参数发送位
//    uint8_t  reserved;

//    COMData32  Modify;     // 电机参数修改 的数据 
//    //实际给FOC的指令力矩为：
//    //K_P*delta_Pos + K_W*delta_W + T
//    q15_t     T;      // 期望关节的输出力矩（电机本身的力矩）x256, 7 + 8 描述
//    q15_t     W;      // 期望关节速度 （电机本身的速度） x128,       8 + 7描述	
//    int32_t   Pos;      // 期望关节位置 x 16384/6.2832, 14位编码器（主控0点修正，电机关节还是以编码器0点为准）

//    q15_t    K_P;      // 关节刚度系数 x2048  4+11 描述
//    q15_t    K_W;      // 关节速度系数 x1024  5+10 描述

//    uint8_t LowHzMotorCmdIndex;     // 电机低频率控制命令的索引, 0-7, 分别代表LowHzMotorCmd中的8个字节
//    uint8_t LowHzMotorCmdByte;      // 电机低频率控制命令的字节
//	
//     COMData32  Res[1];    // 通讯 保留字节  用于实现别的一些通讯内容
//	
//}MasterComdV3;   // 加上数据包的包头 和CRC 34字节

//typedef struct {
//	// 定义 电机控制命令数据包	
//    COMHead head;    
//    MasterComdV3 Mdata;
//    COMData32 CRCdata;
//}MasterComdDataV3;//返回数据

//typedef struct {  // 以 4个字节一组排列 ，不然编译器会凑整
//    // 定义 数据
//    uint8_t  mode;        // 当前关节模式
//    uint8_t  ReadBit;     // 电机控制参数修改     是否成功位
//    int8_t  Temp;        // 电机当前平均温度   
//    uint8_t  MError;      // 电机错误 标识
// 
//    COMData32  Read;     // 读取的当前 电机 的控制数据 
//    int16_t     T;      // 当前实际电机输出力矩       7 + 8 描述

//    int16_t     W;      // 当前实际电机速度（高速）   8 + 7 描述
//    float      LW;      // 当前实际电机速度（低速）   

//    int16_t     W2;      // 当前实际关节速度（高速）   8 + 7 描述
//    float      LW2;      // 当前实际关节速度（低速）   

//    int16_t    Acc;           // 电机转子加速度       15+0 描述  惯量较小
//    int16_t    OutAcc;        // 输出轴加速度         12+3 描述  惯量较大
//		 
//    int32_t   Pos;      // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
//    int32_t   Pos2;     // 关节编码器位置(输出编码器)

//    int16_t     gyro[3];  // 电机驱动板6轴传感器数据
//    int16_t     acc[3];   

//    // 力传感器的数据   
//    int16_t     Fgyro[3];  //  
//    int16_t     Facc[3];
//    int16_t     Fmag[3];
//    uint8_t     Ftemp;     // 8位表示的温度  7位（-28~100度）  1位0.5度分辨率
//    
//    int16_t     Force16;   // 力传感器高16位数据
//    int8_t      Force8;    // 力传感器低8位数据
//		
//    uint8_t     FError;    //  足端传感器错误标识
//		
//    int8_t      Res[1];    // 通讯 保留字节
//	
//}ServoComdV3;  // 加上数据包的包头 和CRC 78字节（4+70+4）

//typedef struct {
//    // 定义 电机控制命令数据包	
//    COMHead        head;
//    ServoComdV3      Mdata;

//    COMData32    CRCdata;

//}ServoComdDataV3;	//发送数据


//struct MOTOR_send{
//	// 定义 发送格式化数据
//    MasterComdDataV3  motor_send_data;  //电机控制数据结构体，详见motor_msg.h
//    MotorType A1;
//	  //int hex_len = 34;                //发送的16进制命令数组长度, 34
//    // long long send_time;            //发送该命令的时间, 微秒(us)
//    // 待发送的各项数据
//    unsigned short id;              //电机ID，0xBB代表全部电机
//    unsigned short mode;            //0:空闲, 5:开环转动, 10:闭环FOC控制
//    //实际给FOC的指令力矩为：
//    //K_P*delta_Pos + K_W*delta_W + T
//    float T;                        //期望关节的输出力矩（电机本身的力矩）（Nm）
//    float W;                        //期望关节速度（电机本身的速度）(rad/s)
//    float Pos;                      //期望关节位置（rad）
//    float K_P;                      //关节刚度系数
//    float K_W;                      //关节速度系数
//    COMData32 Res;                  // 通讯 保留字节  用于实现别的一些通讯内容
//};

//struct MOTOR_recv{
//    // 定义 接收数据
//    ServoComdDataV3 motor_recv_data;   //电机接收数据结构体，详见motor_msg.h
//    MotorType A1;
//    //int hex_len;                     //接收的16进制命令数组长度, 78
//	  //hex_len = 78;
//    // long long resv_time;            //接收该命令的时间, 微秒(us)
//    //int correct;      // 0 表示 false，非零值表示 true                   //接收数据是否完整（true完整，false不完整）
//	  //correct = 0; 
//    //解读得出的电机数据
//    unsigned char motor_id;         //电机ID
//    unsigned char mode;             //0:空闲, 5:开环转动, 10:闭环FOC控制
//    int Temp;                       //温度
//    unsigned char MError;           //错误码

//    float T;                        // 当前实际电机输出力矩
//    float W;                        // 当前实际电机速度（高速）
//    float LW;                       // 当前实际电机速度（低速）
//    int Acc;                        // 电机转子加速度
//    float Pos;                      // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）

//    float gyro[3];                  // 电机驱动板6轴传感器数据
//    float acc[3];
//};


// 该结构体定义与 A1电机使用手册一致
typedef struct{
		uint8_t start[2];   // 包头1 固定为0xFE 包头2 固定为0xEE
		uint8_t Motor_ID;   // 电机ID，可以为0,1,2,0xBB，0xBB表示向所有电机广播
		uint8_t reserved;   // 可忽略

		uint8_t mode;		// 电机运行模式，可为0(停转),5(开环),10(闭环),11(修改ID模式，不建议在这里开启)
		uint8_t ModifyBit;  // 可忽略
		uint8_t ReadBit;    // 可忽略
		uint8_t reserved_b; // 可忽略
		uint32_t Modify;    // 可忽略
		int16_t T;          // 电机前馈力矩，×256倍描述         |T|<128      (乘倍率前) 单位：Nm
		int16_t W;          // 电机速度命令，×128倍描述         |W|<256      (乘倍率前) 单位：rad/s
		int32_t Pos;        // 电机位置命令，×16384/pi倍描述    |Pos|<823549 (乘倍率前) 单位：rad
		int16_t kp;         // 电机位置刚度，×2048倍描述         0<K_P<16    (乘倍率前)
		int16_t kw;         // 电机速度刚度，×1024倍描述         0<K_W<32    (乘倍率前)
}Send_Data;


typedef struct{
		uint8_t mode;
		uint8_t Temp;
		uint16_t T;
		uint16_t W;    //这里原先为uint16_t
		uint16_t Acc;
		uint32_t Pos;
}Recv_Data;




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
void A1_Motor_Torque_Control(int ID,float Torque);
void A1_Motor_0Torque_Control(int ID);
void Received_Data_Dealer(const uint8_t *sbus_buf);
void A1_Motor_Speed_Control(int ID,float W);
/* USER CODE END Private defines */



/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */

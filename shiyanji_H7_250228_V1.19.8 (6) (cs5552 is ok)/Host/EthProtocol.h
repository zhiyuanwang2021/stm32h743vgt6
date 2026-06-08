#ifndef  _ETHPROTOCOL_H_
#define  _ETHPROTOCOL_H_

#include "stm32h7xx_hal.h"
#include "ETHw5500.h"
#include "CRC16.h"
#include "myFifo.h"


//UTC功能码
/********************上位机发给下位机**********************************/
#define LINK_ACK										0x00//对0xA0功能码应答，用于判断网络状态
#define DOPE_OPEN_CLOSE_LINK							0x01//建立或断开连接
#define DOPEON_OFF 										0x02//激活/停用控制器
#define DOPESETCTRL 									0x03//使能/不使能闭环控制
#define DOPETRANSMITDATA								0x04//激活/停用测量数据传输				

#define	DOPERDNOMINALACCSPEED							0x07//读取位置生成器标称值
#define DOPESETNOMINALACCSPEED							0x08//设置位置生成器标称值
#define DOPERDCTRLPARAMETER								0x09//读取所有闭环控制参数
#define	DOPEDEADBANDCTRL								0x0A//设定误差死区控制参数
#define	DOPEDEFAULTACC									0x0B//设置默认加速度
#define DOPESPEEDLIMIT									0x0C//设定最大限定速度
#define	DOPESETDATATRANSMISSIONRATE						0x0D//设定数据发送周期  单位1ms
#define	DOPEINTGR										0x0E//设定传感器滤波时间
#define	DOPEDESTWND										0x0F//设定目标误差/时间窗口
#define DOPESFT											0x10//设定软限位

#define DOPEPOSPID										0x11//设置位置闭环控制参数
#define	DOPERDPOSPID									0x12//读取位置闭环控制参数
#define	DOPEWRPOSPID									0x13//写入定位置闭环控制参数

#define DOPESPEEDPID									0x14//设置速度闭环控制参数
#define	DOPERDSPEEDPID									0x15//读取速度闭环控制参数
#define	DOPEWRSPEEDPID									0x16//写入速度闭环控制参数

#define DOPEFEEDFORWARD									0x17//设置速度前馈参数
#define DOPERDFEEDFORWARD								0x18//读取速度前馈参数
#define DOPEWRFEEDFORWARD								0x19//写入速度前馈参数

#define DOPESETSENSORCORRECTION							0x1A//设定传感器校正表
#define RDSENSORDATA									0x1B//读取传感器数据
#define WRSENSORDATA									0x1C//写入传感器数据
#define RDSENSORBIGDEFORMATIONDATA						0x1D//读取大变形传感器数据
#define WRSENSORBIGDEFORMATIONDATA						0x1E//读取大变形传感器数据

#define RDOUTPUTPARA									0x20//读取输出参数
#define WROUTPUTPARA									0x21//写入输出参数
#define	RDETHPARA 										0x22//读以太网参数
#define	WRETHPARA 										0x23//写入以太网参数	下位机收到后重启
#define RDSYSPARA 										0x24//读配置参数
#define WRSYSPARA 										0x25//写入配置参数
#define DOPEWRDVERSION 									0x26//读版本信息
#define SETTARE											0x27//测量通道的数值清零
#define WRSERVOPARA										0x28//写入伺服参数
#define RDSERVOPARA										0x29//读伺服参数

#define DOPESETOPENLOOPCOMMAND  						0x30 //开环测试指令
#define DOPEMOVE										0x31 //以默认加速度上下移动
#define DOPEMOVE_A										0x32 //以指定加速度上下移动
#define DOPEPOS											0x33 //以默认加速度移动至指定位置
#define DOPEPOS_A										0x34 //以指定加/减速度移动至指定位置
/*	
在指定的控制模式下，以默认加速度移动至指定位置，若达到限位位置，则以默认减速度停止；
若即将达到指定位置，则根据参数“DestinationMode”确定到达指定位置的控制方式以及到达指定位置后的操作
*/
#define DOPEPOSEXT										0x35 
/*
同DoPEPosExt，但在加减速过程中，需要指定加速度，限位减速度，靠近目标位置后的减速度
*/
#define DOPEPOSEXT_A									0x36

#define DOPEHALT										0x40	//以默认减速度从现Command速度减速至0
#define DOPEHALT_A										0x41	//以默认减速度从现Command速度减速至0
#define DOPESHALT										0x42	//以标称减速度从当前位置减速至0，没有MoveCtrl选择，只有位置模式

#define DOPETRIG                                        0x47    //以指定速度移动到限制位置,如果到达触发条件位置，将发送一条消息（如果在组合移动序列中使用，则激活下一个命令）
#define DOPETRIG_A                                      0x48    //同DoPETrig，但在加减速过程中，需要指定加速度，限位减速度
#define DOPEBLOCKHEADER                                 0x4E    //所有简单命令组合移动的指令序列头
#define DOPEBLOCKEXECUTE                                0x4F    //所有简单命令组合移动的执行命令(开始/结束)
		
#define	DOPECYCLE										0x50	//用斜坡函数执行周期运动

#define DOPEDYNCYCLE									0x60	//动态循环指令
#define	DOPESETPEAKCTRL									0x61	//修改正在运行的动态循环指令的峰值控制
/********************下位机发给上位机**********************************/
#define DOPEOPENDEVICEID								0x80	//对功能码0x01应答，默认成功0x01
#define UTCON_OFF										0x81	//对功能码0x02应答
#define	UTCSETCTRL										0x82	//对功能码0x03应答

#define	UTCRDNOMINALACCSPEED							0x83	//对功能码0x07应答
#define	UTCRDCTRLPARAMETER								0x84	//对功能码0x09应答
#define	UTCRDPOSPID										0x85	//对功能码0x12应答
#define	UTCRDSPEEDPID									0x86	//对功能码0x15应答
#define	UTCRDFEEDFORWARD								0x87	//对功能码0x18应答
#define	UTCRDSENSORDATA									0x88	//对功能码0x1B应答
#define	UTCRDSENSORBIGDEFORMATIONDATA					0x89	//对功能码0x1D应答
#define	UTCRDOUTPUTPARA									0x8A	//对功能码0x20应答
#define	UTCRDETHPARA									0x8B	//对功能码0x22应答
#define	UTCRDSYSPARA									0x8C	//对功能码0x24/0x25应答
#define UTCWRDVERSION									0x8D	//对功能码0x26应答
#define UTCRDSERVOPARA									0x8E	//对功能码0x29应答

#define UTC_SET_ON_POS_MSG                              0x90    //移动指令完成后,下位机发送通知PC
#define UTC_SET_ON_TPOS_MSG                             0x91    //DoPETrig指令到达触发条件
#define UTC_SET_ON_LPOS_MSG                             0x92    //当DoPEPosExt达到限制值会触发这个消息
#define UTC_SET_ON_SFT_MSG                              0x93    //软件限位报警
#define UTC_SET_ON_RUNTIME_MSG                          0x94    //运行错误消息
#define UTC_SET_ON_SYSTEM_MSG                           0x95    //系统消息
#define UTC_SET_ON_DEBUG_MSG                            0x96    //调试消息

#define	DOPECURRENTDATA									0xA0	//上报各输入接口状态
#define LPUSTAN_REPORT                                  0xA1    //send lpusTAN to PC,when related command complete.


#define	SYSTEM_MSG_TEXT_LEN                             256     //系统消息的最大长度
#define DEBUG_MSG_TEXT_LEN                             256     //调试消息的最大长度
typedef struct
{
    uint16_t DoPError;      //表示事件发生时，其他的错误。
    uint8_t  Reached;       //=1时，表示在目标窗口内，=0时，表示在目标窗口外。
    float    Time;          //到达目的位置的下位机系统时间
    uint8_t  Control;       //控制模式(指的是下面Position参数的)
    float    Position;      //如果Reached=1 位置为目标设定值位置;如果Reached=0 位置为当前位置
    uint8_t  DControl;      //控制模式(指的是下面Destination参数的)
    float    Destination;   //移动指令的目标设定值
    uint16_t usTAN;         //该条移动指令的传输编号

    float    timeoutAllowed;//允许超时时间
}utcSetOnPosMsg_t;

extern utcSetOnPosMsg_t utcSetOnPosMsg;

typedef struct
{
    uint16_t DoPError;      //表示事件发生时，其他的错误。
    uint8_t  Reached;       //永远是1
    float    Time;          //到达触发条件时的下位机系统时间
    uint8_t  Control;       //控制模式(指的是下面Position参数的)
    float    Position;      //DoPETrig指令的Limit设定值
    uint8_t  DControl;      //控制模式(指的是下面Destination参数的)
    float    Destination;   //DoPETrig指令的触发条件位置Trigger Position 
    uint16_t usTAN;         //该条移动指令的传输编号
}utcSetOnTPosMsg_t;

extern utcSetOnTPosMsg_t utcSetOnTPosMsg;

typedef struct
{
    uint16_t DoPError;      //表示事件发生时，其他的错误。
    uint8_t  Reached;       //永远是1
    float    Time;          //到达触发条件时的下位机系统时间
    uint8_t  Control;       //控制模式(指的是下面Position参数的)
    float    Position;      //DoPEPosExt指令的Limit设定值
    uint8_t  DControl;      //控制模式(指的是下面Destination参数的)
    float    Destination;   //DoPEPosExt指令的Destination目标位置
    uint16_t usTAN;         //该条移动指令的传输编号
}utcSetOnLPosMsg_t;

extern utcSetOnLPosMsg_t utcSetOnLPosMsg;

typedef struct
{
    uint16_t DoPError;      //表示事件发生时，其他的错误。
    uint8_t  Upper;         //=1 上软限位触发，=0 下限位触发
    float    Time;          //软限位触发时的下位机系统时间
    uint8_t  Control;       //控制模式(指的是下面Position参数的)
    float    Position;      //软限位触发的位置
    uint16_t usTAN;         //该条移动指令的传输编号
}utcSetOnSftMsg_t;

extern utcSetOnSftMsg_t utcSetOnSftMsg;

typedef struct
{
    uint16_t DoPError;      //表示事件发生时，其他的错误。
    uint8_t  ErrorNumber;   //错误代码
    float    Time;          //软限位触发时的下位机系统时间
    uint8_t  Device;        //负责该错误的设备编号，帮助识别哪个设备发生了错误。
    uint8_t  Bits;          //设备状态位，Bits 提供关于设备内部哪个部分或功能出现问题的详细信息。
    uint16_t usTAN;         //该条移动指令的传输编号
}utcSetOnRuntimeError_t;

extern utcSetOnRuntimeError_t utcSetOnRuntimeError;

typedef struct
{
    uint16_t DoPError;      //表示事件发生时，其他的错误。
    uint8_t  MsgNumber;     //系统消息编号
    float    Time;          //发送消息时的下位机系统时间
    char     Text[SYSTEM_MSG_TEXT_LEN];//消息文本，SYSTEM_MSG_TEXT_LEN暂定设置为256
}utcSetOnSystemMsg_t;

extern utcSetOnSystemMsg_t utcSetOnSystemMsg;

typedef struct
{
    uint16_t DoPError;      //表示事件发生时，其他的错误。
    uint8_t  MsgType;       //调试消息的类型
    float    Time;          //发送消息时的下位机系统时间
    char     Text[DEBUG_MSG_TEXT_LEN];//消息文本，DEBUG_MSG_TEXT_LEN暂定设置为256
}utcSetOnDebugMsg_t;

extern utcSetOnDebugMsg_t utcSetOnDebugMsg;

uint8_t UTC_sendbuf_process(uint8_t* Data,UTC_ETH* EthStruct);
uint8_t UTC_revbuf_process(UTC_ETH* EthStruct);

void utc_rev_fifo(void);

void utc_send_fifo(void);

void utcSetOnSystemMsgUpdate(char* _msgContent,utcSetOnSystemMsg_t* _utcSetOnSystemMsg);

#endif


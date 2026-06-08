#ifndef  _GPARAMETER_H_
#define  _GPARAMETER_H_  
#include "stdint.h"
#include "posGenerator.h"


#define CDTADTRANS_NUM 200
//55
#ifndef PHY_LINK_OFF 
		#define PHY_LINK_OFF 0
#endif
#ifndef PHY_LINK_ON 
		#define PHY_LINK_ON 1
#endif
#ifndef PHY_LINK_UNKNOW
		#define PHY_LINK_UNKNOW 2
#endif

#define	Device_Open  1
#define Device_Off 0

#define DoPE_On  1
#define DoPE_Off 0
#define CtrlTRUE    1
#define CtrlFALSE	0

#define TRUE  0x01
#define FALSE 0x00

#define DIRECTION_UP 0x01
#define DIRECTION_DOWN 0x00

#define ETH_HEARTBEAT_GOT 0x00
#define ETH_HEARTBEAT_LOST 0x01
#define ETH_HEARTBEAT_THRESHOLD 200 //1000ms

#define SERVO_ENCODER_GAIN 4.0f

enum WM_ENUM{
	wmInit = 1,//init
	wmParaConfig = 1<<1,//paramters configration
	wmMoveReady = 1<<2,//ready to execute movement command
	wmCalibrate = 1<<3,//senser calibration mode
	wmLoopctrl = 1<<4,//executing close/open loop control 
	wmEmergency = 1<<5,//emergency trigger
	wmMoveRQ = 1<<6,//move required
};

typedef struct{
	uint8_t now;
	uint8_t last;
}WORK_MODE_STRUCT;

extern WORK_MODE_STRUCT wm;

enum initS_ENUM{
	initScomplete = 1,
	initSexecuting = 1<<1,
	initSfailure = 1<<2,
	initSreinit = 1<<3
};

enum paraConfigS_ENUM{
	paraConfigScomplete = 1,
	paraConfigSexecuting = 1<<1,
	paraConfigSfailure = 1<<2,
	paraConfigSreceive = 1<<3
};

enum moveReadyS_ENUM{
	moveReadySok = 1,
};

enum loopCtrlS_ENUM{
	loopCtrlScomplete = 1,
	//executing
	loopCtrlSdown = 1<<1,
	loopCtrlSup = 1<<2,
	loopCtrlSmove = 1<<3,
	loopCtrlScycleActive = 1<<4,
	loopCtrlShighPressure = 1<<5,
	//failure
	loopCtrlSinitE = 1<<6,
	loopCtrlSEActive = 1<<7,
	loopCtrlShalt = 1<<8,
	loopCtrlSdeviation = 1<<9,
	//receive
	loopCtrlSreceive = 1<<10,
};

enum calibrateS_ENUM{
	calibrateScomplete = 1,
	//executing
	calibrateSmove = 1<<1,
	calibrateSreadPara = 1<<2,
	calibrateSwritePara = 1<<3,
	//failure
	calibrateSinitE = 1<<4,
	calibrateSEActive = 1<<5,
	calibrateShalt = 1<<6,
	//receive
	calibrateSreceive = 1<<7,
};

enum emergencyS_ENUM{
	emergencyScomplete = 1,
	emergencySexecuting = 1<<1,
	emergencyDeviceOffMoving = 1<<2,
	emergencyDopeOffMoving = 1<<3,
	emergencyCtrlFalseMoving = 1<<4,
	emergencyPhyOff = 1<<5,
	emergencySocketUnestablished = 1<<6,
	emergencyHeartBeatLost = 1<<7,
	emergencyLimitUpper = 1<<8,
	emergencyLimitDown = 1<<9,
	emergencySftUpper = 1<<10,//software limit
	emergencySftDown = 1<<11,//software limit
};

typedef enum{
	//HW means hardware
	limitUpperHWpos = 1,
	limitDownHWpos = 1<<1,
	//SR means sensor range
	limitUpperSRload = 1<<2,
	limitDownSRload = 1<<3,
	limitUpperSRext1 = 1<<4,
	limitDownSRext1 = 1<<5,
	limitUpperSRext2 = 1<<6,
	limitDownSRext2 = 1<<7,
	//SW means software
	limitUpperSWpos = 1<<8,
	limitDownSWpos = 1<<9,
	limitUpperSWload = 1<<10,
	limitDownSWload = 1<<11,
	limitUpperSWext1 = 1<<12,
	limitDownSWext1 = 1<<13,
	limitUpperSWext2 = 1<<14,
	limitDownSWext2 = 1<<15,
}limitType_e;

enum moveRQS_ENUM{
	moveRQSenable = 1,
};

typedef enum{
	noTrigger = 0,
	successTrigger = 1,
	limitArrive = 1<<1,
}trigState_e;

typedef enum{
	SENPS_noProtect = 0,//SENPS means sensor protect state
	SENPS_protectUpperTrig = 1,//SENPS means sensor protect state
	SENPS_protectDownTrig = 2,//SENPS means sensor protect state
}sensorProtectState_e;//SENPS means sensor protect state

typedef enum{
	LOTPS_noProtect = 0,//LOTPS means load protect state
	LOTPS_protectUpperTrig = 1,//LOTPS means load protect state
	LOTPS_protectDownTrig = 2,//LOTPS means load protect state
}loadProtectState_e;//LOTPS means extensometer protect state

typedef struct{
	uint8_t init;
	uint8_t paraConfig;
	uint8_t moveReady;
	uint8_t calibrate;
	uint8_t moveRQ;
	uint16_t emergency;
	uint16_t limitTypeWord;
	uint16_t loopCtrl;
}WORK_STATE_STRUCT;

extern WORK_STATE_STRUCT ws;

typedef enum{
	reachNone = 0,
	reachApproach = 1,
	reachPosition = 2,
	reachMaintain = 3,
}posExtDestReach_e;

typedef struct{
	uint8_t eth_phy_state;
	uint8_t eth_scoket_state;
	uint8_t PC_ack_state;//?????ж?UTC??PC???紫????
	uint8_t os_start;//??????????????
	uint8_t sys_init_ok;//????????????
	uint8_t OpenDevice_state;//??????????????
	uint8_t DoPE_state;//DoPEOn/DoPEOff  ????/????????
	uint8_t DoPESetCtrl;//???/???????????  TRUE???  FALSE?????
	uint8_t DoPETransmitData;
	uint8_t UpperLimits;
	uint8_t LowerLimits;
	uint8_t DynCycle_OffsetReach;
	uint8_t posExtDestReach;
	uint8_t openloop;		//open loop
	uint8_t lostEthHeartbeat;
}StateFlag;
extern StateFlag stateFlag;
extern StateFlag stateFlagLast;


#define POS_MODE 				0 //POS MODE
#define LOAD_MODE				1 //LOAD MODE
#define EXTEN_MODE				2 //EXTEN MODE
extern uint16_t lpusTAN;//communicate TAN

typedef struct{
	uint16_t lpusTAN;
	uint8_t  flag;
	uint8_t  send_flag;
}COMPLETE_STATE;

typedef struct{
	trigState_e flag;
}trigState_t;

#define EMERGENCY_DEVICE_OFF 		0x01//1  0B00000001
#define EMERGENCY_DOPE_OFF 			0x02//2	 0B00000010
#define EMERGENCY_CTRL_FALSE 		0x04//4  0B00000100
#define EMERGENCY_PHY_OFF 			0x08//8  0B00001000
#define EMERGENCY_SOCKET_UNESTAB	0x10//16 0B00010000
#define EMERGENCY_ETH_LOST_HEARTBEAT 0x20//16 0B00010000
#define EMERGENCY_HARDWARE_LIMIT_UPPER 	0x40//16 0B00100000
#define EMERGENCY_HARDWARE_LIMIT_LOWER 	0x80//16 0B00100000

typedef struct{
	uint8_t occur;
	uint8_t processing;
	uint8_t complete;
	uint8_t error;
	uint16_t counter;
}EMERGENCY_STATE;

typedef struct{

uint8_t scram_button;
uint8_t state;

}EMERGENCY_SRTUCT;

extern EMERGENCY_SRTUCT emerency_struct;


 /* Sensor classes */
 /* ---------------------------------- */
#define SEN_UNDEF 			0 	/* unknown sensor class */
#define SEN_ANALOGUE 		1 	/* analogue sensor */
#define SEN_INC 			2 	/* incremental sensor */
#define SEN_ABS 			3 	/* absolute value sensor */
 /* Analogue sensor types */
 /* ---------------------------------- */
#define SIG_STRAINGAUGE 	0 	/* Strain gauge */
#define SIG_LVDT 			1	/* LVDT */
#define SIG_DC 				2 	/* DC */
 /* Incremental sensor types */
 /* ---------------------------------- */
#define SIG_TTL 			0 	/* TTL Signal */
#define SIG_LINE 			1 	/* RS422 (line driver) */
#define SIG_SINE11uA 		2 	/* Sine 11??A */
#define SIG_SINE1V 			3 	/* Sine 1V */
 /* Absolute value sensor types */
 /* ---------------------------------- */
#define SIG_UNDEF 			0 	/* undefined */
#define SIG_TR_LT_S 		1 	/* TR LT-S Sensor */
/* Transducer types */
 /* ---------------------------------- */
#define TRANSDUCER_LINEAR 	0 	/* Linear transducer */
#define TRANSDUCER_ROTARY 	1 	/* Rotary transducer */
/* Reference mark types */
 /* ---------------------------------- */
#define REFMARK_NON 		0 	/* Transducer has no reference mark */
#define REFMARK_ONE 		1 	/* Transducer has one reference mark */
#define REFMARK_DISTCODE 	2 	/* Transducer has distance coded */

typedef struct
{
	uint16_t PartNo;			//?????????????
	uint8_t  Version;			//???????汾
	uint32_t SerNo;				//?????????к?
	uint16_t Class;				//??????????
	uint8_t  DatVersion;		//????汾
}SENSORHEADER;

extern SENSORHEADER SenHeader;

typedef struct
{
	uint8_t CalculatedSensor;	//????????????
	uint8_t SENSOR_CORR_MAX;	//??????У????????????  *** ???????????????鳤???
	uint8_t CorrNo;				//??Ч???????  ????????????鱻??????????
	float 	S1Correction[32];	//S1??У????????S2???????????У???
	uint32_t S2Value[32];		//S2???(??????????????)??ADC???
}SENSORCORRECT;

typedef struct
{
	uint8_t 	SensorNo;		//?????????
	uint32_t 	Intgr;			//????????????????
	SENSORCORRECT Correct;		//?????????ò???
}SENSORSETPARA;					

/* Analogue sensor EEPROM data */
#define SEN_LIN_DATA_MAX 32

typedef struct 
{ 
 float 		MaxExcitation; 		/* Maximum excitation voltage [V] ????????*/
 uint16_t 	MinImpedance; 		/* Impedance [Ohm] ??С?迹*/
 float 		NominalValue; 		/* Nominal value of the sensor [Unit] ??????????*/
 uint16_t 	Unit; 				/* Unit of sensor UNIT_xxx [No] ????????λ*/
 float 		Offset; 			/* Sensor offset [Unit] ???????????*/
 uint16_t 	NegLimit; 			/* Range limit - min. [%] ??? ??С? %*/
 uint16_t 	PosLimit; 			/* Range limit - max. [%] ??? ???? %*/
 float 		Reference; 			/* Nominal value of the reference [*] ?ο???????*/
 float 		CorrReference;		/* Corr. value of the reference [No]?ο????У???*/
 uint16_t 	Sensortype; 		/* Sensor type ??????????*/
 float 		NominalSensitive; 	/* Sensitivity at Nominal value [*] ?????????*/
 uint16_t   GaugeLength;		/* Gauge Length [*] ???????*/
 uint16_t 	Sign; 				/* Invert sign of channel [1/0] ??????????*/
 uint16_t 	Day; 				/* Date of last change [No] ??θ??????????*/
 uint16_t 	Month; 				/* Date of last change [No]	??θ??????????*/
 uint16_t 	Year; 				/* Date of last change [No]	??θ??????????*/
 uint16_t 	LinPoint; 			/* Number of linearization steps [No] ?????????*/
 struct LinVal2
 {
 float MeasValue; 				/* Measured value [Unit] ?????*/
 float RefValue; 				/* Reference [Unit] ?ο??*/
 } LinV2[SEN_LIN_DATA_MAX]; 		/* Linearization table ???????????????*/
}SENSORANALOGUEDATA;

extern SENSORANALOGUEDATA SenAData;
extern SENSORANALOGUEDATA SenAData2;

/* sensor EEPROM data including PosSensor, LoadSensor, Extensometer*/
#define SENSOR_CH_NUM 5 //??????????????
#define SEN_LIN_DATA_MAX 32
typedef enum{
	ch0Pose,//pose 位移
	ch1Bd,//bigderformation 大变形
	ch2Ext1,//ext1 引伸计1
	ch3Ext2,//ext2 引伸计2(扩展)
	ch4Load,//load 载荷
	ch5Ext1Code,//ext1Code 引伸计1原码值
	ch6Ext2Code,//ext1Code 引伸计1原码值
	ch7LoadCode,//ext1Code 引伸计1原码值
}sensorConnector_e;

typedef struct 
{ 
 uint8_t    Sensortype;			/* Sensor type [No] ??????????*/
 uint8_t 	Sign; 				/* Invert sign of channel [1/0] ??????????*/
 uint8_t 	Day; 				/* Date of last change [No] ??θ??????????*/
 uint8_t 	Month; 				/* Date of last change [No]	??θ??????????*/
 uint8_t 	LinPoint; 			/* Number of linearization steps [No] ??Ч????*/
 uint16_t 	Year; 				/* Date of last change [No]	??θ??????????*/
 uint32_t 	sensorIntgr;
 float 		NominalValue; 		/* Nominal value of the sensor [Unit] ??????????*/
 float 		NominalSensitive; 	/* Sensitivity at Nominal value [*] ?????????*/
 struct LinVal
 {
	float CorrectionFactor; 	/* Correction Factor У??????*/
	float RefValue; 			/* Reference ?ο??*/
	int32_t ADCCode;			/* ADC Code ????*/
 } LinV[SEN_LIN_DATA_MAX]; 		/* Linearization table ???????????????*/
}sensorData_t;

extern sensorConnector_e sensorConnector;
extern sensorData_t SenData[SENSOR_CH_NUM],senDataCommbuf[SENSOR_CH_NUM],SenDatatest;

typedef struct 
{ 
 float 		NominalValue; 		/* Nominal value of the sensor [Unit] ??????????*/
 float 		NominalSensitive1; 	/* Sensitivity at Nominal value [*] ?????????1*/
 float 		NominalSensitive2; 	/* Sensitivity at Nominal value [*] ?????????2*/
 uint8_t	CollectType;		/* type of Collect [No] ????β??????*/
 uint8_t   Sensortype;			/* Sensor type [No] ??????????*/
 uint8_t 	Sign; 				/* Invert sign of channel [1/0] ??????????*/
 uint8_t 	Day; 				/* Date of last change [No] ??θ??????????*/
 uint8_t 	Month; 				/* Date of last change [No]	??θ??????????*/
 uint16_t 	Year; 				/* Date of last change [No]	??θ??????????*/
}SENSORBIGDEFORMATIONDATA; 

extern SENSORBIGDEFORMATIONDATA SensorBigDeformationData[SENSOR_CH_NUM];

typedef struct
{
	float P;
	float I;
	float D;
}PID;

typedef struct
{
	float SpeedFFP;
	uint32_t PosDelay;
}FFD;

typedef struct
{
	float WndSize;
	uint32_t WndTime;
}WND;
extern WND wndHaltPos;
extern WND wndHaltLoad;
extern WND wndHaltExt;
extern WND wndPosExtPos;
extern WND wndPosExtLoad;
extern WND wndPosExtExt;

typedef struct{
    float Deadband;
	uint8_t PercentP;
}DEADBAND;

typedef struct{
	float 	UpperSft;
	float	LowerSft;
	uint8_t Reaction;
}SFT;

typedef struct{
	float 	DitherFrequency;
	float	DitherAmplitude;
}DITHER;

typedef struct{
	float Acc;//accelerate
	float Speed;//speed
}NOMINALVALUE;//Nomial Acc & Speed

typedef struct
{
    DEADBAND Deadband;//Deadband
	float DefaultAcc;//DefaultAcc
	float DefaultDec;//DefaultDec
	float DefaultSpeed;//DefaultSpeed
	//Wnd
	WND Wnd;
	//Sft
	SFT Sft;
	//PIDPos
	uint8_t HighPressure;
	PID Pid;
	PID Pid_Hp;
	//PidSpeed
	PID PidSpeed;
	PID PidSpeed_Hp;
	//Ffd
	FFD Ffd;
	FFD Ffd_Hp;
	NOMINALVALUE Nominal;
}SETPARA;//POS_MODE LOAD_MODE EXTEN_MODE

typedef struct {
	uint32_t 	electronicGear;
	uint32_t 	encoderResolution;
	float	 	analogGain;	
	float	 	ResolutionGearRate;
}servoParam_t;
extern servoParam_t servoParam;

//Outputpara Signal 
#define SGINAL_A_B 				0x01
#define SGINAL_PULSE_SIGN 		0x02
#define SGINAL_UP_DOWN 			0x03
#define SGINAL_ANALOGUE			0x04
#define SGINAL_DC_MOTOR 		0x05
#define SGINAL_DC_LINEAR_MOTOR 	0x06

typedef struct{
	uint8_t Signal;//???????
	uint8_t Sign;  //????
	float	MaxValue;//???? ???????λ???????100.0%?????????????????????????
	float 	MinValue;//??С? ???????λ???????-100.0%?????????????????????????
	float	InitValue;//???  ?????????????λ
	float   Offset;	 //???	
	float   SignalFrequency;//??????
	float	MaxVoltage;//?????
	float	MaxCurrent;//??????
}OUTPUTPARA;//???????

extern OUTPUTPARA OutputPara,OutputPara_combuf;

typedef struct{
	uint16_t TransmitDataPeriod;//???????????????
	uint8_t ControllerStructure;//????????
}SYSPARA;//??????

extern SYSPARA SysPara;

typedef struct{
	uint8_t TriggerCtrl;
	float Limit;
	float Trigger;
}utcTrig_t;

extern utcTrig_t utcTrig;

typedef enum{
	BHM_CMD_DWND = 0x01,
	BHM_CMD_MESSAGE = 0x02,
}BHM_e;//BHM means block header mode

typedef enum{
	BEM_CMD_IDLE = 0x00,
	BEM_CMD_START = 0x01,
	BEM_CMD_DISCARD = 0x02,
}BEM_e;//BEM means block execute mode

typedef struct{
	uint8_t Cycles;//assembly move instruction cycle times
	uint8_t ModeFlags;//CMD_DWND=0x00;CMD_MESSAGE=0x01;
}utcBlockHeader_t;

typedef struct{
	uint8_t Operation;//assembly move instruction cycle times
}utcBlockExecute_t;
extern utcBlockExecute_t utcBlockExecute;

#define COMBINED_MOVE_MAX_LEN 10
typedef struct{
	uint8_t executeStep;
	uint8_t executeCycleCounter;
	uint8_t counter;
	uint8_t command[COMBINED_MOVE_MAX_LEN];
	uint8_t MoveCtrl[COMBINED_MOVE_MAX_LEN];
	uint8_t LimitMode[COMBINED_MOVE_MAX_LEN];
	uint8_t DestinationCtrl[COMBINED_MOVE_MAX_LEN];
	uint8_t DestinationMode[COMBINED_MOVE_MAX_LEN];
	uint8_t TriggerCtrl[COMBINED_MOVE_MAX_LEN];
	uint16_t lpusTAN[COMBINED_MOVE_MAX_LEN];
	uint16_t Delay[COMBINED_MOVE_MAX_LEN];
	utcBlockHeader_t blockHeader;
	float DecDestination[COMBINED_MOVE_MAX_LEN];
	float Speed[COMBINED_MOVE_MAX_LEN];
	float Destination[COMBINED_MOVE_MAX_LEN];
	float Acc[COMBINED_MOVE_MAX_LEN];				//accelrate
	float Dec[COMBINED_MOVE_MAX_LEN];				//deceleration
	float DecLimit[COMBINED_MOVE_MAX_LEN];			//deceleration
	float Limit[COMBINED_MOVE_MAX_LEN];				//limit 
	float Trigger[COMBINED_MOVE_MAX_LEN];
}combinedMove_t;

extern combinedMove_t combinedMove;

typedef struct//??????????????????
{
	uint8_t OpenDevice_state;
	uint8_t DoPE_state;
	uint8_t DoPESetCtrl;
	uint8_t Command;//??????
	uint8_t MoveCtrl; //?????????
	uint8_t HighPressure;
	uint8_t Direction;//???????
	uint8_t LimitMode;//??λ??
	uint8_t DestinationCtrl;//???λ????????
	uint8_t DestinationMode;//????????
	uint8_t Cycles;			//??????????
	uint8_t DO;				//DO??????
	uint8_t OpenLoopflag;	//?????????????λ
	uint8_t Connector;		//??????????
	uint16_t haltDelay;
	uint16_t lpusTAN; //communication command serial number
	uint32_t DataTransmitPeriod;//??????????????????
	int32_t openloopFrq;//??????????? -200000~200000
	float Speed;			//???
	float Speed1;
	float Speed2;
	float Destination;//???λ??
	float Destination1;
	float Destination2;
	float DecDestination;
	float Acc;					//?????
	float Dec;					//?????
	float DecLimit;
	float Limit;				//??λ?
	float Halt1;        //??????1
	float Halt2;				//??????2
	float openloopAO;//??????DAC???? -100%~100%
	servoParam_t servoParam;
	utcBlockExecute_t blockExecute;
	utcBlockHeader_t blockHeader;
	SENSORSETPARA sensorset;
	//Pos Load Ext ???ò???
	SETPARA Pos;
	SETPARA Load;
	SETPARA Ext;
	PGDYN pgdyn;
	NOMINALVALUE Nominal;//??????
	OUTPUTPARA OutputPara;//???????
	SYSPARA SysPara;//??????
	utcTrig_t trig;
}commBuf_t;//????????????建????

extern commBuf_t commBuf,commBufLast;

typedef struct{
uint8_t EthMode;
uint8_t ServerIp[4];
uint16_t ServerPort;
uint8_t	LocalIp[4];
uint8_t LocalMac[6];
}ethConfig_t;//ethernet config

extern ethConfig_t ethConfig;

typedef struct{
	uint8_t CTRL_HALT;								//??
	uint8_t CTRL_DOWN;
	uint8_t CTRL_UP;
	uint8_t CTRL_MOVE;
	uint8_t CTRL_READY;
	uint8_t CTRL_FREE;
	uint8_t CTRL_INIT_E;
	uint8_t CTRL_SFTSET;
	uint8_t CTRL_SYNCINPUT;
	uint8_t CTRL_SYNCHWAIT;
	uint8_t CTRL_SLAVE;
	uint8_t CTRL_E_ACTIVE;						//??
}CONTROLLER_STATUS_STRUCT;

extern CONTROLLER_STATUS_STRUCT controllerStatus;
//extern uint16_t controller_status_WORD;

typedef struct{//???????	????  //????С??????
	float Position;
	float Load;
	float Extensometer1;
	float Extensometer2;
	float BigDeformation;
	uint16_t ManualBox;//????
	uint16_t InSignals;
	uint16_t OutSignals;
	uint16_t CtrlState1;
	uint8_t  CtrlState2;
	uint8_t	 UpperLimits_state;
	uint8_t  LowerLimits_state;
	uint8_t	 SensorConnected;
	uint8_t  SensorKeyPressed;
	uint32_t ModuleError;
	float    PosSpeed;
	float    LoadSpeed;
	float    Extensometer1Speed;
	float    Extensometer2Speed;
	float	 BigDeformationSpeed;
	uint32_t PosEncoderCode;//Pos?????????
	uint32_t LoadADCCode;//LOAD???
	uint32_t Extension1ADCCode;
	uint32_t Extension2ADCCode;
	uint32_t BigDeformationCode;
	float 	 pgst;
	float 	 pgLoadst;
	float	 pgExtst;
	uint32_t outputFrequency;
	float	 outputDAC;
}CDATATRANS;

extern CDATATRANS cdatatrans;
extern uint8_t	cdatabuf[CDTADTRANS_NUM];

typedef struct{
	uint32_t PeInterface[6];
	uint32_t Application[13];
	uint32_t Subsy[6];
	uint32_t SubsyCustVer[6];
	uint32_t SubsyCustName[9];
	uint32_t Bios[6];
	uint32_t HwCtrl[7];
	uint32_t PeInterfacePC[6];
	uint32_t DpxVer[6];
	uint32_t SerialNumber[17];
	uint8_t VersionEnd;
}VERSION_STRUCT;

extern VERSION_STRUCT utc_version;

typedef struct{
	PID PosPid;
	PID PosPid_Hp;//λ??PID?????????  ?????п?????????????
	PID SpeedPid;
	PID SpeedPid_Hp;
	PID PosExtPid;
	FFD Ffd;
	FFD Ffd_Hp;//???????????????  ?????п?????????????
	float DefaultAcc;//???????
	float Speed;//???????
	float Deviation;//??????
	uint8_t DevReaction;//??????
	WND Wnd;//?????????
	SFT Sft;//????λ
	float MinAcceleration;//??С?????
	float MaxAcceleration;//???????
	float MinDeceleration;//??С?????
	float MaxDeceleration;//???????
	float MinSpeed;//??С???
	float MaxSpeed;//??????
	DEADBAND Deadband;//???????????
	DITHER Dither;//????????
	NOMINALVALUE Nominal;//???? Acc & Speed
}ALLCTRLPARA;

extern ALLCTRLPARA posAllCtrlPara;
extern ALLCTRLPARA loadAllCtrlPara;
extern ALLCTRLPARA extAllCtrlPara;

#define LIMIT_NOT_ACTIVE	0x00
#define LIMIT_ABSOLUTE		0x01
#define LIMIT_RELATIVE		0x02

#define DEST_APPROACH	0x00
#define DEST_POSITION	0x01
#define DEST_MAINTAIN	0x02

typedef struct{
	uint8_t moveCtrlOrig;
	uint8_t limitmode;
	uint8_t destinationctrl;
	uint8_t destinationmode;
	float declimit;
	float decdestination;
	float limit;
	float Destination;
}POSEXT;
extern POSEXT posext;

#define CONNECTOR_NUM_MAX 8
typedef struct{
	uint8_t flag;
	uint8_t connector;
	int32_t	value[CONNECTOR_NUM_MAX];//int32_t tare value
	float	fValue[CONNECTOR_NUM_MAX];//float tare value
}TARE;
extern TARE tare;//传感器清零结构体


typedef struct{
	int8_t 		sign;
	uint32_t	sensorIntgr;
	float 		posLoopCtrlOut;
	float 		speedLoopCtrlOut;
	float		detaLoopCtrlOut;
	float 		pgSpeed;
	float 		setPos;
	float		NominalSensitive;
	float 		NominalValue;	    
}LOOPCTRL_STRUCT;

typedef struct
{
	uint8_t   	command;
	uint8_t 	command_last;
	uint8_t 	movectrl;
	uint8_t 	movectrl_last;
	uint8_t     halt_flag;
	uint8_t		cycleT;
	uint8_t		cyclef;
	uint8_t		creepFlag;
	uint16_t	lpusTAN;
	uint16_t	usTAN;
	int32_t		svPWMforProtect;
	float		AOforProtect;
    float 		servoOutput;
	float		Servo_output_last;
	float 		delta;//time interval
	float 		limit;
	float 		halt;
	float	 	defaultacc; 
	float 		defaultdec; 
	float 		defaultspeed; 
	float 		defaultspeedlimit;
	float 		upperlimit_phy;
	float 		lowerlimit_phy;
	float		utcTime;
	double 		lesoU;

	LOOPCTRL_STRUCT posCtrl;
	LOOPCTRL_STRUCT loadCtrl;
	LOOPCTRL_STRUCT ext1Ctrl;
	LOOPCTRL_STRUCT ext2Ctrl;    
	COMPLETE_STATE complete_state;//?????????
	EMERGENCY_STATE emergency_state;//?????????????
	//trigger
	trigState_t trigState;
	//Pos Load Ext
	SETPARA Pos;
	SETPARA Load;
	SETPARA Ext;
	//WND 
	uint32_t wnd_timer;
	uint32_t wnd_timerPos;
	uint32_t wnd_timerLoad;
	uint32_t wnd_timerExt;
	volatile uint16_t DataTransmitPeriod;
	int32_t openloopFrq;//??????DAC???? -200000~200000
	int32_t openloopFrq_Last;//??????DAC???? -200000~200000
	float openloopAO;//??????DAC???? -100%~100%
	SYSPARA SysPara;//??????
	TARE	tare;
}APP_LAYER_VARIABLE_STRUCT;
extern APP_LAYER_VARIABLE_STRUCT AL;

typedef struct {
	uint16_t 	posDeadzone;
	uint16_t 	loadDeadzone;
	uint16_t 	extDeadzone;
	float 	pos;
	float 	load;
	float 	ext;
	int32_t posCompensate;
}SERVOPWM;
extern SERVOPWM svPWM;

typedef struct {
	uint16_t 	posOffset;
	uint16_t 	loadOffset;
	uint16_t 	extOffset;
	float 	pos;
	float 	load;
	float 	ext;
}SERVOAO;
extern SERVOAO svAO;

typedef struct{
	uint8_t reset;
}SYSTEMRESET_MYSEMAPHORE;

typedef struct{
	uint8_t write_sensorcode;
	uint8_t write_outputpara;
	uint8_t write_syspara;
	uint8_t write_sendataPose;
	uint8_t write_sendataBigDerformation;
	uint8_t write_sendataOther;//include load extensometer1、2
	uint8_t write_defalutacc_pos;
	uint8_t write_defalutacc_load;
	uint8_t write_defalutacc_ext;
	uint8_t write_speed_pos;
	uint8_t write_speed_load;
	uint8_t write_speed_ext;
	uint8_t write_nominal_pos;
	uint8_t write_nominal_load;
	uint8_t write_nominal_ext;
	uint8_t write_transmitdataperiod;
	uint8_t write_wnd_pos;
	uint8_t write_wnd_load;
	uint8_t write_wnd_ext;
	uint8_t write_sft_pos;
	uint8_t write_sft_load;
	uint8_t write_sft_ext;
	uint8_t write_pospid_pos;
	uint8_t write_pospid_load;
	uint8_t write_pospid_ext;
	uint8_t write_speedpid_pos;
	uint8_t write_speedpid_load;
	uint8_t write_speedpid_ext;
	uint8_t write_ffd_pos;
	uint8_t write_ffd_load;
	uint8_t write_ffd_ext;
	uint8_t write_intgr;
	uint8_t write_sensorCheck;
	uint8_t readSensorOnceLoad;
	uint8_t readSensorOnceExt1;
	uint8_t readSensorOnceExt2;
	uint8_t writeEthConfig;
	uint8_t writeServoParam;
}STORAGE_PARAM_MYSEMAPHORE;

typedef struct{
	uint8_t DISem;
	uint8_t DOSem;
}MAP_FUNC_MYSEMAPHORE;

typedef struct{
	uint8_t CON1;
	uint8_t CON2;
	uint8_t CON3;
	uint8_t ADJ1;
	uint8_t ADJ2;
	uint8_t ADJ3;
}DB9_MYSEMAPHORE;

typedef struct{
	uint8_t pgInit;
	uint8_t combinedMoveExecute;
}MOVE_MYSEMAPHORE;

typedef struct{
	uint8_t sendataUpdate;//sensor data update semaphore
	uint8_t outputUpdate;//output parameter update semaphore
	uint8_t transperiodUpdate;//transmit period update semaphore
	uint8_t ctrlstructUpdate;//control struct update semaphore
}COMMUNICATION_MYSEMAPHORE;

typedef struct 
{
	uint8_t pos;
	uint8_t load;
	uint8_t ext;
	uint8_t speedPos;
	uint8_t speedLoad;
	uint8_t speedext;
}PID_PARAM_MYSEMAPHORE;//when pid parameter is updated

typedef struct{
	uint8_t set;
	uint8_t write;
}PARACONFIG_MYSEMAPHORE;

typedef struct{
	uint8_t write;
}CALIBRATE_MYSEMAPHORE;

typedef struct{
	uint8_t up;
	uint8_t rapidUp;
	uint8_t down;
	uint8_t rapidDown;
	uint8_t testStart;
	uint8_t reset;
	uint8_t halt;
	uint8_t clampUpperFixture;
	uint8_t releaseUpperFixture;
	uint8_t clampLowerFixture;
	uint8_t releaseLowerFixture;
}MANUAL_BOX_485_MYSEMAPHORE;

typedef struct{
	uint8_t SetOnPos;
	uint8_t SetOnTPos;
	uint8_t SetOnLPos;
	uint8_t SetOnSft;
	uint8_t SetOnRuntime;
	uint8_t SetOnSystem;
	uint8_t SetOnDebug;
}UTC_SET_ON_MYSEMAPHORE;

typedef struct{
	SYSTEMRESET_MYSEMAPHORE systemReset;
	STORAGE_PARAM_MYSEMAPHORE eeprom;
	STORAGE_PARAM_MYSEMAPHORE mram;
	MAP_FUNC_MYSEMAPHORE mapFunc;
	DB9_MYSEMAPHORE db9;
	MOVE_MYSEMAPHORE move;
	COMMUNICATION_MYSEMAPHORE comm;
	PID_PARAM_MYSEMAPHORE pid;
	PARACONFIG_MYSEMAPHORE paraconfig;
	CALIBRATE_MYSEMAPHORE calibrate;
	MANUAL_BOX_485_MYSEMAPHORE manualBox485;
	UTC_SET_ON_MYSEMAPHORE	utcSetOn;
}mySemaphore_t;
extern mySemaphore_t mySemaphore;

typedef struct{
	uint8_t DImapNum[10];
}COM_MAP_STRUCT;
extern COM_MAP_STRUCT comMap;

#define FILTER_LEN 1000
#define STRAIN_FILTER_LEN 500
typedef struct{
	float speed_temp;
	float force_temp;
	float strain_temp;
	float strain2_temp;
	float speedload_temp;
	float speedStrain_temp;
	float speed_buf[5];
	float speed_buf_load[1000];
	float speed_buf_strain[10];
	float force_buf[FILTER_LEN];
	float strain_buf[FILTER_LEN];
	float strain2_buf[FILTER_LEN];
}FILTER;
extern FILTER filter;
extern FILTER filterSL;
extern FILTER filterSL2;

typedef struct{
	int32_t speed_buf[10];
	int32_t force_buf[FILTER_LEN];
	int32_t speed_buf_load[10];
	int32_t speed_temp;
	int32_t force_temp;
	int32_t speedload_temp;
}FILTER_INT;
extern FILTER_INT filter_Int;

typedef struct{
	uint8_t index;
	uint32_t reloadVal;
	uint32_t reloadValHalf;
	uint32_t pcs;
	float fclk;
	uint32_t fr;
	uint32_t nr;
	uint32_t nrLast;
	uint32_t detaNr;
	int32_t monitorCode1;
	int32_t monitorCode2;
	float set;
	float pidout;
	float ns;
	float fs;
	float real;
	float realLast;
	float detaReal;
	float monitor1;
	float monitor2;
	float monitor3;
	float monitor4;
	float monitor5;
	float monitor6;
	uint32_t esoCounter;
}TEST_STRUCT;
extern TEST_STRUCT test;

typedef struct{
	uint16_t real; //ms
	uint16_t expect;//ms
	uint16_t max;//ms
	uint16_t min;//ms
	uint16_t count;//ms
	double fclkExp;//Hz
}ctrlPeriod_t;
extern ctrlPeriod_t posCtrlPeriod;
extern ctrlPeriod_t extCtrlPeriod;

extern float pwms1;
extern float pwms2;
extern float pwms3;

typedef struct{
	uint8_t ethFood;
	uint8_t ethDog;
	uint8_t commFood;
	uint8_t commDog;
}TASK_MONITOR_STRUCT;

extern TASK_MONITOR_STRUCT taskMonitor;


void parameter_default_init(void);//?????????
void sendata_default_init(void);
float servoParamCalcu(servoParam_t* _servoParam);
void commBufResume(commBuf_t* x,commBuf_t* x_last);
void commBufSave(commBuf_t* x,commBuf_t* x_last);
float constraint(float in,float upperlimit,float lowerlimit);
int constraintINT(int in,int upperlimit,int lowerlimit);
void storageParaDefaultInit(void);
void workModeDefaultInit(WORK_MODE_STRUCT* Wm);
void workStateDefaultInit(WORK_STATE_STRUCT* ws);
void workModeMonitor(WORK_MODE_STRUCT* Wm,WORK_STATE_STRUCT* Ws);
void stm32SoftReset(void);
#endif


#include "gParameter.h"
#include "EthProtocol.h"
#include <string.h>
#include "in_out.h"
#include "control.h"
#include "sensor.h"
#include "pid.h"
#include <elog.h>
WORK_MODE_STRUCT wm;
WORK_STATE_STRUCT ws;
WND wndHaltPos;
WND wndHaltLoad;
WND wndHaltExt;
WND wndPosExtPos;
WND wndPosExtLoad;
WND wndPosExtExt;
StateFlag stateFlag;
StateFlag stateFlagLast;
uint16_t lpusTAN=0;//通信指令编号
APP_LAYER_VARIABLE_STRUCT AL;
commBuf_t commBuf,commBufLast;
CONTROLLER_STATUS_STRUCT controllerStatus;
//uint16_t controller_status_WORD;
CDATATRANS cdatatrans;
uint8_t	cdatabuf[CDTADTRANS_NUM]={0};
VERSION_STRUCT utc_version;
ethConfig_t ethConfig;
ALLCTRLPARA posAllCtrlPara;
ALLCTRLPARA loadAllCtrlPara;
ALLCTRLPARA extAllCtrlPara;
SENSORANALOGUEDATA SenAData;
SENSORANALOGUEDATA SenAData2;
SENSORHEADER SenHeader;
sensorConnector_e sensorConnector;
sensorData_t SenData[SENSOR_CH_NUM],senDataCommbuf[SENSOR_CH_NUM],SenDatatest;
SENSORBIGDEFORMATIONDATA SensorBigDeformationData[SENSOR_CH_NUM];
POSEXT posext;
OUTPUTPARA OutputPara,OutputPara_combuf;
SYSPARA SysPara;
mySemaphore_t mySemaphore;
COM_MAP_STRUCT comMap;
FILTER filter;
FILTER filterSL;
FILTER filterSL2;
FILTER_INT filter_Int;
EMERGENCY_SRTUCT emerency_struct;
SERVOPWM svPWM;
SERVOAO svAO;
servoParam_t servoParam;
TEST_STRUCT test;
ctrlPeriod_t posCtrlPeriod;
ctrlPeriod_t extCtrlPeriod;
utcTrig_t utcTrig;
combinedMove_t combinedMove;
utcBlockExecute_t utcBlockExecute;
float pwms1;
float pwms2;
float pwms3;
TASK_MONITOR_STRUCT taskMonitor;
extern void measureData_init(void);
void EmergencyErrorJudge(void);
void Emergency_Monitor(void);
void Emergency_Process(void);

void parameter_default_init(void)//参数初始化
{
	//state_flag初始化
	stateFlag.OpenDevice_state  =	Device_Off;
	stateFlag.DoPE_state  =	DoPE_Off;
	stateFlag.DoPESetCtrl = CtrlFALSE;
	stateFlag.DynCycle_OffsetReach = 0;

	stateFlag.openloop = 0;
	
	measureData_init();
	AL.posCtrl.setPos=0;	//期望位置
	AL.loadCtrl.setPos=0;	//期望力矩
	AL.ext1Ctrl.setPos = 0;
	AL.ext2Ctrl.setPos = 0;
	AL.posCtrl.sign = 1;//位移方向
	AL.loadCtrl.sign = -1;//力方向
	AL.ext1Ctrl.sign = 1;//direction of extensometer
	AL.movectrl = 9;//默认位置模式
	AL.movectrl_last = 9;

	svPWM.pos = 0;
	svPWM.load = 0;
	svPWM.ext = 0;
	svPWM.posDeadzone = 470;
	svPWM.loadDeadzone = 470;
	svPWM.extDeadzone = 470;
	svAO.pos = 0;
	svAO.load = 0;
	svAO.ext = 0;
	svAO.posOffset = 0;
	svAO.loadOffset = 0;
	svAO.extOffset = 0;

	AL.delta = 0.001f; //速度采样周期1ms
	AL.defaultacc = 5;
	AL.defaultdec = -5;
	AL.defaultspeed = 10;
	AL.defaultspeedlimit = 50;
	
	//AL.speed_ext = 0;
	AL.upperlimit_phy = 1000;
	AL.lowerlimit_phy = -1000;
	AL.emergency_state.occur = 0;
	AL.emergency_state.complete = 0;
	AL.emergency_state.processing = 0;
	AL.complete_state.flag = 1;
	AL.posCtrl.NominalSensitive = 0.0000428482053;
	AL.loadCtrl.NominalSensitive = 3.0764;
	AL.ext1Ctrl.NominalSensitive = 1.61;

	AL.loadCtrl.NominalValue = 10;
	AL.ext1Ctrl.NominalValue = 10;
	AL.ext2Ctrl.NominalValue = 10;
	AL.openloopAO = 0;
	AL.openloopFrq = 0;

	commBuf.openloopAO = 0;
	commBuf.SysPara.TransmitDataPeriod = SysPara.TransmitDataPeriod; //20ms
	
	//传感器滤波初始化
	commBuf.sensorset.SensorNo = 0;
	commBuf.sensorset.Intgr = 10;
	
	//死区初始化
	commBuf.Pos.Deadband.Deadband = 0;
	commBuf.Pos.Deadband.PercentP = 0;
	commBuf.Load.Deadband.Deadband = 0;
	commBuf.Load.Deadband.PercentP = 0;
	commBuf.Ext.Deadband.Deadband = 0;
	commBuf.Ext.Deadband.PercentP = 0;
	
	//默认加/减速度初始化
	commBuf.Pos.DefaultAcc = 20;
	commBuf.Pos.DefaultDec = -20;
	commBuf.Load.DefaultAcc = 200;
	commBuf.Load.DefaultDec = -200;
	commBuf.Ext.DefaultAcc = 0.01;
	commBuf.Ext.DefaultDec = -0.01;
	
	//默认速度初始化
	commBuf.Pos.DefaultSpeed = 20;
	commBuf.Load.DefaultSpeed = 20;
	commBuf.Ext.DefaultSpeed = 0.01;
	
	//高低压模式
	commBuf.Pos.HighPressure = 0;
	commBuf.Load.HighPressure = 0;
	commBuf.Ext.HighPressure = 0;
	
	//位置闭环控制参数初始化
	//Pos
	commBuf.Pos.Pid.P = 1;
	commBuf.Pos.Pid.I = 0.01;
	commBuf.Pos.Pid.D = 0.1;
	commBuf.Pos.Pid_Hp.P = 2;
	commBuf.Pos.Pid_Hp.I = 0.02;
	commBuf.Pos.Pid_Hp.D = 0.2;
	//Load
	commBuf.Load.Pid.P = 1;
	commBuf.Load.Pid.I = 0.01;
	commBuf.Load.Pid.D = 0;
	commBuf.Load.Pid_Hp.P = 2;
	commBuf.Load.Pid_Hp.I = 0.02;
	commBuf.Load.Pid_Hp.D = 0;
	//Ext
	commBuf.Ext.Pid.P = 1;
	commBuf.Ext.Pid.I = 0.001;
	commBuf.Ext.Pid.D = 0;
	commBuf.Ext.Pid_Hp.P = 1;
	commBuf.Ext.Pid_Hp.I = 0.001;
	commBuf.Ext.Pid_Hp.D = 0;
	
	//速度前馈参数初始化
	//Pos
	commBuf.Pos.Ffd.SpeedFFP = 0;
	commBuf.Pos.Ffd.PosDelay = 0;
	commBuf.Pos.Ffd_Hp.SpeedFFP = 0;
	commBuf.Pos.Ffd_Hp.PosDelay = 0;
	//Load
	commBuf.Load.Ffd.SpeedFFP = 0;
	commBuf.Load.Ffd.PosDelay = 0;
	commBuf.Load.Ffd_Hp.SpeedFFP = 0;
	commBuf.Load.Ffd_Hp.PosDelay = 0;
	//Ext
	commBuf.Ext.Ffd.SpeedFFP = 0;
	commBuf.Ext.Ffd.PosDelay = 0;
	commBuf.Ext.Ffd_Hp.SpeedFFP = 0;
	commBuf.Ext.Ffd_Hp.PosDelay = 0;
	
	//软限位
	commBuf.Pos.Sft.UpperSft = 1000;
	commBuf.Pos.Sft.LowerSft = -1000;
	commBuf.Load.Sft.UpperSft = 1000;
	commBuf.Load.Sft.LowerSft = -1000;
	commBuf.Ext.Sft.UpperSft = 5;
	commBuf.Ext.Sft.LowerSft = -5;
	
	//设定目标误差/时间窗口
	commBuf.Pos.Wnd.WndSize = 0.01;
	commBuf.Pos.Wnd.WndTime = 50;
	commBuf.Load.Wnd.WndSize = 1;
	commBuf.Load.Wnd.WndTime = 50;
	commBuf.Ext.Wnd.WndSize = 0.001;
	commBuf.Ext.Wnd.WndTime = 50;
	
	//dyncycle_halttime
	dyncycle_halttime.haltamplitude_countingflag = 0;
	dyncycle_halttime.haltatamplitude_time = 0;
	dyncycle_halttime.haltatamplitude_timestamp = 0;
	dyncycle_halttime.haltatminusamplitude_counter = 0;
	dyncycle_halttime.haltatplusamplitude_counter = 0;
	dyncycle_halttime.wave_time = 0;

	//filter
	memset(filter.speed_buf,0,sizeof(filter.speed_buf));
	memset(filter.speed_buf_load,0,sizeof(filter.speed_buf_load));
	memset(filter.force_buf,0,sizeof(filter.force_buf));
	memset(filter.strain_buf,0,sizeof(filter.strain_buf));
	filter.speed_temp = 0;
	filter.speedload_temp = 0;
	filter.force_temp = 0;
	filter.strain_temp = 0;

	//wnd halt
	wndHaltPos.WndSize = 0.2;
	wndHaltPos.WndTime = 100;
	wndHaltLoad.WndSize = 1;
	wndHaltLoad.WndTime = 100;
	wndHaltExt.WndSize = 0.001;
	wndHaltExt.WndTime = 50;
	wndPosExtPos.WndSize = 0.05;
	wndPosExtPos.WndTime = 1;
	wndPosExtLoad.WndSize = 1;
	wndPosExtLoad.WndTime = 1;
	wndPosExtExt.WndSize = 0.001;
	wndPosExtExt.WndTime = 1;

	//pos ctrl period
	posCtrlPeriod.real = 1;
	posCtrlPeriod.expect = 1;
	posCtrlPeriod.max = 1500;
	posCtrlPeriod.min = 1;
	posCtrlPeriod.fclkExp = 0;

// wiz_NetInfo gWIZNETINFO = { .mac = {0x00, 0x08, 0xdc,0x00, 0xab, 0xcd},
//                             .ip = {192, 168, 0, 11},
//                             .sn = {255,255,255,0},
//                             .gw = {192, 168, 0, 1},
//                             .dns = {8,8,8,8},
//                             .dhcp = NETINFO_STATIC};
	//ethernet config
	ethConfig.EthMode = 0x00;//TCP 0x00 UDP 0x01
	ethConfig.LocalIp[0] = 192;ethConfig.LocalIp[1] = 168;ethConfig.LocalIp[2] = 0;ethConfig.LocalIp[3] = 11;
	ethConfig.LocalMac[0] = 0x00;ethConfig.LocalMac[1] = 0x08;ethConfig.LocalMac[2] = 0xdc;ethConfig.LocalMac[3] = 0x00;ethConfig.LocalMac[4] = 0xab;ethConfig.LocalMac[5] = 0xcd;
	ethConfig.ServerIp[0] = 192;ethConfig.ServerIp[1] = 168;ethConfig.ServerIp[2] = 0;ethConfig.ServerIp[3] = 10;
	ethConfig.ServerPort = 5001;

	//semaphore init 
	mySemaphore.move.combinedMoveExecute = BEM_CMD_IDLE;

}

/**
 * @brief Storage parameter init by default 
*/
void storageParaDefaultInit(void){
	//default acc dec
	commBuf.Pos.DefaultAcc = 20;
	commBuf.Pos.DefaultDec = -20;
	commBuf.Load.DefaultAcc = 200;
	commBuf.Load.DefaultDec = -200;
	commBuf.Ext.DefaultAcc = 0.01;
	commBuf.Ext.DefaultDec = -0.01;

	//upper/lower limit phy
	AL.upperlimit_phy = 1000;
	AL.lowerlimit_phy = -1000;

	//speed speedlimit
	AL.defaultspeed = 20;
	AL.defaultspeedlimit = 20;

	//posAllCtrlPara
	posAllCtrlPara.PosPid.P = 1;
	posAllCtrlPara.PosPid.I = 0;
	posAllCtrlPara.PosPid.D = 0;
	posAllCtrlPara.PosPid_Hp.P = 1;
	posAllCtrlPara.PosPid_Hp.I = 0;
	posAllCtrlPara.PosPid_Hp.D = 0;
	posAllCtrlPara.SpeedPid.P = 0;
	posAllCtrlPara.SpeedPid.I = 0;
	posAllCtrlPara.SpeedPid.D = 0;
	posAllCtrlPara.Ffd.SpeedFFP = 0;
	posAllCtrlPara.Ffd.PosDelay = 0;
	posAllCtrlPara.Ffd_Hp.SpeedFFP = 0;
	posAllCtrlPara.Ffd_Hp.PosDelay = 0;
	posAllCtrlPara.DefaultAcc = 30;
	posAllCtrlPara.Speed = 20;
	posAllCtrlPara.Deviation = 0;
	posAllCtrlPara.DevReaction = 0;
	posAllCtrlPara.Wnd.WndSize = 0.002;
	posAllCtrlPara.Wnd.WndTime = 100 ;//ms
	posAllCtrlPara.Sft.UpperSft = 1000;
	posAllCtrlPara.Sft.LowerSft = -1000;
	posAllCtrlPara.Sft.Reaction = 0;
	posAllCtrlPara.MinAcceleration = 0.001;
	posAllCtrlPara.MaxAcceleration = 100;
	posAllCtrlPara.MinDeceleration = -0.001;
	posAllCtrlPara.MaxDeceleration = -100;
	posAllCtrlPara.MinSpeed = 0.0001;
	posAllCtrlPara.MaxSpeed = 20;
	posAllCtrlPara.Deadband.Deadband = 0;
	posAllCtrlPara.Deadband.PercentP = 0;
	posAllCtrlPara.Dither.DitherAmplitude = 0;
	posAllCtrlPara.Dither.DitherFrequency = 0;
	posAllCtrlPara.Nominal.Acc = 50;
	posAllCtrlPara.Nominal.Speed = 20; 

	//loadAllCtrlPara
	loadAllCtrlPara.PosPid.P = 10;
	loadAllCtrlPara.PosPid.I = 0.1;
	loadAllCtrlPara.PosPid.D = 0;
	loadAllCtrlPara.PosPid_Hp.P = 1;
	loadAllCtrlPara.PosPid_Hp.I = 0.001;
	loadAllCtrlPara.PosPid_Hp.D = 0;
	loadAllCtrlPara.SpeedPid.P = 0;
	loadAllCtrlPara.SpeedPid.I = 0;
	loadAllCtrlPara.SpeedPid.D = 0;
	loadAllCtrlPara.Ffd.SpeedFFP = 0;
	loadAllCtrlPara.Ffd.PosDelay = 0;
	loadAllCtrlPara.Ffd_Hp.SpeedFFP = 0;
	loadAllCtrlPara.Ffd_Hp.PosDelay = 0;
	loadAllCtrlPara.DefaultAcc = 5;
	loadAllCtrlPara.Speed = 10;
	loadAllCtrlPara.Deviation = 0;
	loadAllCtrlPara.DevReaction = 0;
	loadAllCtrlPara.Wnd.WndSize = 1;
	loadAllCtrlPara.Wnd.WndTime = 100 ;//ms
	loadAllCtrlPara.Sft.UpperSft = 10000;
	loadAllCtrlPara.Sft.LowerSft = -10000;
	loadAllCtrlPara.Sft.Reaction = 0;
	loadAllCtrlPara.MinAcceleration = 0.001;
	loadAllCtrlPara.MaxAcceleration = 200;
	loadAllCtrlPara.MinDeceleration = -0.001;
	loadAllCtrlPara.MaxDeceleration = -200;
	loadAllCtrlPara.MinSpeed = 1;
	loadAllCtrlPara.MaxSpeed = 2000;
	loadAllCtrlPara.Deadband.Deadband = 0;
	loadAllCtrlPara.Deadband.PercentP = 0;
	loadAllCtrlPara.Dither.DitherAmplitude = 0;
	loadAllCtrlPara.Dither.DitherFrequency = 0;
	loadAllCtrlPara.Nominal.Acc = 200;
	loadAllCtrlPara.Nominal.Speed = 1000; 

	//extAllCtrlPara
	extAllCtrlPara.PosPid.P = 1;
	extAllCtrlPara.PosPid.I = 0.001;
	extAllCtrlPara.PosPid.D = 0;
	extAllCtrlPara.PosPid_Hp.P = 1;
	extAllCtrlPara.PosPid_Hp.I = 0;
	extAllCtrlPara.PosPid_Hp.D = 0;
	extAllCtrlPara.SpeedPid.P = 0;
	extAllCtrlPara.SpeedPid.I = 0;
	extAllCtrlPara.SpeedPid.D = 0;
	extAllCtrlPara.Ffd.SpeedFFP = 0;
	extAllCtrlPara.Ffd.PosDelay = 0;
	extAllCtrlPara.Ffd_Hp.SpeedFFP = 0;
	extAllCtrlPara.Ffd_Hp.PosDelay = 0;
	extAllCtrlPara.DefaultAcc = 0.01;
	extAllCtrlPara.Speed = 0.005;
	extAllCtrlPara.Deviation = 0;
	extAllCtrlPara.DevReaction = 0;
	extAllCtrlPara.Wnd.WndSize = 0.001;
	extAllCtrlPara.Wnd.WndTime = 100 ;//ms
	extAllCtrlPara.Sft.UpperSft = 1000;
	extAllCtrlPara.Sft.LowerSft = -1000;
	extAllCtrlPara.Sft.Reaction = 0;
	extAllCtrlPara.MinAcceleration = 0.0001;
	extAllCtrlPara.MaxAcceleration = 1;
	extAllCtrlPara.MinDeceleration = -0.0001;
	extAllCtrlPara.MaxDeceleration = -1;
	extAllCtrlPara.MinSpeed = 0.0001;
	extAllCtrlPara.MaxSpeed = 1;
	extAllCtrlPara.Deadband.Deadband = 0;
	extAllCtrlPara.Deadband.PercentP = 0;
	extAllCtrlPara.Dither.DitherAmplitude = 0;
	extAllCtrlPara.Dither.DitherFrequency = 0;
	extAllCtrlPara.Nominal.Acc = 1;
	extAllCtrlPara.Nominal.Speed = 1; 

	//system parameters
	SysPara.TransmitDataPeriod = 20;
	SysPara.ControllerStructure = 0x02;

	//output parameter
	OutputPara.Signal = SGINAL_PULSE_SIGN;
	OutputPara.Sign = 0x00;
	OutputPara.MaxValue = 100;
	OutputPara.MinValue = -100;
	OutputPara.InitValue = 0;
	OutputPara.Offset = 0;
	OutputPara.SignalFrequency = 600000;
	OutputPara.MaxVoltage = 10;//V
	OutputPara.MaxCurrent = 0;//A

	//servoParam
	servoParam.electronicGear = 10000;
	servoParam.encoderResolution = 10000;
	servoParam.analogGain = 1;
	servoParamCalcu(&servoParam);

	//these data need to be saved realtime
	pose.code = 0;
	AL.tare.fValue[ch4Load] = 0;
	AL.tare.fValue[ch2Ext1] = 0;

	//senserCheck
	sensorCheck.Register[0] = 0;
	sensorCheck.Register[1] = 0;
	sensorCheck.Register[2] = 0;
	sensorCheck.MapNum[0] = SENSOR_NO_CHANNEL;
	sensorCheck.MapNum[1] = SENSOR_NO_CHANNEL;
	sensorCheck.MapNum[2] = SENSOR_NO_CHANNEL;

	//utc version
	utc_version.Bios[0] = '7';
	utc_version.Bios[1] = '9';
	utc_version.Bios[2] = '1';
	utc_version.Bios[3] = '3';
	utc_version.Bios[5] = '7';
	utc_version.Bios[5] = '7';

	//tare
	AL.tare.value[ch0Pose] = 0;
	AL.tare.value[ch1Bd] = 0;
	AL.tare.value[ch2Ext1] = 0;
	AL.tare.value[ch3Ext2] = 0;
	AL.tare.value[ch4Load] = 0;
	AL.tare.fValue[ch2Ext1] = 0;
	AL.tare.fValue[ch3Ext2] = 0;
	AL.tare.fValue[ch4Load] = 0;


}

void sendata_default_init(void)
{
	SenData[ch0Pose].Sign = 0x01;//pose
	SenData[ch1Bd].Sign = 0x00;//large deformation
	SenData[ch2Ext1].Sign = 0x01;//ext1
	SenData[ch3Ext2].Sign = 0x00;//ext2
	SenData[ch4Load].Sign = 0x00;//Load
	
	SenData[ch0Pose].NominalSensitive = 0.0000181352;//0.0000282462
	SenData[ch1Bd].NominalSensitive = 1;
	SenData[ch2Ext1].NominalSensitive = 1.705;
	SenData[ch3Ext2].NominalSensitive = 1.705;
	SenData[ch4Load].NominalSensitive = 3;

	SenData[ch2Ext1].NominalValue = 10;//mm
	SenData[ch4Load].NominalValue = 10;//kN

	SenData[ch0Pose].sensorIntgr = 20;
	SenData[ch1Bd].sensorIntgr = 20;
	SenData[ch2Ext1].sensorIntgr = 20;
	SenData[ch3Ext2].sensorIntgr = 20;
	SenData[ch4Load].sensorIntgr = 20;
}

/**
 * @brief packup the parameters of configuration
 * @version 1.09.92
*/
void paraConfigurePackup(void){
	//default acc dec
	commBuf.Pos.DefaultAcc = 20;
	commBuf.Pos.DefaultDec = -20;
	commBuf.Load.DefaultAcc = 5;
	commBuf.Load.DefaultDec = -5;
	commBuf.Ext.DefaultAcc = 5;
	commBuf.Ext.DefaultDec = -5;

	//upper/lower limit phy
	AL.upperlimit_phy = 1000;
	AL.lowerlimit_phy = -1000;

	//speed speedlimit
	AL.defaultspeed = 10;
	AL.defaultspeedlimit = 20;

	//posAllCtrlPara
	posAllCtrlPara.PosPid.P = 15;
	posAllCtrlPara.PosPid.I = 0;
	posAllCtrlPara.PosPid.D = 1;
	posAllCtrlPara.PosPid_Hp.P = 15;
	posAllCtrlPara.PosPid_Hp.I = 0;
	posAllCtrlPara.PosPid_Hp.D = 1;
	posAllCtrlPara.SpeedPid.P = 0;
	posAllCtrlPara.SpeedPid.I = 0;
	posAllCtrlPara.SpeedPid.D = 0;
	posAllCtrlPara.Ffd.SpeedFFP = 0;
	posAllCtrlPara.Ffd.PosDelay = 0;
	posAllCtrlPara.Ffd_Hp.SpeedFFP = 0;
	posAllCtrlPara.Ffd_Hp.PosDelay = 0;
	posAllCtrlPara.DefaultAcc = 30;
	posAllCtrlPara.Speed = 20;
	posAllCtrlPara.Deviation = 0;
	posAllCtrlPara.DevReaction = 0;
	posAllCtrlPara.Wnd.WndSize = 0.002;
	posAllCtrlPara.Wnd.WndTime = 100 ;//ms
	posAllCtrlPara.Sft.UpperSft = 1000;
	posAllCtrlPara.Sft.LowerSft = -1000;
	posAllCtrlPara.Sft.Reaction = 0;
	posAllCtrlPara.MinAcceleration = 0.001;
	posAllCtrlPara.MaxAcceleration = 100;
	posAllCtrlPara.MinDeceleration = -0.1;
	posAllCtrlPara.MaxDeceleration = -100;
	posAllCtrlPara.MinSpeed = 0.0001;
	posAllCtrlPara.MaxSpeed = 20;
	posAllCtrlPara.Deadband.Deadband = 0;
	posAllCtrlPara.Deadband.PercentP = 0;
	posAllCtrlPara.Dither.DitherAmplitude = 0;
	posAllCtrlPara.Dither.DitherFrequency = 0;
	posAllCtrlPara.Nominal.Acc = 50;
	posAllCtrlPara.Nominal.Speed = 20; 

	//loadAllCtrlPara
	loadAllCtrlPara.PosPid.P = 0.04;
	loadAllCtrlPara.PosPid.I = 0.00035;
	loadAllCtrlPara.PosPid.D = 0;
	loadAllCtrlPara.PosPid_Hp.P = 0.04;
	loadAllCtrlPara.PosPid_Hp.I = 0.00035;
	loadAllCtrlPara.PosPid_Hp.D = 0;
	loadAllCtrlPara.SpeedPid.P = 0;
	loadAllCtrlPara.SpeedPid.I = 0;
	loadAllCtrlPara.SpeedPid.D = 0;
	loadAllCtrlPara.Ffd.SpeedFFP = 100;
	loadAllCtrlPara.Ffd.PosDelay = 0;
	loadAllCtrlPara.Ffd_Hp.SpeedFFP = 0;
	loadAllCtrlPara.Ffd_Hp.PosDelay = 0;
	loadAllCtrlPara.DefaultAcc = 5;
	loadAllCtrlPara.Speed = 10;
	loadAllCtrlPara.Deviation = 0;
	loadAllCtrlPara.DevReaction = 0;
	loadAllCtrlPara.Wnd.WndSize = 1;
	loadAllCtrlPara.Wnd.WndTime = 100 ;//ms
	loadAllCtrlPara.Sft.UpperSft = 1000;
	loadAllCtrlPara.Sft.LowerSft = -1000;
	loadAllCtrlPara.Sft.Reaction = 0;
	loadAllCtrlPara.MinAcceleration = 0.001;
	loadAllCtrlPara.MaxAcceleration = 100;
	loadAllCtrlPara.MinDeceleration = -0.001;
	loadAllCtrlPara.MaxDeceleration = -100;
	loadAllCtrlPara.MinSpeed = 0.01;
	loadAllCtrlPara.MaxSpeed = 20;
	loadAllCtrlPara.Deadband.Deadband = 0;
	loadAllCtrlPara.Deadband.PercentP = 0;
	loadAllCtrlPara.Dither.DitherAmplitude = 0;
	loadAllCtrlPara.Dither.DitherFrequency = 0;
	loadAllCtrlPara.Nominal.Acc = 50;
	loadAllCtrlPara.Nominal.Speed = 50; 

	//extAllCtrlPara
	extAllCtrlPara.PosPid.P = 1;
	extAllCtrlPara.PosPid.I = 0;
	extAllCtrlPara.PosPid.D = 0;
	extAllCtrlPara.PosPid_Hp.P = 1;
	extAllCtrlPara.PosPid_Hp.I = 0;
	extAllCtrlPara.PosPid_Hp.D = 0;
	extAllCtrlPara.SpeedPid.P = 0;
	extAllCtrlPara.SpeedPid.I = 0;
	extAllCtrlPara.SpeedPid.D = 0;
	extAllCtrlPara.Ffd.SpeedFFP = 0;
	extAllCtrlPara.Ffd.PosDelay = 0;
	extAllCtrlPara.Ffd_Hp.SpeedFFP = 0;
	extAllCtrlPara.Ffd_Hp.PosDelay = 0;
	extAllCtrlPara.DefaultAcc = 5;
	extAllCtrlPara.Speed = 10;
	extAllCtrlPara.Deviation = 0;
	extAllCtrlPara.DevReaction = 0;
	extAllCtrlPara.Wnd.WndSize = 0.002;
	extAllCtrlPara.Wnd.WndTime = 100 ;//ms
	extAllCtrlPara.Sft.UpperSft = 1000;
	extAllCtrlPara.Sft.LowerSft = -1000;
	extAllCtrlPara.Sft.Reaction = 0;
	extAllCtrlPara.MinAcceleration = 0.001;
	extAllCtrlPara.MaxAcceleration = 100;
	extAllCtrlPara.MinDeceleration = -0.1;
	extAllCtrlPara.MaxDeceleration = -100;
	extAllCtrlPara.MinSpeed = 0.01;
	extAllCtrlPara.MaxSpeed = 20;
	extAllCtrlPara.Deadband.Deadband = 0;
	extAllCtrlPara.Deadband.PercentP = 0;
	extAllCtrlPara.Dither.DitherAmplitude = 0;
	extAllCtrlPara.Dither.DitherFrequency = 0;
	extAllCtrlPara.Nominal.Acc = 30;
	extAllCtrlPara.Nominal.Speed = 15; 

	//system parameters
	SysPara.TransmitDataPeriod = 20;
	SysPara.ControllerStructure = 0x02;

	//output parameter
	OutputPara.Signal = SGINAL_PULSE_SIGN;
	OutputPara.Sign = 0x01;
	OutputPara.MaxValue = 100;
	OutputPara.MinValue = -100;
	OutputPara.InitValue = 0;
	OutputPara.Offset = 0;
	OutputPara.SignalFrequency = 700000;
	OutputPara.MaxVoltage = 10;//V
	OutputPara.MaxValue = 0;//A

	//these data need to be saved realtime
	pose.code = 0;
	AL.tare.fValue[ch4Load] = 0;
	AL.tare.fValue[ch2Ext1] = 0;

	//senserCheck
	sensorCheck.Register[0] = 0;
	sensorCheck.Register[1] = 0;
	sensorCheck.Register[2] = 0;
	sensorCheck.MapNum[0] = SENSOR_NO_CHANNEL;
	sensorCheck.MapNum[1] = SENSOR_NO_CHANNEL;
	sensorCheck.MapNum[2] = SENSOR_NO_CHANNEL;

	//utc version
	utc_version.Bios[0] = '1';
	utc_version.Bios[1] = '.';
	utc_version.Bios[2] = '0';
	utc_version.Bios[3] = '9';
	utc_version.Bios[5] = '9';
	utc_version.Bios[5] = '2';


	//sensor
	SenData[ch0Pose].Sign = 0x00;//pose
	SenData[ch1Bd].Sign = 0x00;//large deformation
	SenData[ch2Ext1].Sign = 0x00;//ext1
	SenData[ch3Ext2].Sign = 0x00;//ext2
	SenData[ch4Load].Sign = 0x00;//Load
	
	SenData[ch0Pose].NominalSensitive = 0.0000181352;
	SenData[ch1Bd].NominalSensitive = 1;
	SenData[ch2Ext1].NominalSensitive = 1;
	SenData[ch3Ext2].NominalSensitive = 1;
	SenData[ch4Load].NominalSensitive = 3;

	SenData[ch0Pose].sensorIntgr = 20;
	SenData[ch1Bd].sensorIntgr = 20;
	SenData[ch2Ext1].sensorIntgr = 20;
	SenData[ch3Ext2].sensorIntgr = 20;
	SenData[ch4Load].sensorIntgr = 20;
	
}

/**
 * @brief Calculate the gear resolution ratio in servo parameters
 * @param _servoParam Pointer to the servo parameter structure, which contains parameters such as electronic gear and encoder resolution.
 * @return Returns the calculated gear resolution ratio.
 */
float servoParamCalcu(servoParam_t* _servoParam){
	_servoParam->ResolutionGearRate = SERVO_ENCODER_GAIN * (float)_servoParam->encoderResolution/ (float)_servoParam->electronicGear;
	return _servoParam->ResolutionGearRate;
}

void commBufResume(commBuf_t* x,commBuf_t* x_last)
{
	memcpy(x,x_last,sizeof(*x));
}

void commBufSave(commBuf_t* x,commBuf_t* x_last)
{
	memcpy(x_last,x,sizeof(*x_last));
}

float constraint(float in,float upperlimit,float lowerlimit)
{	
	float out;
	out = in;
	if(out > upperlimit) out = upperlimit;
	if(out < lowerlimit) out = lowerlimit;
	return out;
}

int constraintINT(int in,int upperlimit,int lowerlimit)
{	
	int out;
	out = in;
	if(out > upperlimit) out = upperlimit;
	if(out < lowerlimit) out = lowerlimit;
	return out;
}

/**
 * @brief init work mode
*/
void workModeDefaultInit(WORK_MODE_STRUCT* Wm){
	Wm->now = wmInit;
	Wm->last = Wm->now;
}

/**
 * @brief init work state
*/
void workStateDefaultInit(WORK_STATE_STRUCT* ws){
	ws->init = initSexecuting;
	ws->paraConfig = paraConfigScomplete;
	ws->moveReady &= (~moveReadySok);
	ws->loopCtrl = loopCtrlScomplete;
	ws->calibrate = calibrateScomplete;
	ws->emergency = 0;
	ws->moveRQ &= (~moveRQSenable);
}

//#define WORK_MODE_DEBUG
void paraConfigSem_Process(uint8_t* set,uint8_t* write,WORK_STATE_STRUCT* Ws);
/**
 * @note Fun_13
 * @brief work mode monitor
 * @param 		Wm	work mode pointer
 * @param[in] 	Ws	work state pointer
 * 
*/
void workModeMonitor(WORK_MODE_STRUCT* Wm,WORK_STATE_STRUCT* Ws){
	//emergency monitor
	Emergency_Monitor();//紧急情况检测
	switch (Wm->now)
	{
	case wmInit:

		/*****mode mointor and switch*****/
		//skip to paraConfig
		if((Ws->init & initScomplete) != 0){
			Wm->last = Wm->now;
			Wm->now = wmParaConfig;
#ifdef WORK_MODE_DEBUG
			log_i("wm.now=wmParaConfig wm.last=wmInit");
#endif
		}
		break;
	case wmParaConfig:
		/***state mointor and switch***/
		paraConfigSem_Process(&mySemaphore.paraconfig.set,&mySemaphore.paraconfig.write,Ws);

		/*****mode mointor and switch*****/
		//skip to wmMoveReady
		if((Ws->paraConfig & paraConfigScomplete) != 0){
			Wm->last = Wm->now;
			Wm->now = wmMoveReady;
#ifdef WORK_MODE_DEBUG
			log_i("wm.now=wmMoveReady wm.last=wmParaConfig");
#endif
		}
		break;
	case wmMoveReady:
		/***state mointor and switch***/
		if(stateFlag.OpenDevice_state == Device_Open
		&& stateFlag.DoPE_state == DoPE_On
		&& stateFlag.DoPESetCtrl == CtrlTRUE){
			Ws->moveReady = moveReadySok;//ok
		}else{
			Ws->moveReady &= (~moveReadySok);//not ok
		}
		//receive loopctrl command
		if(mySemaphore.move.pgInit == 1){
			Ws->loopCtrl = loopCtrlSreceive;
		}
		//receive parameter configration
		if(mySemaphore.paraconfig.write == 1 || mySemaphore.paraconfig.set == 1){
			Ws->paraConfig = paraConfigSreceive;
			AL.command = commBuf.Command;
		}
		//receive calibrate command(we have no idea of whether system is in calibration mode)
		// if(mySemaphore.calibrate.write== 1){
		// 	Ws->calibrate = calibrateSreceive;
		// }

		/*****mode mointor and switch*****/
		//skip to wmParaConfig
		if((Ws->paraConfig & paraConfigSreceive) != 0){
			Wm->last = Wm->now;
			Wm->now = wmParaConfig;
			#ifdef WORK_MODE_DEBUG
			log_i("wm.now=wmParaConfig wm.last=wmMoveReady");
			#endif
		}else if((Ws->moveReady & moveReadySok) != 0){
			//skip to wmLoopctrl
			if((Ws->loopCtrl & loopCtrlSreceive) != 0){
				Wm->last = Wm->now;
				Wm->now = wmLoopctrl;
			#ifdef WORK_MODE_DEBUG
				log_i("wm.now=wmLoopctrl wm.last=wmMoveReady");
			#endif
			}
			//skip to wmCalibrate
			else if((Ws-> calibrate & calibrateSreceive) != 0){
				Wm->last = Wm->now;
				Wm->now = wmCalibrate;
			#ifdef WORK_MODE_DEBUG
				log_i("wm.now=wmCalibrate wm.last=wmMoveReady");
			#endif
			}
		}
		break;
	case wmCalibrate:
		/***state mointor and switch***/
		//receive parameter configration
		if(mySemaphore.paraconfig.write == 1 || mySemaphore.paraconfig.set == 1){
			mySemaphore.paraconfig.write = 0;
			mySemaphore.paraconfig.set = 0;
		}


		/*****mode mointor and switch*****/
		//skip to emergency
		if((Ws->emergency & emergencySexecuting) != 0){
			Wm->last = Wm->now;
			Wm->now = wmEmergency;
			#ifdef WORK_MODE_DEBUG
			log_i("wm.now=wmEmergency wm.last=wmCalibrate");
			#endif
		}
		//skip to wmMoveReady
		else if((Ws->calibrate & calibrateScomplete) != 0){
			Wm->last = Wm->now;
			Wm->now = wmMoveReady;
			#ifdef WORK_MODE_DEBUG
			log_i("wm.now=wmMoveReady wm.last=wmCalibrate");
			#endif
		}
		break;
	case wmLoopctrl:
		/***state mointor and switch***/
		if(Ws->loopCtrl == loopCtrlSreceive){
			Ws->loopCtrl &= (~loopCtrlSreceive);
		}
		//receive parameter configration
		if(mySemaphore.paraconfig.write == 1 || mySemaphore.paraconfig.set == 1){
			mySemaphore.paraconfig.write = 0;
			mySemaphore.paraconfig.set = 0;
		}
		CommandUpdate();  //一些状态的重置，应该分开，在cyclectrl、dyn、ext等
		//loopctrl complete
		if(AL.complete_state.flag == 1){
			Ws->loopCtrl = loopCtrlScomplete;
		}else{
			Ws->loopCtrl |= loopCtrlSmove;
			// if(speedPose.filter > 1.0E-5f){
			if(speedPose.filter > 1.0E-5f){
				Ws->loopCtrl |= loopCtrlSup;
			}else{
				Ws->loopCtrl &= (~loopCtrlSup);
			}

			if(speedPose.filter < -(1.0E-5f)){
				Ws->loopCtrl |= loopCtrlSdown;
			}else{
				Ws->loopCtrl &= (~loopCtrlSdown);
			}
		}

		/*****mode mointor and switch*****/
		//skip to emergency
		if((ws.emergency & 0x00FC) != 0){
			Wm->last = Wm->now;
			Wm->now = wmEmergency;
			#ifdef WORK_MODE_DEBUG
			log_i("wm.now=wmEmergency wm.last=wmLoopctrl");
			#endif
		}
		//2025-3-4 00:04:16：不进行限位的紧急跳转
		// else if((ws.emergency & emergencyLimitUpper) != 0){
		// 	Wm->last = Wm->now;
		// 	Wm->now = wmEmergency;
		// 	log_i("wm.now=wmEmergency wm.last=loopCtrlSup");
		// }else if((ws.emergency & emergencyLimitDown) != 0){
		// 	Wm->last = Wm->now;
		// 	Wm->now = wmEmergency;
		// 	log_i("wm.now=wmEmergency wm.last=loopCtrlSdown");
		// }
		//skip to wmMoveReady
		else if((Ws->loopCtrl & loopCtrlScomplete) != 0){
			Wm->last = Wm->now;
			Wm->now = wmMoveReady;
			#ifdef WORK_MODE_DEBUG
			log_i("wm.now=wmMoveReady wm.last=wmLoopctrl");
			#endif
		}
		break;
	case wmEmergency:
		//emergency executing
		if(Ws->emergency > (emergencySexecuting + emergencyScomplete) 
			&& (Ws->emergency & 0xF00) == 0x00	//区分是紧急模式还是限位模式，在这里限位不需要处理，限位会在输出的地方直接限制输出。
			&& (Ws->emergency & emergencyScomplete) == 0
			&& (Ws->emergency & emergencySexecuting) == 0){
			AL.emergency_state.counter = 0;

			log_i("emergency reason:%4X",Ws->emergency);
			Ws->emergency |= emergencySexecuting;
			Emergency_Process();//配置紧急处理模式
			log_i("emergency executing");
		}
		else if((Ws->emergency & emergencySexecuting) != 0){
			AL.emergency_state.counter++;
			if(AL.emergency_state.counter >= 1000){
				AL.emergency_state.counter = 0;
				//emergency process complete
				Ws->emergency = emergencyScomplete;
				AL.command = 0x00;
				AL.complete_state.flag = 1;
				log_i("emergency emergencyScomplete");
			}
		}

		//receive parameter configration
		if(mySemaphore.paraconfig.write == 1 || mySemaphore.paraconfig.set == 1){
			mySemaphore.paraconfig.write = 0;
			mySemaphore.paraconfig.set = 0;
		}
		
		/*****mode mointor and switch*****/
		//skip to wmMoveReady
		if((Ws->emergency & emergencyScomplete) != 0){
			Ws->emergency = 0;
			Wm->last = Wm->now;
			Wm->now = wmMoveReady;
			log_i("wm.now=wmMoveReady wm.last=wmEmergency");
		}
		//skip to wmMoveRQ
		if((Ws->moveRQ & moveRQSenable) != 0){
			Wm->last = wmEmergency;
			Wm->now = wmMoveRQ;
			log_i("wm.now=wmMoveRQ wm.last=wmEmergency");
		}
		break;
	case wmMoveRQ://Forget about moveRQ for a moment, because there is no moveRQ button on the system.
		
		/*****mode mointor and switch*****/
		//skip to emergency
		if((Ws->moveRQ & moveRQSenable) == 0){
			Wm->last = wmEmergency;
			Wm->now = wmMoveReady;
			log_i("wm.now=wmMoveReady wm.last=wmEmergency");
		}
		break;
	default:
		break;
	}


}


void ctrstruct_update(void);
void sensorIntgrUpdate(void);
void ethernetConfigUpdate(void);
void servoParamUpdate(void);

/**
 * @note	Fun16 includes Fun_162
 * @brief in paraConfig wrok mode,update the parameter configration and judge the work state of paraconfig 
 * @param set paraconfig set semaphore 
 * @param write paraconfig write semaphore 
 * @param Ws WORK_STATE_STRUCT*
*/
void paraConfigSem_Process(uint8_t* set,uint8_t* write,WORK_STATE_STRUCT* Ws){
		if(*set == 1){
			switch (commBuf.Command){
			case DOPEPOSPID:
				switch(commBuf.MoveCtrl)
				{
				case POS_MODE:
					posAllCtrlPara.PosPid.P = commBuf.Pos.Pid.P;
					posAllCtrlPara.PosPid.I = commBuf.Pos.Pid.I;
					posAllCtrlPara.PosPid.D = commBuf.Pos.Pid.D;
					mySemaphore.pid.pos = 1;
					break;
				case LOAD_MODE:
					loadAllCtrlPara.PosPid.P = commBuf.Load.Pid.P;
					loadAllCtrlPara.PosPid.I = commBuf.Load.Pid.I;
					loadAllCtrlPara.PosPid.D = commBuf.Load.Pid.D;
					mySemaphore.pid.load = 1;
					break;
				case EXTEN_MODE:
					extAllCtrlPara.PosPid.P = commBuf.Ext.Pid.P;
					extAllCtrlPara.PosPid.I = commBuf.Ext.Pid.I;
					extAllCtrlPara.PosPid.D = commBuf.Ext.Pid.D;
					mySemaphore.pid.ext = 1;
					break;
				default:break;
				}
			    break;
			case DOPESPEEDPID:
				switch(commBuf.MoveCtrl){
				case POS_MODE:
					posAllCtrlPara.SpeedPid.P = commBuf.Pos.PidSpeed.P;
					posAllCtrlPara.SpeedPid.I = commBuf.Pos.PidSpeed.I;
					posAllCtrlPara.SpeedPid.D = commBuf.Pos.PidSpeed.D;
					//wait for adding speedpid semaphore
					break;
				case LOAD_MODE:
					loadAllCtrlPara.SpeedPid.P = commBuf.Load.PidSpeed.P;
					loadAllCtrlPara.SpeedPid.I = commBuf.Load.PidSpeed.I;
					loadAllCtrlPara.SpeedPid.D = commBuf.Load.PidSpeed.D;
					//wait for adding speedpid semaphore
					break;
				case EXTEN_MODE:
					extAllCtrlPara.SpeedPid.P = commBuf.Ext.PidSpeed.P;
					extAllCtrlPara.SpeedPid.I = commBuf.Ext.PidSpeed.I;
					extAllCtrlPara.SpeedPid.D = commBuf.Ext.PidSpeed.D;
					//wait for adding speedpid semaphore
					break;
				default:break;
				}
				break;
			case DOPEFEEDFORWARD:
				switch(commBuf.MoveCtrl){
				case POS_MODE:	
						posAllCtrlPara.Ffd.SpeedFFP = commBuf.Pos.Ffd.SpeedFFP;
						posAllCtrlPara.Ffd.PosDelay = commBuf.Pos.Ffd.PosDelay;
				break;
				case LOAD_MODE:  
						loadAllCtrlPara.Ffd.SpeedFFP = commBuf.Load.Ffd.SpeedFFP;
						loadAllCtrlPara.Ffd.PosDelay = commBuf.Load.Ffd.PosDelay;
				break;
				case EXTEN_MODE:	
						extAllCtrlPara.Ffd.SpeedFFP = commBuf.Ext.Ffd.SpeedFFP;
						extAllCtrlPara.Ffd.PosDelay = commBuf.Ext.Ffd.PosDelay;
				break;
				default:break;
				}
				break;
			default:break;
			}
			*set = 0;
			Ws->paraConfig = paraConfigScomplete;
		}
		if(*write == 1){
			switch (commBuf.Command){
			case DOPESETNOMINALACCSPEED:
				switch(commBuf.MoveCtrl){
				case POS_MODE:
					posAllCtrlPara.Nominal.Acc = commBuf.Pos.Nominal.Acc;
					posAllCtrlPara.Nominal.Speed = commBuf.Pos.Nominal.Speed;
					mySemaphore.mram.write_nominal_pos = 1;
				break;
				case LOAD_MODE:
					loadAllCtrlPara.Nominal.Acc = commBuf.Load.Nominal.Acc;
					loadAllCtrlPara.Nominal.Speed = commBuf.Load.Nominal.Speed;
					mySemaphore.mram.write_nominal_load = 1;
				break;
				case EXTEN_MODE:
					extAllCtrlPara.Nominal.Acc = commBuf.Pos.Nominal.Acc;
					extAllCtrlPara.Nominal.Speed = commBuf.Pos.Nominal.Speed;
					mySemaphore.mram.write_nominal_ext = 1;
				break;
				default:break;
				}
				break;
			case DOPEDEFAULTACC:
				switch(commBuf.MoveCtrl){
				case POS_MODE:	
					posAllCtrlPara.DefaultAcc = commBuf.Pos.DefaultAcc;
					mySemaphore.mram.write_defalutacc_pos = 1;
				break;
				case LOAD_MODE:
					loadAllCtrlPara.DefaultAcc = commBuf.Load.DefaultAcc;
					mySemaphore.mram.write_defalutacc_load = 1;
				break;
				case EXTEN_MODE:
					extAllCtrlPara.DefaultAcc = commBuf.Ext.DefaultAcc;
					mySemaphore.mram.write_defalutacc_ext = 1;
				break;
				default:break;
				}
				break;
			case DOPESPEEDLIMIT:
				switch(commBuf.MoveCtrl){
				case POS_MODE:
							posAllCtrlPara.Speed = commBuf.Pos.DefaultSpeed;
							mySemaphore.mram.write_speed_pos = 1;
				break;
				case LOAD_MODE:
							loadAllCtrlPara.Speed = commBuf.Load.DefaultSpeed;
							mySemaphore.mram.write_speed_load = 1;
				break;
				case EXTEN_MODE:
							extAllCtrlPara.Speed = commBuf.Ext.DefaultSpeed;
							mySemaphore.mram.write_speed_ext = 1;
				break;
				default:break;
				}
				break;
			case DOPEDESTWND:
				switch(commBuf.MoveCtrl)
				{
				case POS_MODE:
				log_d("mySemaphore.mram.write_wnd_pos = 1;");
							posAllCtrlPara.Wnd.WndSize = commBuf.Pos.Wnd.WndSize;
							posAllCtrlPara.Wnd.WndTime = commBuf.Pos.Wnd.WndTime;
							mySemaphore.mram.write_wnd_pos = 1;
				break;
				case LOAD_MODE:	
							loadAllCtrlPara.Wnd.WndSize = commBuf.Load.Wnd.WndSize;
							loadAllCtrlPara.Wnd.WndTime = commBuf.Load.Wnd.WndTime;
							mySemaphore.mram.write_wnd_load = 1;
				break;
				case EXTEN_MODE:
							extAllCtrlPara.Wnd.WndSize = commBuf.Ext.Wnd.WndSize;
							extAllCtrlPara.Wnd.WndTime = commBuf.Ext.Wnd.WndTime;
							mySemaphore.mram.write_wnd_ext = 1;
				break;
				default:break;
				}
				break;
			case DOPESFT:
				switch(commBuf.MoveCtrl){
				case POS_MODE:
					posAllCtrlPara.Sft.UpperSft = commBuf.Pos.Sft.UpperSft;
					posAllCtrlPara.Sft.LowerSft = commBuf.Pos.Sft.LowerSft;
					posAllCtrlPara.Sft.Reaction = commBuf.Pos.Sft.Reaction;
					mySemaphore.mram.write_sft_pos = 1;
					break;
				case LOAD_MODE:
					loadAllCtrlPara.Sft.UpperSft = commBuf.Load.Sft.UpperSft;
					loadAllCtrlPara.Sft.LowerSft = commBuf.Load.Sft.LowerSft;
					loadAllCtrlPara.Sft.Reaction = commBuf.Load.Sft.Reaction;
					mySemaphore.mram.write_sft_load = 1;
					break;
				case EXTEN_MODE:
					extAllCtrlPara.Sft.UpperSft = commBuf.Ext.Sft.UpperSft;
					extAllCtrlPara.Sft.LowerSft = commBuf.Ext.Sft.LowerSft;
					extAllCtrlPara.Sft.Reaction = commBuf.Ext.Sft.Reaction;
					mySemaphore.mram.write_sft_ext = 1;
					break;
				default:break;
				}
				break;
			case DOPEWRPOSPID:
				switch(commBuf.MoveCtrl){
				case POS_MODE:
					if(commBuf.HighPressure == 0){
					posAllCtrlPara.PosPid.P = commBuf.Pos.Pid.P;
					posAllCtrlPara.PosPid.I = commBuf.Pos.Pid.I;
					posAllCtrlPara.PosPid.D = commBuf.Pos.Pid.D;
					mySemaphore.pid.pos = 1;
					mySemaphore.mram.write_pospid_pos = 1;
					}
					else{
					commBuf.Pos.Pid_Hp.P = commBuf.Pos.Pid_Hp.P;
					commBuf.Pos.Pid_Hp.I = commBuf.Pos.Pid_Hp.I;
					commBuf.Pos.Pid_Hp.D = commBuf.Pos.Pid_Hp.D;
					//wait for adding semaphore

					}
					break;
				case LOAD_MODE:
					if(commBuf.HighPressure == 0){
					loadAllCtrlPara.PosPid.P = commBuf.Load.Pid.P;
					loadAllCtrlPara.PosPid.I = commBuf.Load.Pid.I;
					loadAllCtrlPara.PosPid.D = commBuf.Load.Pid.D;
					mySemaphore.pid.load = 1;
					mySemaphore.mram.write_pospid_load = 1;
					}
					else{
					commBuf.Load.Pid_Hp.P = commBuf.Load.Pid_Hp.P;
					commBuf.Load.Pid_Hp.I = commBuf.Load.Pid_Hp.I;
					commBuf.Load.Pid_Hp.D = commBuf.Load.Pid_Hp.D;
					//wait for adding semaphore
					
					}
					break;
				case EXTEN_MODE:
					if(commBuf.HighPressure == 0){
					extAllCtrlPara.PosPid.P = commBuf.Ext.Pid.P;
					extAllCtrlPara.PosPid.I = commBuf.Ext.Pid.I;
					extAllCtrlPara.PosPid.D = commBuf.Ext.Pid.D;
					mySemaphore.pid.ext = 1;
					mySemaphore.mram.write_pospid_ext = 1;
					}
					else{
					commBuf.Ext.Pid_Hp.P = commBuf.Ext.Pid_Hp.P;
					commBuf.Ext.Pid_Hp.I = commBuf.Ext.Pid_Hp.I;
					commBuf.Ext.Pid_Hp.D = commBuf.Ext.Pid_Hp.D;
					//wait for adding semaphore
					
					}
					break;
				default:break;
				}
				break;
			case DOPEWRSPEEDPID:
				switch(commBuf.MoveCtrl)
				{
				case POS_MODE:
					if(commBuf.HighPressure == 0){
					posAllCtrlPara.SpeedPid.P = commBuf.Pos.PidSpeed.P;
					posAllCtrlPara.SpeedPid.I = commBuf.Pos.PidSpeed.I;
					posAllCtrlPara.SpeedPid.D = commBuf.Pos.PidSpeed.D;
					mySemaphore.mram.write_speedpid_pos = 1;
					}
					else{
					posAllCtrlPara.SpeedPid_Hp.P = commBuf.Pos.PidSpeed_Hp.P;
					posAllCtrlPara.SpeedPid_Hp.I = commBuf.Pos.PidSpeed_Hp.I;
					posAllCtrlPara.SpeedPid_Hp.D = commBuf.Pos.PidSpeed_Hp.D;
					}
					break;
				case LOAD_MODE:
					if(commBuf.HighPressure == 0){
					loadAllCtrlPara.SpeedPid.P = commBuf.Pos.PidSpeed.P;
					loadAllCtrlPara.SpeedPid.I = commBuf.Pos.PidSpeed.I;
					loadAllCtrlPara.SpeedPid.D = commBuf.Pos.PidSpeed.D;
					mySemaphore.mram.write_speedpid_load = 1;
					}
					else{
					loadAllCtrlPara.SpeedPid_Hp.P = commBuf.Pos.PidSpeed_Hp.P;
					loadAllCtrlPara.SpeedPid_Hp.I = commBuf.Pos.PidSpeed_Hp.I;
					loadAllCtrlPara.SpeedPid_Hp.D = commBuf.Pos.PidSpeed_Hp.D;
					}
					break;
				case EXTEN_MODE:
					if(commBuf.HighPressure == 0){
					extAllCtrlPara.SpeedPid.P = commBuf.Pos.PidSpeed.P;
					extAllCtrlPara.SpeedPid.I = commBuf.Pos.PidSpeed.I;
					extAllCtrlPara.SpeedPid.D = commBuf.Pos.PidSpeed.D;
					mySemaphore.mram.write_speedpid_ext = 1;
					}
					else{
					extAllCtrlPara.SpeedPid_Hp.P = commBuf.Pos.PidSpeed_Hp.P;
					extAllCtrlPara.SpeedPid_Hp.I = commBuf.Pos.PidSpeed_Hp.I;
					extAllCtrlPara.SpeedPid_Hp.D = commBuf.Pos.PidSpeed_Hp.D;
					}
					break;
				default:break;
				}
				break;
			case DOPEWRFEEDFORWARD:
				switch(commBuf.MoveCtrl)
				{
				case POS_MODE:
					if(commBuf.Pos.HighPressure == 0)
					{
					posAllCtrlPara.Ffd.SpeedFFP = commBuf.Pos.Ffd.SpeedFFP;
					posAllCtrlPara.Ffd.PosDelay = commBuf.Pos.Ffd.PosDelay;
					mySemaphore.mram.write_ffd_pos = 1;
					}
					else
					{
					posAllCtrlPara.Ffd_Hp.SpeedFFP = commBuf.Pos.Ffd_Hp.SpeedFFP;
					posAllCtrlPara.Ffd_Hp.PosDelay = commBuf.Pos.Ffd_Hp.PosDelay;
					}
				break;
				case LOAD_MODE:  
					if(commBuf.Load.HighPressure == 0)
					{
					loadAllCtrlPara.Ffd.SpeedFFP = commBuf.Load.Ffd.SpeedFFP;
					loadAllCtrlPara.Ffd.PosDelay = commBuf.Load.Ffd.PosDelay;
					mySemaphore.mram.write_ffd_load = 1;
					}
					else
					{
					loadAllCtrlPara.Ffd_Hp.SpeedFFP = commBuf.Load.Ffd_Hp.SpeedFFP;
					loadAllCtrlPara.Ffd_Hp.PosDelay = commBuf.Load.Ffd_Hp.PosDelay;
					}
				break;
				case EXTEN_MODE:	
					if(commBuf.Ext.HighPressure == 0)
					{
					extAllCtrlPara.Ffd.SpeedFFP = commBuf.Ext.Ffd.SpeedFFP;
					extAllCtrlPara.Ffd.PosDelay = commBuf.Ext.Ffd.PosDelay;
					mySemaphore.mram.write_ffd_ext = 1;
					}
					else
					{
					extAllCtrlPara.Ffd_Hp.SpeedFFP = commBuf.Ext.Ffd_Hp.SpeedFFP;
					extAllCtrlPara.Ffd_Hp.PosDelay = commBuf.Ext.Ffd_Hp.PosDelay;
					}
				break;
				default:break;
				}
				break;
			case WROUTPUTPARA:
				//输出参数更新
				output_param_update();
				//释放EEPROM存储信号量
				mySemaphore.mram.write_outputpara = 1;
				break;
			case WRETHPARA:
				ethernetConfigUpdate();
				break;
			case WRSYSPARA:
				ctrstruct_update();//系统控制结构更新
				break;
			case WRSERVOPARA:
				servoParamUpdate();
				break;
			case DOPEINTGR:
				sensorIntgrUpdate();
				break;
			default:break;
			}
			*write = 0;
			Ws->paraConfig = paraConfigScomplete;
		}
		if(*write == 1 || *set == 1){
			Ws->paraConfig = paraConfigSreceive;
		}
}

/* Fun_166 */
void ctrstruct_update(void){
	SysPara.ControllerStructure = commBuf.SysPara.ControllerStructure;
	//释放MRAM存储信号量
	mySemaphore.mram.write_syspara = 1;
}

void sensorIntgrUpdate(void){
	SenData[sensorConnector].sensorIntgr = senDataCommbuf[sensorConnector].sensorIntgr;
	//释放MRAM存储信号量
	mySemaphore.mram.write_intgr = 1;
}

void ethernetConfigUpdate(void){
	//释放MRAM存储信号量
	mySemaphore.mram.writeEthConfig = 1;
}

void servoParamUpdate(void){
	servoParam.electronicGear = commBuf.servoParam.electronicGear;
	servoParam.encoderResolution = commBuf.servoParam.encoderResolution;
	servoParam.analogGain = commBuf.servoParam.analogGain;
	servoParamCalcu(&servoParam);
	//释放MRAM存储信号量
	mySemaphore.mram.writeServoParam = 1;
}

/*
紧急情况包括：
1.正在执行运动控制指令  系统SetCtrlOff DoPEOff CloseLink触发
2.正在执行运动控制指令  w5500检测到以太网连接断开（包括established断开 和 phy物理层断连，即非established状态）
3.运动指令执行出错

判断出三种情况，执行HALT指令，使执行器平稳停下
1.难点：SetCtrlOff DoPEOff CloseLink触发，如果系统在运动，并不能第一时间切换系统到对应状态，应该要等HALT执行完毕后再切换
2.难点：将w5500状态传递 并且直到紧急停止过程完成，不再受以太网状态影响
3.难点：如何判断指令出错 ①位置误差过大？ ②PWM过大 ③瞬间指令变化过大

2023年10月31日20:26:50 先实现1. 
*/

void Emergency_Halt(void)
{
	commBuf.Command = DOPEHALT;//紧急停止
	commBuf.MoveCtrl = AL.movectrl;//保持之前的控制状态 ***
	switch(commBuf.MoveCtrl)
	{
		case POS_MODE:
			commBuf.Acc = posAllCtrlPara.DefaultAcc;
			commBuf.Dec = -posAllCtrlPara.DefaultAcc;
		break;
		case LOAD_MODE:
			commBuf.Acc = loadAllCtrlPara.DefaultAcc;
			commBuf.Dec = -loadAllCtrlPara.DefaultAcc;
		break;
		case EXTEN_MODE:
			commBuf.Acc = extAllCtrlPara.DefaultAcc;
			commBuf.Dec = -extAllCtrlPara.DefaultAcc;
		break;
	}
	mySemaphore.move.pgInit = 1;
}

/**
 * @brief Force halt directly by zeroing the output
 * 
*/
void emergencyForceHalt(void)
{
	//以下的这些操作 总结起来是HALTS函数，应该去实现调用
	//Clear all output
	svPWM.pos = 0;
	svPWM.load = 0;
	svPWM.ext = 0;
	svAO.pos = 0;
	svAO.load = 0;
	svAO.ext = 0;
	//SON OFF
	DO.SON = GPIO_PIN_RESET;
	//clear pid
	pidInitPos();
	 pid_pos.calcuRun = 0;
	pidInitLoad();
	 pid_load.calcuRun = 0;
	pidInitExt();
	 pid_ext.calcuRun = 0;
	memset(&pg,0,sizeof(pg));
	pg.st = pose.orig;
	pg.s0 = pg.st;
	pg.s3 = pg.st;
	pg.st_last = pg.st;
	pg.v0 = 0;
	memset(&pgLoad,0,sizeof(pgLoad));
	pgLoad.st = force.filter;
	pgLoad.s0 = pgLoad.st;
	pgLoad.s3 = pgLoad.st;
	pgLoad.st_last = pgLoad.st;
	pgLoad.v0 = 0;
	//speed filter clear
	filter.speed_temp = 0;
	speedPose.filter = 0;
	memset(&filter.speed_buf,0,sizeof(filter.speed_buf));
}

/* Fun_1321 */
void Emergency_Monitor(void)//emergency mointor
{
	EmergencyErrorJudge();//emergency error judge
	//if error occurs, change work mode into wmEmergency
	//下一步是将以下内容，合理的放入wmEmergency中处理
}

/* Fun_1322 */
void Emergency_Process(void)//应该在创建一个紧急停止宏定义 便于后续完成停止 设置系统状态 HALT_EMERGENCY
{
	//Emergency_Halt();//紧急停止HALT指令
	emergencyForceHalt();//强制停止
	//还要添加当前指令相关参数和状态清零 2023/11/2
	//log_i("EmergencyError:0x%2x",AL.emergency_state.error);
}

void EmergencyErrorJudge(void)//紧急情况错误情况判断
{
	//loop ctrl moving
	if((ws.loopCtrl & loopCtrlSmove) != 0){
		//相关使能信号ERROR
		if(commBuf.OpenDevice_state == Device_Off){
			AL.emergency_state.error |= EMERGENCY_DEVICE_OFF;
			ws.emergency |= emergencyDeviceOffMoving;
		}else{
			ws.emergency &= (~emergencyDeviceOffMoving);
		}
		if(commBuf.DoPE_state == DoPE_Off){
			AL.emergency_state.error |= EMERGENCY_DOPE_OFF;
			ws.emergency |= emergencyDopeOffMoving;
		}else{
			ws.emergency &= (~emergencyDopeOffMoving);
		}
		if(commBuf.DoPESetCtrl == CtrlFALSE){
			AL.emergency_state.error |= EMERGENCY_CTRL_FALSE;
			ws.emergency |= emergencyCtrlFalseMoving;
		}else{
			ws.emergency &= (~emergencyCtrlFalseMoving);
		}

		//software limit error

		//deviation error	

	}
	//ETH NET error
	//网络故障信号ERROR
	if(stateFlag.eth_phy_state == PHY_LINK_OFF || stateFlag.eth_phy_state == PHY_LINK_UNKNOW )//phy off
	{
		AL.emergency_state.error |= EMERGENCY_PHY_OFF;
		ws.emergency |= emergencyPhyOff;
	}else{
		ws.emergency &= (~emergencyPhyOff);
	}
	if(stateFlag.eth_scoket_state != SOCK_ESTABLISHED)//socket unestablished
	{
		AL.emergency_state.error |= EMERGENCY_SOCKET_UNESTAB;	
		ws.emergency |= emergencySocketUnestablished;
	}else{
		ws.emergency &= (~emergencySocketUnestablished);
	}
	//Eth heartbeat lost
	if(stateFlag.lostEthHeartbeat == ETH_HEARTBEAT_LOST){
		AL.emergency_state.error |= EMERGENCY_ETH_LOST_HEARTBEAT;
		ws.emergency |= emergencyHeartBeatLost;
	}else{
		ws.emergency &= (~emergencyHeartBeatLost);
	}

	//hardware limit error
	if(DI.DI7.value == GPIO_PIN_RESET){
		AL.emergency_state.error |= EMERGENCY_HARDWARE_LIMIT_UPPER;
		ws.emergency |= emergencyLimitUpper;
		//log_i("emergencyLimitUpper");
	}else{
		ws.emergency &= (~emergencyLimitUpper);
	}
		
	if(DI.DI8.value == GPIO_PIN_RESET){
		AL.emergency_state.error |= EMERGENCY_HARDWARE_LIMIT_LOWER;
		ws.emergency |= emergencyLimitDown;
		//log_i("emergencyLimitDown");
	}else{
		ws.emergency &= (~emergencyLimitDown);
	}
}

/**
 * @brief system software reset
 */
void stm32SoftReset(void)
 {
  // __set_FAULTMASK(1);//禁止所有的可屏蔽中断
	 __set_PRIMASK(1);
   HAL_NVIC_SystemReset();//软件复位
 }

 

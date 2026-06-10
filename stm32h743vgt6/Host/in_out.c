#include "in_out.h"
#include "DI.h"
#include "DO.h"
#include "usart.h"
#include "tim.h"
#include "adc_scale.h"
#include "cmsis_os.h"
#include "Servo_driver.h"
#include "Encoder.h"
#include "control.h"
#include "gParameter.h"
#include "communicate.h"
#include "posGenerator.h"
#include "sensor.h"
#include "td.h"
#include "lsm.h"
#include <elog.h>
#include "adrc.h"
#include "CS5552.h"

HAL_OUTPUT hal_output;
DOTypeDef openloopDO;
MDSTRUCT pose;
MDSTRUCT speedPose;
MDSTRUCT bigDeformationUpper;
MDSTRUCT bigDeformationLower;
MDSTRUCT bigDeformationSum;
MDSTRUCT force;
MDSTRUCT forceLsm;
MDSTRUCT forceSL2;
MDSTRUCT speedForce;
MDSTRUCT strain1;
MDSTRUCT strain2;
MDSTRUCT strain1Lsm;
MDSTRUCT strain2Lsm;
MDSTRUCT speedStrain;
DIMAPSTRUCT DIMap_init[10];
DIMAPSTRUCT DIMap[10];
MANUAL_BOX_STRUCT manualBox;
static float filterMaxArrayLoad[FILTER_LEN] = {0};
static float filterTempLoad = 0;
static float filterMaxArrayStrain[FILTER_LEN] = {0};
static float filterTempStrain = 0;
static float filterMaxArrayStrain2[FILTER_LEN] = {0};
static float filterTempStrain2 = 0;

void SetTareMonitor(APP_LAYER_VARIABLE_STRUCT *p); // 浼犳劅鍣ㄥ€兼竻闆舵娴?
void output_param_update(void);					   // 杈撳嚭鍙傛暟鏇存柊
void output_switch(OUTPUTPARA *op);				   // output switch
void output_switch_openloop(OUTPUTPARA *op);	   // output switch openloop
void posespeed_filter(uint8_t filter_depth);
void loadCalcu(MDSTRUCT *force, APP_LAYER_VARIABLE_STRUCT *AL);
void load_filter(MDSTRUCT *x, uint16_t filter_depth);
void loadFilterWithLenSwitch(MDSTRUCT *x, FILTER *filter, uint16_t filter_depth);
void load_filter_SL2(uint16_t filter_depth);
void loadFilterLsm(MDSTRUCT *x, uint16_t filter_depth,const float _orig);
void rigidityLsmIdentify(uint16_t filter_depth,const float _xOrig,const float _yOrig);
void loadspeed_filter(uint16_t filter_depth);
uint16_t loadFilterSwitch(uint8_t Wm, uint16_t MaxFiterLen, uint16_t FilterDepth);
void extensometerCalcu(MDSTRUCT *_strain, APP_LAYER_VARIABLE_STRUCT *AL);
void extensometerFilterLsm(MDSTRUCT *x, uint16_t filter_depth,const float _orig);
uint16_t extensometerFilterSwitch(const uint8_t Wm,const uint16_t MaxFiterLen,const uint16_t FilterDepth);
void extensometerFilterWithLenSwitch(MDSTRUCT *_x,MDSTRUCT *_xLsm ,
									FILTER *_filter,const APP_LAYER_VARIABLE_STRUCT *_AL,
									const uint16_t _filter_depth);
void extensometer2Calcu(MDSTRUCT *_strain, APP_LAYER_VARIABLE_STRUCT *_AL);
void extensometer2FilterLsm(MDSTRUCT *x, uint16_t filter_depth,const float _orig);
uint16_t extensometer2FilterSwitch(const uint8_t Wm,const uint16_t MaxFiterLen,const uint16_t FilterDepth);
void extensometer2FilterWithLenSwitch(MDSTRUCT *_x,MDSTRUCT *_xLsm,
									FILTER *_filter,const APP_LAYER_VARIABLE_STRUCT *_AL,
									const uint16_t _filter_depth);
void sendata_pg_process(void);
void openloop_parameters_update(void);
void openloopOutputMap(OUTPUTPARA *op);
void closeloopOutputMap(OUTPUTPARA *op);
void SONMap(void);
void emergency_scram_button_process(void);
void DI_map(HAL_DI_STRUCT *x);
void DB9_map(HAL_DB9_STRUCT *x);
void ServoIO_map(HAL_DI_STRUCT *x);
void MD_CODE_map(uint8_t SensorMapNum[3]);
static void MD_ResetEngineeringValue(MDSTRUCT *md);
static uint8_t MD_SinglePointConfigReady(uint8_t connector);
static void MD_UpdateEngineeringValue(MDSTRUCT *md, uint8_t connector, const sensorData_t *sensorData);
/*DI Function map*/
void DIFuncMap_init(void);
void DIFunc_map(void);
//extensometer hardware protect
sensorProtectState_e sensorRangeProtect(const float _valueNS,const float _max,const float _min,sensorConnector_e _sensorConnetor);
void sftLimitMonitor(float _pose,float _force,float _strain1,float _strain2);
void sensorRangeMonitor(float _force,float _strain1,float _strain2);
void limitProtectProcess(void);
void bigDeformationCalcu(int32_t *_code1,int32_t *_code2);

void inputGetValue()
{
	// // Encoder
	// Encoder_Get(Encoder0, &encoder);
	// AI 鍦ㄥ閮ㄤ腑鏂腑鑾峰彇
	extern uint16_t time13Count;
	__HAL_TIM_SetCounter(&htim13, 0);
	// static uint8_t cs5552SampleDivider = 0;
	// if(++cs5552SampleDivider >= 40)
	// {
	// 	cs5552SampleDivider = 0;
	// 	CS5552_CompatDataGet(&cs5552_compat_ctx);
	// }
	// CS5552 sampling is moved to StartDefaultTask().
	time13Count = __HAL_TIM_GetCounter(&htim13);
	// DI
	DIDetect_ReadALL();
	// DB9
	DB9Detect_ReadALL();
}

void inputMapping(){
	// DI ordinary IO mapping
	DI_map(&halDI);
	// DI function map
	DIFunc_map();
	// Servo mapping
	ServoIO_map(&halDI);
	// DB9 ordinary IO mapping
	DB9_map(&halDB9);
	// CON ADJ monitor
	sensorCON_ADJ_Monitor(&DB9);
	// measure data code value mapping
	// MD_CODE_map(sensorCheck.MapNum);
	// sensor settare monitor
	SetTareMonitor(&AL);
	// pose calculate
	poseCalcu();
	// big deformation calculate
	bigDeformationCalcu(&bigDeformationUpper.code,&bigDeformationLower.code);
	// pose speed filter
	posespeed_filter(10);
	// load calculate
	loadCalcu(&force, &AL);
	// load filter lsm
	loadFilterLsm(&forceLsm, 500 ,force.orig);

	// switch load filter length
	// filterLen.loadLen = loadFilterSwitch(wm.now, FILTER_LEN, SenData[ch4Load].sensorIntgr); // SenData[ch4Load].sensorIntgr
	filterLen.loadLen = loadFilterSwitch(wm.now, FILTER_LEN, 20); // SenData[ch4Load].sensorIntgr
	// load filter with length switch
	loadFilterWithLenSwitch(&force, &filter, filterLen.loadLen);

	// //rigidity lsm identify
	rigidityLsmIdentify(100,pose.orig,force.filter);
	
	// load_filter_SL2(10);
	// extensometer calculate
	extensometerCalcu(&strain1, &AL);
	// extensometer filter lsm
	extensometerFilterLsm(&strain1Lsm, 500, strain1.orig);
	// switch strain filter length
	filterLen.strainLen = extensometerFilterSwitch(wm.now, STRAIN_FILTER_LEN, 20); // SenData[ch4Load].sensorIntgr
	// exten filter with length switch
	extensometerFilterWithLenSwitch(&strain1,&strain1Lsm,&filter,&AL,filterLen.strainLen);
	// extensometer2 calculate
	extensometer2Calcu(&strain2, &AL);
	// extensometer2 filter lsm
	extensometer2FilterLsm(&strain2Lsm, 500, strain2.orig);
	// switch strain filter length
	filterLen.strain2Len = extensometer2FilterSwitch(wm.now, STRAIN_FILTER_LEN, SenData[ch3Ext2].sensorIntgr); // SenData[ch4Load].sensorIntgr
	// exten filter with length switch
	extensometer2FilterWithLenSwitch(&strain2,&strain2Lsm,&filter,&AL,filterLen.strain2Len);
	/**************************************************************************************************
	//load_filter(&force,1);
	//x-pos y-load least square fast method
	//ls_slide_t.x_m = mean_fast(ls_filter_len,&ls_slide_t.i_xm,&ls_slide_t.x_mtemp,ls_slide_t.x_mbuf,++ls_slide_t.n);
	***************************************************************************************************/
	//load speed filter
	loadspeed_filter(1000);
	// pos generator update if sendata is updated
	sendata_pg_process();
	// software limit monitor
	//sftLimitMonitor(pose.orig,force.filter,strain1.filter,strain2.filter);
	// sensor range limit monitor
	//sensorRangeMonitor(float _force,float _strain1,float _strain2);
}

#define EXTEN_PROTECT_MAX 2.0f
#define EXTEN_PROTECT_MIN -2.0f
void outputMapping()
{
	// sensorProtectState_e extps = SENPS_noProtect;
	// openloop mapping
	if (stateFlag.openloop == 1) // OPENLOOP
		openloopOutputMap(&OutputPara);
	// closeloop maping
	else
		closeloopOutputMap(&OutputPara);
	// DO.SON Map
	SONMap();
	// emergency scram button process (use for testing)
	// emergency_scram_button_process();
	//extps = sensorRangeProtect(strain1.orig,EXTEN_PROTECT_MAX,EXTEN_PROTECT_MIN);
}
/* Fun鈥斺€?78 */
void outputControl()
{
	// upper and lower limit protect process
	limitProtectProcess();
	// Pulse ouput
	//if((AL.openloopFrq_Last != AL.openloopFrq) || (stateFlag.openloop == 0))
		Servo_mod_fclk(PULSE, hal_output.PWM);
	hal_output.PWM_last = hal_output.PWM;
	// PWM start enable
	if (hal_output.PWM == 0 && AL.complete_state.flag == 1)
	{
		Servo_PWM_Disable(PULSE);
		// hal_output.DO.SON = 0;
	}
	// SRV DIR
	(hal_output.DO.DIR == 1) ? DIR_SET : DIR_RESET;
	// SRV ENBALE
	(hal_output.DO.SON == 1) ? SRV_ON : SRV_OFF;

	// Darlington
	Darlington_Write(P_DO1_Pin, hal_output.DO.DO1);
	Darlington_Write(P_DO2_Pin, hal_output.DO.DO2);
	Darlington_Write(P_DO3_Pin, hal_output.DO.DO3);
	// Relay
	Relay_Write(P_DO4_RL_Pin, hal_output.DO.DO4_RL);
	Relay_Write(P_DO5_RL_Pin, hal_output.DO.DO5_RL);

}

/**
 * @brief ADC code map
 */
static void MD_ResetEngineeringValue(MDSTRUCT *md)
{
	if (md == NULL)
		return;

	md->orig_last = md->orig;
	md->orig = 0.0f;
	md->origTare = 0.0f;
}

static uint8_t MD_SinglePointConfigReady(uint8_t connector)
{
	switch (connector)
	{
	case ch3Ext2:
		return (AL.ext2Ctrl.NominalSensitive != 0.0f) && (AL.ext2Ctrl.NominalValue != 0.0f);
	case ch4Load:
		return (AL.loadCtrl.NominalSensitive != 0.0f) && (AL.loadCtrl.NominalValue != 0.0f);
	default:
		return 0u;
	}
}

static void MD_UpdateEngineeringValue(MDSTRUCT *md, uint8_t connector, const sensorData_t *sensorData)
{
	if ((md == NULL) || (sensorData == NULL))
		return;

	md->orig_last = md->orig;

	if ((sensorData->LinPoint > 0u) && (sensorCalibrate.multipointCalibrate != NULL))
	{
		sensorCalibrate.multipointCalibrate(md->code, connector, sensorData, &md->orig);
		return;
	}

	if ((sensorCalibrate.singlepointCalibrate == NULL) || !MD_SinglePointConfigReady(connector))
	{
		md->orig = 0.0f;
		md->origTare = 0.0f;
		return;
	}

	sensorCalibrate.singlepointCalibrate(md->code, connector, sensorData, &md->orig);
}

void MD_CODE_map(uint8_t SensorMapNum[3])
{
	if (SensorMapNum == NULL)
	{
		force.code = 0;
		strain1.code = 0;
		strain2.code = 0;
		MD_ResetEngineeringValue(&force);
		MD_ResetEngineeringValue(&strain1);
		MD_ResetEngineeringValue(&strain2);
		return;
	}

	if (SensorMapNum[0] == SENSOR_NO_CHANNEL)
	{
		force.code = 0;
		MD_ResetEngineeringValue(&force);
	}
	else
	{
		force.code = cs5552_compat_ctx.Code[CS5552_COMPAT_CHANNEL_FORCE] - AL.tare.value[ch4Load];
		MD_UpdateEngineeringValue(&force, ch4Load, &SenData[ch4Load]);
	}

	/*
	 * Current CS5552 migration does not use chip0.ch0,
	 * so strain1 is intentionally kept at zero for now.
	 */
	strain1.code = 0;
	MD_ResetEngineeringValue(&strain1);

	if (SensorMapNum[2] == SENSOR_NO_CHANNEL)
	{
		strain2.code = 0;
		MD_ResetEngineeringValue(&strain2);
	}
	else
	{
		strain2.code = cs5552_compat_ctx.Code[CS5552_COMPAT_CHANNEL_STRAIN2] - AL.tare.value[ch3Ext2];
		MD_UpdateEngineeringValue(&strain2, ch3Ext2, &SenData[ch3Ext2]);
	}

	//pose.code = encdr->count0;
}

/**
 * @brief DI map
 * @param[in] x hal DI
 */
void DI_map(HAL_DI_STRUCT *x)
{
	DI.DI2.value = x->DI2.value;
	DI.DI2.timPress = x->DI2.pressTime;
	DI.DI2.timUnPress = x->DI2.unpressTime;
	DI.DI3.value = x->DI3.value;
	DI.DI3.timPress = x->DI3.pressTime;
	DI.DI3.timUnPress = x->DI3.unpressTime;
	DI.DI4.value = x->DI4.value;
	DI.DI4.timPress = x->DI4.pressTime;
	DI.DI4.timUnPress = x->DI4.unpressTime;
	DI.DI5.value = x->DI5.value;
	DI.DI5.timPress = x->DI5.pressTime;
	DI.DI5.timUnPress = x->DI5.unpressTime;
	DI.DI6.value = x->DI6.value;
	DI.DI6.timPress = x->DI6.pressTime;
	DI.DI6.timUnPress = x->DI6.unpressTime;
	DI.DI7.value = x->DI7.value;
	DI.DI7.timPress = x->DI7.pressTime;
	DI.DI7.timUnPress = x->DI7.unpressTime;
	DI.DI8.value = x->DI8.value;
	DI.DI8.timPress = x->DI8.pressTime;
	DI.DI8.timUnPress = x->DI8.unpressTime;
	DI.DI9.value = x->DI9.value;
	DI.DI9.timPress = x->DI9.pressTime;
	DI.DI9.timUnPress = x->DI9.unpressTime;
	DI.DI10.value = x->DI10.value;
	DI.DI10.timPress = x->DI10.pressTime;
	DI.DI10.timUnPress = x->DI10.unpressTime;
	DI.ALM.value = x->ALM.value;
	DI.ALM.timPress = x->ALM.pressTime;
	DI.ALM.timUnPress = x->ALM.unpressTime;
	if(manualBox.type == manualBoxDI_Type)
		DI.ALL = x->ALL;
	else if (manualBox.type == manualBox485_Type){
		DI.ALL = (x->ALL & 0xFFC0) + (manualBoxCoil.word & 0x003F);
	}
	// emergency button map
	//emerency_struct.scram_button = (uint8_t)x->DI4.value;
}

/**
 * @brief ServoIO map
 * @param[in] x hal DI
 */
void ServoIO_map(HAL_DI_STRUCT *x)
{
	DI.ALM.value = x->ALM.value;
}

/**
 * @brief DB9 IO map
 * @param[in] x hal DIB9 IO
 */
void DB9_map(HAL_DB9_STRUCT *x)
{
#if USE_DB9_CON
	DB9.CON1.value = x->CON1.value;
	DB9.CON1.timPress = x->CON1.pressTime;
	DB9.CON1.timUnPress = x->CON1.unpressTime;
	DB9.CON2.value = x->CON2.value;
	DB9.CON2.timPress = x->CON2.pressTime;
	DB9.CON2.timUnPress = x->CON2.unpressTime;
	DB9.CON3.value = x->CON3.value;
	DB9.CON3.timPress = x->CON3.pressTime;
	DB9.CON3.timUnPress = x->CON3.unpressTime;
#else
	DB9.CON1.value = x->CON1.value;
	DB9.CON1.timPress = 0;
	DB9.CON1.timUnPress = 0;
	DB9.CON2.value = x->CON2.value;
	DB9.CON2.timPress = 0;
	DB9.CON2.timUnPress = 0;
	DB9.CON3.value = x->CON3.value;
	DB9.CON3.timPress = 0;
	DB9.CON3.timUnPress = 0;
#endif

	DB9.ADJ1.value = x->ADJ1.value;
	DB9.ADJ1.timPress = x->ADJ1.pressTime;
	DB9.ADJ1.timUnPress = x->ADJ1.unpressTime;
	DB9.ADJ2.value = x->ADJ2.value;
	DB9.ADJ2.timPress = x->ADJ2.pressTime;
	DB9.ADJ2.timUnPress = x->ADJ2.unpressTime;
	DB9.ADJ3.value = x->ADJ3.value;
	DB9.ADJ3.timPress = x->ADJ3.pressTime;
	DB9.ADJ3.timUnPress = x->ADJ3.unpressTime;
	DB9.ALL = x->ALL;
}

/**
 * @note	Fun_153
 * @brief	monitor if tare.flag is true,then set tare for relevant sensor
 * @param	p type:APP_LAYER_VARIABLE_STRUCT*
 * @retval	null
 */
void SetTareMonitor(APP_LAYER_VARIABLE_STRUCT *p)
{
	if (p->tare.flag == 1 && p->tare.connector < CONNECTOR_NUM_MAX)
	{
		switch (p->tare.connector)
		{
		case 0x00: // 浣嶇Щ
			pose.code = sensorCalibrate.zeroCode[ch0Pose];
			log_i("SetTare pos.code:%d",pose.code);
			encoder.count0 = sensorCalibrate.zeroCode[ch0Pose];
			__HAL_TIM_SetCounter(&htim1, 30000);
			memset(filter.speed_buf, 0, sizeof(filter.speed_buf));
			filter.speed_temp = 0;
			memset(&pg, 0, sizeof(pg));
			// leso reference parameters clear
			leso.z1 = 0;
			leso.z2 = 0;
			leso.fe1 = 0;
			leso.fe2 = 0;
			break;
		case 0x01:				  // 澶у彉褰?
			p->tare.value[ch1Bd] = 0 - sensorCalibrate.zeroCode[ch1Bd]; // 淇濈暀
			p->tare.fValue[ch1Bd] = 0;
			break;
		case 0x02: // 寮曚几璁?
			//p->tare.value[ch2Ext1] = strain1.code - sensorCalibrate.zeroCode[ch2Ext1];
			p->tare.fValue[ch2Ext1] = strain1.filterTrans + p->tare.fValue[ch2Ext1];//Find the current true value before taring
			//p->tare.fValue[ch2Ext1] = strain1.origTare;
			
			memset(filterMaxArrayStrain, 0, sizeof(filterMaxArrayStrain));
			filterTempStrain = 0;
			memset(filter.strain_buf, 0, sizeof(filter.strain_buf));
			filter.strain_temp = 0;
			memset(filter.speed_buf_strain, 0, sizeof(filter.speed_buf_strain));
			filter.speedStrain_temp = 0;
			memset(&pgExt, 0, sizeof(pgExt));
			break;
		case 0x03: // 鎵╁睍1
			//p->tare.value[ch3Ext2] = strain2.code - sensorCalibrate.zeroCode[ch3Ext2];
			p->tare.fValue[ch3Ext2] = strain2.filterTrans + p->tare.fValue[ch3Ext2];//Find the current true value before taring
			//p->tare.fValue[ch3Ext2] = strain2.origTare;
			break;
		case 0x04: // 杞借嵎
			//p->tare.value[ch4Load] = force.code - sensorCalibrate.zeroCode[ch4Load];
			p->tare.fValue[ch4Load] = force.filterTrans + p->tare.fValue[ch4Load];//Find the current true value before taring
			//p->tare.fValue[ch4Load] = force.origTare;
			//p->tare.fValue[ch4Load] = force.filterTransTare;
			// force.code;
			memset(filterMaxArrayLoad, 0, sizeof(filterMaxArrayLoad));
			filterTempLoad = 0;
			memset(filter.force_buf, 0, sizeof(filter.force_buf));
			filter.force_temp = 0;
			memset(filter.speed_buf_load, 0, sizeof(filter.speed_buf_load));
			filter.speedload_temp = 0;
			memset(&pgLoad, 0, sizeof(pgLoad));
			break;
		case ch5Ext1Code:
			p->tare.value[ch2Ext1] = strain1.code - sensorCalibrate.zeroCode[ch2Ext1] + p->tare.value[ch2Ext1];
			printf("SetTare ext1.code:%d\r\n",p->tare.value[ch2Ext1]);
			break;
		case ch6Ext2Code:
			p->tare.value[ch3Ext2] = strain2.code - sensorCalibrate.zeroCode[ch3Ext2] + p->tare.value[ch3Ext2];
			printf("SetTare ext2.code:%d\r\n",p->tare.value[ch3Ext2]);
			break;
		case ch7LoadCode:
			p->tare.value[ch4Load] = force.code - sensorCalibrate.zeroCode[ch4Load] + p->tare.value[ch4Load];
			printf("SetTare force.code:%d\r\n",p->tare.value[ch4Load]);
			break;
		}
	}
	p->tare.flag = 0;
}

/**
 * @brief	pose calculate from Encoder
 * @param	null
 * @retval	null
 * @date	2024-3-5 20:11:40
 * @author	Pan
 */
void poseCalcu(void){
	pose.orig_last = pose.orig;
	//pose.orig = AL.posCtrl.sign * pose.code * AL.posCtrl.NominalSensitive; // calculate encoder0 movement pulse
	if(SenData[ch0Pose].LinPoint > 0){
		sensorCalibrate.multipointCalibrate(pose.code,ch0Pose,&SenData[ch0Pose],&pose.orig);
	}
	else
		sensorCalibrate.singlepointCalibrate(pose.code,ch0Pose,&SenData[ch0Pose],&pose.orig);
}

/**
 * @brief	big deformation calculate from Encoder
 * @param	_code1 big deformation upper
 * @param	_code2 big deformation lower
 * @retval	null
 * @date	2025-3-21 17:25:01
 * @author	Pan
 */
void bigDeformationCalcu(int32_t *_code1,int32_t *_code2){
	bigDeformationUpper.orig_last = bigDeformationUpper.orig;
	bigDeformationLower.orig_last = bigDeformationLower.orig;
	bigDeformationSum.orig_last = bigDeformationSum.orig;
	//pose.orig = AL.posCtrl.sign * pose.code * AL.posCtrl.NominalSensitive; // calculate encoder0 movement pulse
	bigDeformationUpper.orig =  *_code1 * 0.01215;
	bigDeformationLower.orig =  *_code2 * 0.0056844;
	bigDeformationSum.orig = bigDeformationUpper.orig + bigDeformationLower.orig;
}

/**
 * @brief	pose calculate from Encoder in interrput
 * @param[out] pose	-struct of MDSTRUCT
 * @param[in] count -encoder count in delta time
 * @retval	null
 * @date	2024-3-5 20:11:40
 * @author	Pan
 */
void poseCodeCalculate_Int(MDSTRUCT *pose,const int32_t count)
{
	pose->code_last = pose->code;
	pose->code = count;
	pose->codeErr = pose->code - pose->code_last;
}

/**
 * @brief	pose speed filter
 * @param	filter_depth type:uint8_t
 * @retval	null
 * @date	2024-3-5 20:11:40
 * @author	Pan
 */
void posespeed_filter(uint8_t filter_depth)
{
	// static uint8_t i = 0;
	 speedPose.filter_last = speedPose.filter;
	// i++;
	// if (i >= filter_depth)
	// 	i = 0;
	// filter.speed_temp -= filter.speed_buf[i];
	// filter.speed_buf[i] = (pose.orig - pose.orig_last) / AL.delta; // 璁＄畻閫熷害 r/s
	// filter.speed_temp += filter.speed_buf[i];
	// speedPose.orig = filter.speed_buf[i];
	// speedPose.filter = filter.speed_temp / (filter_depth * 1.0f); // 璁＄畻閫熷害 r/s

	speedPose.codeFiter = filter_Int.speed_temp / filter_depth; // 璁＄畻閫熷害 r/s
	speedPose.filter = speedPose.codeFiter*1.0f / (AL.posCtrl.sign*1.0f) *AL.posCtrl.NominalSensitive;


	if (fabs(speedPose.filter) <= 1.0E-7)
		speedPose.filter = 0.0f;
}

/**
 * @brief	pose speed filter in interrupt
 * @param	filter_depth type:uint8_t
 * @param[inout]	filter
 * @param[in]		pose
 * @param[out]		speedPose
 * @retval	null
 * @date	2024-11-8 14:40:49
 * @author	Pan
 */
void poseSpeedFilter_Int(FILTER_INT *_filter,MDSTRUCT *_pose,MDSTRUCT *_speedPose,const uint8_t _filter_depth)
{
	static uint8_t i = 0;
	_speedPose->code_last = _speedPose->code;
	if (++i >= _filter_depth)
		i = 0;
	_filter->speed_temp -= _filter->speed_buf[i];
	_filter->speed_buf[i] = _pose->codeErr * 2000; // 璁＄畻閫熷害 r/s  in 2k
	_filter->speed_temp += _filter->speed_buf[i];
	_speedPose->code = _filter->speed_buf[i];
}

/**
 * @brief	load calculate from ADC code
 * @param[in]	AL type:APP_LAYER_VARIABLE_STRUCT*
 * @retval	null
 * @date	2024-3-5 20:11:40
 * @author	Pan
 */
void loadCalcu(MDSTRUCT *force, APP_LAYER_VARIABLE_STRUCT *AL)
{
	(void)AL;
	MD_UpdateEngineeringValue(force, ch4Load, &SenData[ch4Load]);
}

/**
 * @brief	load lsm filter
 * @param	x the object needed to filter
 * @param	filter_depth type:uint16_t
 * @retval	null
 * @date	2024-3-5 20:11:40
 * @author	Pan
 */
void loadFilterLsm(MDSTRUCT *x, uint16_t filter_depth,const float _orig)
{
	x->orig = _orig;
	x->filter_last = x->filter;
	ls_slide_t.y_m = mean_fast(filter_depth, &ls_slide_t.i_ym, &ls_slide_t.y_mtemp, ls_slide_t.y_mbuf, x->orig);
	lsm_pre_load.y_pre_last = lsm_pre_load.y_pre;
	// printf("x->orig:%f",x->orig);
	lsm_pre_load.y_pre = least_square_fast(filter_depth, &ls_slide_t, 0, x->orig);
	x->filter = lsm_pre_load.y_pre;
}

/**
 * @brief	rigidity lsm identify 
 * @param	x the object horizontal axis
 * @param	y the object vertical axis
 * @param	filter_depth type:uint16_t
 * @retval	null
 * @date	2024-3-5 20:11:40
 * @author	Pan
 */
void rigidityLsmIdentify(uint16_t filter_depth,const float _xOrig,const float _yOrig)
{
	static uint8_t _periodCounter = 0;
	if(++_periodCounter > 10){
		_periodCounter = 1;
		lsRigidity.x_m = mean_fast(filter_depth, &lsRigidity.i_xm, &lsRigidity.x_mtemp, lsRigidity.x_mbuf, _xOrig);
		lsRigidity.y_m = mean_fast(filter_depth, &lsRigidity.i_ym, &lsRigidity.y_mtemp, lsRigidity.y_mbuf, _yOrig);
		// printf("x->orig:%f",x->orig);
		leastSquareStandard(filter_depth, &lsRigidity, _xOrig, _yOrig);
		lsRigidity.kFilter = lsRigidity.kFilter*0.7 + lsRigidity.k*0.3; 
	}
}

/**
 * @brief	load filter
 * @param	filter_depth type:uint16_t
 * @retval	null
 * @date	2024-3-5 20:11:40
 * @author	Pan
 */
void load_filter(MDSTRUCT *x, uint16_t filter_depth)
{
	static uint16_t i = 0;
	x->filter_last = x->filter;
	i++;
	if (i >= filter_depth)
		i = 0;
	filterSL.force_temp -= filterSL.force_buf[i];
	filterSL.force_buf[i] = force.orig;
	filterSL.force_temp += filterSL.force_buf[i];
	x->filter = filterSL.force_temp / (filter_depth * 1.0f);
}

/**
 * @brief	load filter sigmod len
 * @param	Wm work mode
 * @param	MaxFiterLen max filter len
 * @param	FilterDepth filter depth len
 * @return  len fliter len
 * @date	2024-6-6 21:29:50
 * @author	Pan
 */
uint16_t loadFilterSwitch(uint8_t Wm, uint16_t MaxFiterLen, uint16_t FilterDepth)
{
	uint16_t len;
	static uint32_t readyCounter = 0;
	if (Wm == wmMoveReady)
	{
		readyCounter++;
		if (readyCounter >= MaxFiterLen)
		{
			readyCounter = MaxFiterLen;
			len = MaxFiterLen;
		}
		else
			len = FilterDepth;
	}
	else
	{
		readyCounter = 0;
		len = FilterDepth;
	}
	// filterLen.loadLen = switchLen(0.005,ls_slide_t.k,FILTER_LEN,50,AL.loadCtrl.NominalValue,force.orig-force.filter);
	return len;
}

/**
 * @brief	load filter sigmod len
 * @param	x the object needed to filter
 * @param	filter	relevant variable of filter
 * @param	filter_depth type:uint16_t
 * @retval	null
 * @date	2024-3-5 20:11:40
 * @author	Pan
 */
void loadFilterWithLenSwitch(MDSTRUCT *x, FILTER *filter, uint16_t filter_depth)
{
	static uint16_t i = 0, depthLast = FILTER_LEN, k = 0;
	uint16_t  m;

	x->filter_last = x->filter;
	if (depthLast > filter_depth)
	{
		filter->force_temp = 0;
		i = filter_depth - 1;
		for (m = 0; m < filter_depth; m++)
		{
			filter->force_buf[m] = filterMaxArrayLoad[(k - filter_depth + m + 1) % FILTER_LEN];
			filter->force_temp += filter->force_buf[m];
		}
	}
	k++;
	if (k >= FILTER_LEN)
		k = 0;
	filterTempLoad -= filterMaxArrayLoad[k];
	filterMaxArrayLoad[k] = force.orig;
	filterTempLoad += filterMaxArrayLoad[k];

	i++;
	if (i >= filter_depth)
		i = 0;
	filter->force_temp -= filter->force_buf[i];
	filter->force_buf[i] = force.orig;
	filter->force_temp += filter->force_buf[i];

	if (filter_depth >= FILTER_LEN)
	{
		x->filter = filterTempLoad / (filter_depth * 1.0);
	}
	else
	{
		x->filter = filter->force_temp / (filter_depth * 1.0);
	}

	if (filter_depth < FILTER_LEN)
	{
		x->filterTrans = forceLsm.filter;
	}
	else
	{
		x->filterTrans = x->filter;
	}

	depthLast = filter_depth;
	//force.codeFiter_last = force.codeFiter;
	//force.codeFiter = (int32_t)(x->filter * AL.loadCtrl.sign * 1.0 * AL.loadCtrl.NominalSensitive / 1.201226008469083e-4 / AL.loadCtrl.NominalValue + AL.tare.value[4]);
	// if(SenData[ch4Load].LinPoint == 0){
	// 	sensorCalibrate.antiSinglepointCalibrate(x->filterTrans,ch4Load,&SenData[ch4Load],&force.codeFiter);
	// 	//force.codeFiter = force.codeFiter + AL.tare.value[ch4Load];
	// }else{
	// 	sensorCalibrate.antiMultipointCalibrate(x->filterTrans,&SenData[ch4Load],&force.codeFiter);
	// }
}

/**
 * @brief	load filter sigmod len
 * @param	filter_depth type:uint16_t
 * @retval	null
 * @date	2024-3-5 20:11:40
 * @author	Pan
 */
#define FILTER_LEN2 10
void load_filter_SL2(uint16_t filter_depth)
{
	static uint16_t i = 0;
	// uint16_t j;
	i++;
	if (i >= FILTER_LEN2)
		i = 0;
	filterSL2.force_temp -= filterSL2.force_buf[i];
	filterSL2.force_buf[i] = force.orig;
	filterSL2.force_temp += filterSL2.force_buf[i];
	forceSL2.filter = filterSL2.force_temp / (filter_depth * 1.0);
	// forceSL.filter = forceSL.filter*0.9 + forceSL.filter_last*0.1;
}

/**
 * @brief	load speed filter
 * @param	filter_depth type:uint8_t
 * @retval	null
 * @date	2024-3-5 20:11:40
 * @author	Pan
 */
void loadspeed_filter(uint16_t filter_depth)
{
	static uint16_t i = 0;
	speedForce.filter_last = speedForce.filter;
	i++;
	if (i >= filter_depth)
		i = 0;
	filter.speedload_temp -= filter.speed_buf_load[i];
	filter.speed_buf_load[i] = (force.filter - force.filter_last) / AL.delta; // 璁＄畻閫熷害 r/s
	filter.speedload_temp += filter.speed_buf_load[i];
	speedForce.orig = filter.speed_buf_load[i];
	speedForce.filter = filter.speedload_temp / (filter_depth * 1.0f); // 璁＄畻閫熷害 r/s
}


/**
 * @brief	extensometer calculate from ADC code
 * @param[in]	AL type:APP_LAYER_VARIABLE_STRUCT*
 * @retval	null
 * @date	2024-3-5 20:11:40
 * @author	Pan
 */
void extensometerCalcu(MDSTRUCT *_strain, APP_LAYER_VARIABLE_STRUCT *_AL)
{
	_strain->orig_last = _strain->orig;
	// ADC_FACTOR_ROUND: ADC code -> engineering value scaling factor
	//_strain->orig = _AL->ext1Ctrl.sign * (_strain->code - _AL->tare.value[2]) * ADC_FACTOR / _AL->ext1Ctrl.NominalSensitive * _AL->ext1Ctrl.NominalValue;
	if(SenData[ch2Ext1].LinPoint > 0){
		sensorCalibrate.multipointCalibrate(_strain->code,ch2Ext1,&SenData[ch2Ext1],&_strain->orig);
	}
	else
		sensorCalibrate.singlepointCalibrate(_strain->code,ch2Ext1,&SenData[ch2Ext1],&_strain->orig);
}

/**
 * @brief	extensometer lsm filter
 * @param	x the object needed to filter
 * @param	filter_depth type:uint16_t
 * @retval	null
 * @date	2024-3-5 20:11:40
 * @author	Pan
 */
void extensometerFilterLsm(MDSTRUCT *x, uint16_t filter_depth,const float _orig)
{
	x->orig = _orig;
	x->filter_last = x->filter;
	lsSlideExt.y_m = mean_fast(filter_depth, &lsSlideExt.i_ym, &lsSlideExt.y_mtemp, lsSlideExt.y_mbuf, x->orig);
	lsm_pre_ext.y_pre_last = lsm_pre_ext.y_pre;
	lsm_pre_ext.y_pre = least_square_fast(filter_depth, &lsSlideExt, 0, x->orig);
	x->filter = lsm_pre_ext.y_pre;
}

/**
 * @brief	extensometer filter sigmod len
 * @param[in]	Wm work mode
 * @param[in]	MaxFiterLen max filter len
 * @param[in]	FilterDepth filter depth len
 * @return  len fliter len
 * @date	2024-11-21 11:34:34
 * @author	Pan
 */
uint16_t extensometerFilterSwitch(const uint8_t Wm,const uint16_t MaxFiterLen,const uint16_t FilterDepth)
{
	uint16_t len;
	static uint32_t readyCounter = 0;
	if (Wm == wmMoveReady)
	{
		readyCounter++;
		if (readyCounter >= MaxFiterLen)
		{
			readyCounter = MaxFiterLen;
			len = MaxFiterLen;
		}
		else
			len = FilterDepth;
	}
	else
	{
		readyCounter = 0;
		len = FilterDepth;
	}
	return len;
}


/**
 * @brief	extensomter filter sigmod len
 * @param	_x the object needed to filter
 * @param   _xLsm the lsm object needed to filter
 * @param	_filter	relevant variable of filter
 * @param[in] _AL type:APP_LAYER_VARIABLE_STRUCT*
 * @param	_filter_depth type:uint16_t
 * @retval	null
 * @date	2024-11-20 17:57:06
 * @author	Pan
 */
void extensometerFilterWithLenSwitch(MDSTRUCT *_x,MDSTRUCT *_xLsm ,
									FILTER *_filter,const APP_LAYER_VARIABLE_STRUCT *_AL,
									const uint16_t _filter_depth)
{
	static uint16_t i = 0, depthLast = STRAIN_FILTER_LEN,k = 0;
	uint16_t  m;

	_x->filter_last = _x->filter;
	if (depthLast > _filter_depth)
	{
		_filter->strain_temp = 0;
		i = _filter_depth - 1;
		for (m = 0; m < _filter_depth; m++)
		{
			_filter->strain_buf[m] = filterMaxArrayStrain[(k - _filter_depth + m + 1) % STRAIN_FILTER_LEN];
			_filter->strain_temp += _filter->strain_buf[m];
		}
	}
	k++;
	if (k >= STRAIN_FILTER_LEN)
		k = 0;
	filterTempStrain -= filterMaxArrayStrain[k];
	filterMaxArrayStrain[k] = _x->orig;
	filterTempStrain += filterMaxArrayStrain[k];

	i++;
	if (i >= _filter_depth)
		i = 0;
	_filter->strain_temp -= _filter->strain_buf[i];
	_filter->strain_buf[i] = _x->orig;
	_filter->strain_temp += _filter->strain_buf[i];

	if (_filter_depth >= STRAIN_FILTER_LEN)
	{
		_x->filter = filterTempStrain / (_filter_depth * 1.0);
	}
	else
	{
		_x->filter = _filter->strain_temp / (_filter_depth * 1.0);
	}

	if (_filter_depth < STRAIN_FILTER_LEN)
	{
		_x->filterTrans = _xLsm->filter;
	}
	else
	{
		_x->filterTrans = _x->filter;
	}

	depthLast = _filter_depth;
	_x->codeFiter_last = _x->codeFiter;
	_x->codeFiter = (int32_t)(_x->filter * _AL->ext1Ctrl.sign * 1.0f * _AL->ext1Ctrl.NominalSensitive / ADC_FACTOR_ROUND / _AL->ext1Ctrl.NominalValue + _AL->tare.value[ch2Ext1]);
}

/**
 * @brief	extensometer2 calculate from ADC code
 * @param[in]	AL type:APP_LAYER_VARIABLE_STRUCT*
 * @retval	null
 * @date	2025-2-6 23:10:40
 * @author	Pan
 */
void extensometer2Calcu(MDSTRUCT *_strain, APP_LAYER_VARIABLE_STRUCT *_AL)
{
	(void)_AL;
	MD_UpdateEngineeringValue(_strain, ch3Ext2, &SenData[ch3Ext2]);
}

/**
 * @brief	extensometer2 lsm filter
 * @param	x the object needed to filter
 * @param	filter_depth type:uint16_t
 * @retval	null
 * @date	2025-2-6 23:17:20
 * @author	Pan
 */
void extensometer2FilterLsm(MDSTRUCT *x, uint16_t filter_depth,const float _orig)
{
	x->orig = _orig;
	x->filter_last = x->filter;
	lsSlideExt2.y_m = mean_fast(filter_depth, &lsSlideExt2.i_ym, &lsSlideExt2.y_mtemp, lsSlideExt2.y_mbuf, x->orig);
	lsm_pre_ext2.y_pre_last = lsm_pre_ext2.y_pre;
	lsm_pre_ext2.y_pre = least_square_fast(filter_depth, &lsSlideExt2, 0, x->orig);
	x->filter = lsm_pre_ext2.y_pre;
}

/**
 * @brief	extensometer2 filter sigmod len
 * @param[in]	Wm work mode
 * @param[in]	MaxFiterLen max filter len
 * @param[in]	FilterDepth filter depth len
 * @return  len fliter len
 * @date	2025-2-6 23:25:14
 * @author	Pan
 */
uint16_t extensometer2FilterSwitch(const uint8_t Wm,const uint16_t MaxFiterLen,const uint16_t FilterDepth)
{
	uint16_t len;
	static uint32_t readyCounter = 0;
	if (Wm == wmMoveReady)
	{
		readyCounter++;
		if (readyCounter >= MaxFiterLen)
		{
			readyCounter = MaxFiterLen;
			len = MaxFiterLen;
		}
		else
			len = FilterDepth;
	}
	else
	{
		readyCounter = 0;
		len = FilterDepth;
	}
	return len;
}

/**
 * @brief	extensomter2 filter sigmod len
 * @param	_x the object needed to filter
 * @param   _xLsm the lsm object needed to filter
 * @param	_filter	relevant variable of filter
 * @param[in] _AL type:APP_LAYER_VARIABLE_STRUCT*
 * @param	_filter_depth type:uint16_t
 * @retval	null
 * @date	2025-2-6 23:29:48
 * @author	Pan
 */
void extensometer2FilterWithLenSwitch(MDSTRUCT *_x,MDSTRUCT *_xLsm,
									FILTER *_filter,const APP_LAYER_VARIABLE_STRUCT *_AL,
									const uint16_t _filter_depth)
{
	static uint16_t i = 0, depthLast = STRAIN_FILTER_LEN,k = 0;
	uint16_t  m;

	_x->filter_last = _x->filter;
	if (depthLast > _filter_depth)
	{
		_filter->strain2_temp = 0;
		i = _filter_depth - 1;
		for (m = 0; m < _filter_depth; m++)
		{
			_filter->strain2_buf[m] = filterMaxArrayStrain2[(k - _filter_depth + m + 1) % STRAIN_FILTER_LEN];
			_filter->strain2_temp += _filter->strain2_buf[m];
		}
	}
	k++;
	if (k >= STRAIN_FILTER_LEN)
		k = 0;
	filterTempStrain2 -= filterMaxArrayStrain2[k];
	filterMaxArrayStrain2[k] = _x->orig;
	filterTempStrain2 += filterMaxArrayStrain2[k];

	i++;
	if (i >= _filter_depth)
		i = 0;
	_filter->strain2_temp -= _filter->strain2_buf[i];
	_filter->strain2_buf[i] = _x->orig;
	_filter->strain2_temp += _filter->strain2_buf[i];

	if (_filter_depth >= STRAIN_FILTER_LEN)
	{
		_x->filter = filterTempStrain2 / (_filter_depth * 1.0);
	}
	else
	{
		_x->filter = _filter->strain2_temp / (_filter_depth * 1.0);
	}

	if (_filter_depth < STRAIN_FILTER_LEN)
	{
		_x->filterTrans = _xLsm->filter;
	}
	else
	{
		_x->filterTrans = _x->filter;
	}

	depthLast = _filter_depth;
	_x->codeFiter_last = _x->codeFiter;
	_x->codeFiter = (int32_t)(_x->filter * _AL->ext2Ctrl.sign * 1.0f * _AL->ext2Ctrl.NominalSensitive / ADC_FACTOR_ROUND / _AL->ext2Ctrl.NominalValue + _AL->tare.value[ch2Ext1]);
}

/**
 * @brief	pos generator should be updated if sendata is updated
 * @note	because the change of sendata will affect the calculation of measurement data
 * @param	null
 * @retval	null
 * @date	2024-3-5 20:11:40
 * @author	Pan
 */
void sendata_pg_process(void)
{
	if (mySemaphore.comm.sendataUpdate == 1)
	{
		switch (AL.movectrl)
		{
		case POS_MODE:
			sendata_pg_update(&pose.orig);
			break;
		case LOAD_MODE:
			sendata_pg_update(&force.filter);
			break;
		case EXTEN_MODE:
			// sendata_pg_update(&pose_now);
			break;
		}
		mySemaphore.comm.sendataUpdate = 0;
	}
}

/**
 * @note	Fun_165
 * @brief	output parameters will sync from combuf if mySemaphore.paraconfig.write == 1 && command == WROUTPUTPARA
 * @param	null
 * @retval	null
 * @date	2024-3-5 20:11:40
 * @author	Pan
 */
void output_param_update(void)
{
	// Data rationality checks should be added.
	memcpy(&OutputPara, &OutputPara_combuf, sizeof(OutputPara_combuf));
}

/**
 * @brief map hal DO
 * @param[in] in mapping data of DO
 * @param[out] out hal data of DO
 */
void halDOMap(DOTypeDef *out, DOTypeDef *in)
{
	out->DO1 = in->DO1;
	out->DO2 = in->DO2;
	out->DO3 = in->DO3;
	out->DO4_RL = in->DO4_RL;
	out->DO5_RL = in->DO5_RL;
	out->SON = in->SON;
	out->DIR = in->DIR;
}

/**
 * @brief	open loop parameters update
 * @param	null
 * @retval	null
 * @date	2024-3-5 20:11:40
 * @author	Pan
 */
void openloop_parameters_update(void)
{
	// The open loop flag bit is 1 to update the relevant value
	if (commBuf.OpenLoopflag == 1)
	{
		AL.command = commBuf.Command;
		// Frequency
		AL.openloopFrq = commBuf.openloopFrq;
		// AO
		AL.openloopAO = commBuf.openloopAO;
		// DO
		openloopDO.SON = (GPIO_PinState)(commBuf.DO & 0x01);
		openloopDO.DIR = (GPIO_PinState)((commBuf.DO >> 1) & 0x01);
		openloopDO.DO1 = (GPIO_PinState)((commBuf.DO >> 2) & 0x01);
		openloopDO.DO2 = (GPIO_PinState)((commBuf.DO >> 3) & 0x01);
		openloopDO.DO3 = (GPIO_PinState)((commBuf.DO >> 4) & 0x01);
		openloopDO.DO4_RL = (GPIO_PinState)((commBuf.DO >> 5) & 0x01);
		openloopDO.DO5_RL = (GPIO_PinState)((commBuf.DO >> 6) & 0x01);
		commBuf.OpenLoopflag = 0;
	}
	// openloop stateflag update,and openloop output reset
	if (AL.command == DOPESETOPENLOOPCOMMAND)
	{
		stateFlag.openloop = 1;
	}
	else if (AL.command != DOPESETOPENLOOPCOMMAND && stateFlag.openloop == 1)
	{
		stateFlag.openloop = 0;
		AL.openloopFrq = 0;
		AL.openloopAO = 0;
		openloopDO.SON = GPIO_PIN_RESET;
		openloopDO.DIR = GPIO_PIN_RESET;
		openloopDO.DO1 = GPIO_PIN_RESET;
		openloopDO.DO2 = GPIO_PIN_RESET;
		openloopDO.DO3 = GPIO_PIN_RESET;
		openloopDO.DO4_RL = GPIO_PIN_RESET;
		openloopDO.DO5_RL = GPIO_PIN_RESET;
	}
}

/**
 * @note	Fun_177
 * @brief	open loop output process
 * @param[in]	op OUTPUTPARA*
 * @retval	null
 * @date	2024-3-6 15:09:01
 * @author	Pan
 */
void openloopOutputMap(OUTPUTPARA *op)
{
	// map hal DO
	halDOMap(&DO, &openloopDO);
	halDOMap(&hal_output.DO, &DO);
	AL.openloopFrq_Last = (int32_t)hal_output.PWM;
	AL.svPWMforProtect = AL.openloopFrq;
	// PWM AO mapping
	if (AL.openloopFrq >= 0)
	{
		hal_output.DO.DIR = DO.DIR; // 鏂瑰悜鍒ゆ柇  //姝ｈ浆
		hal_output.PWM = AL.openloopFrq;
	}
	else
	{
		hal_output.DO.DIR = (GPIO_PinState)!DO.DIR; // 鍙嶈浆
		hal_output.PWM = -AL.openloopFrq;
	}
	hal_output.AO = 0;
}

/**
 * @brief according to OutputPara,limit the amplitude of the output signal
 * @param[out]	out 	output signal
 * @param[in]	max 	MaxValue
 * @param[in]	min		MinValue
 * @param[in]   range	SignalFrequency or MaxVoltage or MaxCurrent
 */
void outputSignalConstraint(float *out, float max, float min, float range)
{
	float tmp;
	if (*out > (tmp = (max / 100.0f * range)))
	{
		*out = (float)tmp;
	}
	else if (*out < (tmp = (min / 100.0f * range)))
	{
		*out = (float)tmp;
	}
}

/**
 * @brief according to OutputPara,Set offset of the output signal
 * @param[out]	out 	output signal
 * @param[in]	offset	Offset
 * @param[in]	range	SignalFrequency or MaxVoltage or MaxCurrent
 */
void outputSignalOffset(float *out, float offset, float range)
{
	if (*out > 0)
	{
		*out = *out + offset / 100.0f * range;
	}
	else if (*out < 0)
	{
		*out = *out - offset / 100.0f * range;
	}
}

/**
 * @brief according to OutputPara,Set initial value	of the output signal
 * @param[out]	out 	output signal
 * @param[in]	initValue	initial value
 * @param[in]	range	SignalFrequency or MaxVoltage or MaxCurrent
 */
void outputSignalInitValue(float *out, float initValue, float range)
{
	*out = initValue / 100.0f * range;
}

/**
 * @brief according to OutputPara,Set sign of the output signal
 * @param[out]	out 	output signal
 * @param[in]	sign	sign
 */
void outputSignalSign(float *out, uint8_t sign)
{
	if (sign == 0x01)
	{
		*out *= -1;
	}
}

/**
 * @brief according to OutputPara,Set the output signal
 * @param[out]	out 	output signal
 * @param[in]	op		OUTPUTPARA*
 */
void outputSignalProcess(float *out, OUTPUTPARA *op)
{
	if (op->Signal <= SGINAL_UP_DOWN)
	{
		outputSignalOffset(out, op->Offset, op->SignalFrequency);
		outputSignalSign(out, op->Sign);
		outputSignalConstraint(out, op->MaxValue, op->MinValue, op->SignalFrequency);
	}
	else if (op->Signal > SGINAL_UP_DOWN)
	{
		outputSignalOffset(out, op->Offset, op->MaxVoltage);
		outputSignalSign(out, op->Sign);
		outputSignalConstraint(out, op->MaxValue, op->MinValue, op->MaxVoltage);
	}
}

extern uint16_t pwmFinishcounter;
/**
 * @brief	close loop control ouput process
 * @param	op type:OUTPUTPARA*
 * @retval	null
 * @date	2024-3-6 15:31:10
 * @author	Pan
 * @bug		InitValue, the variable of OutputPara, i have no idea of its usage.
 */
void closeloopOutputMap(OUTPUTPARA *op)
{
	// uint32_t utmp = 0;
	// static int32_t fclkcounters;
	// map hal DO
	halDOMap(&hal_output.DO, &DO);
	// PWM AO mapping
	switch (op->Signal)
	{
	case SGINAL_A_B:

		break;
	case SGINAL_PULSE_SIGN:
		switch (AL.movectrl)
		{
		case POS_MODE:
			outputSignalProcess(&svPWM.pos, &OutputPara);
			AL.svPWMforProtect = svPWM.pos;
			if (svPWM.pos >= 0)
			{
				hal_output.PWM = svPWM.pos;
				DO.DIR = (GPIO_PinState)0; // 鏂瑰悜鍒ゆ柇  //姝ｈ浆
			}
			else
			{
				hal_output.PWM = -svPWM.pos;
				DO.DIR = (GPIO_PinState)1; // 鍙嶈浆
			}
			if (AL.complete_state.flag == 1 || stateFlag.DoPESetCtrl == CtrlFALSE)
				hal_output.PWM = 0; // 浠ｈ〃瀹屾垚姝ゆ杩愬姩鎴栭棴鐜姝?绂佹杈撳嚭
			break;
		case LOAD_MODE:
			outputSignalProcess(&svPWM.load, &OutputPara);
			AL.svPWMforProtect = svPWM.load;
			if (svPWM.load >= 0)
			{
				hal_output.PWM = svPWM.load;
				DO.DIR = (GPIO_PinState)0; // 鏂瑰悜鍒ゆ柇  //姝ｈ浆
			}
			else
			{
				hal_output.PWM = -svPWM.load;
				DO.DIR = (GPIO_PinState)1; // 鍙嶈浆
			}
			if (AL.complete_state.flag == 1 || stateFlag.DoPESetCtrl == CtrlFALSE)
				hal_output.PWM = 0; // 浠ｈ〃瀹屾垚姝ゆ杩愬姩鎴栭棴鐜姝?绂佹杈撳嚭
			break;
		case EXTEN_MODE:
			outputSignalProcess(&svPWM.ext, &OutputPara);
			AL.svPWMforProtect = svPWM.ext;
			if (svPWM.ext >= 0)
			{
				hal_output.PWM = svPWM.ext;
				DO.DIR = (GPIO_PinState)0; // 鏂瑰悜鍒ゆ柇  //姝ｈ浆
			}
			else
			{
				hal_output.PWM = -svPWM.ext;
				DO.DIR = (GPIO_PinState)1; // 鍙嶈浆
			}
			if (AL.complete_state.flag == 1 || stateFlag.DoPESetCtrl == CtrlFALSE){
				hal_output.PWM = 0; // 浠ｈ〃瀹屾垚姝ゆ杩愬姩鎴栭棴鐜姝?绂佹杈撳嚭
			}
			break;
		default:
			break;
		}
		hal_output.DO.DIR = DO.DIR;
		break;
	case SGINAL_UP_DOWN:

		break;
	case SGINAL_ANALOGUE:
		AL.svPWMforProtect = 0;
		AL.AOforProtect = 0;
		hal_output.PWM = 0;
		hal_output.AO = 0;
		break;
	case SGINAL_DC_MOTOR:

		break;
	case SGINAL_DC_LINEAR_MOTOR:

		break;
	}
}
/**
 * @brief According to the state of stateflag and emergency, enable or disenable SON.
 *
 */
void SONMap(void)
{
	if(stateFlag.openloop == 0){
		if (stateFlag.DoPE_state == DoPE_On && AL.emergency_state.processing == 0)
		{
			DO.SON = GPIO_PIN_SET;
		}
		else
		{
			DO.SON = GPIO_PIN_RESET;
		}
	}
	// //Hardware limit
	// if(DI.DI7.value == GPIO_PIN_RESET || DI.DI8.value == GPIO_PIN_RESET){
	// 	DO.SON = 0;
	// }
}

#define SCRAM_TRIGGER_TIME 20
#define SCRAM_RELEASE_TIME1 3000
#define SCRAM_RELEASE_TIME2 2000

void emergency_scram_button_process(void)
{
	static uint16_t scram_counter = 0;
	static uint16_t scram_release_counter = 0;
	if (emerency_struct.scram_button == 0 && emerency_struct.state != 2)
	{
		scram_counter++;
		if (scram_counter >= SCRAM_TRIGGER_TIME) //>=20ms
		{
			emerency_struct.state = 1; // stop system
		}
		if (scram_counter >= SCRAM_RELEASE_TIME1) //>=3000ms
		{
			emerency_struct.state = 2; // ready to release system
		}
	}

	if (emerency_struct.scram_button == 1 && emerency_struct.state == 2)
	{
		scram_release_counter++;
		if (scram_release_counter >= SCRAM_RELEASE_TIME2)
		{
			scram_release_counter = 0;
			scram_counter = 0;
			emerency_struct.state = 0;
		}
	}

	if (emerency_struct.state == 1 || emerency_struct.state == 2)
	{
		svPWM.pos = 0;
		svAO.pos = 0;
		hal_output.PWM = 0;
		hal_output.AO = 0;
		hal_output.DO.SON = GPIO_PIN_RESET; // SRV OFF
		memset(&pg, 0, sizeof(pg));
		memset(&pgLoad, 0, sizeof(pgLoad));
		memset(&pgdyn, 0, sizeof(pgdyn));
		AL.servoOutput = 0;
		AL.command = 0;
		AL.complete_state.flag = 1;
	}
}

/**
 * @brief measure data init
 *
 */
void measureData_init(void)
{
	pose.orig = 0; // 鍒濆浣嶇疆

	force.orig = 0;

	strain1.orig = 0;

	speedPose.filter = 0; // pos speed init

	speedForce.filter = 0; // load speed init
}

/*****************************DI FUNC MAP***********************************************/
static void DIFunction01(PINSTATE_MAP_STRUCT *x)
{
	// log_i("DIFunction01 x:%d",x->value);
}

static void DIFunction02(PINSTATE_MAP_STRUCT *x)
{
	static GPIO_PinState stateLast = GPIO_PIN_SET;
	if(mySemaphore.manualBox485.up == 1){
		mySemaphore.manualBox485.up = 0;
		manualBox.up(manualBox.speed);
		log_i("manualBox485.up speed:%f", manualBox.speed);
	}
	else if(mySemaphore.manualBox485.halt == 1){
		mySemaphore.manualBox485.halt = 0;
		manualBox.halt();
		log_i("manualBox485.halt");
	}

	if (stateLast == GPIO_PIN_SET && x->value == GPIO_PIN_RESET)
	{
		manualBox.up(manualBox.speed);
		log_i("manualBox.up speed:%f", manualBox.speed);
	}
	else if (stateLast == GPIO_PIN_RESET && x->value == GPIO_PIN_SET)
	{
		manualBox.halt();
		log_i("manualBox.halt");
	}
	stateLast = x->value;
	// log_i("DIFunction02 x:%d",x->value);
}

static void DIFunction03(PINSTATE_MAP_STRUCT *x)
{
	static GPIO_PinState stateLast = GPIO_PIN_SET;
	if(mySemaphore.manualBox485.down == 1){
		mySemaphore.manualBox485.down = 0;
		manualBox.down(manualBox.speed);
		log_i("manualBox485.down speed:%f", manualBox.speed);
	}
	else if(mySemaphore.manualBox485.halt == 1){
		mySemaphore.manualBox485.halt = 0;
		manualBox.halt();
		log_i("manualBox485.halt");
	}

	if (stateLast == GPIO_PIN_SET && x->value == GPIO_PIN_RESET)
	{
		manualBox.down(manualBox.speed);
		log_i("manualBox.down speed:%f", manualBox.speed);
	}
	else if (stateLast == GPIO_PIN_RESET && x->value == GPIO_PIN_SET)
	{
		manualBox.halt();
		log_i("manualBox.halt");
	}
	stateLast = x->value;
	// log_i("DIFunction03 x:%d",x->value);
}

static void DIFunction04(PINSTATE_MAP_STRUCT *x)
{
	static GPIO_PinState stateLast = GPIO_PIN_SET;
	if(mySemaphore.manualBox485.rapidUp == 1){
		mySemaphore.manualBox485.rapidUp = 0;
		manualBox.up(manualBox.rapidSpeed);
		log_i("manualBox485.rapidUp rapidSpeed:%f", manualBox.rapidSpeed);
	}
	else if(mySemaphore.manualBox485.halt == 1){
		mySemaphore.manualBox485.halt = 0;
		manualBox.halt();
		log_i("manualBox485.halt");
	}
	if (stateLast == GPIO_PIN_SET && x->value == GPIO_PIN_RESET)
	{
		manualBox.up(manualBox.rapidSpeed);
		log_i("manualBox.up rapidSpeed:%f", manualBox.rapidSpeed);
	}
	else if (stateLast == GPIO_PIN_RESET && x->value == GPIO_PIN_SET)
	{
		manualBox.halt();
		log_i("manualBox.halt");
	}
	stateLast = x->value;
	// log_i("DIFunction04 x:%d",x->value);
}

static void DIFunction05(PINSTATE_MAP_STRUCT *x)
{
	static GPIO_PinState stateLast = GPIO_PIN_SET;
	if(mySemaphore.manualBox485.rapidDown == 1){
		mySemaphore.manualBox485.rapidDown = 0;
		manualBox.down(manualBox.rapidSpeed);
		log_i("manualBox485.rapidDown rapidSpeed:%f", manualBox.rapidSpeed);
	}
	else if(mySemaphore.manualBox485.halt == 1){
		mySemaphore.manualBox485.halt = 0;
		manualBox.halt();
		log_i("manualBox485.halt");
	}
	if (stateLast == GPIO_PIN_SET && x->value == GPIO_PIN_RESET)
	{
		manualBox.down(manualBox.rapidSpeed);
		log_i("manualBox.down rapidSpeed:%f", manualBox.rapidSpeed);
	}
	else if (stateLast == GPIO_PIN_RESET && x->value == GPIO_PIN_SET)
	{
		manualBox.halt();
		log_i("manualBox.halt");
	}
	stateLast = x->value;
	// log_i("DIFunction05 x:%d",x->value);
}

static void DIFunction06(PINSTATE_MAP_STRUCT *x)
{
	static GPIO_PinState stateLast = GPIO_PIN_SET;
	if(mySemaphore.manualBox485.reset == 1){
		mySemaphore.manualBox485.reset = 0;
		manualBox.reset(manualBox.rapidSpeed);
		log_i("manualBox485.reset speed:%f", manualBox.rapidSpeed);
	}
	if (stateLast == GPIO_PIN_SET && x->value == GPIO_PIN_RESET)
	{
		manualBox.reset(manualBox.rapidSpeed);
		log_i("manualBox.reset speed:%f", manualBox.rapidSpeed);
	}
	stateLast = x->value;
	// log_i("DIFunction06 x:%d",x->value);
}

static void DIFunction07(PINSTATE_MAP_STRUCT *x)
{
	// log_i("DIFunction07 x:%d",x->value);
}

static void DIFunction08(PINSTATE_MAP_STRUCT *x)
{
	// log_i("DIFunction08 x:%d",x->value);
}

static void DIFunction09(PINSTATE_MAP_STRUCT *x)
{
	// log_i("DIFunction09 x:%d",x->value);
}

static void DIFunction10(PINSTATE_MAP_STRUCT *x)
{
	// log_i("DIFunction10 x:%d",x->value);
}

void DIFuncMap_init(void)
{
	DIMap_init[0].func = &DIFunction01;
	DIMap_init[1].func = &DIFunction02;
	DIMap_init[2].func = &DIFunction03;
	DIMap_init[3].func = &DIFunction04;
	DIMap_init[4].func = &DIFunction05;
	DIMap_init[5].func = &DIFunction06;
	DIMap_init[6].func = &DIFunction07;
	DIMap_init[7].func = &DIFunction08;
	DIMap_init[8].func = &DIFunction09;
	DIMap_init[9].func = &DIFunction10;

	DIMap[0].func = DIMap_init[0].func;
	DIMap[1].func = DIMap_init[1].func;
	DIMap[2].func = DIMap_init[2].func;
	DIMap[3].func = DIMap_init[3].func;
	DIMap[4].func = DIMap_init[4].func;
	DIMap[5].func = DIMap_init[5].func;
	DIMap[6].func = DIMap_init[6].func;
	DIMap[7].func = DIMap_init[7].func;
	DIMap[8].func = DIMap_init[8].func;
	DIMap[9].func = DIMap_init[9].func;
}
/**
 * @brief according to the arry of mapNum from upper computer by ETH communication,remap the DI Function.
 * @param mapNum new arry of DI Function map
 * @note For example, if the upper computer sends an array sequence of "2,1,3,5,7,6,8,4,9,0",
 * 						the DIMap[0].func = DIMap_init[2].func; // DIFunction03
 * 						the DIMap[1].func = DIMap_init[1].func;	// DIFunction02
 * 						the DIMap[2].func = DIMap_init[3].func; // DIFunction04
 * 						...
 * 						the DIMap[9].func = DIMap_init[0].func;	// DIFunction01
 */
static void DIFuncMap_update(uint8_t mapNum[10])
{
	uint8_t i;
	for (i = 0; i < 10; i++)
	{
		DIMap[i].func = DIMap_init[mapNum[i]].func;
	}
}

void DIFunc_map(void)
{
	// determine whether the semaphore of map function has been updated
	if (mySemaphore.mapFunc.DISem == 1)
		DIFuncMap_update(comMap.DImapNum);
	// call DI Map Function
	DIMap[0].func(&DI.DI1);
	DIMap[1].func(&DI.DI2);
	DIMap[2].func(&DI.DI3);
	DIMap[3].func(&DI.DI4);
	DIMap[4].func(&DI.DI5);
	DIMap[5].func(&DI.DI6);
	DIMap[6].func(&DI.DI7);
	DIMap[7].func(&DI.DI8);
	DIMap[8].func(&DI.DI9);
	DIMap[9].func(&DI.DI10);
}
/******************************DI FUNC MAP**********************************************/

/********************************** MANUAL BOX *****************************************/
void manualBoxUp(float speed);
void manualBoxDown(float speed);
void manualBoxReset(float speed);
void manualBoxHalt(void);
void manualBoxFuncRegister(void);

/**
 *@brief Function for initing manual box
 */
void manualBoxInit(void)
{
	manualBoxFuncRegister();
	manualBox.speed = 1;
	manualBox.rapidSpeed = 5;
	manualBox.type = manualBoxDI_Type;
}
/**
 * @brief Function for registering manual box command function
 */
void manualBoxFuncRegister(void)
{
	manualBox.up = &manualBoxUp;
	manualBox.down = &manualBoxDown;
	manualBox.reset = &manualBoxReset;
	manualBox.halt = &manualBoxHalt;
}

void manualBoxUp(float speed)
{
	commBuf.Command = DOPEMOVE; // 鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	// 杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.Direction = 0x01;
	commBuf.MoveCtrl = POS_MODE;
	commBuf.Speed = speed;
	if (posAllCtrlPara.Speed < commBuf.Speed)
		commBuf.Speed = posAllCtrlPara.Speed; // 鏈€澶ч€熷害榛樿鍊紆nit/s
	commBuf.Acc = posAllCtrlPara.DefaultAcc;
	commBuf.Dec = -commBuf.Acc;
	log_d("manualBoxUp speed:%f", speed);
	mySemaphore.move.pgInit = 1;
}

void manualBoxDown(float speed)
{
	commBuf.Command = DOPEMOVE; // 鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	// 杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.Direction = 0x00;
	commBuf.MoveCtrl = POS_MODE;
	commBuf.Speed = speed;
	if (posAllCtrlPara.Speed < commBuf.Speed)
		commBuf.Speed = posAllCtrlPara.Speed; // 鏈€澶ч€熷害榛樿鍊紆nit/s
	commBuf.Acc = posAllCtrlPara.DefaultAcc;
	commBuf.Dec = -commBuf.Acc;
	log_d("manualBoxDown speed:%f", speed);
	mySemaphore.move.pgInit = 1;
}

void manualBoxReset(float speed)
{
	commBuf.Command = DOPEPOS; // 鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	commBuf.MoveCtrl = POS_MODE;
	commBuf.Speed = speed;
	commBuf.Destination = 0;
	commBuf.Acc = posAllCtrlPara.DefaultAcc;
	commBuf.Dec = -posAllCtrlPara.DefaultAcc;
	if (posAllCtrlPara.Speed < commBuf.Speed)
		commBuf.Speed = posAllCtrlPara.Speed; // 鏈€澶ч€熷害榛樿鍊紆nit/s
	log_d("manualBoxReset speed:%f", speed);
	mySemaphore.move.pgInit = 1;
}

void manualBoxHalt(void)
{
	commBuf.Command = DOPEHALT; // 鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	commBuf.MoveCtrl = POS_MODE;
	commBuf.Acc = posAllCtrlPara.DefaultAcc;
	commBuf.Dec = -posAllCtrlPara.DefaultAcc;
	mySemaphore.move.pgInit = 1;
}

void manualBoxTypeMonitor(MANUAL_BOX_STRUCT *_manualBox){
	static uint16_t _timCounter = 0;
	if(_manualBox->type == manualBox485_Type){
		if(++_timCounter >= 100){
			_timCounter = 0;
			_manualBox->type = manualBoxDI_Type;
		}
	}
}

/**
 * @brief sensor range protect
 * @param[in] _value the value needed to protect
 * @param[in] _max 	 max protect value
 * @param[in] _min   min protect value
 * @param[in] _sensorConnetor the connector of sensor
 * @retval SENPS_noProtect means no pretect  
 * 		   SENPS_protectTrig means protect trigger
 */
sensorProtectState_e sensorRangeProtect(const float _valueNS,const float _max,const float _min,sensorConnector_e _sensorConnetor){
	//NS means no sign,the variable should be passed by sign,so that the value can be original without sign.
	if(_valueNS >= _max){
		ws.emergency |= emergencyLimitUpper;
		switch (_sensorConnetor)
		{
		case ch4Load:
			ws.limitTypeWord |=limitUpperSRload; 
			ws.limitTypeWord &=(~limitDownSRload); 
			break;
		case ch2Ext1:
			ws.limitTypeWord |=limitUpperSRext1;
			ws.limitTypeWord &=(~limitDownSRext1); 
			break;
		case ch3Ext2:
			ws.limitTypeWord |=limitUpperSRext2; 
			ws.limitTypeWord &=(~limitDownSRext2);
			break;
		default:
			break;
		}
		return SENPS_protectUpperTrig;
	}else if(_valueNS <= _min){
		ws.emergency |= emergencyLimitDown;
		switch (_sensorConnetor)
		{
		case ch4Load:
			ws.limitTypeWord |=limitDownSRload; 
			ws.limitTypeWord &=(~limitUpperSRload); 
			break;
		case ch2Ext1:
			ws.limitTypeWord |=limitDownSRext1; 
			ws.limitTypeWord &=(~limitUpperSRext1); 
			break;
		case ch3Ext2:
			ws.limitTypeWord |=limitDownSRext2;
			ws.limitTypeWord &=(~limitUpperSRext2); 
			break;
		default:
			break;
		}
		return SENPS_protectDownTrig;
	}else{
		switch (_sensorConnetor)
		{
		case ch4Load:
			ws.limitTypeWord &=(~limitDownSRload); 
			ws.limitTypeWord &=(~limitUpperSRload); 
			break;
		case ch2Ext1:
			ws.limitTypeWord &=(~limitDownSRext1); 
			ws.limitTypeWord &=(~limitUpperSRext1); 
			break;
		case ch3Ext2:
			ws.limitTypeWord &=(~limitDownSRext2);
			ws.limitTypeWord &=(~limitUpperSRext2); 
			break;
		default:
			break;
		}
		return SENPS_noProtect;
	}
}

void sftLimitMonitor(float _pose,float _force,float _strain1,float _strain2){
	static uint16_t _emergencyRecord = 0;
	uint8_t _control = POS_MODE;
	float _position = 0;
	_emergencyRecord = ws.emergency;

	//pose
	if(_pose > posAllCtrlPara.Sft.UpperSft){
		//upper limit trigger
		ws.limitTypeWord |= limitUpperSWpos;
		ws.limitTypeWord &= (~limitDownSWpos);
		_control = POS_MODE;
		_position = _pose;
	}else if(_pose < posAllCtrlPara.Sft.LowerSft){
		//lower limit trigger
		ws.limitTypeWord |= limitDownSWpos;
		ws.limitTypeWord &= (~limitUpperSWpos);
		_control = POS_MODE;
		_position = _pose;
	}else{
		ws.limitTypeWord &= (~limitDownSWpos);
		ws.limitTypeWord &= (~limitUpperSWpos);
	}
	
	//force
	if(_force > loadAllCtrlPara.Sft.UpperSft){
		//upper limit trigger
		ws.limitTypeWord |= limitUpperSWload;
		ws.limitTypeWord &= (~limitDownSWload);
		_control = LOAD_MODE;
		_position = _force;
	}else if(_force < loadAllCtrlPara.Sft.LowerSft){
		//lower limit trigger
		ws.limitTypeWord |= limitDownSWload;
		ws.limitTypeWord &= (~limitUpperSWload);
		_control = LOAD_MODE;
		_position = _force;
	}else{
		ws.limitTypeWord &= (~limitDownSWload);
		ws.limitTypeWord &= (~limitUpperSWload);
	}
	
	//strain1
	if(_strain1 > extAllCtrlPara.Sft.UpperSft){
		//upper limit trigger
		ws.limitTypeWord |= limitUpperSWext1;
		ws.limitTypeWord &= (~limitDownSWext1);
		_control = EXTEN_MODE;
		_position = _strain1;		
	}else if (_strain1 < extAllCtrlPara.Sft.LowerSft)
	{
		//lower limit trigger
		ws.limitTypeWord |= limitDownSWext1;
		ws.limitTypeWord &= (~limitUpperSWext1);
		_control = EXTEN_MODE;
		_position = _strain1;	
	}
	else{
		ws.limitTypeWord &= (~limitDownSWext1);
		ws.limitTypeWord &= (~limitUpperSWext1);
	}
	/*
	//maybe need to add a new variable called "ext2AllCtrlPara",so that new sft in ext2AllCtrlPara can be used to protect strain2.
	//strain2 
	if(_strain2 > extAllCtrlPara.Sft.UpperSft){
		//upper limit trigger
		ws.emergency |= emergencySftUpper;
		ws.limitTypeWord |= limitUpperSWext2;
		ws.limitTypeWord &= (~limitDownSWext2);
		_control = EXTEN_MODE;
		_position = _strain2;			
	}else if (_strain2 < extAllCtrlPara.Sft.LowerSft)
	{
		//lower limit trigger
		ws.emergency |= emergencySftDown;
		ws.limitTypeWord |= limitDownSWext2;
		ws.limitTypeWord &= (~limitUpperSWext2);
		_control = EXTEN_MODE;
		_position = _strain2;
	}else{
		ws.limitTypeWord &= (~limitDownSWext2);
		ws.limitTypeWord &= (~limitUpperSWext2);
	}
	*/
	if(ws.limitTypeWord & 0x5500!= 0){
		ws.emergency |= emergencySftUpper;
	}else{
		ws.emergency &= (~emergencySftUpper);
	}

	if(ws.limitTypeWord & 0xAA00 != 0){
		ws.emergency |= emergencySftDown;	
	}else{
		ws.emergency &= (~emergencySftDown);
	}

	
	if((ws.emergency & emergencySftUpper) == 1 && _emergencyRecord != ws.emergency ){
		utcSetOnSftMsg.Upper = 1;
		utcSetOnSftMsg.Time = AL.utcTime;
		utcSetOnSftMsg.Control = _control;
		utcSetOnSftMsg.Position = _position;
		mySemaphore.utcSetOn.SetOnSft = 1;
	}else if((ws.emergency & emergencySftDown) == 1 && _emergencyRecord != ws.emergency){
		utcSetOnSftMsg.Upper = 0;	
		utcSetOnSftMsg.Time = AL.utcTime;
		utcSetOnSftMsg.Control = _control;
		utcSetOnSftMsg.Position = _position;
		mySemaphore.utcSetOn.SetOnSft = 1;
	}

}

void sensorRangeMonitor(float _forceNS,float _strain1NS,float _strain2NS){
	//NS means no sign,the variable should be passed by sign,so that the value can be original without sign.
	//load sensor
	if (_forceNS > AL.loadCtrl.NominalValue)
	{
		/* code */
	}
	

	//extensometer sensor1


	//extensometer sensor2



}

/**
 * @brief 澶勭悊闄愪綅淇濇姢閫昏緫锛屽寘鎷‖浠堕檺浣嶅拰杞檺浣嶄繚鎶?
 * 
 * 姝ゅ嚱鏁颁細妫€鏌ョ‖浠跺拰杞欢鐨勪笂涓嬮檺浣嶈Е鍙戞儏鍐碉紝鏍规嵁涓嶅悓鐨勯檺浣嶈Е鍙戞潯浠跺拰褰撳墠鐨凱WM淇濇姢鍊硷紝
 * 绂佺敤鐩稿簲鐨凱WM杈撳嚭锛屼互纭繚绯荤粺鐨勫畨鍏ㄨ繍琛屻€?
 * 
 * 娉ㄦ剰锛?.淇濇姢鍑芥暟鏈夊疄闄呮晥鏋滅殑鍓嶆彁鏄細鍦ㄥ簲鐢ㄥ眰鏄剧ず鐨勪綅缃柟鍚戜笌瀹為檯鐨勪綅缃柟鍚戠浉鍚岋紒
 * 		 2.鏈繚鎶ら€昏緫閮芥槸鍩轰簬闂幆鎺у埗鐨勮緭鍑篜WM鎴朅O鍊硷紝杩欓噷鐗规寚娌℃湁杩涘叆杈撳嚭鏄犲皠鍓嶇殑鍊笺€?
 * 		   鍥犱负鍦ㄨ緭鍑烘槧灏勫悗浼氳杈撳嚭绗﹀彿鏀瑰彉PWM杈撳嚭鐨勬柟鍚戯紝涓嶈兘浠ユ敼鍙樺悗鐨凱WM鍊兼潵鍒ゆ柇銆?
 * 		   鎵€浠ヨ繖閲岀殑PWM淇濇姢鍊兼槸娌℃湁缁忚繃杈撳嚭鏄犲皠鐨勩€?
 * 
 * 淇濇姢娴佺▼锛氣憼->鈶?>鈶?
 * 			鈶燛mergency_Monitor()鍑芥暟鍦╨oopctrl鏈€寮€濮嬭璋冪敤锛屾娴嬬‖浠堕檺浣嶈Е鍙戞儏鍐?
 * 			鈶ftLimitMonitor()鍑芥暟鍦ㄦ暟鎹緭鍏ユ槧灏勫悗琚皟鐢紝妫€娴嬭蒋浠堕檺浣嶈Е鍙戞儏鍐?
 * 			鈶imitProtectProcess()鍑芥暟鍦╨oopctrl鏈€鍚庤璋冪敤锛屾牴鎹檺浣嶈Е鍙戞儏鍐电鐢ㄧ浉鍏宠緭鍑?
 * 
 * @param 鏃?
 * @date:2025-3-3 23:30:58
 * @retval 鏃?
 */
void limitProtectProcess(void){
	static uint16_t upperCounter = 0,lowerCounter = 0;
	//sensor range protect monitor
	//娴嬭瘯浣跨敤 閲忕▼鐨?%鐨刴ax min
	sensorRangeProtect(force.orig*AL.loadCtrl.sign,AL.loadCtrl.NominalValue*1000.0f*0.75f,-AL.loadCtrl.NominalValue*1000.0f*0.75f,ch4Load);
	sensorRangeProtect(strain1.orig*AL.ext1Ctrl.sign,AL.ext1Ctrl.NominalValue*0.5f,-AL.ext1Ctrl.NominalValue*0.5f,ch2Ext1);
	//sensorRangeProtect(force.orig*AL.ext2Ctrl.sign,AL.ext2Ctrl.NominalValue*1000.0f,-AL.ext2Ctrl.NominalValue*1000.0f,ch3Ext2);
	//hardware limit protect process
	//hardware upper limit
	if((ws.emergency & emergencyLimitUpper) != 0){
		if(AL.svPWMforProtect > 0){
			hal_output.PWM = 0;
			upperCounter++;
			lowerCounter = 0;
			if(upperCounter >= 5){
				upperCounter = 0;
				//鎸囦护娓呴浂
				AL.complete_state.flag = 1;
			}

		}
		if(AL.AOforProtect > 0){
			hal_output.AO = 0;
			AL.complete_state.flag = 1;
		}
	}
	//hardware lower limit
	if((ws.emergency & emergencyLimitDown) != 0){
		if(AL.svPWMforProtect < 0){
			hal_output.PWM = 0;
			lowerCounter++;
			upperCounter = 0;
			if(lowerCounter >= 5){
				lowerCounter = 0;
				//鎸囦护娓呴浂
				AL.complete_state.flag = 1;
			}
		}
		if(AL.AOforProtect < 0){
			hal_output.AO = 0;
			AL.complete_state.flag = 1;
		}
	}
	//sft limit protect process 
	//闇€瑕佹敼鎴愰拡瀵逛笉鍚屾ā寮忚Е鍙戠殑杞檺浣嶈繘琛屼笉鍚岀殑澶勭悊锛屾柟娉曟坊鍔犺蒋闄愪綅瑙﹀彂鐨勬爣蹇楃绫?渚嬪锛歟mergencySftUpperPos
	//software upper limit
	if((ws.emergency & emergencySftUpper) != 0){
		if(AL.svPWMforProtect > 0){
			hal_output.PWM = 0;
			AL.complete_state.flag = 1;
		}
		if(AL.AOforProtect > 0){
			hal_output.AO = 0;
			AL.complete_state.flag = 1;
		}
	}
	//software lower limit
	if((ws.emergency & emergencySftDown) != 0){
		if(AL.svPWMforProtect < 0){
			hal_output.PWM = 0;
			AL.complete_state.flag = 1;
		}
		if(AL.AOforProtect < 0){
			hal_output.AO = 0;
			AL.complete_state.flag = 1;
		}
	}
}

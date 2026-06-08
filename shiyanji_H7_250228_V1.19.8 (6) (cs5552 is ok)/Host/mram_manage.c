#include "mram_manage.h"
#include <elog.h>
#include "usart.h"
#include "sensor.h"

/**
 * @brief read check num,and then verify whether the num is equal to MRAM_CHECK
 * @retval err if equal, err = 0. if not equal,err = 1. if read err,err = 2.
*/
uint8_t mramCheck(void){
	uint8_t err = 0,check_num,rc = 0;
	rc = MRAM_READ(CHEAK_ADDR,&check_num,1);//read 1 byte
	if(rc){
		log_e("read mramCheckNum failure:%d",rc);
		err = 2;
	}
	if(check_num == MRAM_CHECK)
		err = 0;
	else
		err = 1;
	return err;
}

/**
 * @brief		write DefaultAcc in marm 
 * @param[out]	x 			type:ALLCTRLPARA*
 * @param[in]	movectrl	type:uint8_t
 * @retval		err-number of data write errors
*/
uint8_t mramDefaultAccWrite(ALLCTRLPARA* x,uint8_t movectrl)
{
	uint8_t err=0;
	uint16_t acc_addr=0;

	switch (movectrl)
	{
	case POS_MODE:
		acc_addr = Posallctrlpara_DefaultAcc_ADDR;
		break;
	case LOAD_MODE:
		acc_addr = Loadallctrlpara_DefaultAcc_ADDR;
		break;
	case EXTEN_MODE:
		acc_addr = Extallctrlpara_DefaultAcc_ADDR;
		break;
	default:
		break;
	}
	err += MRAM_WRITE(acc_addr, (uint8_t*)(&x->DefaultAcc),4);//write 4 byte

	if(err > 0)
		log_e("MRAM %2x DefaultAcc WRerr:%d\r\n",movectrl,err);
	return err;
}

/**
 * @brief		write Speed in mram 
 * @param[out]	x 			type:ALLCTRLPARA*
 * @param[in]	movectrl	type:uint8_t
 * @retval		err-number of data write errors
*/
uint8_t mramSpeedWrite(ALLCTRLPARA* x,uint8_t movectrl)
{
	uint8_t err=0;
	uint16_t speed_addr=0;

	switch (movectrl)
	{
	case POS_MODE:
		speed_addr = Posallctrlpara_Speed_ADDR;
		break;
	case LOAD_MODE:
		speed_addr = Loadallctrlpara_Speed_ADDR;
		break;
	case EXTEN_MODE:
		speed_addr = Extallctrlpara_Speed_ADDR;
		break;
	default:
		break;
	}
	err += MRAM_WRITE(speed_addr, (uint8_t*)(&x->Speed),4);//write 4 byte
	
	if(err > 0)
		log_e("MRAM %2x Speed WRerr:%d\r\n",movectrl,err);
	return err;
}

/**
 * @brief		write Nominal acc and speed in mram 
 * @param[out]	x 			type:NOMINALVALUE*
 * @param[in]	movectrl	type:uint8_t
 * @retval		err-number of data write errors
*/
uint8_t mramNominalWrite(NOMINALVALUE* x,uint8_t movectrl)
{
	uint8_t err=0;
	uint16_t acc_addr=0,speed_addr=0;

	switch (movectrl)
	{
	case POS_MODE:
		acc_addr   = Posallctrlpara_NominalAcc_ADDR;
		speed_addr = Posallctrlpara_NominalSpeed_ADDR;
		break;
	case LOAD_MODE:
		acc_addr   = Loadallctrlpara_NominalAcc_ADDR;
		speed_addr = Loadallctrlpara_NominalSpeed_ADDR;
		break;
	case EXTEN_MODE:
		acc_addr   = Extallctrlpara_NominalAcc_ADDR;
		speed_addr = Extallctrlpara_NominalSpeed_ADDR;
		break;
	default:
		break;
	}
	err += MRAM_WRITE(acc_addr, (uint8_t*)(&x->Acc),4);//write 4 byte
	err += MRAM_WRITE(speed_addr, (uint8_t*)(&x->Speed),4);//write 4 byte
	if(err > 0)
		log_e("MRAM %2x Nominal acc and speed WRerr:%d\r\n",movectrl,err);
	return err;
}

/** 
 * @brief		write transmit data period in mram
 * @param[out]	x 			type:SysPara*
 * @retval		err-number of data write errors
*/
uint8_t mramTansmitDataPeriodWrite(SYSPARA* x)
{
	uint8_t err=0;
	err += MRAM_WRITE(SYSPARA_TRANSMITDATAPERIOD_ADDR, (uint8_t*)(&x->TransmitDataPeriod),4);//write 4 byte
	if(err > 0)
		log_e("MRAM %2x TansmitDataPeriod WRerr:%d\r\n",err);
	return err;
}

/** 
 * @brief		write Wnd size and time in mram
 * @param[out]	x 			type:WND*
 * @param[in]	movectrl	type:uint8_t
 * @retval		err-number of data write errors
*/
uint8_t mramWndWrite(WND* x,uint8_t movectrl)
{
	uint8_t err=0;
	uint16_t size_addr=0,time_addr=0;
	switch (movectrl)
	{
	case POS_MODE:
		size_addr = Posallctrlpara_Wnd_WndSize_ADDR;
		time_addr = Posallctrlpara_Wnd_WndTime_ADDR;
		break;
	case LOAD_MODE:
		size_addr = Loadallctrlpara_Wnd_WndSize_ADDR;
		time_addr = Loadallctrlpara_Wnd_WndTime_ADDR;
		break;
	case EXTEN_MODE:
		size_addr = Extallctrlpara_Wnd_WndSize_ADDR;
		time_addr = Extallctrlpara_Wnd_WndTime_ADDR;
		break;
	default:
		break;
	}
	err += MRAM_WRITE(size_addr, (uint8_t*)(&x->WndSize),4);
	err += MRAM_WRITE(time_addr, (uint8_t*)(&x->WndTime),4);//write 4 byte
	if(err > 0)
		log_e("MRAM %2x Wnd size&time WRerr:%d\r\n",movectrl,err);
	return err;
}

/**
 * @brief		write software lower, upper and reaction in mram
 * @param[out]	x 			type:SFT*
 * @param[in]	movectrl	type:uint8_t
 * @retval		err-number of data write errors
*/
uint8_t mramSftWrite(SFT* x,uint8_t movectrl)
{
	uint8_t err=0;
	uint16_t lowersft_addr=0,uppersft_addr=0,reaction_addr=0;
	switch (movectrl)
	{
	case POS_MODE:
		lowersft_addr = Posallctrlpara_Sft_LowerStf_ADDR;
		uppersft_addr = Posallctrlpara_Sft_UpperSft_ADDR;
		reaction_addr = Posallctrlpara_Sft_Reaction_ADDR;
		break;
	case LOAD_MODE:
		lowersft_addr = Loadallctrlpara_Sft_LowerStf_ADDR;
		uppersft_addr = Loadallctrlpara_Sft_UpperSft_ADDR;
		reaction_addr = Loadallctrlpara_Sft_Reaction_ADDR;
		break;
	case EXTEN_MODE:
		lowersft_addr = Extallctrlpara_Sft_LowerStf_ADDR;
		uppersft_addr = Extallctrlpara_Sft_UpperSft_ADDR;
		reaction_addr = Extallctrlpara_Sft_Reaction_ADDR;
		break;
	default:
		break;
	}
	err += MRAM_WRITE(lowersft_addr, (uint8_t*)(&x->LowerSft),4);
	err += MRAM_WRITE(uppersft_addr, (uint8_t*)(&x->UpperSft),4);
	err += MRAM_WRITE(reaction_addr, (uint8_t*)(&x->Reaction),4);
	if(err > 0)
		log_e("MRAM %2x Sft lower&upper&reaction WRerr:%d\r\n",movectrl,err);
	return err;
}

/**
 * @brief		write PosPID parameters in mram 
 * @param[out]	x 			type:PID*
 * @param[in]	movectrl	type:uint8_t
 * @retval		err-number of data write errors
*/
uint8_t mramPosPIDWrite(PID* x,uint8_t movectrl)
{
	uint8_t err=0;
	uint16_t p_addr=0,i_addr=0,d_addr=0;
	switch (movectrl)
	{
	case POS_MODE:
		p_addr = Posallctrlpara_PosPid_P_ADDR;
		i_addr = Posallctrlpara_PosPid_I_ADDR;
		d_addr = Posallctrlpara_PosPid_D_ADDR;
		break;
	case LOAD_MODE:
		p_addr = Loadallctrlpara_PosPid_P_ADDR;
		i_addr = Loadallctrlpara_PosPid_I_ADDR;
		d_addr = Loadallctrlpara_PosPid_D_ADDR;
		break;
	case EXTEN_MODE:
		p_addr = Extallctrlpara_PosPid_P_ADDR;
		i_addr = Extallctrlpara_PosPid_I_ADDR;
		d_addr = Extallctrlpara_PosPid_D_ADDR;
		break;
	default:
		break;
	}
	err += MRAM_WRITE(p_addr, (uint8_t*)(&x->P),4);
	err += MRAM_WRITE(i_addr, (uint8_t*)(&x->I),4);
	err += MRAM_WRITE(d_addr, (uint8_t*)(&x->D),4);
	if(err > 0)
		log_e("MRAM %2x PosPID  WRerr:%d\r\n",movectrl,err);
	return err;
}

/**
 * @brief		write SpeedPID parameters in mram 
 * @param[out]	x 			type:PID*
 * @param[in]	movectrl	type:uint8_t
 * @retval		err-number of data write errors
*/
uint8_t mramSpeedPIDWrite(PID* x,uint8_t movectrl)
{
	uint8_t err=0;
	uint16_t p_addr=0,i_addr=0,d_addr=0;
	switch (movectrl)
	{
	case POS_MODE:
		p_addr = Posallctrlpara_SpeedPid_P_ADDR;
		i_addr = Posallctrlpara_SpeedPid_I_ADDR;
		d_addr = Posallctrlpara_SpeedPid_D_ADDR;
		break;
	case LOAD_MODE:
		p_addr = Loadallctrlpara_SpeedPid_P_ADDR;
		i_addr = Loadallctrlpara_SpeedPid_I_ADDR;
		d_addr = Loadallctrlpara_SpeedPid_D_ADDR;
		break;
	case EXTEN_MODE:
		p_addr = Extallctrlpara_SpeedPid_P_ADDR;
		i_addr = Extallctrlpara_SpeedPid_I_ADDR;
		d_addr = Extallctrlpara_SpeedPid_D_ADDR;
		break;
	default:
		break;
	}
	err += MRAM_WRITE(p_addr, (uint8_t*)(&x->P),4);
	err += MRAM_WRITE(i_addr, (uint8_t*)(&x->I),4);
	err += MRAM_WRITE(d_addr, (uint8_t*)(&x->D),4);
	if(err > 0)
		log_e("MRAM %2x SpeedPid  WRerr:%d\r\n",movectrl,err);
	return err;
}

/**
 * @brief		write Ffd parameters in mram
 * @param[out]	x 			type:FFD*
 * @param[in]	movectrl	type:uint8_t
 * @retval		err-number of data write errors
*/
uint8_t mramFfdWrite(FFD* x,uint8_t movectrl)
{
	uint8_t err=0;
	uint16_t ffp_addr=0,posdelay_addr=0;
	switch (movectrl)
	{
	case POS_MODE:
		ffp_addr = Posallctrlpara_Ffd_SpeedFFP_ADDR;
		posdelay_addr = Posallctrlpara_Ffd_PosDelay_ADDR;
		break;
	case LOAD_MODE:
		ffp_addr = Loadallctrlpara_Ffd_SpeedFFP_ADDR;
		posdelay_addr = Loadallctrlpara_Ffd_PosDelay_ADDR;
		break;
	case EXTEN_MODE:
		ffp_addr = Extallctrlpara_Ffd_SpeedFFP_ADDR;
		posdelay_addr = Extallctrlpara_Ffd_PosDelay_ADDR;
		break;
	default:
		break;
	}
	err += MRAM_WRITE(ffp_addr, (uint8_t*)(&x->SpeedFFP),4);
	err += MRAM_WRITE(posdelay_addr, (uint8_t*)(&x->PosDelay),4);
	if(err > 0)
		log_e("MRAM %2x Ffd  WRerr:%d\r\n",movectrl,err);
	return err;
}

/**
 * @brief		write output parameters in mram
 * @param		x	type:outputpara*
 * @retval		err-number of data write errors
*/
uint8_t mramOutputParaWriteAll(OUTPUTPARA* x)
{
	uint8_t err=0;
	err += MRAM_WRITE(OUTPUTPARA_SIGNAL_ADDR, (&x->Signal), 1);
	err += MRAM_WRITE(OUTPUTPARA_SIGN_ADDR, (&x->Sign), 1);
	err += MRAM_WRITE(OUTPUTPARA_MAXVALUE_ADDR, (uint8_t*)(&x->MaxValue), 4);
	err += MRAM_WRITE(OUTPUTPARA_MINVALUE_ADDR, (uint8_t*)(&x->MinValue), 4);
	err += MRAM_WRITE(OUTPUTPARA_INITVALUE_ADDR, (uint8_t*)(&x->InitValue), 4);
	err += MRAM_WRITE(OUTPUTPARA_OFFSET_ADDR, (uint8_t*)(&x->Offset), 4);
	err += MRAM_WRITE(OUTPUTPARA_SIGNALFREQUENCY_ADDR, (uint8_t*)(&x->SignalFrequency), 4);
	err += MRAM_WRITE(OUTPUTPARA_MAXVOLTAGE_ADDR, (uint8_t*)(&x->MaxVoltage), 4);
	err += MRAM_WRITE(OUTPUTPARA_MAXCURRENT_ADDR, (uint8_t*)(&x->MaxCurrent), 4);

	if(err > 0)
		log_e("MRAM OutputPara WRerr:%d\r\n",err);
	return err;
}

/**
 * @brief		write system parameters in mram
 * @param		x	type:syspara*
 * @retval		err-number of data write errors
*/
uint8_t mramSysParaWriteAll(SYSPARA* x)
{
	uint8_t err=0;
	err += MRAM_WRITE(SYSPARA_TRANSMITDATAPERIOD_ADDR,(uint8_t*)(&x->TransmitDataPeriod), 2);
	err += MRAM_WRITE(SYSPARA_CONTROLLERSTRUCTURE_ADDR, (uint8_t*)(&x->ControllerStructure), 1);
	if(err > 0)
		log_e("MRAM SysPara  WRerr:%d\r\n",err);
	return err;
}

/**
 * @brief		write all senser parameter in mram
 * @note		The applicable sensors include: position sensor, force sensor, extensometer
 * @param		x type:sensorData_t*
 * @retval		err-number of data write errors
*/

//Sensor Content 
//400Byte 100-499
uint8_t mramSensorWriteAll(sensorData_t* x)
{
	uint8_t err=0,i;
	err += MRAM_WRITE(SensorType_ADDR, (&x->Sensortype), 1);
	err += MRAM_WRITE(Sign_ADDR, (&x->Sign), 1);
	err += MRAM_WRITE(NominalValue_ADDR, (uint8_t*)(&x->NominalValue), 4);
	err += MRAM_WRITE(NominalSensitive_ADDR, (uint8_t*)(&x->NominalSensitive), 4);
	err += MRAM_WRITE(Day_ADDR, (&x->Day), 1);
	err += MRAM_WRITE(Month_ADDR, (&x->Month), 1);
	err += MRAM_WRITE(Year_ADDR, (uint8_t*)(&x->Year), 2);
	err += MRAM_WRITE(LinPoint_ADDR, (&x->LinPoint), 1);

	for(i=0;i<SEN_LIN_DATA_MAX;i++)
	{
		err += MRAM_WRITE(LinV_ADDR+12*i, (uint8_t*)(&x->LinV[i].ADCCode), 4);
		err += MRAM_WRITE(LinV_ADDR+4+12*i,(uint8_t*)(&x->LinV[i].RefValue), 4);
		err += MRAM_WRITE(LinV_ADDR+8+12*i,(uint8_t*)(&x->LinV[i].CorrectionFactor), 4);
	}
	if(err > 0)
		log_e("mram WRerr:%d\r\n",err);
	return err;
}

//Sensor Pose Content 
//400Byte 5600-5999
uint8_t mramSensorPoseWriteAll(sensorData_t* x)
{
	uint8_t err=0,i;
	err += MRAM_WRITE(SensorType_POSE_ADDR, (&x->Sensortype), 1);
	err += MRAM_WRITE(Sign_POSE_ADDR, (&x->Sign), 1);
	err += MRAM_WRITE(NominalValue_POSE_ADDR, (uint8_t*)(&x->NominalValue), 4);
	err += MRAM_WRITE(NominalSensitive_POSE_ADDR, (uint8_t*)(&x->NominalSensitive), 4);
	err += MRAM_WRITE(Day_POSE_ADDR, (&x->Day), 1);
	err += MRAM_WRITE(Month_POSE_ADDR, (&x->Month), 1);
	err += MRAM_WRITE(Year_POSE_ADDR, (uint8_t*)(&x->Year), 2);
	err += MRAM_WRITE(LinPoint_POSE_ADDR, (&x->LinPoint), 1);

	for(i=0;i<SEN_LIN_DATA_MAX;i++)
	{
		err += MRAM_WRITE(LinV_POSE_ADDR+12*i, (uint8_t*)(&x->LinV[i].ADCCode), 4);
		err += MRAM_WRITE(LinV_POSE_ADDR+4+12*i,(uint8_t*)(&x->LinV[i].RefValue), 4);
		err += MRAM_WRITE(LinV_POSE_ADDR+8+12*i,(uint8_t*)(&x->LinV[i].CorrectionFactor), 4);
	}
	if(err > 0)
		log_e("mram WRerr:%d\r\n",err);
	return err;
}

/**
 * @brief		write sensor check parameters in mram
 * @param		x	type:SENSER_CHECK_STRUCT*
 * @retval		err-number of data write errors
*/
uint8_t mramSensorCheckWriteAll(SENSER_CHECK_STRUCT* x)
{
	uint8_t err=0;
	err += MRAM_WRITE(SENSOR_CHECK_REGISTER0_ADDR, (uint8_t*)(&x->Register[0]), 1);
	err += MRAM_WRITE(SENSOR_CHECK_REGISTER1_ADDR, (uint8_t*)(&x->Register[1]), 1);
	err += MRAM_WRITE(SENSOR_CHECK_REGISTER2_ADDR, (uint8_t*)(&x->Register[2]), 1);
	err += MRAM_WRITE(SENSOR_CHECK_MAPNUM0_ADDR, (uint8_t*)(&x->MapNum[0]), 1);
	err += MRAM_WRITE(SENSOR_CHECK_MAPNUM1_ADDR, (uint8_t*)(&x->MapNum[1]), 1);
	err += MRAM_WRITE(SENSOR_CHECK_MAPNUM2_ADDR, (uint8_t*)(&x->MapNum[2]), 1);
	if(err > 0)
		log_e("MRAM SensorCheck WRerr:%d\r\n",err);
	return err;
}

/**
 * @brief		write sensor intgr in mram
 * @param[out]	x 			sensor intgr  type:uint32_t 
 * @param[in]	channel		sensor connector type:SENSOR_CONNECTOR
 * @retval		err-number of data write errors
*/
uint8_t mramIntgrWrite(uint32_t* x,sensorConnector_e channel){
	uint8_t err=0;
	uint16_t intgr_addr=0;
	switch (channel)
	{
	case ch0Pose:
		intgr_addr = INTGR_POSE_ADDR;
		break;
	case ch1Bd:
		intgr_addr = INTGR_BIGDERFORMATION_ADDR;
		break;
	case ch2Ext1:
		intgr_addr = INTGR_EXTEN1_ADDR;
		break;
	case ch3Ext2:
		intgr_addr = INTGR_EXTEN2_ADDR;
		break;
	case ch4Load:
		intgr_addr = INTGR_LOAD_ADDR;
		break;
	default:
		break;
	}
	err += MRAM_WRITE(intgr_addr, (uint8_t*)x,4);
	if(err > 0)
		log_e("MRAM channel %d Intgr WRerr:%d\r\n",channel,err);
	log_d("MRAM channel %d Intgr WR ok\r\n",channel);
	return err;
}

/**
 * @brief		write ethernet configuration in mram
 * @param[in]	x 			ethernet configuration content  type:ethConfig_t 
 * @retval		err-number of data write errors
*/
uint8_t mramEthWrite(const ethConfig_t* x){
	uint8_t err=0;
	err += MRAM_WRITE(ETH_MODE_ADDR, (uint8_t*)(&x->EthMode),1);
	err += MRAM_WRITE(ETH_SERVER_IP_ADDR, (uint8_t*)(&x->ServerIp),4);
	err += MRAM_WRITE(ETH_SERVER_PORT_ADDR, (uint8_t*)(&x->ServerPort),2);
	err += MRAM_WRITE(ETH_LOCAL_IP_ADDR, (uint8_t*)(&x->LocalIp),4);
	err += MRAM_WRITE(ETH_LOCAL_MAC_ADDR, (uint8_t*)(&x->LocalMac),6);

	if(err > 0){
		if(elog_init_ok)
			log_e("MRAM ethernet configure WRerr:%d\r\n",err);
		else
			printf("MRAM ethernet configure WRerr:%d\r\n",err);
	}else{
		if(elog_init_ok)
			log_d("MRAM ethernet configure WR ok\r\n");
		else
			printf("MRAM ethernet configure WR ok\r\n");
	}
	return err;
}


uint8_t mramServoParamWrite(const servoParam_t* x){
	uint8_t err=0;
	err += MRAM_WRITE(SERVOPARA_ELECTRONICGEAR_ADDR, (uint8_t*)(&x->electronicGear),4);
	err += MRAM_WRITE(SERVOPARA_ENCODERRESOLUTION_ADDR, (uint8_t*)(&x->encoderResolution),4);
	err += MRAM_WRITE(SERVOPARA_ANALOGGAIN_ADDR, (uint8_t*)(&x->analogGain),4);

	if(err > 0){
		log_e("MRAM servo param WRerr:%d\r\n",err);
	}else{
		log_d("MRAM servo param WR ok\r\n");
	}
	return err;
}

/**
 * @brief read all storage parameter, when system init stage.
 * @note function includes check step.
 * 
*/
uint8_t mramReadOnInit(void){
	uint8_t err=0;

	return err;
}

/**
 * @note	Fun_11011
 * @brief	write all storage parameter in mram
 * @retval		err-Number of data write errors
*/
uint8_t mramWriteAll(void){
uint8_t err=0,check_num=MRAM_CHECK,i=0;
	err += MRAM_WRITE(CHEAK_ADDR, &check_num,1);
	err += MRAM_WRITE(PosDefaultAcc_ADDR, (uint8_t*)(&commBuf.Pos.DefaultAcc), 4);
	err += MRAM_WRITE(PosDefaultDec_ADDR, (uint8_t*)(&commBuf.Pos.DefaultDec), 4);
	err += MRAM_WRITE(LoadDefaultAcc_ADDR, (uint8_t*)(&commBuf.Load.DefaultAcc), 4);
	err += MRAM_WRITE(LoadDefaultDec_ADDR, (uint8_t*)(&commBuf.Load.DefaultDec), 4);
	err += MRAM_WRITE(ExtDefaultAcc_ADDR, (uint8_t*)(&commBuf.Ext.DefaultAcc), 4);
	err += MRAM_WRITE(ExtDefaultDec_ADDR, (uint8_t*)(&commBuf.Ext.DefaultDec), 4);
	err += MRAM_WRITE(upperlimit_phy_ADDR, (uint8_t*)(&AL.upperlimit_phy), 4);
	err += MRAM_WRITE(lowerlimit_phy_ADDR, (uint8_t*)(&AL.lowerlimit_phy), 4);
	err += MRAM_WRITE(defaultspeed_ADDR, (uint8_t*)(&AL.defaultspeed), 4);
	err += MRAM_WRITE(defaultspeedlimit_ADDR, (uint8_t*)(&AL.defaultspeedlimit), 4);
		
	
//POS_MODE ???????? 400-514
	err += MRAM_WRITE(Posallctrlpara_PosPid_P_ADDR, (uint8_t*)(&posAllCtrlPara.PosPid.P), 4);
	err += MRAM_WRITE(Posallctrlpara_PosPid_I_ADDR, (uint8_t*)(&posAllCtrlPara.PosPid.I), 4);
	err += MRAM_WRITE(Posallctrlpara_PosPid_D_ADDR, (uint8_t*)(&posAllCtrlPara.PosPid.D), 4);
	err += MRAM_WRITE(Posallctrlpara_PosPid_Hp_P_ADDR, (uint8_t*)(&posAllCtrlPara.PosPid_Hp.P), 4);
	err += MRAM_WRITE(Posallctrlpara_PosPid_Hp_I_ADDR, (uint8_t*)(&posAllCtrlPara.PosPid_Hp.P), 4);
	err += MRAM_WRITE(Posallctrlpara_PosPid_Hp_D_ADDR, (uint8_t*)(&posAllCtrlPara.PosPid_Hp.P), 4);
	err += MRAM_WRITE(Posallctrlpara_SpeedPid_P_ADDR, (uint8_t*)(&posAllCtrlPara.SpeedPid.P), 4);
	err += MRAM_WRITE(Posallctrlpara_SpeedPid_I_ADDR, (uint8_t*)(&posAllCtrlPara.SpeedPid.I), 4);
	err += MRAM_WRITE(Posallctrlpara_SpeedPid_D_ADDR, (uint8_t*)(&posAllCtrlPara.SpeedPid.D), 4);
	err += MRAM_WRITE(Posallctrlpara_Ffd_SpeedFFP_ADDR, (uint8_t*)(&posAllCtrlPara.Ffd.SpeedFFP), 4);
	err += MRAM_WRITE(Posallctrlpara_Ffd_PosDelay_ADDR, (uint8_t*)(&posAllCtrlPara.Ffd.PosDelay), 4);
	err += MRAM_WRITE(Posallctrlpara_Ffd_Hp_SpeedFFP_ADDR, (uint8_t*)(&posAllCtrlPara.Ffd_Hp.SpeedFFP), 4);
	err += MRAM_WRITE(Posallctrlpara_Ffd_Hp_PosDelay_ADDR, (uint8_t*)(&posAllCtrlPara.Ffd_Hp.SpeedFFP), 4);
	err += MRAM_WRITE(Posallctrlpara_DefaultAcc_ADDR, (uint8_t*)(&posAllCtrlPara.DefaultAcc), 4);
	err += MRAM_WRITE(Posallctrlpara_Speed_ADDR, (uint8_t*)(&posAllCtrlPara.Speed), 4);
	err += MRAM_WRITE(Posallctrlpara_Deviation_ADDR, (uint8_t*)(&posAllCtrlPara.Deviation), 4);
	err += MRAM_WRITE(Posallctrlpara_DevReaction_ADDR, (&posAllCtrlPara.DevReaction), 1);
	err += MRAM_WRITE(Posallctrlpara_Wnd_WndSize_ADDR, (uint8_t*)(&posAllCtrlPara.Wnd.WndSize), 4);
	err += MRAM_WRITE(Posallctrlpara_Wnd_WndTime_ADDR, (uint8_t*)(&posAllCtrlPara.Wnd.WndTime), 4);
	err += MRAM_WRITE(Posallctrlpara_Sft_UpperSft_ADDR, (uint8_t*)(&posAllCtrlPara.Sft.UpperSft), 4);
	err += MRAM_WRITE(Posallctrlpara_Sft_LowerStf_ADDR, (uint8_t*)(&posAllCtrlPara.Sft.LowerSft), 4);
	err += MRAM_WRITE(Posallctrlpara_Sft_Reaction_ADDR, (&posAllCtrlPara.Sft.Reaction), 1);
	err += MRAM_WRITE(Posallctrlpara_MinAcceleration_ADDR, (uint8_t*)(&posAllCtrlPara.MinAcceleration), 4);
	err += MRAM_WRITE(Posallctrlpara_MaxAcceleration_ADDR, (uint8_t*)(&posAllCtrlPara.MaxAcceleration), 4);
	err += MRAM_WRITE(Posallctrlpara_MinDeceleration_ADDR, (uint8_t*)(&posAllCtrlPara.MinDeceleration), 4);
	err += MRAM_WRITE(Posallctrlpara_MaxDeceleration_ADDR, (uint8_t*)(&posAllCtrlPara.MaxDeceleration), 4);
	err += MRAM_WRITE(Posallctrlpara_MinSpeed_ADDR, (uint8_t*)(&posAllCtrlPara.MinSpeed), 4);
	err += MRAM_WRITE(Posallctrlpara_MaxSpeed_ADDR, (uint8_t*)(&posAllCtrlPara.MaxSpeed), 4);
	err += MRAM_WRITE(Posallctrlpara_Deadband_Deadband_ADDR, (uint8_t*)(&posAllCtrlPara.Deadband.Deadband), 4);
	err += MRAM_WRITE(Posallctrlpara_Deadband_PercentP_ADDR, (&posAllCtrlPara.Deadband.PercentP), 1);
	err += MRAM_WRITE(Posallctrlpara_Dither_DitherAmplitude_ADDR, (uint8_t*)(&posAllCtrlPara.Dither.DitherAmplitude), 4);
	err += MRAM_WRITE(Posallctrlpara_Dither_DitherFrequency_ADDR, (uint8_t*)(&posAllCtrlPara.Dither.DitherFrequency), 4);
	err += MRAM_WRITE(Posallctrlpara_NominalAcc_ADDR, (uint8_t*)(&posAllCtrlPara.Nominal.Acc), 4);
	err += MRAM_WRITE(Posallctrlpara_NominalSpeed_ADDR, (uint8_t*)(&posAllCtrlPara.Nominal.Speed), 4);

//LOAD_MODE ???????? 600-714
	err += MRAM_WRITE(Loadallctrlpara_PosPid_P_ADDR, (uint8_t*)(&loadAllCtrlPara.PosPid.P), 4);
	err += MRAM_WRITE(Loadallctrlpara_PosPid_I_ADDR, (uint8_t*)(&loadAllCtrlPara.PosPid.I), 4);
	err += MRAM_WRITE(Loadallctrlpara_PosPid_D_ADDR, (uint8_t*)(&loadAllCtrlPara.PosPid.D), 4);
	err += MRAM_WRITE(Loadallctrlpara_PosPid_Hp_P_ADDR, (uint8_t*)(&loadAllCtrlPara.PosPid_Hp.P), 4);
	err += MRAM_WRITE(Loadallctrlpara_PosPid_Hp_I_ADDR, (uint8_t*)(&loadAllCtrlPara.PosPid_Hp.P), 4);
	err += MRAM_WRITE(Loadallctrlpara_PosPid_Hp_D_ADDR, (uint8_t*)(&loadAllCtrlPara.PosPid_Hp.P), 4);
	err += MRAM_WRITE(Loadallctrlpara_SpeedPid_P_ADDR, (uint8_t*)(&loadAllCtrlPara.SpeedPid.P), 4);
	err += MRAM_WRITE(Loadallctrlpara_SpeedPid_I_ADDR, (uint8_t*)(&loadAllCtrlPara.SpeedPid.I), 4);
	err += MRAM_WRITE(Loadallctrlpara_SpeedPid_D_ADDR, (uint8_t*)(&loadAllCtrlPara.SpeedPid.D), 4);
	err += MRAM_WRITE(Loadallctrlpara_Ffd_SpeedFFP_ADDR, (uint8_t*)(&loadAllCtrlPara.Ffd.SpeedFFP), 4);
	err += MRAM_WRITE(Loadallctrlpara_Ffd_PosDelay_ADDR, (uint8_t*)(&loadAllCtrlPara.Ffd.PosDelay), 4);
	err += MRAM_WRITE(Loadallctrlpara_Ffd_Hp_SpeedFFP_ADDR, (uint8_t*)(&loadAllCtrlPara.Ffd_Hp.SpeedFFP), 4);
	err += MRAM_WRITE(Loadallctrlpara_Ffd_Hp_PosDelay_ADDR, (uint8_t*)(&loadAllCtrlPara.Ffd_Hp.SpeedFFP), 4);
	err += MRAM_WRITE(Loadallctrlpara_DefaultAcc_ADDR, (uint8_t*)(&loadAllCtrlPara.DefaultAcc), 4);
	err += MRAM_WRITE(Loadallctrlpara_Speed_ADDR, (uint8_t*)(&loadAllCtrlPara.Speed), 4);
	err += MRAM_WRITE(Loadallctrlpara_Deviation_ADDR, (uint8_t*)(&loadAllCtrlPara.Deviation), 4);
	err += MRAM_WRITE(Loadallctrlpara_DevReaction_ADDR, (&loadAllCtrlPara.DevReaction), 1);
	err += MRAM_WRITE(Loadallctrlpara_Wnd_WndSize_ADDR, (uint8_t*)(&loadAllCtrlPara.Wnd.WndSize), 4);
	err += MRAM_WRITE(Loadallctrlpara_Wnd_WndTime_ADDR, (uint8_t*)(&loadAllCtrlPara.Wnd.WndTime), 4);
	err += MRAM_WRITE(Loadallctrlpara_Sft_UpperSft_ADDR, (uint8_t*)(&loadAllCtrlPara.Sft.UpperSft), 4);
	err += MRAM_WRITE(Loadallctrlpara_Sft_LowerStf_ADDR, (uint8_t*)(&loadAllCtrlPara.Sft.LowerSft), 4);
	err += MRAM_WRITE(Loadallctrlpara_Sft_Reaction_ADDR, (&loadAllCtrlPara.Sft.Reaction), 1);
	err += MRAM_WRITE(Loadallctrlpara_MinAcceleration_ADDR, (uint8_t*)(&loadAllCtrlPara.MinAcceleration), 4);
	err += MRAM_WRITE(Loadallctrlpara_MaxAcceleration_ADDR, (uint8_t*)(&loadAllCtrlPara.MaxAcceleration), 4);
	err += MRAM_WRITE(Loadallctrlpara_MinDeceleration_ADDR, (uint8_t*)(&loadAllCtrlPara.MinDeceleration), 4);
	err += MRAM_WRITE(Loadallctrlpara_MaxDeceleration_ADDR, (uint8_t*)(&loadAllCtrlPara.MaxDeceleration), 4);
	err += MRAM_WRITE(Loadallctrlpara_MinSpeed_ADDR, (uint8_t*)(&loadAllCtrlPara.MinSpeed), 4);
	err += MRAM_WRITE(Loadallctrlpara_MaxSpeed_ADDR, (uint8_t*)(&loadAllCtrlPara.MaxSpeed), 4);
	err += MRAM_WRITE(Loadallctrlpara_Deadband_Deadband_ADDR, (uint8_t*)(&loadAllCtrlPara.Deadband.Deadband), 4);
	err += MRAM_WRITE(Loadallctrlpara_Deadband_PercentP_ADDR, (&loadAllCtrlPara.Deadband.PercentP), 1);
	err += MRAM_WRITE(Loadallctrlpara_Dither_DitherAmplitude_ADDR, (uint8_t*)(&loadAllCtrlPara.Dither.DitherAmplitude), 4);
	err += MRAM_WRITE(Loadallctrlpara_Dither_DitherFrequency_ADDR, (uint8_t*)(&loadAllCtrlPara.Dither.DitherFrequency), 4);
	err += MRAM_WRITE(Loadallctrlpara_NominalAcc_ADDR, (uint8_t*)(&loadAllCtrlPara.Nominal.Acc), 4);
	err += MRAM_WRITE(Loadallctrlpara_NominalSpeed_ADDR, (uint8_t*)(&loadAllCtrlPara.Nominal.Speed), 4);

//EXTEN_MODE ???????? 800-914
	err += MRAM_WRITE(Loadallctrlpara_PosPid_P_ADDR, (uint8_t*)(&extAllCtrlPara.PosPid.P), 4);
	err += MRAM_WRITE(Extallctrlpara_PosPid_I_ADDR, (uint8_t*)(&extAllCtrlPara.PosPid.I), 4);
	err += MRAM_WRITE(Extallctrlpara_PosPid_D_ADDR, (uint8_t*)(&extAllCtrlPara.PosPid.D), 4);
	err += MRAM_WRITE(Extallctrlpara_PosPid_Hp_P_ADDR, (uint8_t*)(&extAllCtrlPara.PosPid_Hp.P), 4);
	err += MRAM_WRITE(Extallctrlpara_PosPid_Hp_I_ADDR, (uint8_t*)(&extAllCtrlPara.PosPid_Hp.P), 4);
	err += MRAM_WRITE(Extallctrlpara_PosPid_Hp_D_ADDR, (uint8_t*)(&extAllCtrlPara.PosPid_Hp.P), 4);
	err += MRAM_WRITE(Extallctrlpara_SpeedPid_P_ADDR, (uint8_t*)(&extAllCtrlPara.SpeedPid.P), 4);
	err += MRAM_WRITE(Extallctrlpara_SpeedPid_I_ADDR, (uint8_t*)(&extAllCtrlPara.SpeedPid.I), 4);
	err += MRAM_WRITE(Extallctrlpara_SpeedPid_D_ADDR, (uint8_t*)(&extAllCtrlPara.SpeedPid.D), 4);
	err += MRAM_WRITE(Extallctrlpara_Ffd_SpeedFFP_ADDR, (uint8_t*)(&extAllCtrlPara.Ffd.SpeedFFP), 4);
	err += MRAM_WRITE(Extallctrlpara_Ffd_PosDelay_ADDR, (uint8_t*)(&extAllCtrlPara.Ffd.PosDelay), 4);
	err += MRAM_WRITE(Extallctrlpara_Ffd_Hp_SpeedFFP_ADDR, (uint8_t*)(&extAllCtrlPara.Ffd_Hp.SpeedFFP), 4);
	err += MRAM_WRITE(Extallctrlpara_Ffd_Hp_PosDelay_ADDR, (uint8_t*)(&extAllCtrlPara.Ffd_Hp.SpeedFFP), 4);
	err += MRAM_WRITE(Extallctrlpara_DefaultAcc_ADDR, (uint8_t*)(&extAllCtrlPara.DefaultAcc), 4);
	err += MRAM_WRITE(Extallctrlpara_Speed_ADDR, (uint8_t*)(&extAllCtrlPara.Speed), 4);
	err += MRAM_WRITE(Extallctrlpara_Deviation_ADDR, (uint8_t*)(&extAllCtrlPara.Deviation), 4);
	err += MRAM_WRITE(Extallctrlpara_DevReaction_ADDR, (&extAllCtrlPara.DevReaction), 1);
	err += MRAM_WRITE(Extallctrlpara_Wnd_WndSize_ADDR, (uint8_t*)(&extAllCtrlPara.Wnd.WndSize), 4);
	err += MRAM_WRITE(Extallctrlpara_Wnd_WndTime_ADDR, (uint8_t*)(&extAllCtrlPara.Wnd.WndTime), 4);
	err += MRAM_WRITE(Extallctrlpara_Sft_UpperSft_ADDR, (uint8_t*)(&extAllCtrlPara.Sft.UpperSft), 4);
	err += MRAM_WRITE(Extallctrlpara_Sft_LowerStf_ADDR, (uint8_t*)(&extAllCtrlPara.Sft.LowerSft), 4);
	err += MRAM_WRITE(Extallctrlpara_Sft_Reaction_ADDR, (&extAllCtrlPara.Sft.Reaction),1);
	err += MRAM_WRITE(Extallctrlpara_MinAcceleration_ADDR, (uint8_t*)(&extAllCtrlPara.MinAcceleration), 4);
	err += MRAM_WRITE(Extallctrlpara_MaxAcceleration_ADDR, (uint8_t*)(&extAllCtrlPara.MaxAcceleration), 4);
	err += MRAM_WRITE(Extallctrlpara_MinDeceleration_ADDR, (uint8_t*)(&extAllCtrlPara.MinDeceleration), 4);
	err += MRAM_WRITE(Extallctrlpara_MaxDeceleration_ADDR, (uint8_t*)(&extAllCtrlPara.MaxDeceleration), 4);
	err += MRAM_WRITE(Extallctrlpara_MinSpeed_ADDR, (uint8_t*)(&extAllCtrlPara.MinSpeed), 4);
	err += MRAM_WRITE(Extallctrlpara_MaxSpeed_ADDR, (uint8_t*)(&extAllCtrlPara.MaxSpeed), 4);
	err += MRAM_WRITE(Extallctrlpara_Deadband_Deadband_ADDR, (uint8_t*)(&extAllCtrlPara.Deadband.Deadband), 4);
	err += MRAM_WRITE(Extallctrlpara_Deadband_PercentP_ADDR, (&extAllCtrlPara.Deadband.PercentP),1);
	err += MRAM_WRITE(Extallctrlpara_Dither_DitherAmplitude_ADDR, (uint8_t*)(&extAllCtrlPara.Dither.DitherAmplitude), 4);
	err += MRAM_WRITE(Extallctrlpara_Dither_DitherFrequency_ADDR, (uint8_t*)(&extAllCtrlPara.Dither.DitherFrequency), 4);
	err += MRAM_WRITE(Extallctrlpara_NominalAcc_ADDR, (uint8_t*)(&extAllCtrlPara.Nominal.Acc), 4);
	err += MRAM_WRITE(Extallctrlpara_NominalSpeed_ADDR, (uint8_t*)(&extAllCtrlPara.Nominal.Speed), 4);

//system parameters 5000-5002
	err += MRAM_WRITE(SYSPARA_TRANSMITDATAPERIOD_ADDR, (uint8_t*)(&SysPara.TransmitDataPeriod), 2);
	err += MRAM_WRITE(SYSPARA_CONTROLLERSTRUCTURE_ADDR, (uint8_t*)(&SysPara.ControllerStructure),1);

//servo parameters 5050-5061
	err += MRAM_WRITE(SERVOPARA_ELECTRONICGEAR_ADDR, (uint8_t*)(&servoParam.electronicGear), 4);
	err += MRAM_WRITE(SERVOPARA_ENCODERRESOLUTION_ADDR, (uint8_t*)(&servoParam.encoderResolution), 4);
	err += MRAM_WRITE(SERVOPARA_ANALOGGAIN_ADDR, (uint8_t*)(&servoParam.analogGain), 4);

//output parameters 5100-5129
	err += MRAM_WRITE(OUTPUTPARA_SIGNAL_ADDR, (&OutputPara.Signal),1);
	err += MRAM_WRITE(OUTPUTPARA_SIGN_ADDR, (&OutputPara.Sign),1);
	err += MRAM_WRITE(OUTPUTPARA_MAXVALUE_ADDR, (uint8_t*)(&OutputPara.MaxValue), 4);
	err += MRAM_WRITE(OUTPUTPARA_MINVALUE_ADDR, (uint8_t*)(&OutputPara.MinValue), 4);
	err += MRAM_WRITE(OUTPUTPARA_INITVALUE_ADDR, (uint8_t*)(&OutputPara.InitValue), 4);
	err += MRAM_WRITE(OUTPUTPARA_OFFSET_ADDR, (uint8_t*)(&OutputPara.Offset), 4);
	err += MRAM_WRITE(OUTPUTPARA_SIGNALFREQUENCY_ADDR, (uint8_t*)(&OutputPara.SignalFrequency), 4);
	err += MRAM_WRITE(OUTPUTPARA_MAXVOLTAGE_ADDR,(uint8_t*)(&OutputPara.MaxVoltage), 4);
	err += MRAM_WRITE(OUTPUTPARA_MAXCURRENT_ADDR, (uint8_t*)(&OutputPara.MaxCurrent), 4);

//these data need to be saved realtime 5200-5211
	err += MRAM_WRITE(POSE_ENCODER_CODE_ADDR, (uint8_t*)(&pose.code), 4);
	err += MRAM_WRITE(MRAM_LOAD_TARE_VALUE_ADDR,(uint8_t*)(&AL.tare.fValue[ch4Load]), 4);
	err += MRAM_WRITE(MRAM_EXTEN_TARE_VALUE_ADDR, (uint8_t*)(&AL.tare.fValue[ch2Ext1]), 4);
	err += MRAM_WRITE(MRAM_EXTEN2_TARE_VALUE_ADDR,(uint8_t*)(&AL.tare.fValue[ch3Ext2]), 4);
	err += MRAM_WRITE(MRAM_LOAD_TARE_CODE_ADDR, (uint8_t*)(&AL.tare.value[ch4Load]), 4);
	err += MRAM_WRITE(MRAM_EXTEN_TARE_CODE_ADDR,(uint8_t*)(&AL.tare.value[ch2Ext1]), 4);
	err += MRAM_WRITE(MRAM_EXTEN2_TARE_CODE_ADDR, (uint8_t*)(&AL.tare.value[ch3Ext2]), 4);

//senserCheck 5400-5405
	err += MRAM_WRITE(SENSOR_CHECK_REGISTER0_ADDR, (uint8_t*)(&sensorCheck.Register[0]), 1);
	err += MRAM_WRITE(SENSOR_CHECK_REGISTER1_ADDR, (uint8_t*)(&sensorCheck.Register[1]), 1);
	err += MRAM_WRITE(SENSOR_CHECK_REGISTER2_ADDR, (uint8_t*)(&sensorCheck.Register[2]), 1);
	err += MRAM_WRITE(SENSOR_CHECK_MAPNUM0_ADDR, (uint8_t*)(&sensorCheck.MapNum[0]), 1);
	err += MRAM_WRITE(SENSOR_CHECK_MAPNUM1_ADDR, (uint8_t*)(&sensorCheck.MapNum[1]), 1);
	err += MRAM_WRITE(SENSOR_CHECK_MAPNUM2_ADDR, (uint8_t*)(&sensorCheck.MapNum[2]), 1);

//Sensor Pose Content 5600-5999
	err += MRAM_WRITE(SensorType_POSE_ADDR, (&SenData[ch0Pose].Sensortype), 1);
	err += MRAM_WRITE(Sign_POSE_ADDR, (&SenData[ch0Pose].Sign), 1);
	err += MRAM_WRITE(NominalValue_POSE_ADDR, (uint8_t*)(&SenData[ch0Pose].NominalValue), 4);
	err += MRAM_WRITE(NominalSensitive_POSE_ADDR, (uint8_t*)(&SenData[ch0Pose].NominalSensitive), 4);
	err += MRAM_WRITE(Day_POSE_ADDR, (&SenData[ch0Pose].Day), 1);
	err += MRAM_WRITE(Month_POSE_ADDR, (&SenData[ch0Pose].Month), 1);
	err += MRAM_WRITE(Year_POSE_ADDR, (uint8_t*)(&SenData[ch0Pose].Year), 2);
	err += MRAM_WRITE(LinPoint_POSE_ADDR, (&SenData[ch0Pose].LinPoint), 1);

	for(i=0;i<SEN_LIN_DATA_MAX;i++)
	{
		err += MRAM_WRITE(LinV_POSE_ADDR+12*i, (uint8_t*)(&SenData[ch0Pose].LinV[i].ADCCode), 4);
		err += MRAM_WRITE(LinV_POSE_ADDR+4+12*i,(uint8_t*)(&SenData[ch0Pose].LinV[i].RefValue), 4);
		err += MRAM_WRITE(LinV_POSE_ADDR+8+12*i,(uint8_t*)(&SenData[ch0Pose].LinV[i].CorrectionFactor), 4);
	}
//sensor intgr content 6000-6019
	err += MRAM_WRITE(INTGR_POSE_ADDR, (uint8_t*)(&SenData[ch0Pose].sensorIntgr), 4);
	err += MRAM_WRITE(INTGR_BIGDERFORMATION_ADDR,(uint8_t*)(&SenData[ch1Bd].sensorIntgr), 4);
	err += MRAM_WRITE(INTGR_EXTEN1_ADDR,(uint8_t*)(&SenData[ch2Ext1].sensorIntgr), 4);
	err += MRAM_WRITE(INTGR_EXTEN2_ADDR,(uint8_t*)(&SenData[ch3Ext2].sensorIntgr), 4);
	err += MRAM_WRITE(INTGR_LOAD_ADDR,(uint8_t*)(&SenData[ch4Load].sensorIntgr), 4);

//ethernet config 6020-6036
	err += MRAM_WRITE(ETH_MODE_ADDR, (uint8_t*)(&ethConfig.EthMode),1);
	err += MRAM_WRITE(ETH_SERVER_IP_ADDR, (uint8_t*)(&ethConfig.ServerIp),4);
	err += MRAM_WRITE(ETH_SERVER_PORT_ADDR, (uint8_t*)(&ethConfig.ServerPort),2);
	err += MRAM_WRITE(ETH_LOCAL_IP_ADDR, (uint8_t*)(&ethConfig.LocalIp),4);
	err += MRAM_WRITE(ETH_LOCAL_MAC_ADDR, (uint8_t*)(&ethConfig.LocalMac),6);

	//???? 20000-
	for(i=0;i<6;i++)
		err += MRAM_WRITE(PeInterface_ADDR, (uint8_t*)(&utc_version.PeInterface[i]), 4);
	for(i=0;i<13;i++)
		err += MRAM_WRITE(Application_ADDR, (uint8_t*)(&utc_version.Application[i]), 4);
	for(i=0;i<6;i++)
		err += MRAM_WRITE(Subsy_ADDR,(uint8_t*)(&utc_version.Subsy[i]), 4);
	for(i=0;i<6;i++)
		err += MRAM_WRITE(SubsyCustVer_ADDR, (uint8_t*)(&utc_version.SubsyCustVer[i]), 4);
	for(i=0;i<9;i++)
		err += MRAM_WRITE(SubsyCustName_ADDR, (uint8_t*)(&utc_version.SubsyCustName[i]), 4);
	for(i=0;i<6;i++)
		err += MRAM_WRITE(Bios_ADDR, (uint8_t*)(&utc_version.Bios[i]), 4);
	for(i=0;i<7;i++)
		err += MRAM_WRITE(HwCtrl_ADDR, (uint8_t*)(&utc_version.HwCtrl[i]), 4);
	for(i=0;i<6;i++)
		err += MRAM_WRITE(PeInterfacePC_ADDR, (uint8_t*)(&utc_version.PeInterfacePC[i]), 4);
	for(i=0;i<6;i++)
		err += MRAM_WRITE(DpxVer_ADDR, (uint8_t*)(&utc_version.DpxVer[i]), 4);
	for(i=0;i<17;i++)
		err += MRAM_WRITE(SerialNumber_ADDR, (uint8_t*)(&utc_version.SerialNumber[i]), 4);
	err += MRAM_WRITE( VersionEnd_ADDR, &utc_version.VersionEnd, 1);
	
	if(err > 0)
		if(elog_init_ok)
			log_e("mram write all para WRerr:%d",err);
		else
			printf("mram write all para WRerr:%d\r\n",err);
	return err;
}

/**
 * @note	Fun_11012
 * @brief	read all storage parameter in mram
 * @retval		err-Number of data read errors
*/
uint8_t mramReadAll(void){
uint8_t err=0,check_num=MRAM_CHECK,i=0;
	err += MRAM_READ(CHEAK_ADDR, &check_num,1);
	err += MRAM_READ(PosDefaultAcc_ADDR, (uint8_t*)(&commBuf.Pos.DefaultAcc), 4);
	err += MRAM_READ(PosDefaultDec_ADDR, (uint8_t*)(&commBuf.Pos.DefaultDec), 4);
	err += MRAM_READ(LoadDefaultAcc_ADDR, (uint8_t*)(&commBuf.Load.DefaultAcc), 4);
	err += MRAM_READ(LoadDefaultDec_ADDR, (uint8_t*)(&commBuf.Load.DefaultDec), 4);
	err += MRAM_READ(ExtDefaultAcc_ADDR, (uint8_t*)(&commBuf.Ext.DefaultAcc), 4);
	err += MRAM_READ(ExtDefaultDec_ADDR, (uint8_t*)(&commBuf.Ext.DefaultDec), 4);
	err += MRAM_READ(upperlimit_phy_ADDR, (uint8_t*)(&AL.upperlimit_phy), 4);
	err += MRAM_READ(lowerlimit_phy_ADDR, (uint8_t*)(&AL.lowerlimit_phy), 4);
	err += MRAM_READ(defaultspeed_ADDR, (uint8_t*)(&AL.defaultspeed), 4);
	err += MRAM_READ(defaultspeedlimit_ADDR, (uint8_t*)(&AL.defaultspeedlimit), 4);
		
//POS_MODE ???????? 400-514
	err += MRAM_READ(Posallctrlpara_PosPid_P_ADDR, (uint8_t*)(&posAllCtrlPara.PosPid.P), 4);
	err += MRAM_READ(Posallctrlpara_PosPid_I_ADDR, (uint8_t*)(&posAllCtrlPara.PosPid.I), 4);
	err += MRAM_READ(Posallctrlpara_PosPid_D_ADDR, (uint8_t*)(&posAllCtrlPara.PosPid.D), 4);
	err += MRAM_READ(Posallctrlpara_PosPid_Hp_P_ADDR, (uint8_t*)(&posAllCtrlPara.PosPid_Hp.P), 4);
	err += MRAM_READ(Posallctrlpara_PosPid_Hp_I_ADDR, (uint8_t*)(&posAllCtrlPara.PosPid_Hp.P), 4);
	err += MRAM_READ(Posallctrlpara_PosPid_Hp_D_ADDR, (uint8_t*)(&posAllCtrlPara.PosPid_Hp.P), 4);
	err += MRAM_READ(Posallctrlpara_SpeedPid_P_ADDR, (uint8_t*)(&posAllCtrlPara.SpeedPid.P), 4);
	err += MRAM_READ(Posallctrlpara_SpeedPid_I_ADDR, (uint8_t*)(&posAllCtrlPara.SpeedPid.I), 4);
	err += MRAM_READ(Posallctrlpara_SpeedPid_D_ADDR, (uint8_t*)(&posAllCtrlPara.SpeedPid.D), 4);
	err += MRAM_READ(Posallctrlpara_Ffd_SpeedFFP_ADDR, (uint8_t*)(&posAllCtrlPara.Ffd.SpeedFFP), 4);
	err += MRAM_READ(Posallctrlpara_Ffd_PosDelay_ADDR, (uint8_t*)(&posAllCtrlPara.Ffd.PosDelay), 4);
	err += MRAM_READ(Posallctrlpara_Ffd_Hp_SpeedFFP_ADDR, (uint8_t*)(&posAllCtrlPara.Ffd_Hp.SpeedFFP), 4);
	err += MRAM_READ(Posallctrlpara_Ffd_Hp_PosDelay_ADDR, (uint8_t*)(&posAllCtrlPara.Ffd_Hp.SpeedFFP), 4);
	err += MRAM_READ(Posallctrlpara_DefaultAcc_ADDR, (uint8_t*)(&posAllCtrlPara.DefaultAcc), 4);
	err += MRAM_READ(Posallctrlpara_Speed_ADDR, (uint8_t*)(&posAllCtrlPara.Speed), 4);
	err += MRAM_READ(Posallctrlpara_Deviation_ADDR, (uint8_t*)(&posAllCtrlPara.Deviation), 4);
	err += MRAM_READ(Posallctrlpara_DevReaction_ADDR, (&posAllCtrlPara.DevReaction), 1);
	err += MRAM_READ(Posallctrlpara_Wnd_WndSize_ADDR, (uint8_t*)(&posAllCtrlPara.Wnd.WndSize), 4);
	err += MRAM_READ(Posallctrlpara_Wnd_WndTime_ADDR, (uint8_t*)(&posAllCtrlPara.Wnd.WndTime), 4);
	err += MRAM_READ(Posallctrlpara_Sft_UpperSft_ADDR, (uint8_t*)(&posAllCtrlPara.Sft.UpperSft), 4);
	err += MRAM_READ(Posallctrlpara_Sft_LowerStf_ADDR, (uint8_t*)(&posAllCtrlPara.Sft.LowerSft), 4);
	err += MRAM_READ(Posallctrlpara_Sft_Reaction_ADDR, (&posAllCtrlPara.Sft.Reaction), 1);
	err += MRAM_READ(Posallctrlpara_MinAcceleration_ADDR, (uint8_t*)(&posAllCtrlPara.MinAcceleration), 4);
	err += MRAM_READ(Posallctrlpara_MaxAcceleration_ADDR, (uint8_t*)(&posAllCtrlPara.MaxAcceleration), 4);
	err += MRAM_READ(Posallctrlpara_MinDeceleration_ADDR, (uint8_t*)(&posAllCtrlPara.MinDeceleration), 4);
	err += MRAM_READ(Posallctrlpara_MaxDeceleration_ADDR, (uint8_t*)(&posAllCtrlPara.MaxDeceleration), 4);
	err += MRAM_READ(Posallctrlpara_MinSpeed_ADDR, (uint8_t*)(&posAllCtrlPara.MinSpeed), 4);
	err += MRAM_READ(Posallctrlpara_MaxSpeed_ADDR, (uint8_t*)(&posAllCtrlPara.MaxSpeed), 4);
	err += MRAM_READ(Posallctrlpara_Deadband_Deadband_ADDR, (uint8_t*)(&posAllCtrlPara.Deadband.Deadband), 4);
	err += MRAM_READ(Posallctrlpara_Deadband_PercentP_ADDR, (&posAllCtrlPara.Deadband.PercentP), 1);
	err += MRAM_READ(Posallctrlpara_Dither_DitherAmplitude_ADDR, (uint8_t*)(&posAllCtrlPara.Dither.DitherAmplitude), 4);
	err += MRAM_READ(Posallctrlpara_Dither_DitherFrequency_ADDR, (uint8_t*)(&posAllCtrlPara.Dither.DitherFrequency), 4);
	err += MRAM_READ(Posallctrlpara_NominalAcc_ADDR, (uint8_t*)(&posAllCtrlPara.Nominal.Acc), 4);
	err += MRAM_READ(Posallctrlpara_NominalSpeed_ADDR, (uint8_t*)(&posAllCtrlPara.Nominal.Speed), 4);

//LOAD_MODE ???????? 600-714
	err += MRAM_READ(Loadallctrlpara_PosPid_P_ADDR, (uint8_t*)(&loadAllCtrlPara.PosPid.P), 4);
	err += MRAM_READ(Loadallctrlpara_PosPid_I_ADDR, (uint8_t*)(&loadAllCtrlPara.PosPid.I), 4);
	err += MRAM_READ(Loadallctrlpara_PosPid_D_ADDR, (uint8_t*)(&loadAllCtrlPara.PosPid.D), 4);
	err += MRAM_READ(Loadallctrlpara_PosPid_Hp_P_ADDR, (uint8_t*)(&loadAllCtrlPara.PosPid_Hp.P), 4);
	err += MRAM_READ(Loadallctrlpara_PosPid_Hp_I_ADDR, (uint8_t*)(&loadAllCtrlPara.PosPid_Hp.P), 4);
	err += MRAM_READ(Loadallctrlpara_PosPid_Hp_D_ADDR, (uint8_t*)(&loadAllCtrlPara.PosPid_Hp.P), 4);
	err += MRAM_READ(Loadallctrlpara_SpeedPid_P_ADDR, (uint8_t*)(&loadAllCtrlPara.SpeedPid.P), 4);
	err += MRAM_READ(Loadallctrlpara_SpeedPid_I_ADDR, (uint8_t*)(&loadAllCtrlPara.SpeedPid.I), 4);
	err += MRAM_READ(Loadallctrlpara_SpeedPid_D_ADDR, (uint8_t*)(&loadAllCtrlPara.SpeedPid.D), 4);
	err += MRAM_READ(Loadallctrlpara_Ffd_SpeedFFP_ADDR, (uint8_t*)(&loadAllCtrlPara.Ffd.SpeedFFP), 4);
	err += MRAM_READ(Loadallctrlpara_Ffd_PosDelay_ADDR, (uint8_t*)(&loadAllCtrlPara.Ffd.PosDelay), 4);
	err += MRAM_READ(Loadallctrlpara_Ffd_Hp_SpeedFFP_ADDR, (uint8_t*)(&loadAllCtrlPara.Ffd_Hp.SpeedFFP), 4);
	err += MRAM_READ(Loadallctrlpara_Ffd_Hp_PosDelay_ADDR, (uint8_t*)(&loadAllCtrlPara.Ffd_Hp.SpeedFFP), 4);
	err += MRAM_READ(Loadallctrlpara_DefaultAcc_ADDR, (uint8_t*)(&loadAllCtrlPara.DefaultAcc), 4);
	err += MRAM_READ(Loadallctrlpara_Speed_ADDR, (uint8_t*)(&loadAllCtrlPara.Speed), 4);
	err += MRAM_READ(Loadallctrlpara_Deviation_ADDR, (uint8_t*)(&loadAllCtrlPara.Deviation), 4);
	err += MRAM_READ(Loadallctrlpara_DevReaction_ADDR, (&loadAllCtrlPara.DevReaction), 1);
	err += MRAM_READ(Loadallctrlpara_Wnd_WndSize_ADDR, (uint8_t*)(&loadAllCtrlPara.Wnd.WndSize), 4);
	err += MRAM_READ(Loadallctrlpara_Wnd_WndTime_ADDR, (uint8_t*)(&loadAllCtrlPara.Wnd.WndTime), 4);
	err += MRAM_READ(Loadallctrlpara_Sft_UpperSft_ADDR, (uint8_t*)(&loadAllCtrlPara.Sft.UpperSft), 4);
	err += MRAM_READ(Loadallctrlpara_Sft_LowerStf_ADDR, (uint8_t*)(&loadAllCtrlPara.Sft.LowerSft), 4);
	err += MRAM_READ(Loadallctrlpara_Sft_Reaction_ADDR, (&loadAllCtrlPara.Sft.Reaction), 1);
	err += MRAM_READ(Loadallctrlpara_MinAcceleration_ADDR, (uint8_t*)(&loadAllCtrlPara.MinAcceleration), 4);
	err += MRAM_READ(Loadallctrlpara_MaxAcceleration_ADDR, (uint8_t*)(&loadAllCtrlPara.MaxAcceleration), 4);
	err += MRAM_READ(Loadallctrlpara_MinDeceleration_ADDR, (uint8_t*)(&loadAllCtrlPara.MinDeceleration), 4);
	err += MRAM_READ(Loadallctrlpara_MaxDeceleration_ADDR, (uint8_t*)(&loadAllCtrlPara.MaxDeceleration), 4);
	err += MRAM_READ(Loadallctrlpara_MinSpeed_ADDR, (uint8_t*)(&loadAllCtrlPara.MinSpeed), 4);
	err += MRAM_READ(Loadallctrlpara_MaxSpeed_ADDR, (uint8_t*)(&loadAllCtrlPara.MaxSpeed), 4);
	err += MRAM_READ(Loadallctrlpara_Deadband_Deadband_ADDR, (uint8_t*)(&loadAllCtrlPara.Deadband.Deadband), 4);
	err += MRAM_READ(Loadallctrlpara_Deadband_PercentP_ADDR, (&loadAllCtrlPara.Deadband.PercentP), 1);
	err += MRAM_READ(Loadallctrlpara_Dither_DitherAmplitude_ADDR, (uint8_t*)(&loadAllCtrlPara.Dither.DitherAmplitude), 4);
	err += MRAM_READ(Loadallctrlpara_Dither_DitherFrequency_ADDR, (uint8_t*)(&loadAllCtrlPara.Dither.DitherFrequency), 4);
	err += MRAM_READ(Loadallctrlpara_NominalAcc_ADDR, (uint8_t*)(&loadAllCtrlPara.Nominal.Acc), 4);
	err += MRAM_READ(Loadallctrlpara_NominalSpeed_ADDR, (uint8_t*)(&loadAllCtrlPara.Nominal.Speed), 4);

//EXTEN_MODE ???????? 800-914
	err += MRAM_READ(Extallctrlpara_PosPid_P_ADDR, (uint8_t*)(&extAllCtrlPara.PosPid.P), 4);
	err += MRAM_READ(Extallctrlpara_PosPid_I_ADDR, (uint8_t*)(&extAllCtrlPara.PosPid.I), 4);
	err += MRAM_READ(Extallctrlpara_PosPid_D_ADDR, (uint8_t*)(&extAllCtrlPara.PosPid.D), 4);
	err += MRAM_READ(Extallctrlpara_PosPid_Hp_P_ADDR, (uint8_t*)(&extAllCtrlPara.PosPid_Hp.P), 4);
	err += MRAM_READ(Extallctrlpara_PosPid_Hp_I_ADDR, (uint8_t*)(&extAllCtrlPara.PosPid_Hp.P), 4);
	err += MRAM_READ(Extallctrlpara_PosPid_Hp_D_ADDR, (uint8_t*)(&extAllCtrlPara.PosPid_Hp.P), 4);
	err += MRAM_READ(Extallctrlpara_SpeedPid_P_ADDR, (uint8_t*)(&extAllCtrlPara.SpeedPid.P), 4);
	err += MRAM_READ(Extallctrlpara_SpeedPid_I_ADDR, (uint8_t*)(&extAllCtrlPara.SpeedPid.I), 4);
	err += MRAM_READ(Extallctrlpara_SpeedPid_D_ADDR, (uint8_t*)(&extAllCtrlPara.SpeedPid.D), 4);
	err += MRAM_READ(Extallctrlpara_Ffd_SpeedFFP_ADDR, (uint8_t*)(&extAllCtrlPara.Ffd.SpeedFFP), 4);
	err += MRAM_READ(Extallctrlpara_Ffd_PosDelay_ADDR, (uint8_t*)(&extAllCtrlPara.Ffd.PosDelay), 4);
	err += MRAM_READ(Extallctrlpara_Ffd_Hp_SpeedFFP_ADDR, (uint8_t*)(&extAllCtrlPara.Ffd_Hp.SpeedFFP), 4);
	err += MRAM_READ(Extallctrlpara_Ffd_Hp_PosDelay_ADDR, (uint8_t*)(&extAllCtrlPara.Ffd_Hp.SpeedFFP), 4);
	err += MRAM_READ(Extallctrlpara_DefaultAcc_ADDR, (uint8_t*)(&extAllCtrlPara.DefaultAcc), 4);
	err += MRAM_READ(Extallctrlpara_Speed_ADDR, (uint8_t*)(&extAllCtrlPara.Speed), 4);
	err += MRAM_READ(Extallctrlpara_Deviation_ADDR, (uint8_t*)(&extAllCtrlPara.Deviation), 4);
	err += MRAM_READ(Extallctrlpara_DevReaction_ADDR, (&extAllCtrlPara.DevReaction), 1);
	err += MRAM_READ(Extallctrlpara_Wnd_WndSize_ADDR, (uint8_t*)(&extAllCtrlPara.Wnd.WndSize), 4);
	err += MRAM_READ(Extallctrlpara_Wnd_WndTime_ADDR, (uint8_t*)(&extAllCtrlPara.Wnd.WndTime), 4);
	err += MRAM_READ(Extallctrlpara_Sft_UpperSft_ADDR, (uint8_t*)(&extAllCtrlPara.Sft.UpperSft), 4);
	err += MRAM_READ(Extallctrlpara_Sft_LowerStf_ADDR, (uint8_t*)(&extAllCtrlPara.Sft.LowerSft), 4);
	err += MRAM_READ(Extallctrlpara_Sft_Reaction_ADDR, (&extAllCtrlPara.Sft.Reaction),1);
	err += MRAM_READ(Extallctrlpara_MinAcceleration_ADDR, (uint8_t*)(&extAllCtrlPara.MinAcceleration), 4);
	err += MRAM_READ(Extallctrlpara_MaxAcceleration_ADDR, (uint8_t*)(&extAllCtrlPara.MaxAcceleration), 4);
	err += MRAM_READ(Extallctrlpara_MinDeceleration_ADDR, (uint8_t*)(&extAllCtrlPara.MinDeceleration), 4);
	err += MRAM_READ(Extallctrlpara_MaxDeceleration_ADDR, (uint8_t*)(&extAllCtrlPara.MaxDeceleration), 4);
	err += MRAM_READ(Extallctrlpara_MinSpeed_ADDR, (uint8_t*)(&extAllCtrlPara.MinSpeed), 4);
	err += MRAM_READ(Extallctrlpara_MaxSpeed_ADDR, (uint8_t*)(&extAllCtrlPara.MaxSpeed), 4);
	err += MRAM_READ(Extallctrlpara_Deadband_Deadband_ADDR, (uint8_t*)(&extAllCtrlPara.Deadband.Deadband), 4);
	err += MRAM_READ(Extallctrlpara_Deadband_PercentP_ADDR, (&extAllCtrlPara.Deadband.PercentP),1);
	err += MRAM_READ(Extallctrlpara_Dither_DitherAmplitude_ADDR, (uint8_t*)(&extAllCtrlPara.Dither.DitherAmplitude), 4);
	err += MRAM_READ(Extallctrlpara_Dither_DitherFrequency_ADDR, (uint8_t*)(&extAllCtrlPara.Dither.DitherFrequency), 4);
	err += MRAM_READ(Extallctrlpara_NominalAcc_ADDR, (uint8_t*)(&extAllCtrlPara.Nominal.Acc), 4);
	err += MRAM_READ(Extallctrlpara_NominalSpeed_ADDR, (uint8_t*)(&extAllCtrlPara.Nominal.Speed), 4);

//system parameters 5000-5002
	err += MRAM_READ(SYSPARA_TRANSMITDATAPERIOD_ADDR, (uint8_t*)(&SysPara.TransmitDataPeriod), 2);
	err += MRAM_READ(SYSPARA_CONTROLLERSTRUCTURE_ADDR, (uint8_t*)(&SysPara.ControllerStructure),1);

//servo parameters 5050-5061
	err += MRAM_READ(SERVOPARA_ELECTRONICGEAR_ADDR, (uint8_t*)(&servoParam.electronicGear), 4);
	err += MRAM_READ(SERVOPARA_ENCODERRESOLUTION_ADDR, (uint8_t*)(&servoParam.encoderResolution), 4);
	err += MRAM_READ(SERVOPARA_ANALOGGAIN_ADDR, (uint8_t*)(&servoParam.analogGain), 4);

//output parameters 5100-5129
	err += MRAM_READ(OUTPUTPARA_SIGNAL_ADDR, (&OutputPara.Signal),1);
	err += MRAM_READ(OUTPUTPARA_SIGN_ADDR, (&OutputPara.Sign),1);
	err += MRAM_READ(OUTPUTPARA_MAXVALUE_ADDR, (uint8_t*)(&OutputPara.MaxValue), 4);
	err += MRAM_READ(OUTPUTPARA_MINVALUE_ADDR, (uint8_t*)(&OutputPara.MinValue), 4);
	err += MRAM_READ(OUTPUTPARA_INITVALUE_ADDR, (uint8_t*)(&OutputPara.InitValue), 4);
	err += MRAM_READ(OUTPUTPARA_OFFSET_ADDR, (uint8_t*)(&OutputPara.Offset), 4);
	err += MRAM_READ(OUTPUTPARA_SIGNALFREQUENCY_ADDR, (uint8_t*)(&OutputPara.SignalFrequency), 4);
	err += MRAM_READ(OUTPUTPARA_MAXVOLTAGE_ADDR,(uint8_t*)(&OutputPara.MaxVoltage), 4);
	err += MRAM_READ(OUTPUTPARA_MAXCURRENT_ADDR, (uint8_t*)(&OutputPara.MaxCurrent), 4);

//these data need to be saved realtime 5200-5211
	err += MRAM_READ(POSE_ENCODER_CODE_ADDR, (uint8_t*)(&pose.code), 4);
	err += MRAM_READ(MRAM_LOAD_TARE_VALUE_ADDR,(uint8_t*)(&AL.tare.fValue[ch4Load]), 4);
	err += MRAM_READ(MRAM_EXTEN_TARE_VALUE_ADDR, (uint8_t*)(&AL.tare.fValue[ch2Ext1]), 4);
	err += MRAM_READ(MRAM_EXTEN2_TARE_VALUE_ADDR,(uint8_t*)(&AL.tare.fValue[ch3Ext2]), 4);
	err += MRAM_READ(MRAM_LOAD_TARE_CODE_ADDR, (uint8_t*)(&AL.tare.value[ch4Load]), 4);
	err += MRAM_READ(MRAM_EXTEN_TARE_CODE_ADDR,(uint8_t*)(&AL.tare.value[ch2Ext1]), 4);
	err += MRAM_READ(MRAM_EXTEN2_TARE_CODE_ADDR, (uint8_t*)(&AL.tare.value[ch3Ext2]), 4);

//senserCheck 5400-5405
	err += MRAM_READ(SENSOR_CHECK_REGISTER0_ADDR, (uint8_t*)(&sensorCheck.Register[0]), 1);
	err += MRAM_READ(SENSOR_CHECK_REGISTER1_ADDR, (uint8_t*)(&sensorCheck.Register[1]), 1);
	err += MRAM_READ(SENSOR_CHECK_REGISTER2_ADDR, (uint8_t*)(&sensorCheck.Register[2]), 1);
	err += MRAM_READ(SENSOR_CHECK_MAPNUM0_ADDR, (uint8_t*)(&sensorCheck.MapNum[0]), 1);
	err += MRAM_READ(SENSOR_CHECK_MAPNUM1_ADDR, (uint8_t*)(&sensorCheck.MapNum[1]), 1);
	err += MRAM_READ(SENSOR_CHECK_MAPNUM2_ADDR, (uint8_t*)(&sensorCheck.MapNum[2]), 1);

//Sensor Pose Content 5600-5999
	err += MRAM_READ(SensorType_POSE_ADDR, (&SenData[ch0Pose].Sensortype), 1);
	err += MRAM_READ(Sign_POSE_ADDR, (&SenData[ch0Pose].Sign), 1);
	err += MRAM_READ(NominalValue_POSE_ADDR, (uint8_t*)(&SenData[ch0Pose].NominalValue), 4);
	err += MRAM_READ(NominalSensitive_POSE_ADDR, (uint8_t*)(&SenData[ch0Pose].NominalSensitive), 4);
	err += MRAM_READ(Day_POSE_ADDR, (&SenData[ch0Pose].Day), 1);
	err += MRAM_READ(Month_POSE_ADDR, (&SenData[ch0Pose].Month), 1);
	err += MRAM_READ(Year_POSE_ADDR, (uint8_t*)(&SenData[ch0Pose].Year), 2);
	err += MRAM_READ(LinPoint_POSE_ADDR, (&SenData[ch0Pose].LinPoint), 1);

	for(i=0;i<SEN_LIN_DATA_MAX;i++)
	{
		err += MRAM_READ(LinV_POSE_ADDR+12*i, (uint8_t*)(&SenData[ch0Pose].LinV[i].ADCCode), 4);
		err += MRAM_READ(LinV_POSE_ADDR+4+12*i,(uint8_t*)(&SenData[ch0Pose].LinV[i].RefValue), 4);
		err += MRAM_READ(LinV_POSE_ADDR+8+12*i,(uint8_t*)(&SenData[ch0Pose].LinV[i].CorrectionFactor), 4);
	}

//sensor intgr content
	err += MRAM_READ(INTGR_POSE_ADDR, (uint8_t*)(&SenData[ch0Pose].sensorIntgr), 4);
	err += MRAM_READ(INTGR_BIGDERFORMATION_ADDR,(uint8_t*)(&SenData[ch1Bd].sensorIntgr), 4);
	err += MRAM_READ(INTGR_EXTEN1_ADDR,(uint8_t*)(&SenData[ch2Ext1].sensorIntgr), 4);
	err += MRAM_READ(INTGR_EXTEN2_ADDR,(uint8_t*)(&SenData[ch3Ext2].sensorIntgr), 4);
	err += MRAM_READ(INTGR_LOAD_ADDR,(uint8_t*)(&SenData[ch4Load].sensorIntgr), 4);

//ethernet config
	err += MRAM_READ(ETH_MODE_ADDR, (uint8_t*)(&ethConfig.EthMode),1);
	err += MRAM_READ(ETH_SERVER_IP_ADDR, (uint8_t*)(&ethConfig.ServerIp),4);
	err += MRAM_READ(ETH_SERVER_PORT_ADDR, (uint8_t*)(&ethConfig.ServerPort),2);
	err += MRAM_READ(ETH_LOCAL_IP_ADDR, (uint8_t*)(&ethConfig.LocalIp),4);
	err += MRAM_READ(ETH_LOCAL_MAC_ADDR, (uint8_t*)(&ethConfig.LocalMac),6);

	//utc version 20000-
	for(i=0;i<6;i++)
		err += MRAM_READ(PeInterface_ADDR, (uint8_t*)(&utc_version.PeInterface[i]), 4);
	for(i=0;i<13;i++)
		err += MRAM_READ(Application_ADDR, (uint8_t*)(&utc_version.Application[i]), 4);
	for(i=0;i<6;i++)
		err += MRAM_READ(Subsy_ADDR,(uint8_t*)(&utc_version.Subsy[i]), 4);
	for(i=0;i<6;i++)
		err += MRAM_READ(SubsyCustVer_ADDR, (uint8_t*)(&utc_version.SubsyCustVer[i]), 4);
	for(i=0;i<9;i++)
		err += MRAM_READ(SubsyCustName_ADDR, (uint8_t*)(&utc_version.SubsyCustName[i]), 4);
	for(i=0;i<6;i++)
		err += MRAM_READ(Bios_ADDR, (uint8_t*)(&utc_version.Bios[i]), 4);
	for(i=0;i<7;i++)
		err += MRAM_READ(HwCtrl_ADDR, (uint8_t*)(&utc_version.HwCtrl[i]), 4);
	for(i=0;i<6;i++)
		err += MRAM_READ(PeInterfacePC_ADDR, (uint8_t*)(&utc_version.PeInterfacePC[i]), 4);
	for(i=0;i<6;i++)
		err += MRAM_READ(DpxVer_ADDR, (uint8_t*)(&utc_version.DpxVer[i]), 4);
	for(i=0;i<17;i++)
		err += MRAM_READ(SerialNumber_ADDR, (uint8_t*)(&utc_version.SerialNumber[i]), 4);
	err += MRAM_READ( VersionEnd_ADDR, &utc_version.VersionEnd, 1);
	
	if(err > 0)
		if(elog_init_ok)
			log_e("mram read all para WRerr:%d",err);
		else
			printf("mram read all para WRerr:%d\r\n",err);
	return err;
}

/**
 * @brief		sensor code value save
 * @param		null
 * @retval		err-number of error
*/
uint8_t mramSensorCodeWrite(void)
{	
	uint8_t err=0;
	err += MRAM_WRITE(POSE_ENCODER_CODE_ADDR, (uint8_t*)(&pose.code), 4);
	err += MRAM_WRITE(MRAM_LOAD_TARE_VALUE_ADDR,(uint8_t*)(&AL.tare.fValue[ch4Load]), 4);
	err += MRAM_WRITE(MRAM_EXTEN_TARE_VALUE_ADDR, (uint8_t*)(&AL.tare.fValue[ch2Ext1]), 4);
	err += MRAM_WRITE(MRAM_EXTEN2_TARE_VALUE_ADDR,(uint8_t*)(&AL.tare.fValue[ch3Ext2]), 4);
	err += MRAM_WRITE(MRAM_LOAD_TARE_CODE_ADDR, (uint8_t*)(&AL.tare.value[ch4Load]), 4);
	err += MRAM_WRITE(MRAM_EXTEN_TARE_CODE_ADDR,(uint8_t*)(&AL.tare.value[ch2Ext1]), 4);
	err += MRAM_WRITE(MRAM_EXTEN2_TARE_CODE_ADDR, (uint8_t*)(&AL.tare.value[ch3Ext2]), 4);


	if(err > 0)
		log_e("mram sensor code write:%d\r\n",err);
	return err;
}

/**
 * @brief		sensor code value read
 * @param		null
 * @retval		err-number of error
*/
uint8_t mramSensorCodeRead(void)
{	
	uint8_t err=0;
	err += MRAM_READ(POSE_ENCODER_CODE_ADDR, (uint8_t*)(&pose.code), 4);
	err += MRAM_READ(MRAM_LOAD_TARE_VALUE_ADDR,(uint8_t*)(&AL.tare.fValue[ch4Load]), 4);
	err += MRAM_READ(MRAM_EXTEN_TARE_VALUE_ADDR, (uint8_t*)(&AL.tare.fValue[ch2Ext1]), 4);
	err += MRAM_READ(MRAM_EXTEN2_TARE_VALUE_ADDR,(uint8_t*)(&AL.tare.fValue[ch3Ext2]), 4);
	err += MRAM_READ(MRAM_LOAD_TARE_CODE_ADDR, (uint8_t*)(&AL.tare.value[ch4Load]), 4);
	err += MRAM_READ(MRAM_EXTEN_TARE_CODE_ADDR,(uint8_t*)(&AL.tare.value[ch2Ext1]), 4);
	err += MRAM_READ(MRAM_EXTEN2_TARE_CODE_ADDR, (uint8_t*)(&AL.tare.value[ch3Ext2]), 4);
	if(err > 0)
		log_e("mram sensor code read:%d\r\n",err);
	return err;
}

/**
 * @brief According to the semaphore, write relevant data to mram
 * @param[in] semaphore mram storage param semaphore
 * @date
 * @author Pan
*/
void mram_write_monitor(STORAGE_PARAM_MYSEMAPHORE* semaphore){
uint8_t rc=0;
	//sensor code and tare store updated
	if(semaphore->write_sensorcode == 1)
	{
	rc += mramSensorCodeWrite();
	semaphore->write_sensorcode = 0;
	}
	//DefaultAcc Pos store updated
	if(semaphore->write_defalutacc_pos == 1)
	{
	rc += mramDefaultAccWrite(&posAllCtrlPara,POS_MODE);
	semaphore->write_defalutacc_pos = 0;
	}
	//DefaultAcc Load store updated
	if(semaphore->write_defalutacc_load == 1)
	{
	rc += mramDefaultAccWrite(&loadAllCtrlPara,LOAD_MODE);
	semaphore->write_defalutacc_load = 0;
	}
	//DefaultAcc Ext store updated
	if(semaphore->write_defalutacc_ext == 1)
	{
	rc += mramDefaultAccWrite(&extAllCtrlPara,EXTEN_MODE);
	semaphore->write_defalutacc_ext = 0;
	}
	//Speed Pos store updated
	if(semaphore->write_speed_pos == 1)
	{
	rc += mramSpeedWrite(&posAllCtrlPara,POS_MODE);
	semaphore->write_speed_pos = 0;
	}
	//Speed Load store updated
	if(semaphore->write_speed_load == 1)
	{
	rc += mramSpeedWrite(&loadAllCtrlPara,LOAD_MODE);
	semaphore->write_speed_load = 0;
	}
	//Speed Load store updated
	if(semaphore->write_speed_ext == 1)
	{
	rc += mramSpeedWrite(&extAllCtrlPara,EXTEN_MODE);
	semaphore->write_speed_ext = 0;
	}
	//Nominal Pos store updated
	if(semaphore->write_nominal_pos == 1)
	{
	rc += mramNominalWrite(&(posAllCtrlPara.Nominal),POS_MODE);
	//log_i("mramNominalWrite err:%d",rc);
	semaphore->write_nominal_pos = 0;
	}
	//Nominal Load store updated
	if(semaphore->write_nominal_load == 1)
	{
	rc += mramNominalWrite(&(loadAllCtrlPara.Nominal),LOAD_MODE);
	semaphore->write_nominal_load = 0;
	}
	//Nominal Ext store updated
	if(semaphore->write_nominal_ext == 1)
	{
	rc += mramNominalWrite(&(extAllCtrlPara.Nominal),EXTEN_MODE);
	semaphore->write_nominal_ext = 0;
	}
	//transmit data period store updated
	if(semaphore->write_transmitdataperiod == 1)
	{
	rc += mramTansmitDataPeriodWrite(&SysPara);
	semaphore->write_transmitdataperiod = 0;
	}
	//Wnd size and time pos store updated
	if(semaphore->write_wnd_pos == 1)
	{
	rc += mramWndWrite(&(posAllCtrlPara.Wnd),POS_MODE);
	semaphore->write_wnd_pos = 0;
	}
	//Wnd size and time load store updated
	if(semaphore->write_wnd_load == 1)
	{
	rc += mramWndWrite(&(loadAllCtrlPara.Wnd),LOAD_MODE);
	semaphore->write_wnd_load = 0;
	}
	//Wnd size and time ext store updated
	if(semaphore->write_wnd_ext == 1)
	{
	rc += mramWndWrite(&(extAllCtrlPara.Wnd),EXTEN_MODE);
	semaphore->write_wnd_ext = 0;
	}
	//Sft pos store updated
	if(semaphore->write_sft_pos == 1)
	{
	rc +=mramSftWrite(&(posAllCtrlPara.Sft),POS_MODE);
	semaphore->write_sft_pos = 0;
	}
	//Sft load store updated
	if(semaphore->write_sft_load == 1)
	{
	rc += mramSftWrite(&(loadAllCtrlPara.Sft),LOAD_MODE);
 	semaphore->write_sft_load = 0;
	}
	//Sft ext store updated
	if(semaphore->write_sft_ext == 1)
	{
	rc += mramSftWrite(&(extAllCtrlPara.Sft),EXTEN_MODE);
	semaphore->write_sft_ext = 0;
	}
	//PosPID pos store updated
	if(semaphore->write_pospid_pos == 1)
	{
	rc += mramPosPIDWrite(&(posAllCtrlPara.PosPid),POS_MODE);
	semaphore->write_pospid_pos = 0;
	}
	//PosPID load store updated
	if(semaphore->write_pospid_load == 1)
	{
	rc += mramPosPIDWrite(&(loadAllCtrlPara.PosPid),LOAD_MODE);
	semaphore->write_pospid_load = 0;
	}
	//PosPID ext store updated
	if(semaphore->write_pospid_ext == 1)
	{
	rc += mramPosPIDWrite(&(extAllCtrlPara.PosPid),EXTEN_MODE);
	semaphore->write_pospid_ext = 0;
	}
	//SpeedPID pos store updated
	if(semaphore->write_speedpid_pos == 1)
	{
	rc += mramSpeedPIDWrite(&(posAllCtrlPara.SpeedPid),POS_MODE);
	semaphore->write_speedpid_pos = 0;
	}
	//SpeedPID load store updated
	if(semaphore->write_speedpid_load == 1)
	{
	rc += mramSpeedPIDWrite(&(loadAllCtrlPara.SpeedPid),LOAD_MODE);
	semaphore->write_speedpid_load = 0;
	}
	//SpeedPID ext store updated
	if(semaphore->write_speedpid_ext == 1)
	{
	rc += mramSpeedPIDWrite(&(extAllCtrlPara.SpeedPid),EXTEN_MODE);
	semaphore->write_speedpid_ext = 0;
	}
	//Ffd pos store updated
	if(semaphore->write_ffd_pos == 1)
	{
	rc += mramFfdWrite(&(posAllCtrlPara.Ffd),POS_MODE);
	semaphore->write_ffd_pos = 0;
	}
	//Ffd load store updated
	if(semaphore->write_ffd_load == 1)
	{
	rc += mramFfdWrite(&(loadAllCtrlPara.Ffd),LOAD_MODE);
	semaphore->write_ffd_load = 0;
	}
	//Ffd ext store updated
	if(semaphore->write_ffd_ext == 1)
	{
	rc += mramFfdWrite(&(extAllCtrlPara.Ffd),EXTEN_MODE);
	semaphore->write_ffd_ext = 0;
	}
	//system parameter store updated
	if(semaphore->write_syspara == 1)
	{
	rc += mramSysParaWriteAll(&SysPara);
	semaphore->write_syspara = 0;
	}
	//output parameter store updated
	if(semaphore->write_outputpara == 1)
	{
	rc += mramOutputParaWriteAll(&OutputPara);
	semaphore->write_outputpara = 0;	
	}
	//sendata pose parameter store updated
	if(semaphore->write_sendataPose == 1){
		rc += mramSensorPoseWriteAll(&SenData[ch0Pose]);//pose
		semaphore->write_sendataPose = 0;
	}
	//sendata big derformation parameter store updated
	if(semaphore->write_sendataBigDerformation == 1){

		semaphore->write_sendataBigDerformation = 0;//bigdeformation
	}
	//sensorCheck parameter store updated
	if(semaphore->write_sensorCheck == 1){
		rc += mramSensorCheckWriteAll(&sensorCheck);
		log_d("mramSensorCheckWriteAll :%d",rc);
		semaphore->write_sensorCheck = 0;	
	}
	//sensorIntgr parameter store update
	if(semaphore->write_intgr == 1){
		rc += mramIntgrWrite(&(SenData[sensorConnector].sensorIntgr),sensorConnector);
		semaphore->write_intgr = 0;
	}
	//ethernet configuration store update,and restart system
	if(semaphore->writeEthConfig == 1){
		rc += mramEthWrite(&ethConfig);
		semaphore->writeEthConfig = 0;
		mySemaphore.systemReset.reset = 1;
	}
	//servo parameter store update
	if(semaphore->writeServoParam == 1){
		rc += mramServoParamWrite(&servoParam);
		semaphore->writeServoParam = 0;
	}
}

/**
 * @brief mram init and read storged data, when system start
 * 
*/
void mramInit(void){
	HAL_Delay(10);
	MRAM_WriteEnable();
  	MRAM_WriteStatusRegister();
	//check ok,then read all storaged 
	if(mramCheck() == 0){
		printf("mramCheck() == 0\r\n");
		mramReadAll();
		// printf("sensorCheck.mapNum:%d,%d,%d\r\n",sensorCheck.MapNum[0],sensorCheck.MapNum[1],sensorCheck.MapNum[2]);
		// printf("sensorCheck.Register:%d,%d,%d\r\n",sensorCheck.Register[0],sensorCheck.Register[1],sensorCheck.Register[2]);
	}
	else if(mramCheck()  == 1){
		printf("mramCheck() == 1\r\n");
		mramWriteAll();
		HAL_Delay(1000);
		mramReadAll();
	}


}


void mram_test_val(void){
	SysPara.TransmitDataPeriod = 20;
	//commBuf.Pos.DefaultAcc = 5;
	//commBuf.Pos.DefaultAcc = 5;
	utc_version.Bios[0] = '7';
	utc_version.Bios[1] = '9';
	utc_version.Bios[2] = '1';
	utc_version.Bios[3] = '3';
	utc_version.Bios[5] = '7';
	utc_version.Bios[5] = '7';
	posAllCtrlPara.Deadband.Deadband = 7.77;
	loadAllCtrlPara.Deadband.Deadband = 8.88;
	extAllCtrlPara.Deadband.Deadband = 9.29;
	posAllCtrlPara.Deadband.PercentP = 39;
	loadAllCtrlPara.Deadband.PercentP = 40;
	extAllCtrlPara.Deadband.PercentP = 41;
}

void mram_test_printf(void){
	printf("SysPara.TransmitDataPeriod:%d\r\n",SysPara.TransmitDataPeriod);
	printf("utc_version.Bios[0]:%c\r\n",utc_version.Bios[0]);
	printf("posAllCtrlPara.Deadband.Deadband:%f\r\n",posAllCtrlPara.Deadband.Deadband);
	printf("posAllCtrlPara.Deadband.PercentP:%d\r\n",posAllCtrlPara.Deadband.PercentP);
	printf("extAllCtrlPara.Deadband.PercentP:%d\r\n",extAllCtrlPara.Deadband.PercentP);
	printf("OutputPara.MaxVoltage:%f\r\n",OutputPara.MaxVoltage);
}

/**
 * @brief mram test program
 * 
 * 
*/
void mram_test(void){
	MRAM_WriteEnable();
  	MRAM_WriteStatusRegister();
	mram_test_val();
	mramWriteAll();
	HAL_Delay(1000);
	mramReadAll();
	mram_test_printf();

}




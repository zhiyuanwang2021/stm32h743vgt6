#include "Eeprom_manage.h"
#include <elog.h>


void EEPROM_test(void)
{
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
	
	SenAData.MaxExcitation = 6.88;
	SenAData.MinImpedance = 111;
	SenAData.Sensortype = SIG_DC;
	SenAData.Year = 2023;
	SenAData.Month = 11;
	SenAData.Day = 20;
	
	SenAData2.MaxExcitation =5.77;
	SenAData2.MinImpedance = 222;
	SenAData2.Sensortype = SIG_DC;
	SenAData2.Year = 2024;
	SenAData2.Month = 1;
	SenAData2.Day = 14;
	
	SenDatatest.NominalValue = 88.8f;
	SenDatatest.LinV[1].CorrectionFactor = 1.099f;
	SenDatatest.Year = 2023;
	SenDatatest.Month = 11;
	SenDatatest.Day = 22;
	
	
}

/**
 * @brief		将所有控制相关参数写入控制器上的At24c256
 * @retval		err——写入数据出错数量
*/
uint8_t EepromOnController_WriteAll(void)
{
	uint8_t err=0,check_num=EEPROM_CHECK,i=0;
	err += At24cxx_Write_Byte(M24C02_Controller,CHEAK_ADDR, &check_num);
	err += At24cxx_Write_Dword(M24C02_Controller,PosDefaultAcc_ADDR, (uint32_t*)(&commBuf.Pos.DefaultAcc));
	err += At24cxx_Write_Dword(M24C02_Controller,PosDefaultDec_ADDR, (uint32_t*)(&commBuf.Pos.DefaultDec));
	err += At24cxx_Write_Dword(M24C02_Controller,LoadDefaultAcc_ADDR, (uint32_t*)(&commBuf.Load.DefaultAcc));
	err += At24cxx_Write_Dword(M24C02_Controller,LoadDefaultDec_ADDR, (uint32_t*)(&commBuf.Load.DefaultDec));
	err += At24cxx_Write_Dword(M24C02_Controller,ExtDefaultAcc_ADDR, (uint32_t*)(&commBuf.Ext.DefaultAcc));
	err += At24cxx_Write_Dword(M24C02_Controller,ExtDefaultDec_ADDR, (uint32_t*)(&commBuf.Ext.DefaultDec));
	err += At24cxx_Write_Dword(M24C02_Controller,upperlimit_phy_ADDR, (uint32_t*)(&AL.upperlimit_phy));
	err += At24cxx_Write_Dword(M24C02_Controller,lowerlimit_phy_ADDR, (uint32_t*)(&AL.lowerlimit_phy));
	err += At24cxx_Write_Dword(M24C02_Controller,defaultspeed_ADDR, (uint32_t*)(&AL.defaultspeed));
	err += At24cxx_Write_Dword(M24C02_Controller,defaultspeedlimit_ADDR, (uint32_t*)(&AL.defaultspeedlimit));
		
	
//POS_MODE 所有闭环控制参数 400-514
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_PosPid_P_ADDR, (uint32_t*)(&posAllCtrlPara.PosPid.P));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_PosPid_I_ADDR, (uint32_t*)(&posAllCtrlPara.PosPid.I));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_PosPid_D_ADDR, (uint32_t*)(&posAllCtrlPara.PosPid.D));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_PosPid_Hp_P_ADDR, (uint32_t*)(&posAllCtrlPara.PosPid_Hp.P));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_PosPid_Hp_I_ADDR, (uint32_t*)(&posAllCtrlPara.PosPid_Hp.P));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_PosPid_Hp_D_ADDR, (uint32_t*)(&posAllCtrlPara.PosPid_Hp.P));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_SpeedPid_P_ADDR, (uint32_t*)(&posAllCtrlPara.SpeedPid.P));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_SpeedPid_I_ADDR, (uint32_t*)(&posAllCtrlPara.SpeedPid.I));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_SpeedPid_D_ADDR, (uint32_t*)(&posAllCtrlPara.SpeedPid.D));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_Ffd_SpeedFFP_ADDR, (uint32_t*)(&posAllCtrlPara.Ffd.SpeedFFP));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_Ffd_PosDelay_ADDR, (uint32_t*)(&posAllCtrlPara.Ffd.PosDelay));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_Ffd_Hp_SpeedFFP_ADDR, (uint32_t*)(&posAllCtrlPara.Ffd_Hp.SpeedFFP));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_Ffd_Hp_PosDelay_ADDR, (uint32_t*)(&posAllCtrlPara.Ffd_Hp.SpeedFFP));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_DefaultAcc_ADDR, (uint32_t*)(&posAllCtrlPara.DefaultAcc));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_Speed_ADDR, (uint32_t*)(&posAllCtrlPara.Speed));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_Deviation_ADDR, (uint32_t*)(&posAllCtrlPara.Deviation));
	err += At24cxx_Write_Byte (M24C02_Controller,Posallctrlpara_DevReaction_ADDR, (&posAllCtrlPara.DevReaction));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_Wnd_WndSize_ADDR, (uint32_t*)(&posAllCtrlPara.Wnd.WndSize));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_Wnd_WndTime_ADDR, (uint32_t*)(&posAllCtrlPara.Wnd.WndTime));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_Sft_UpperSft_ADDR, (uint32_t*)(&posAllCtrlPara.Sft.UpperSft));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_Sft_LowerStf_ADDR, (uint32_t*)(&posAllCtrlPara.Sft.LowerSft));
	err += At24cxx_Write_Byte (M24C02_Controller,Posallctrlpara_Sft_Reaction_ADDR, (&posAllCtrlPara.Sft.Reaction));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_MinAcceleration_ADDR, (uint32_t*)(&posAllCtrlPara.MinAcceleration));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_MaxAcceleration_ADDR, (uint32_t*)(&posAllCtrlPara.MaxAcceleration));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_MinDeceleration_ADDR, (uint32_t*)(&posAllCtrlPara.MinDeceleration));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_MaxDeceleration_ADDR, (uint32_t*)(&posAllCtrlPara.MaxDeceleration));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_MinSpeed_ADDR, (uint32_t*)(&posAllCtrlPara.MinSpeed));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_MaxSpeed_ADDR, (uint32_t*)(&posAllCtrlPara.MaxSpeed));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_Deadband_Deadband_ADDR, (uint32_t*)(&posAllCtrlPara.Deadband.Deadband));
	err += At24cxx_Write_Byte (M24C02_Controller,Posallctrlpara_Deadband_PercentP_ADDR, (&posAllCtrlPara.Deadband.PercentP));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_Dither_DitherAmplitude_ADDR, (uint32_t*)(&posAllCtrlPara.Dither.DitherAmplitude));
	err += At24cxx_Write_Dword(M24C02_Controller,Posallctrlpara_Dither_DitherFrequency_ADDR, (uint32_t*)(&posAllCtrlPara.Dither.DitherFrequency));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_NominalAcc_ADDR, (uint32_t*)(&posAllCtrlPara.Nominal.Acc));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_NominalSpeed_ADDR, (uint32_t*)(&posAllCtrlPara.Nominal.Speed));

//LOAD_MODE 所有闭环控制参数 600-714
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_PosPid_P_ADDR, (uint32_t*)(&loadAllCtrlPara.PosPid.P));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_PosPid_I_ADDR, (uint32_t*)(&loadAllCtrlPara.PosPid.I));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_PosPid_D_ADDR, (uint32_t*)(&loadAllCtrlPara.PosPid.D));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_PosPid_Hp_P_ADDR, (uint32_t*)(&loadAllCtrlPara.PosPid_Hp.P));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_PosPid_Hp_I_ADDR, (uint32_t*)(&loadAllCtrlPara.PosPid_Hp.P));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_PosPid_Hp_D_ADDR, (uint32_t*)(&loadAllCtrlPara.PosPid_Hp.P));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_SpeedPid_P_ADDR, (uint32_t*)(&loadAllCtrlPara.SpeedPid.P));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_SpeedPid_I_ADDR, (uint32_t*)(&loadAllCtrlPara.SpeedPid.I));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_SpeedPid_D_ADDR, (uint32_t*)(&loadAllCtrlPara.SpeedPid.D));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_Ffd_SpeedFFP_ADDR, (uint32_t*)(&loadAllCtrlPara.Ffd.SpeedFFP));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_Ffd_PosDelay_ADDR, (uint32_t*)(&loadAllCtrlPara.Ffd.PosDelay));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_Ffd_Hp_SpeedFFP_ADDR, (uint32_t*)(&loadAllCtrlPara.Ffd_Hp.SpeedFFP));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_Ffd_Hp_PosDelay_ADDR, (uint32_t*)(&loadAllCtrlPara.Ffd_Hp.SpeedFFP));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_DefaultAcc_ADDR, (uint32_t*)(&loadAllCtrlPara.DefaultAcc));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_Speed_ADDR, (uint32_t*)(&loadAllCtrlPara.Speed));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_Deviation_ADDR, (uint32_t*)(&loadAllCtrlPara.Deviation));
	err += At24cxx_Write_Byte (M24C02_Controller,Loadallctrlpara_DevReaction_ADDR, (&loadAllCtrlPara.DevReaction));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_Wnd_WndSize_ADDR, (uint32_t*)(&loadAllCtrlPara.Wnd.WndSize));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_Wnd_WndTime_ADDR, (uint32_t*)(&loadAllCtrlPara.Wnd.WndTime));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_Sft_UpperSft_ADDR, (uint32_t*)(&loadAllCtrlPara.Sft.UpperSft));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_Sft_LowerStf_ADDR, (uint32_t*)(&loadAllCtrlPara.Sft.LowerSft));
	err += At24cxx_Write_Byte (M24C02_Controller,Loadallctrlpara_Sft_Reaction_ADDR, (&loadAllCtrlPara.Sft.Reaction));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_MinAcceleration_ADDR, (uint32_t*)(&loadAllCtrlPara.MinAcceleration));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_MaxAcceleration_ADDR, (uint32_t*)(&loadAllCtrlPara.MaxAcceleration));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_MinDeceleration_ADDR, (uint32_t*)(&loadAllCtrlPara.MinDeceleration));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_MaxDeceleration_ADDR, (uint32_t*)(&loadAllCtrlPara.MaxDeceleration));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_MinSpeed_ADDR, (uint32_t*)(&loadAllCtrlPara.MinSpeed));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_MaxSpeed_ADDR, (uint32_t*)(&loadAllCtrlPara.MaxSpeed));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_Deadband_Deadband_ADDR, (uint32_t*)(&loadAllCtrlPara.Deadband.Deadband));
	err += At24cxx_Write_Byte (M24C02_Controller,Loadallctrlpara_Deadband_PercentP_ADDR, (&loadAllCtrlPara.Deadband.PercentP));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_Dither_DitherAmplitude_ADDR, (uint32_t*)(&loadAllCtrlPara.Dither.DitherAmplitude));
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_Dither_DitherFrequency_ADDR, (uint32_t*)(&loadAllCtrlPara.Dither.DitherFrequency));
	err += At24cxx_Read_Dword(M24C02_Controller,Loadallctrlpara_NominalAcc_ADDR, (uint32_t*)(&loadAllCtrlPara.Nominal.Acc));
	err += At24cxx_Read_Dword(M24C02_Controller,Loadallctrlpara_NominalSpeed_ADDR, (uint32_t*)(&loadAllCtrlPara.Nominal.Speed));

//EXTEN_MODE 所有闭环控制参数 800-914
	err += At24cxx_Write_Dword(M24C02_Controller,Loadallctrlpara_PosPid_P_ADDR, (uint32_t*)(&extAllCtrlPara.PosPid.P));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_PosPid_I_ADDR, (uint32_t*)(&extAllCtrlPara.PosPid.I));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_PosPid_D_ADDR, (uint32_t*)(&extAllCtrlPara.PosPid.D));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_PosPid_Hp_P_ADDR, (uint32_t*)(&extAllCtrlPara.PosPid_Hp.P));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_PosPid_Hp_I_ADDR, (uint32_t*)(&extAllCtrlPara.PosPid_Hp.P));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_PosPid_Hp_D_ADDR, (uint32_t*)(&extAllCtrlPara.PosPid_Hp.P));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_SpeedPid_P_ADDR, (uint32_t*)(&extAllCtrlPara.SpeedPid.P));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_SpeedPid_I_ADDR, (uint32_t*)(&extAllCtrlPara.SpeedPid.I));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_SpeedPid_D_ADDR, (uint32_t*)(&extAllCtrlPara.SpeedPid.D));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_Ffd_SpeedFFP_ADDR, (uint32_t*)(&extAllCtrlPara.Ffd.SpeedFFP));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_Ffd_PosDelay_ADDR, (uint32_t*)(&extAllCtrlPara.Ffd.PosDelay));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_Ffd_Hp_SpeedFFP_ADDR, (uint32_t*)(&extAllCtrlPara.Ffd_Hp.SpeedFFP));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_Ffd_Hp_PosDelay_ADDR, (uint32_t*)(&extAllCtrlPara.Ffd_Hp.SpeedFFP));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_DefaultAcc_ADDR, (uint32_t*)(&extAllCtrlPara.DefaultAcc));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_Speed_ADDR, (uint32_t*)(&extAllCtrlPara.Speed));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_Deviation_ADDR, (uint32_t*)(&extAllCtrlPara.Deviation));
	err += At24cxx_Write_Byte(M24C02_Controller,Extallctrlpara_DevReaction_ADDR, (&extAllCtrlPara.DevReaction));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_Wnd_WndSize_ADDR, (uint32_t*)(&extAllCtrlPara.Wnd.WndSize));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_Wnd_WndTime_ADDR, (uint32_t*)(&extAllCtrlPara.Wnd.WndTime));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_Sft_UpperSft_ADDR, (uint32_t*)(&extAllCtrlPara.Sft.UpperSft));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_Sft_LowerStf_ADDR, (uint32_t*)(&extAllCtrlPara.Sft.LowerSft));
	err += At24cxx_Write_Byte(M24C02_Controller,Extallctrlpara_Sft_Reaction_ADDR, (&extAllCtrlPara.Sft.Reaction));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_MinAcceleration_ADDR, (uint32_t*)(&extAllCtrlPara.MinAcceleration));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_MaxAcceleration_ADDR, (uint32_t*)(&extAllCtrlPara.MaxAcceleration));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_MinDeceleration_ADDR, (uint32_t*)(&extAllCtrlPara.MinDeceleration));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_MaxDeceleration_ADDR, (uint32_t*)(&extAllCtrlPara.MaxDeceleration));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_MinSpeed_ADDR, (uint32_t*)(&extAllCtrlPara.MinSpeed));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_MaxSpeed_ADDR, (uint32_t*)(&extAllCtrlPara.MaxSpeed));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_Deadband_Deadband_ADDR, (uint32_t*)(&extAllCtrlPara.Deadband.Deadband));
	err += At24cxx_Write_Byte(M24C02_Controller,Extallctrlpara_Deadband_PercentP_ADDR, (&extAllCtrlPara.Deadband.PercentP));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_Dither_DitherAmplitude_ADDR, (uint32_t*)(&extAllCtrlPara.Dither.DitherAmplitude));
	err += At24cxx_Write_Dword(M24C02_Controller,Extallctrlpara_Dither_DitherFrequency_ADDR, (uint32_t*)(&extAllCtrlPara.Dither.DitherFrequency));
	err += At24cxx_Read_Dword(M24C02_Controller,Extallctrlpara_NominalAcc_ADDR, (uint32_t*)(&extAllCtrlPara.Nominal.Acc));
	err += At24cxx_Read_Dword(M24C02_Controller,Extallctrlpara_NominalSpeed_ADDR, (uint32_t*)(&extAllCtrlPara.Nominal.Speed));

//system parameters 5000-5002
	err += At24cxx_Write_Word(M24C02_Controller,SYSPARA_TRANSMITDATAPERIOD_ADDR, (&SysPara.TransmitDataPeriod));
	err += At24cxx_Write_Byte(M24C02_Controller,SYSPARA_CONTROLLERSTRUCTURE_ADDR, (&SysPara.ControllerStructure));

//output parameters 5100-5129
	err += At24cxx_Write_Byte(M24C02_Controller,OUTPUTPARA_SIGNAL_ADDR, (&OutputPara.Signal));
	err += At24cxx_Write_Byte(M24C02_Controller,OUTPUTPARA_SIGN_ADDR, (&OutputPara.Sign));
	err += At24cxx_Write_Dword(M24C02_Controller,OUTPUTPARA_MAXVALUE_ADDR, (uint32_t*)(&OutputPara.MaxValue));
	err += At24cxx_Write_Dword(M24C02_Controller,OUTPUTPARA_MINVALUE_ADDR, (uint32_t*)(&OutputPara.MinValue));
	err += At24cxx_Write_Dword(M24C02_Controller,OUTPUTPARA_INITVALUE_ADDR, (uint32_t*)(&OutputPara.InitValue));
	err += At24cxx_Write_Dword(M24C02_Controller,OUTPUTPARA_OFFSET_ADDR, (uint32_t*)(&OutputPara.Offset));
	err += At24cxx_Write_Dword(M24C02_Controller,OUTPUTPARA_SIGNALFREQUENCY_ADDR, (uint32_t*)(&OutputPara.SignalFrequency));
	err += At24cxx_Write_Dword(M24C02_Controller,OUTPUTPARA_MAXVOLTAGE_ADDR,(uint32_t*)(&OutputPara.MaxVoltage));
	err += At24cxx_Write_Dword(M24C02_Controller,OUTPUTPARA_MAXCURRENT_ADDR, (uint32_t*)(&OutputPara.MaxCurrent));

//these data need to be saved realtime 5200-5211
	err += At24cxx_Write_Dword(M24C02_Controller,POSE_ENCODER_CODE_ADDR, (uint32_t*)(&pose.code));
	err += At24cxx_Write_Dword(M24C02_Controller,LOAD_TARE_CODE_ADDR,(uint32_t*)(&AL.tare.fValue[ch4Load]));
	err += At24cxx_Write_Dword(M24C02_Controller,EXTEN_TARE_CODE_ADDR, (uint32_t*)(&AL.tare.fValue[ch2Ext1]));

	//版本信息 20000-
	for(i=0;i<6;i++)
		err += At24cxx_Write_Dword(M24C02_Controller,PeInterface_ADDR, (uint32_t*)(&utc_version.PeInterface[i]));
	for(i=0;i<13;i++)
		err += At24cxx_Write_Dword(M24C02_Controller,Application_ADDR, (uint32_t*)(&utc_version.Application[i]));
	for(i=0;i<6;i++)
		err += At24cxx_Write_Dword(M24C02_Controller,Subsy_ADDR,(uint32_t*)(&utc_version.Subsy[i]));
	for(i=0;i<6;i++)
		err += At24cxx_Write_Dword(M24C02_Controller,SubsyCustVer_ADDR, (uint32_t*)(&utc_version.SubsyCustVer[i]));
	for(i=0;i<9;i++)
		err += At24cxx_Write_Dword(M24C02_Controller,SubsyCustName_ADDR, (uint32_t*)(&utc_version.SubsyCustName[i]));
	for(i=0;i<6;i++)
		err += At24cxx_Write_Dword(M24C02_Controller,Bios_ADDR, (uint32_t*)(&utc_version.Bios[i]));
	for(i=0;i<7;i++)
		err += At24cxx_Write_Dword(M24C02_Controller,HwCtrl_ADDR, (uint32_t*)(&utc_version.HwCtrl[i]));
	for(i=0;i<6;i++)
		err += At24cxx_Write_Dword(M24C02_Controller,PeInterfacePC_ADDR, (uint32_t*)(&utc_version.PeInterfacePC[i]));
	for(i=0;i<6;i++)
		err += At24cxx_Write_Dword(M24C02_Controller,DpxVer_ADDR, (uint32_t*)(&utc_version.DpxVer[i]));
	for(i=0;i<17;i++)
		err += At24cxx_Write_Dword(M24C02_Controller,SerialNumber_ADDR, (uint32_t*)(&utc_version.SerialNumber[i]));
	err += At24cxx_Write_Byte(M24C02_Controller, VersionEnd_ADDR, &utc_version.VersionEnd);
	
	printf("M24C02_Controller WRerr:%d\r\n",err);
	return err;
}

/**
 * @brief		将所有控制相关参数从控制器上的At24c256读出
 * @retval		err——读取数据出错数量
*/
uint8_t EepromOnController_ReadAll(void)
{
	uint8_t err=0,check_num=EEPROM_CHECK,i=0;
	err += At24cxx_Read_Byte(M24C02_Controller|1,CHEAK_ADDR, &check_num);
	err += At24cxx_Read_Dword(M24C02_Controller|1,PosDefaultAcc_ADDR, (uint32_t*)(&commBuf.Pos.DefaultAcc));
	err += At24cxx_Read_Dword(M24C02_Controller|1,PosDefaultDec_ADDR, (uint32_t*)(&commBuf.Pos.DefaultDec));
	err += At24cxx_Read_Dword(M24C02_Controller|1,LoadDefaultAcc_ADDR, (uint32_t*)(&commBuf.Load.DefaultAcc));
	err += At24cxx_Read_Dword(M24C02_Controller|1,LoadDefaultDec_ADDR, (uint32_t*)(&commBuf.Load.DefaultDec));
	err += At24cxx_Read_Dword(M24C02_Controller|1,ExtDefaultAcc_ADDR, (uint32_t*)(&commBuf.Ext.DefaultAcc));
	err += At24cxx_Read_Dword(M24C02_Controller|1,ExtDefaultDec_ADDR, (uint32_t*)(&commBuf.Ext.DefaultDec));
	err += At24cxx_Read_Dword(M24C02_Controller|1,upperlimit_phy_ADDR, (uint32_t*)(&AL.upperlimit_phy));
	err += At24cxx_Read_Dword(M24C02_Controller|1,lowerlimit_phy_ADDR, (uint32_t*)(&AL.lowerlimit_phy));
	err += At24cxx_Read_Dword(M24C02_Controller|1,defaultspeed_ADDR, (uint32_t*)(&AL.defaultspeed));
	err += At24cxx_Read_Dword(M24C02_Controller|1,defaultspeedlimit_ADDR, (uint32_t*)(&AL.defaultspeedlimit));
	
	
//POS_MODE 所有闭环控制参数 400-514
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_PosPid_P_ADDR, (uint32_t*)(&posAllCtrlPara.PosPid.P));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_PosPid_I_ADDR, (uint32_t*)(&posAllCtrlPara.PosPid.I));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_PosPid_D_ADDR, (uint32_t*)(&posAllCtrlPara.PosPid.D));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_PosPid_Hp_P_ADDR, (uint32_t*)(&posAllCtrlPara.PosPid_Hp.P));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_PosPid_Hp_I_ADDR, (uint32_t*)(&posAllCtrlPara.PosPid_Hp.P));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_PosPid_Hp_D_ADDR, (uint32_t*)(&posAllCtrlPara.PosPid_Hp.P));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_SpeedPid_P_ADDR, (uint32_t*)(&posAllCtrlPara.SpeedPid.P));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_SpeedPid_I_ADDR, (uint32_t*)(&posAllCtrlPara.SpeedPid.I));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_SpeedPid_D_ADDR, (uint32_t*)(&posAllCtrlPara.SpeedPid.D));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_Ffd_SpeedFFP_ADDR, (uint32_t*)(&posAllCtrlPara.Ffd.SpeedFFP));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_Ffd_PosDelay_ADDR, (uint32_t*)(&posAllCtrlPara.Ffd.PosDelay));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_Ffd_Hp_SpeedFFP_ADDR, (uint32_t*)(&posAllCtrlPara.Ffd_Hp.SpeedFFP));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_Ffd_Hp_PosDelay_ADDR, (uint32_t*)(&posAllCtrlPara.Ffd_Hp.SpeedFFP));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_DefaultAcc_ADDR, (uint32_t*)(&posAllCtrlPara.DefaultAcc));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_Speed_ADDR, (uint32_t*)(&posAllCtrlPara.Speed));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_Deviation_ADDR, (uint32_t*)(&posAllCtrlPara.Deviation));
	err += At24cxx_Read_Byte (M24C02_Controller|1,Posallctrlpara_DevReaction_ADDR, (&posAllCtrlPara.DevReaction));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_Wnd_WndSize_ADDR, (uint32_t*)(&posAllCtrlPara.Wnd.WndSize));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_Wnd_WndTime_ADDR, (uint32_t*)(&posAllCtrlPara.Wnd.WndTime));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_Sft_UpperSft_ADDR, (uint32_t*)(&posAllCtrlPara.Sft.UpperSft));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_Sft_LowerStf_ADDR, (uint32_t*)(&posAllCtrlPara.Sft.LowerSft));
	err += At24cxx_Read_Byte (M24C02_Controller|1,Posallctrlpara_Sft_Reaction_ADDR, (&posAllCtrlPara.Sft.Reaction));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_MinAcceleration_ADDR, (uint32_t*)(&posAllCtrlPara.MinAcceleration));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_MaxAcceleration_ADDR, (uint32_t*)(&posAllCtrlPara.MaxAcceleration));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_MinDeceleration_ADDR, (uint32_t*)(&posAllCtrlPara.MinDeceleration));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_MaxDeceleration_ADDR, (uint32_t*)(&posAllCtrlPara.MaxDeceleration));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_MinSpeed_ADDR, (uint32_t*)(&posAllCtrlPara.MinSpeed));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_MaxSpeed_ADDR, (uint32_t*)(&posAllCtrlPara.MaxSpeed));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_Deadband_Deadband_ADDR, (uint32_t*)(&posAllCtrlPara.Deadband.Deadband));
	err += At24cxx_Read_Byte (M24C02_Controller|1,Posallctrlpara_Deadband_PercentP_ADDR, (&posAllCtrlPara.Deadband.PercentP));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_Dither_DitherAmplitude_ADDR, (uint32_t*)(&posAllCtrlPara.Dither.DitherAmplitude));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_Dither_DitherFrequency_ADDR, (uint32_t*)(&posAllCtrlPara.Dither.DitherFrequency));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_NominalAcc_ADDR, (uint32_t*)(&posAllCtrlPara.Nominal.Acc));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Posallctrlpara_NominalSpeed_ADDR, (uint32_t*)(&posAllCtrlPara.Nominal.Speed));
	
//LOAD_MODE 所有闭环控制参数 600-714
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_PosPid_P_ADDR, (uint32_t*)(&loadAllCtrlPara.PosPid.P));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_PosPid_I_ADDR, (uint32_t*)(&loadAllCtrlPara.PosPid.I));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_PosPid_D_ADDR, (uint32_t*)(&loadAllCtrlPara.PosPid.D));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_PosPid_Hp_P_ADDR, (uint32_t*)(&loadAllCtrlPara.PosPid_Hp.P));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_PosPid_Hp_I_ADDR, (uint32_t*)(&loadAllCtrlPara.PosPid_Hp.P));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_PosPid_Hp_D_ADDR, (uint32_t*)(&loadAllCtrlPara.PosPid_Hp.P));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_SpeedPid_P_ADDR, (uint32_t*)(&loadAllCtrlPara.SpeedPid.P));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_SpeedPid_I_ADDR, (uint32_t*)(&loadAllCtrlPara.SpeedPid.I));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_SpeedPid_D_ADDR, (uint32_t*)(&loadAllCtrlPara.SpeedPid.D));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_Ffd_SpeedFFP_ADDR, (uint32_t*)(&loadAllCtrlPara.Ffd.SpeedFFP));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_Ffd_PosDelay_ADDR, (uint32_t*)(&loadAllCtrlPara.Ffd.PosDelay));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_Ffd_Hp_SpeedFFP_ADDR, (uint32_t*)(&loadAllCtrlPara.Ffd_Hp.SpeedFFP));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_Ffd_Hp_PosDelay_ADDR, (uint32_t*)(&loadAllCtrlPara.Ffd_Hp.SpeedFFP));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_DefaultAcc_ADDR, (uint32_t*)(&loadAllCtrlPara.DefaultAcc));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_Speed_ADDR, (uint32_t*)(&loadAllCtrlPara.Speed));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_Deviation_ADDR, (uint32_t*)(&loadAllCtrlPara.Deviation));
	err += At24cxx_Read_Byte (M24C02_Controller|1,Loadallctrlpara_DevReaction_ADDR, (&loadAllCtrlPara.DevReaction));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_Wnd_WndSize_ADDR, (uint32_t*)(&loadAllCtrlPara.Wnd.WndSize));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_Wnd_WndTime_ADDR, (uint32_t*)(&loadAllCtrlPara.Wnd.WndTime));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_Sft_UpperSft_ADDR, (uint32_t*)(&loadAllCtrlPara.Sft.UpperSft));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_Sft_LowerStf_ADDR, (uint32_t*)(&loadAllCtrlPara.Sft.LowerSft));
	err += At24cxx_Read_Byte (M24C02_Controller|1,Loadallctrlpara_Sft_Reaction_ADDR, (&loadAllCtrlPara.Sft.Reaction));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_MinAcceleration_ADDR, (uint32_t*)(&loadAllCtrlPara.MinAcceleration));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_MaxAcceleration_ADDR, (uint32_t*)(&loadAllCtrlPara.MaxAcceleration));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_MinDeceleration_ADDR, (uint32_t*)(&loadAllCtrlPara.MinDeceleration));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_MaxDeceleration_ADDR, (uint32_t*)(&loadAllCtrlPara.MaxDeceleration));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_MinSpeed_ADDR, (uint32_t*)(&loadAllCtrlPara.MinSpeed));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_MaxSpeed_ADDR, (uint32_t*)(&loadAllCtrlPara.MaxSpeed));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_Deadband_Deadband_ADDR, (uint32_t*)(&loadAllCtrlPara.Deadband.Deadband));
	err += At24cxx_Read_Byte (M24C02_Controller|1,Loadallctrlpara_Deadband_PercentP_ADDR, (&loadAllCtrlPara.Deadband.PercentP));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_Dither_DitherAmplitude_ADDR, (uint32_t*)(&loadAllCtrlPara.Dither.DitherAmplitude));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_Dither_DitherFrequency_ADDR, (uint32_t*)(&loadAllCtrlPara.Dither.DitherFrequency));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_NominalAcc_ADDR, (uint32_t*)(&loadAllCtrlPara.Nominal.Acc));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_NominalSpeed_ADDR, (uint32_t*)(&loadAllCtrlPara.Nominal.Speed));

//EXTEN_MODE 所有闭环控制参数 800-914
	err += At24cxx_Read_Dword(M24C02_Controller|1,Loadallctrlpara_PosPid_P_ADDR, (uint32_t*)(&extAllCtrlPara.PosPid.P));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_PosPid_I_ADDR, (uint32_t*)(&extAllCtrlPara.PosPid.I));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_PosPid_D_ADDR, (uint32_t*)(&extAllCtrlPara.PosPid.D));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_PosPid_Hp_P_ADDR, (uint32_t*)(&extAllCtrlPara.PosPid_Hp.P));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_PosPid_Hp_I_ADDR, (uint32_t*)(&extAllCtrlPara.PosPid_Hp.P));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_PosPid_Hp_D_ADDR, (uint32_t*)(&extAllCtrlPara.PosPid_Hp.P));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_SpeedPid_P_ADDR, (uint32_t*)(&extAllCtrlPara.SpeedPid.P));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_SpeedPid_I_ADDR, (uint32_t*)(&extAllCtrlPara.SpeedPid.I));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_SpeedPid_D_ADDR, (uint32_t*)(&extAllCtrlPara.SpeedPid.D));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_Ffd_SpeedFFP_ADDR, (uint32_t*)(&extAllCtrlPara.Ffd.SpeedFFP));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_Ffd_PosDelay_ADDR, (uint32_t*)(&extAllCtrlPara.Ffd.PosDelay));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_Ffd_Hp_SpeedFFP_ADDR, (uint32_t*)(&extAllCtrlPara.Ffd_Hp.SpeedFFP));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_Ffd_Hp_PosDelay_ADDR, (uint32_t*)(&extAllCtrlPara.Ffd_Hp.SpeedFFP));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_DefaultAcc_ADDR, (uint32_t*)(&extAllCtrlPara.DefaultAcc));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_Speed_ADDR, (uint32_t*)(&extAllCtrlPara.Speed));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_Deviation_ADDR, (uint32_t*)(&extAllCtrlPara.Deviation));
	err += At24cxx_Read_Byte (M24C02_Controller|1,Extallctrlpara_DevReaction_ADDR, (&extAllCtrlPara.DevReaction));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_Wnd_WndSize_ADDR, (uint32_t*)(&extAllCtrlPara.Wnd.WndSize));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_Wnd_WndTime_ADDR, (uint32_t*)(&extAllCtrlPara.Wnd.WndTime));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_Sft_UpperSft_ADDR, (uint32_t*)(&extAllCtrlPara.Sft.UpperSft));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_Sft_LowerStf_ADDR, (uint32_t*)(&extAllCtrlPara.Sft.LowerSft));
	err += At24cxx_Read_Byte (M24C02_Controller|1,Extallctrlpara_Sft_Reaction_ADDR, (&extAllCtrlPara.Sft.Reaction));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_MinAcceleration_ADDR, (uint32_t*)(&extAllCtrlPara.MinAcceleration));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_MaxAcceleration_ADDR, (uint32_t*)(&extAllCtrlPara.MaxAcceleration));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_MinDeceleration_ADDR, (uint32_t*)(&extAllCtrlPara.MinDeceleration));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_MaxDeceleration_ADDR, (uint32_t*)(&extAllCtrlPara.MaxDeceleration));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_MinSpeed_ADDR, (uint32_t*)(&extAllCtrlPara.MinSpeed));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_MaxSpeed_ADDR, (uint32_t*)(&extAllCtrlPara.MaxSpeed));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_Deadband_Deadband_ADDR, (uint32_t*)(&extAllCtrlPara.Deadband.Deadband));
	err += At24cxx_Read_Byte (M24C02_Controller|1,Extallctrlpara_Deadband_PercentP_ADDR, (&extAllCtrlPara.Deadband.PercentP));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_Dither_DitherAmplitude_ADDR, (uint32_t*)(&extAllCtrlPara.Dither.DitherAmplitude));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_Dither_DitherFrequency_ADDR, (uint32_t*)(&extAllCtrlPara.Dither.DitherFrequency));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_NominalAcc_ADDR, (uint32_t*)(&extAllCtrlPara.Nominal.Acc));
	err += At24cxx_Read_Dword(M24C02_Controller|1,Extallctrlpara_NominalSpeed_ADDR, (uint32_t*)(&extAllCtrlPara.Nominal.Speed));

//system parameters 5000-5002
	err += At24cxx_Read_Word(M24C02_Controller|1,SYSPARA_TRANSMITDATAPERIOD_ADDR, (&SysPara.TransmitDataPeriod));
	err += At24cxx_Read_Byte(M24C02_Controller|1,SYSPARA_CONTROLLERSTRUCTURE_ADDR, (&SysPara.ControllerStructure));

//output parameters 5100-5129
	err += At24cxx_Read_Byte(M24C02_Controller|1,OUTPUTPARA_SIGNAL_ADDR, (&OutputPara.Signal));
	err += At24cxx_Read_Byte(M24C02_Controller|1,OUTPUTPARA_SIGN_ADDR, (&OutputPara.Sign));
	err += At24cxx_Read_Dword(M24C02_Controller|1,OUTPUTPARA_MAXVALUE_ADDR, (uint32_t*)(&OutputPara.MaxValue));
	err += At24cxx_Read_Dword(M24C02_Controller|1,OUTPUTPARA_MINVALUE_ADDR, (uint32_t*)(&OutputPara.MinValue));
	err += At24cxx_Read_Dword(M24C02_Controller|1,OUTPUTPARA_INITVALUE_ADDR, (uint32_t*)(&OutputPara.InitValue));
	err += At24cxx_Read_Dword(M24C02_Controller|1,OUTPUTPARA_OFFSET_ADDR, (uint32_t*)(&OutputPara.Offset));
	err += At24cxx_Read_Dword(M24C02_Controller|1,OUTPUTPARA_SIGNALFREQUENCY_ADDR, (uint32_t*)(&OutputPara.SignalFrequency));
	err += At24cxx_Read_Dword(M24C02_Controller|1,OUTPUTPARA_MAXVOLTAGE_ADDR,(uint32_t*)(&OutputPara.MaxVoltage));
	err += At24cxx_Read_Dword(M24C02_Controller|1,OUTPUTPARA_MAXCURRENT_ADDR, (uint32_t*)(&OutputPara.MaxCurrent));

//these data need to be saved realtime 5200-5211
	err += At24cxx_Read_Dword(M24C02_Controller|1,POSE_ENCODER_CODE_ADDR, (uint32_t*)(&pose.code));
	err += At24cxx_Read_Dword(M24C02_Controller|1,LOAD_TARE_CODE_ADDR,(uint32_t*)(&AL.tare.fValue[ch4Load]));
	err += At24cxx_Read_Dword(M24C02_Controller|1,EXTEN_TARE_CODE_ADDR, (uint32_t*)(&AL.tare.fValue[ch2Ext1]));

	//版本信息
	for(i=0;i<6;i++)
		err += At24cxx_Read_Dword(M24C02_Controller|1,PeInterface_ADDR, (uint32_t*)(&utc_version.PeInterface[i]));
	for(i=0;i<13;i++)
		err += At24cxx_Read_Dword(M24C02_Controller|1,Application_ADDR, (uint32_t*)(&utc_version.Application[i]));
	for(i=0;i<6;i++)
		err += At24cxx_Read_Dword(M24C02_Controller|1,Subsy_ADDR, (uint32_t*)(&utc_version.Subsy[i]));
	for(i=0;i<6;i++)
		err += At24cxx_Read_Dword(M24C02_Controller|1,SubsyCustVer_ADDR, (uint32_t*)(&utc_version.SubsyCustVer));
	for(i=0;i<9;i++)
		err += At24cxx_Read_Dword(M24C02_Controller|1,SubsyCustName_ADDR, (uint32_t*)(&utc_version.SubsyCustName));
	for(i=0;i<6;i++)
		err += At24cxx_Read_Dword(M24C02_Controller|1,Bios_ADDR, (uint32_t*)(&utc_version.Bios));
	for(i=0;i<7;i++)
		err += At24cxx_Read_Dword(M24C02_Controller|1,HwCtrl_ADDR, (uint32_t*)(&utc_version.HwCtrl));
	for(i=0;i<6;i++)
		err += At24cxx_Read_Dword(M24C02_Controller|1,PeInterfacePC_ADDR, (uint32_t*)(&utc_version.PeInterfacePC));
	for(i=0;i<6;i++)
		err += At24cxx_Read_Dword(M24C02_Controller|1,DpxVer_ADDR, (uint32_t*)(&utc_version.DpxVer));
	for(i=0;i<17;i++)
		err += At24cxx_Read_Dword(M24C02_Controller|1,SerialNumber_ADDR, (uint32_t*)(&utc_version.SerialNumber));
	err += At24cxx_Read_Byte(M24C02_Controller|1, VersionEnd_ADDR, &utc_version.VersionEnd);
	
	printf("M24C02_Controller RDerr:%d\r\n",err);
	return err;
}

/**
 * @brief		write DefaultAcc in controller's At24c256 
 * @param[in]	Device_Addr	type:uint16_t
 * @param[out]	x 			type:ALLCTRLPARA*
 * @param[in]	movectrl	type:uint8_t
 * @retval		err——number of data write errors
*/
uint8_t EepromDefaultAcc_write(uint16_t Device_Addr,ALLCTRLPARA* x,uint8_t movectrl)
{
	uint8_t err=0,check_num=EEPROM_CHECK;
	uint16_t acc_addr=0;

	err += At24cxx_Read_Byte (M24C02_Controller,CHEAK_ADDR, &check_num);
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
	err += At24cxx_Write_Dword(M24C02_Controller,acc_addr, (uint32_t*)(&x->DefaultAcc));

	printf("M24C02_Controller %2x DefaultAcc WRerr:%d\r\n",movectrl,err);
	return err;
}

/**
 * @brief		write Speed in controller's At24c256 
 * @param[in]	Device_Addr	type:uint16_t
 * @param[out]	x 			type:ALLCTRLPARA*
 * @param[in]	movectrl	type:uint8_t
 * @retval		err——number of data write errors
*/
uint8_t EepromSpeed_write(uint16_t Device_Addr,ALLCTRLPARA* x,uint8_t movectrl)
{
	uint8_t err=0,check_num=EEPROM_CHECK;
	uint16_t speed_addr=0;

	err += At24cxx_Read_Byte (M24C02_Controller,CHEAK_ADDR, &check_num);
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
	err += At24cxx_Write_Dword(M24C02_Controller,speed_addr, (uint32_t*)(&x->Speed));

	printf("M24C02_Controller %2x Speed WRerr:%d\r\n",movectrl,err);
	return err;
}

/**
 * @brief		write Nominal acc and speed in controller's At24c256 
 * @param[in]	Device_Addr	type:uint16_t
 * @param[out]	x 			type:NOMINALVALUE*
 * @param[in]	movectrl	type:uint8_t
 * @retval		err——number of data write errors
*/
uint8_t EepromNominal_write(uint16_t Device_Addr,NOMINALVALUE* x,uint8_t movectrl)
{
	uint8_t err=0,check_num=EEPROM_CHECK;
	uint16_t acc_addr=0,speed_addr=0;

	err += At24cxx_Read_Byte (M24C02_Controller,CHEAK_ADDR, &check_num);
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
	err += At24cxx_Write_Dword(M24C02_Controller,acc_addr, (uint32_t*)(&x->Acc));
	err += At24cxx_Write_Dword(M24C02_Controller,speed_addr, (uint32_t*)(&x->Speed));
	printf("M24C02_Controller %2x Nominal acc and speed WRerr:%d\r\n",movectrl,err);
	return err;
}

/** 
 * @brief		write transmit data period in controller's At24c256 
 * @param[in]	Device_Addr	type:uint16_t
 * @param[out]	x 			type:SysPara*
 * @retval		err——number of data write errors
*/
uint8_t EepromTansmitDataPeriod_write(uint16_t Device_Addr,SYSPARA* x)
{
	uint8_t err=0,check_num=EEPROM_CHECK;

	err += At24cxx_Read_Byte (M24C02_Controller,CHEAK_ADDR, &check_num);
	err += At24cxx_Write_Word(M24C02_Controller,SYSPARA_TRANSMITDATAPERIOD_ADDR, (&x->TransmitDataPeriod));

	printf("M24C02_Controller TansmitDataPeriod WRerr:%d\r\n",err);
	return err;
}

/** 
 * @brief		write Wnd size and time in controller's At24c256 
 * @param[in]	Device_Addr	type:uint16_t
 * @param[out]	x 			type:WND*
 * @param[in]	movectrl	type:uint8_t
 * @retval		err——number of data write errors
*/
uint8_t EepromWnd_write(uint16_t Device_Addr,WND* x,uint8_t movectrl)
{
	uint8_t err=0,check_num=EEPROM_CHECK;
	uint16_t size_addr=0,time_addr=0;

	err += At24cxx_Read_Byte (M24C02_Controller,CHEAK_ADDR, &check_num);
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
	err += At24cxx_Write_Dword(M24C02_Controller,size_addr, (uint32_t*)(&x->WndSize));
	err += At24cxx_Write_Dword(M24C02_Controller,time_addr, (&x->WndTime));

	printf("M24C02_Controller %2x Wnd size&time WRerr:%d\r\n",movectrl,err);
	return err;
}

/**
 * @brief		write software lower, upper and reaction in controller's At24c256 
 * @param[in]	Device_Addr	type:uint16_t
 * @param[out]	x 			type:SFT*
 * @param[in]	movectrl	type:uint8_t
 * @retval		err——number of data write errors
*/
uint8_t EepromSft_write(uint16_t Device_Addr,SFT* x,uint8_t movectrl)
{
	uint8_t err=0,check_num=EEPROM_CHECK;
	uint16_t lowersft_addr=0,uppersft_addr=0,reaction_addr=0;

	err += At24cxx_Read_Byte (M24C02_Controller,CHEAK_ADDR, &check_num);
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
	err += At24cxx_Write_Dword(M24C02_Controller,lowersft_addr, (uint32_t*)(&x->LowerSft));
	err += At24cxx_Write_Dword(M24C02_Controller,uppersft_addr, (uint32_t*)(&x->UpperSft));
	err += At24cxx_Write_Byte(M24C02_Controller,reaction_addr, (&x->Reaction));

	printf("M24C02_Controller %2x Sft lower&upper&reaction WRerr:%d\r\n",movectrl,err);
	return err;
}

/**
 * @brief		write PosPID parameters in controller's At24c256 
 * @param[in]	Device_Addr	type:uint16_t
 * @param[out]	x 			type:PID*
 * @param[in]	movectrl	type:uint8_t
 * @retval		err——number of data write errors
*/
uint8_t EepromPosPID_write(uint16_t Device_Addr,PID* x,uint8_t movectrl)
{
	uint8_t err=0,check_num=EEPROM_CHECK;
	uint16_t p_addr=0,i_addr=0,d_addr=0;

	err += At24cxx_Read_Byte (M24C02_Controller,CHEAK_ADDR, &check_num);
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
	err += At24cxx_Write_Dword(M24C02_Controller,p_addr, (uint32_t*)(&x->P));
	err += At24cxx_Write_Dword(M24C02_Controller,i_addr, (uint32_t*)(&x->I));
	err += At24cxx_Write_Dword(M24C02_Controller,d_addr,  (uint32_t*)(&x->D));

	printf("M24C02_Controller %2x PosPID  WRerr:%d\r\n",movectrl,err);
	return err;
}

/**
 * @brief		write SpeedPID parameters in controller's At24c256 
 * @param[in]	Device_Addr	type:uint16_t
 * @param[out]	x 			type:PID*
 * @param[in]	movectrl	type:uint8_t
 * @retval		err——number of data write errors
*/
uint8_t EepromSpeedPID_write(uint16_t Device_Addr,PID* x,uint8_t movectrl)
{
	uint8_t err=0,check_num=EEPROM_CHECK;
	uint16_t p_addr=0,i_addr=0,d_addr=0;

	err += At24cxx_Read_Byte (M24C02_Controller,CHEAK_ADDR, &check_num);
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
	err += At24cxx_Write_Dword(M24C02_Controller,p_addr, (uint32_t*)(&x->P));
	err += At24cxx_Write_Dword(M24C02_Controller,i_addr, (uint32_t*)(&x->I));
	err += At24cxx_Write_Dword(M24C02_Controller,d_addr,  (uint32_t*)(&x->D));

	printf("M24C02_Controller %2x SpeedPid  WRerr:%d\r\n",movectrl,err);
	return err;
}

/**
 * @brief		write Ffd parameters in controller's At24c256 
 * @param[in]	Device_Addr	type:uint16_t
 * @param[out]	x 			type:FFD*
 * @param[in]	movectrl	type:uint8_t
 * @retval		err——number of data write errors
*/
uint8_t EepromFfd_write(uint16_t Device_Addr,FFD* x,uint8_t movectrl)
{
	uint8_t err=0,check_num=EEPROM_CHECK;
	uint16_t ffp_addr=0,posdelay_addr=0;

	err += At24cxx_Read_Byte (M24C02_Controller,CHEAK_ADDR, &check_num);
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
	err += At24cxx_Write_Dword(M24C02_Controller,ffp_addr, (uint32_t*)(&x->SpeedFFP));
	err += At24cxx_Write_Dword(M24C02_Controller,posdelay_addr, (&x->PosDelay));

	printf("M24C02_Controller %2x Ffd WRerr:%d\r\n",movectrl,err);
	return err;
}

/**
 * @brief		将传感器头数据头写入传感器接头上的At24c256
 * @param		Device_Addr——设备地址
 * @retval		err——写入数据出错数量
*/
uint8_t EepromSensorHeaderData_Write(uint16_t Device_Addr,SENSORHEADER* x)
{
	uint8_t err=0;
//	uint8_t check_num=EEPROM_CHECK,i=0;
//	err += At24cxx_Write_Byte (Device_Addr,CHEAK_ADDR, &check_num);
//	err += At24cxx_Write_Word (Device_Addr,PartNo_ADDR, (uint16_t*)(&x->PartNo));
//	err += At24cxx_Write_Byte (Device_Addr,Version_ADDR, (&x->Version));
//	err += At24cxx_Write_Dword(Device_Addr,SerNo_ADDR, (uint32_t*)(&x->SerNo));
//	err += At24cxx_Write_Word (Device_Addr,Class_ADDR, (&x->Class));
//	err += At24cxx_Write_Byte (Device_Addr,DatVersion_ADDR,(uint8_t*)(&x->DatVersion));

//	printf("Device_Addr:0x%2x SensorHeaderData WRerr:%d\r\n",Device_Addr,err);
	return err;
}


/**
 * @brief		将传感器头数据头从传感器接头上的At24c256读出
 * @param		Device_Addr——设备地址
 * @retval		err——写入数据出错数量
*/
uint8_t EepromSensorHeaderData_Read(uint16_t Device_Addr,SENSORHEADER* x)
{
	uint8_t err=0;
//	uint8_t check_num=EEPROM_CHECK,i=0;
//	err += At24cxx_Read_Byte (Device_Addr,CHEAK_ADDR, &check_num);
//	err += At24cxx_Read_Word (Device_Addr,PartNo_ADDR, (uint16_t*)(&x->PartNo));
//	err += At24cxx_Read_Byte (Device_Addr,Version_ADDR, (&x->Version));
//	err += At24cxx_Read_Dword(Device_Addr,SerNo_ADDR, (uint32_t*)(&x->SerNo));
//	err += At24cxx_Read_Word (Device_Addr,Class_ADDR, (&x->Class));
//	err += At24cxx_Read_Byte (Device_Addr,DatVersion_ADDR,(uint8_t*)(&x->DatVersion));

//	printf("Device_Addr:0x%2x SensorHeaderData RDerr:%d\r\n",Device_Addr,err);
	return err;
}

/**
 * @brief		将所有模拟量传感器数据写入传感器接头上的At24c256
 * @param		Device_Addr——设备地址
 * @retval		err——写入数据出错数量
*/
uint8_t EepromAnalogueSensor_WriteAll(uint16_t Device_Addr,SENSORANALOGUEDATA* x)
{
	uint8_t err=0;
//	uint8_t check_num=EEPROM_CHECK,i=0;
//	err += At24cxx_Write_Byte (Device_Addr,CHEAK_ADDR, &check_num);
//	err += At24cxx_Write_Dword(Device_Addr,MaxExcitation_ADDR, (uint32_t*)(&(x->MaxExcitation)));
//	err += At24cxx_Write_Word (Device_Addr,MinImpedance_ADDR, (uint16_t*)(&x->MinImpedance));
//	err += At24cxx_Write_Dword(Device_Addr,NominalValue_ADDR, (uint32_t*)(&x->NominalValue));
//	err += At24cxx_Write_Word (Device_Addr,Unit_ADDR, (&x->Unit));
//	err += At24cxx_Write_Dword(Device_Addr,Offset_ADDR, (uint32_t*)(&x->Offset));
//	err += At24cxx_Write_Word (Device_Addr,NegLimit_ADDR, (&x->NegLimit));
//	err += At24cxx_Write_Word (Device_Addr,PosLimit_ADDR, (&x->PosLimit));
//	err += At24cxx_Write_Dword(Device_Addr,Reference_ADDR, (uint32_t*)(&x->Reference));
//	err += At24cxx_Write_Dword(Device_Addr,CorrReference_ADDR, (uint32_t*)(&x->CorrReference));
//	err += At24cxx_Write_Word (Device_Addr,Sensortype_ADDR, (&x->Sensortype));
//	err += At24cxx_Write_Dword(Device_Addr,NominalSensitive_ADDR, (uint32_t*)(&x->NominalSensitive));
//	//GaugeLength
//	err += At24cxx_Write_Word (Device_Addr,GaugeLength_ADDR, (&x->GaugeLength));
//	err += At24cxx_Write_Word (Device_Addr,Sign_ADDR, (&x->Sign));
//	err += At24cxx_Write_Word (Device_Addr,Day_ADDR, (&x->Day));
//	err += At24cxx_Write_Word (Device_Addr,Month_ADDR, (&x->Month));
//	err += At24cxx_Write_Word (Device_Addr,Year_ADDR, (&x->Year));
//	err += At24cxx_Write_Word (Device_Addr,LinPoint_ADDR, (&x->LinPoint));
//	for(i=0;i<SEN_LIN_DATA_MAX;i++)
//	{
//		err += At24cxx_Write_Dword(Device_Addr,LinVal_ADDR+8*i, (uint32_t*)(&x->LinV2[i].MeasValue));
//		err += At24cxx_Write_Dword(Device_Addr,LinVal_ADDR+4+8*i, (uint32_t*)(&x->LinV2[i].RefValue));
//	}
//	printf("Device_Addr:0x%2x WRerr:%d\r\n",Device_Addr,err);
	return err;
}

/**
 * @brief		将所有模拟量传感器数据从传感器接头上的At24c256读出
 * @param		Device_Addr——设备地址
 * @retval		err——读取数据出错数量
*/
uint8_t EepromAnalogueSensor_ReadAll(uint16_t Device_Addr,SENSORANALOGUEDATA* x)
{
	uint8_t err=0;
//	uint8_t check_num=EEPROM_CHECK,i=0;
//	err += At24cxx_Read_Byte (Device_Addr|1,CHEAK_ADDR, &check_num);
//	err += At24cxx_Read_Dword(Device_Addr|1,MaxExcitation_ADDR, (uint32_t*)(&x->MaxExcitation));
//	err += At24cxx_Read_Word (Device_Addr|1,MinImpedance_ADDR, (uint16_t*)(&x->MinImpedance));
//	err += At24cxx_Read_Dword(Device_Addr|1,NominalValue_ADDR, (uint32_t*)(&x->NominalValue));
//	err += At24cxx_Read_Word (Device_Addr|1,Unit_ADDR, (&x->Unit));
//	err += At24cxx_Read_Dword(Device_Addr|1,Offset_ADDR, (uint32_t*)(&x->Offset));
//	err += At24cxx_Read_Word (Device_Addr|1,NegLimit_ADDR, (&x->NegLimit));
//	err += At24cxx_Read_Word (Device_Addr|1,PosLimit_ADDR, (&x->PosLimit));
//	err += At24cxx_Read_Dword(Device_Addr|1,Reference_ADDR, (uint32_t*)(&x->Reference));
//	err += At24cxx_Read_Dword(Device_Addr|1,CorrReference_ADDR, (uint32_t*)(&x->CorrReference));
//	err += At24cxx_Read_Word (Device_Addr|1,Sensortype_ADDR, (&x->Sensortype));
//	err += At24cxx_Read_Dword(Device_Addr|1,NominalSensitive_ADDR, (uint32_t*)(&x->NominalSensitive));
//	err += At24cxx_Read_Word (Device_Addr|1,Sign_ADDR, (&x->Sign));
//	err += At24cxx_Read_Word (Device_Addr|1,Day_ADDR, (&x->Day));
//	err += At24cxx_Read_Word (Device_Addr|1,Month_ADDR, (&x->Month));
//	err += At24cxx_Read_Word (Device_Addr|1,Year_ADDR, (&x->Year));
//	err += At24cxx_Read_Word (Device_Addr|1,LinPoint_ADDR, (&x->LinPoint));
//	for(i=0;i<SEN_LIN_DATA_MAX;i++)
//	{
//		err += At24cxx_Read_Dword(Device_Addr|1,(LinVal_ADDR+8*i), (uint32_t*)(&x->LinV2[i].MeasValue));
//		err += At24cxx_Read_Dword(Device_Addr|1,(LinVal_ADDR+4+8*i), (uint32_t*)(&x->LinV2[i].RefValue));
//	}
//	printf("Device_Addr:0x%2x RDerr:%d\r\n",Device_Addr,err);
	return err;
}

/**
 * @brief		检查AT24CXX是否正常，用了24XX的最后一个地址(CHEAK_ADDR)来存储标志字，
				如果用其他24C系列,这个地址要修改
 * @param		Device_Addr	——设备地址
 * @param		x			——SENSORANALOGUEDATA 
 * @retval		返回1:检测失败 返回0:检测成功
*/
uint8_t At24cxx_Check(uint16_t Device_Addr,void* x,uint8_t DeviceType)
{ 
	uint8_t temp;
	uint8_t dat=EEPROM_CHECK;
	At24cxx_Read_Byte(Device_Addr|1,50,&temp);	
	printf("Device_Addr:0x%2x CHEAK_ADDR:0x%2x\r\n",Device_Addr,temp);
	HAL_Delay(10);
	
	if(temp == EEPROM_CHECK)
		return 0;		   
	else//排除第一次初始化的情况
	{
		At24cxx_Write_Byte(Device_Addr,50,&dat);
			HAL_Delay(10);		//写一次和读一次之间需要短暂的延时
		
		if(DeviceType == Dev_Controller)
			EepromOnController_WriteAll();
		else if(DeviceType == Dev_Sensor)
			EepromSensor_WriteAll(Device_Addr,(sensorData_t*)x);
		else if(DeviceType == Dev_SensorBigDeformation)
			EepromSensorBigDeformation_WriteAll(Device_Addr,(SENSORBIGDEFORMATIONDATA*)x);
		HAL_Delay(10);
		
		At24cxx_Read_Byte(Device_Addr|1,50,&temp);	
		printf("Device_Addr:0x%2x CHEAK_ADDR:0x%2x\r\n",Device_Addr,temp);
		HAL_Delay(10);
		
		if(temp == EEPROM_CHECK)
			return 0;
	}
	return 1;											  
}

/**
 * @brief		将传感器所有数据写入传感器接头上的At24c256
 * @note		其中适用传感器包括：位置传感器，力传感器，引伸计
 * @param		Device_Addr——设备地址
 * @retval		err——写入数据出错数量
*/

//Sensor Content 
//400Byte 100-499
uint8_t EepromSensor_WriteAll(uint16_t Device_Addr,sensorData_t* x)
{
	uint8_t err=0,check_num=EEPROM_CHECK,i=0;
	err += At24cxx_Write_Byte (Device_Addr,CHEAK_ADDR, &check_num);
	err += At24cxx_Write_Byte (Device_Addr,SensorType_ADDR, (&x->Sensortype));
	err += At24cxx_Write_Byte (Device_Addr,Sign_ADDR, (&x->Sign));
	err += At24cxx_Write_Dword(Device_Addr,NominalValue_ADDR, (uint32_t*)(&x->NominalValue));
	err += At24cxx_Write_Dword(Device_Addr,NominalSensitive_ADDR, (uint32_t*)(&x->NominalSensitive));
	err += At24cxx_Write_Byte (Device_Addr,Day_ADDR, (&x->Day));
	err += At24cxx_Write_Byte (Device_Addr,Month_ADDR, (&x->Month));
	err += At24cxx_Write_Word (Device_Addr,Year_ADDR, (&x->Year));
	err += At24cxx_Write_Byte (Device_Addr,LinPoint_ADDR, (&x->LinPoint));

	for(i=0;i<SEN_LIN_DATA_MAX;i++)
	{
		err += At24cxx_Write_Dword(Device_Addr,LinV_ADDR+12*i, (uint32_t*)(&x->LinV[i].ADCCode));
		err += At24cxx_Write_Dword(Device_Addr,LinV_ADDR+4+12*i,(uint32_t*)(&x->LinV[i].RefValue));
		err += At24cxx_Write_Dword(Device_Addr,LinV_ADDR+8+12*i,(uint32_t*)(&x->LinV[i].CorrectionFactor));
	}
	printf("Device_Addr:0x%2x WRerr:%d\r\n",Device_Addr,err);
	return err;
}


/**
 * @brief		将传感器所有数据从传感器接头上的At24c256读出
 * @note		其中适用传感器包括：位置传感器，力传感器，引伸计
 * @param		Device_Addr——设备地址
 * @retval		err——写入数据出错数量
*/
uint8_t EepromSensor_ReadAll(uint16_t Device_Addr,sensorData_t* x)
{
	uint8_t err=0,check_num=0,i=0;
	err += At24cxx_Read_Byte (Device_Addr|1,CHEAK_ADDR, &check_num);
	if( check_num != EEPROM_CHECK){
		EepromSensor_WriteAll(Device_Addr,x);
	}
	err += At24cxx_Read_Byte (Device_Addr|1,SensorType_ADDR, (&x->Sensortype));
	err += At24cxx_Read_Byte (Device_Addr|1,Sign_ADDR, (&x->Sign));
	err += At24cxx_Read_Dword(Device_Addr|1,NominalValue_ADDR, (uint32_t*)(&x->NominalValue));
	err += At24cxx_Read_Dword(Device_Addr|1,NominalSensitive_ADDR, (uint32_t*)(&x->NominalSensitive));
	err += At24cxx_Read_Byte (Device_Addr|1,Day_ADDR, (&x->Day));
	err += At24cxx_Read_Byte (Device_Addr|1,Month_ADDR, (&x->Month));
	err += At24cxx_Read_Word (Device_Addr|1,Year_ADDR, (&x->Year));
	err += At24cxx_Read_Byte (Device_Addr|1,LinPoint_ADDR, (&x->LinPoint));

	for(i=0;i<SEN_LIN_DATA_MAX;i++)
	{
		err += At24cxx_Read_Dword(Device_Addr|1,LinV_ADDR+12*i, (uint32_t*)(&x->LinV[i].ADCCode));
		err += At24cxx_Read_Dword(Device_Addr|1,LinV_ADDR+4+12*i, (uint32_t*)(&x->LinV[i].RefValue));
		err += At24cxx_Read_Dword(Device_Addr|1,LinV_ADDR+8+12*i,(uint32_t*)(&x->LinV[i].CorrectionFactor));
	}
	printf("Device_Addr:0x%2x RDerr:%d\r\n",Device_Addr,err);
	return err;
}

/**
 * @brief		将大变形传感器所有数据写入传感器接头上的At24c256
 * @note		其中适用大变形传感器
 * @param		Device_Addr——设备地址
 * @retval		err——写入数据出错数量
*/
uint8_t EepromSensorBigDeformation_WriteAll(uint16_t Device_Addr,SENSORBIGDEFORMATIONDATA* x)
{
	uint8_t err=0,check_num=EEPROM_CHECK;
	err += At24cxx_Write_Byte (Device_Addr,SensorBdSensortype_ADDR, &check_num);
	err += At24cxx_Write_Byte (Device_Addr,SensorBdSign_ADDR, (&x->Sign));
	err += At24cxx_Write_Dword(Device_Addr,SensorBdNominalValue_ADDR,(uint32_t*)(&x->NominalValue));
	err += At24cxx_Write_Byte (Device_Addr,SensorBdCollectType_ADDR, (&x->CollectType));
	err += At24cxx_Write_Dword(Device_Addr,SensorBdNominalSensitive1_ADDR, (uint32_t*)(&x->NominalSensitive1));
	err += At24cxx_Write_Dword (Device_Addr,SensorBdNominalSensitive2_ADDR, (uint32_t*)(&x->NominalSensitive2));
	err += At24cxx_Write_Byte (Device_Addr,SensorBdDay_ADDR, (&x->Day));
	err += At24cxx_Write_Byte (Device_Addr,SensorBdMonth_ADDR,(&x->Month));
	err += At24cxx_Write_Word (Device_Addr,SensorBdYear_ADDR, (&x->Year));

	printf("EepromSensorBigDeformation_WriteAll\r\n");
	printf("Device_Addr:0x%2x WRerr:%d\r\n",Device_Addr,err);
	return err;
}
/**
 * @brief		将大变形传感器所有数据从传感器接头上的At24c256读出
 * @note		其中适用大变形传感器
 * @param		Device_Addr——设备地址
 * @retval		err——写入数据出错数量
*/
uint8_t EepromSensorBigDeformation_ReadAll(uint16_t Device_Addr,SENSORBIGDEFORMATIONDATA* x)
{
	uint8_t err=0,check_num=EEPROM_CHECK;
	err += At24cxx_Read_Byte (Device_Addr|1,SensorBdSensortype_ADDR, &check_num);
	err += At24cxx_Read_Byte (Device_Addr|1,SensorBdSign_ADDR, (&x->Sign));
	err += At24cxx_Read_Dword(Device_Addr|1,SensorBdNominalValue_ADDR,(uint32_t*)(&x->NominalValue));
	err += At24cxx_Read_Byte (Device_Addr|1,SensorBdCollectType_ADDR, (&x->CollectType));
	err += At24cxx_Read_Dword(Device_Addr|1,SensorBdNominalSensitive1_ADDR, (uint32_t*)(&x->NominalSensitive1));
	err += At24cxx_Read_Dword(Device_Addr|1,SensorBdNominalSensitive2_ADDR, (uint32_t*)(&x->NominalSensitive2));
	err += At24cxx_Read_Byte (Device_Addr|1,SensorBdDay_ADDR, (&x->Day));
	err += At24cxx_Read_Byte (Device_Addr|1,SensorBdMonth_ADDR,(&x->Month));
	err += At24cxx_Read_Word (Device_Addr|1,SensorBdYear_ADDR, (&x->Year));

	printf("EepromSensorBigDeformation_ReadAll\r\n");
	printf("Device_Addr:0x%2x WRerr:%d\r\n",Device_Addr,err);
	return err;
}

/**
 * @brief		将输出参数所有数据写入控制器上的At24c256
 * @param		x——输出参数结构体首地址
 * @retval		err——写入数据出错数量
*/
uint8_t EepromOutputPara_WriteAll(OUTPUTPARA* x)
{
	uint8_t err=0,check_num=EEPROM_CHECK;
	err += At24cxx_Write_Byte (M24C02_Controller,CHEAK_ADDR, &check_num);
	err += At24cxx_Write_Byte (M24C02_Controller,OUTPUTPARA_SIGNAL_ADDR, (&x->Signal));
	err += At24cxx_Write_Byte (M24C02_Controller,OUTPUTPARA_SIGN_ADDR, (&x->Sign));
	err += At24cxx_Write_Dword(M24C02_Controller,OUTPUTPARA_MAXVALUE_ADDR, (uint32_t*)(&x->MaxValue));
	err += At24cxx_Write_Dword(M24C02_Controller,OUTPUTPARA_MINVALUE_ADDR, (uint32_t*)(&x->MinValue));
	err += At24cxx_Write_Dword(M24C02_Controller,OUTPUTPARA_INITVALUE_ADDR, (uint32_t*)(&x->InitValue));
	err += At24cxx_Write_Dword(M24C02_Controller,OUTPUTPARA_OFFSET_ADDR, (uint32_t*)(&x->Offset));
	err += At24cxx_Write_Dword(M24C02_Controller,OUTPUTPARA_SIGNALFREQUENCY_ADDR, (uint32_t*)(&x->SignalFrequency));
	err += At24cxx_Write_Dword(M24C02_Controller,OUTPUTPARA_MAXVOLTAGE_ADDR, (uint32_t*)(&x->MaxVoltage));
	err += At24cxx_Write_Dword(M24C02_Controller,OUTPUTPARA_MAXCURRENT_ADDR, (uint32_t*)(&x->MaxCurrent));

	printf("EepromOutputPara_WriteAll\r\n");
	printf("M24C02_Controller:0x%2x WRerr:%d\r\n",M24C02_Controller,err);
	return err;
}

/**
 * @brief		将输出参数所有数据从控制器上的At24c256读出
 * @param		x——输出参数结构体首地址
 * @retval		err——读取数据出错数量
*/
uint8_t EepromOutputPara_ReadAll(OUTPUTPARA* x)
{
	uint8_t err=0,check_num=EEPROM_CHECK;
	err += At24cxx_Read_Byte (M24C02_Controller|1,CHEAK_ADDR, &check_num);
	err += At24cxx_Read_Byte (M24C02_Controller|1,OUTPUTPARA_SIGNAL_ADDR, (&x->Signal));
	err += At24cxx_Read_Byte (M24C02_Controller|1,OUTPUTPARA_SIGN_ADDR, (&x->Sign));
	err += At24cxx_Read_Dword(M24C02_Controller|1,OUTPUTPARA_MAXVALUE_ADDR, (uint32_t*)(&x->MaxValue));
	err += At24cxx_Read_Dword(M24C02_Controller|1,OUTPUTPARA_MINVALUE_ADDR, (uint32_t*)(&x->MinValue));
	err += At24cxx_Read_Dword(M24C02_Controller|1,OUTPUTPARA_INITVALUE_ADDR, (uint32_t*)(&x->InitValue));
	err += At24cxx_Read_Dword(M24C02_Controller|1,OUTPUTPARA_OFFSET_ADDR, (uint32_t*)(&x->Offset));
	err += At24cxx_Read_Dword(M24C02_Controller|1,OUTPUTPARA_SIGNALFREQUENCY_ADDR, (uint32_t*)(&x->SignalFrequency));
	err += At24cxx_Read_Dword(M24C02_Controller|1,OUTPUTPARA_MAXVOLTAGE_ADDR, (uint32_t*)(&x->MaxVoltage));
	err += At24cxx_Read_Dword(M24C02_Controller|1,OUTPUTPARA_MAXCURRENT_ADDR, (uint32_t*)(&x->MaxCurrent));
	
	printf("EepromOutputPara_ReadAll\r\n");
	printf("M24C02_Controller:0x%2x RDerr:%d\r\n",M24C02_Controller,err);
	return err;
}

/**
 * @brief		将系统参数所有数据写入控制器上的At24c256
 * @param		x——输出参数结构体首地址
 * @retval		err——写入数据出错数量
*/
uint8_t EepromSysPara_WriteAll(SYSPARA* x)
{
	uint8_t err=0,check_num=EEPROM_CHECK;
	err += At24cxx_Write_Byte (M24C02_Controller,CHEAK_ADDR, &check_num);
	err += At24cxx_Write_Word  (M24C02_Controller,SYSPARA_TRANSMITDATAPERIOD_ADDR, (&x->TransmitDataPeriod));
	err += At24cxx_Write_Byte (M24C02_Controller,SYSPARA_CONTROLLERSTRUCTURE_ADDR, (&x->ControllerStructure));

	printf("EepromSysPara_WriteAll\r\n");
	printf("M24C02_Controller:0x%2x WRerr:%d\r\n",M24C02_Controller,err);
	return err;
}

/**
 * @brief		将系统参数所有数据从控制器上的At24c256读出
 * @param		x——输出参数结构体首地址
 * @retval		err——读取数据出错数量
*/
uint8_t EepromSysPara_ReadAll(SYSPARA* x)
{
	uint8_t err=0,check_num=EEPROM_CHECK;
	err += At24cxx_Read_Byte (M24C02_Controller|1,CHEAK_ADDR, &check_num);
	err += At24cxx_Read_Word (M24C02_Controller|1,SYSPARA_TRANSMITDATAPERIOD_ADDR, (&x->TransmitDataPeriod));
	err += At24cxx_Read_Byte (M24C02_Controller|1,SYSPARA_CONTROLLERSTRUCTURE_ADDR, (&x->ControllerStructure));
	
	printf("EepromSysPara_ReadAll\r\n");
	printf("M24C02_Controller:0x%2x RDerr:%d\r\n",M24C02_Controller,err);
	return err;
}

/**
 * @brief		将拆囊起参数所有数据从每个传感器接头上的At24c256读出
 * @param		null
 * @retval		err——读取数据出错数量
*/
uint8_t eeprom_sendata_init_read(void)
{	
	uint8_t err=0,rc=0;
	
	rc = At24cxx_Check(M24C02_Controller,&SenData[ch0Pose],Dev_Sensor);
	if(rc == 0)//data have been in eeprom
	{
		printf("EepromSensor_ReadAll ch[0] have data read out\r\n");
		EepromSensor_ReadAll(M24C02_Controller,&SenData[ch0Pose]);
	}
	else//data have not been in eeprom, meanwhile writing init data alse faild
	{
		err +=rc;
		printf("EepromSensor_ReadAll ch[0] error! Please check whether the connector has been insert!\r\n");
	}
	
	rc = At24cxx_Check(M24C02_Load,&SenData[ch4Load],Dev_Sensor);
	if(rc == 0)//data have been in eeprom
	{
		printf("EepromSensor_ReadAll ch[4] have data read out\r\n");
		EepromSensor_ReadAll(M24C02_Load,&SenData[ch4Load]);
		printf("SenData[ch4Load].NominalValue:%f\n SenData[ch4Load].NominalSensitive:%f\n",SenData[ch4Load].NominalValue,SenData[ch4Load].NominalSensitive);
	}
	else//data have not been in eeprom, meanwhile writing init data alse faild
	{
		err +=rc;
		printf("EepromSensor_ReadAll ch[4] error! Please check whether the connector has been insert!\r\n");
	}
	
	return err;
}

/**
 * @brief		sensor code value save
 * @param		null
 * @retval		err——number of error
*/
uint8_t eeprom_sensor_code_write(void)
{	
	uint8_t err=0,check_num=EEPROM_CHECK;
	err += At24cxx_Read_Byte (M24C02_Controller,CHEAK_ADDR, &check_num);
	err += At24cxx_Write_Dword (M24C02_Controller,POSE_ENCODER_CODE_ADDR, (uint32_t*)(&pose.code));//pose code
	err += At24cxx_Write_Dword (M24C02_Controller,LOAD_TARE_CODE_ADDR, (uint32_t*)(&AL.tare.fValue[ch4Load]));//load tare ch4Load
	err += At24cxx_Write_Dword (M24C02_Controller,EXTEN_TARE_CODE_ADDR, (uint32_t*)(&AL.tare.fValue[ch2Ext1]));//ext tare ch2Ext1

	//printf("eeprom_sensor_code_write\r\n");
	//printf("M24C02_Controller:0x%2x WRerr:%d\r\n",M24C02_Controller,err);
	return err;
}

/**
 * @brief		sensor code value read
 * @param		null
 * @retval		err——number of error
*/
uint8_t eeprom_sensor_code_read(void)
{	
	uint8_t err=0,check_num=EEPROM_CHECK;
	err += At24cxx_Read_Byte (M24C02_Controller,CHEAK_ADDR, &check_num);
	err += At24cxx_Read_Dword (M24C02_Controller|1,POSE_ENCODER_CODE_ADDR, (uint32_t*)(&pose.code));//pose code
	err += At24cxx_Read_Dword (M24C02_Controller|1,LOAD_TARE_CODE_ADDR, (uint32_t*)(&AL.tare.fValue[ch4Load]));//load tare ch4Load
	err += At24cxx_Read_Dword (M24C02_Controller|1,EXTEN_TARE_CODE_ADDR, (uint32_t*)(&AL.tare.fValue[ch2Ext1]));//ext tare ch2Ext1

	printf("eeprom_sensor_code_read\r\n");
	printf("M24C02_Controller:0x%2x RDerr:%d\r\n",M24C02_Controller,err);
	return err;
}

/**
 * @brief		According to the semaphore, write relevant data to eeprom
 * @param[in] semaphore mram storage param semaphore
 * @retval		err——number of data write errors
 * @date		2024-3-5 15:12:29
 * @author		Pan
*/
void eeprom_write_monitor(STORAGE_PARAM_MYSEMAPHORE* semaphore)
{
	uint8_t rc=0;
	//sensor code and tare store updated
	// if(semaphore->write_sensorcode == 1)
	// {
	// rc = eeprom_sensor_code_write();
	// 	if(rc != 0)
	// 	{
	// 		printf("Eepromsensor_code_write error:%d\r\n",rc);
	// 	} 
	// semaphore->write_sensorcode = 0;
	// }
	//DefaultAcc Pos store updated
	// if(semaphore->write_defalutacc_pos == 1)
	// {
	// rc = EepromDefaultAcc_write(M24C02_Controller,&posAllCtrlPara,POS_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromDefaultAcc_write pos error:%d\r\n",rc);
	// 	} 
	// semaphore->write_defalutacc_pos = 0;
	// }
	// //DefaultAcc Load store updated
	// if(semaphore->write_defalutacc_load == 1)
	// {
	// rc = EepromDefaultAcc_write(M24C02_Controller,&loadAllCtrlPara,LOAD_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromDefaultAcc_write load error:%d\r\n",rc);
	// 	} 
	// semaphore->write_defalutacc_load = 0;
	// }
	// //DefaultAcc Ext store updated
	// if(semaphore->write_defalutacc_ext == 1)
	// {
	// rc = EepromDefaultAcc_write(M24C02_Controller,&extAllCtrlPara,EXTEN_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromDefaultAcc_write ext error:%d\r\n",rc);
	// 	} 
	// semaphore->write_defalutacc_ext = 0;
	// }
	// //Speed Pos store updated
	// if(semaphore->write_speed_pos == 1)
	// {
	// semaphore->write_speed_pos = 0;
	// rc = EepromSpeed_write(M24C02_Controller,&posAllCtrlPara,POS_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromSpeed_write pos error:%d\r\n",rc);
	// 	} 
	// }
	// //Speed Load store updated
	// if(semaphore->write_speed_load == 1)
	// {
	// semaphore->write_speed_load = 0;
	// rc = EepromSpeed_write(M24C02_Controller,&loadAllCtrlPara,LOAD_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromSpeed_write load error:%d\r\n",rc);
	// 	} 
	// }
	// //Speed Load store updated
	// if(semaphore->write_speed_ext == 1)
	// {
	// semaphore->write_speed_ext = 0;
	// rc = EepromSpeed_write(M24C02_Controller,&extAllCtrlPara,EXTEN_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromSpeed_write ext error:%d\r\n",rc);
	// 	} 
	// }
	// //Nominal Pos store updated
	// if(semaphore->write_nominal_pos == 1)
	// {
	// semaphore->write_nominal_pos = 0;
	// rc = EepromNominal_write(M24C02_Controller,&(posAllCtrlPara.Nominal),POS_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromSpeed_write pos error:%d\r\n",rc);
	// 	} 
	// }
	// //Nominal Load store updated
	// if(semaphore->write_nominal_load == 1)
	// {
	// semaphore->write_nominal_load = 0;
	// rc = EepromNominal_write(M24C02_Controller,&(loadAllCtrlPara.Nominal),LOAD_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromSpeed_write load error:%d\r\n",rc);
	// 	} 
	// }
	// //Nominal Ext store updated
	// if(semaphore->write_nominal_ext == 1)
	// {
	// semaphore->write_nominal_ext = 0;
	// rc = EepromNominal_write(M24C02_Controller,&(extAllCtrlPara.Nominal),EXTEN_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromSpeed_write ext error:%d\r\n",rc);
	// 	} 
	// }
	// //transmit data period store updated
	// if(semaphore->write_transmitdataperiod == 1)
	// {
	// semaphore->write_transmitdataperiod = 0;
	// rc = EepromTansmitDataPeriod_write(M24C02_Controller,&SysPara);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromTansmitDataPeriod_write error:%d\r\n",rc);
	// 	} 
	// }
	// //Wnd size and time pos store updated
	// if(semaphore->write_wnd_pos == 1)
	// {
	// semaphore->write_wnd_pos = 0;
	// rc = EepromWnd_write(M24C02_Controller,&(posAllCtrlPara.Wnd),POS_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromWnd_write pos error:%d\r\n",rc);
	// 	} 
	// }
	// //Wnd size and time load store updated
	// if(semaphore->write_wnd_load == 1)
	// {
	// semaphore->write_wnd_load = 0;
	// rc = EepromWnd_write(M24C02_Controller,&(loadAllCtrlPara.Wnd),LOAD_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromWnd_write load error:%d\r\n",rc);
	// 	} 
	// }
	// //Wnd size and time ext store updated
	// if(semaphore->write_wnd_ext == 1)
	// {
	// semaphore->write_wnd_ext = 0;
	// rc = EepromWnd_write(M24C02_Controller,&(extAllCtrlPara.Wnd),EXTEN_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromWnd_write ext error:%d\r\n",rc);
	// 	} 
	// }
	// //Sft pos store updated
	// if(semaphore->write_sft_pos == 1)
	// {
	// semaphore->write_sft_pos = 0;
	// rc = EepromSft_write(M24C02_Controller,&(posAllCtrlPara.Sft),POS_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromSft_write pos error:%d\r\n",rc);
	// 	} 
	// }
	// //Sft load store updated
	// if(semaphore->write_sft_load == 1)
	// {
	// semaphore->write_sft_load = 0;
	// rc = EepromSft_write(M24C02_Controller,&(loadAllCtrlPara.Sft),LOAD_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromSft_write load error:%d\r\n",rc);
	// 	} 
	// }
	// //Sft ext store updated
	// if(semaphore->write_sft_ext == 1)
	// {
	// semaphore->write_sft_ext = 0;
	// rc = EepromSft_write(M24C02_Controller,&(extAllCtrlPara.Sft),EXTEN_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromSft_write ext error:%d\r\n",rc);
	// 	} 
	// }
	// //PosPID pos store updated
	// if(semaphore->write_pospid_pos == 1)
	// {
	// semaphore->write_pospid_pos = 0;
	// rc = EepromPosPID_write(M24C02_Controller,&(posAllCtrlPara.PosPid),POS_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromPosPID_write pos error:%d\r\n",rc);
	// 	} 
	// }
	// //PosPID load store updated
	// if(semaphore->write_pospid_load == 1)
	// {
	// semaphore->write_pospid_load = 0;
	// rc = EepromPosPID_write(M24C02_Controller,&(loadAllCtrlPara.PosPid),LOAD_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromPosPID_write load error:%d\r\n",rc);
	// 	} 
	// }
	// //PosPID ext store updated
	// if(semaphore->write_pospid_ext == 1)
	// {
	// semaphore->write_pospid_ext = 0;
	// rc = EepromPosPID_write(M24C02_Controller,&(extAllCtrlPara.PosPid),EXTEN_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromPosPID_write ext error:%d\r\n",rc);
	// 	} 
	// }
	// //SpeedPID pos store updated
	// if(semaphore->write_speedpid_pos == 1)
	// {
	// semaphore->write_speedpid_pos = 0;
	// rc = EepromSpeedPID_write(M24C02_Controller,&(posAllCtrlPara.SpeedPid),POS_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromSpeedPID_write pos error:%d\r\n",rc);
	// 	} 
	// }
	// //SpeedPID load store updated
	// if(semaphore->write_speedpid_load == 1)
	// {
	// semaphore->write_speedpid_load = 0;
	// rc = EepromSpeedPID_write(M24C02_Controller,&(loadAllCtrlPara.SpeedPid),LOAD_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromSpeedPID_write load error:%d\r\n",rc);
	// 	} 
	// }
	// //SpeedPID ext store updated
	// if(semaphore->write_speedpid_ext == 1)
	// {
	// semaphore->write_speedpid_ext = 0;
	// rc = EepromSpeedPID_write(M24C02_Controller,&(extAllCtrlPara.SpeedPid),EXTEN_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromSpeedPID_write ext error:%d\r\n",rc);
	// 	} 
	// }
	// //Ffd pos store updated
	// if(semaphore->write_ffd_pos == 1)
	// {
	// semaphore->write_ffd_pos = 0;
	// rc = EepromFfd_write(M24C02_Controller,&(posAllCtrlPara.Ffd),POS_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromFfd_write pos error:%d\r\n",rc);
	// 	} 
	// }
	// //Ffd load store updated
	// if(semaphore->write_ffd_load == 1)
	// {
	// semaphore->write_ffd_load = 0;
	// rc = EepromFfd_write(M24C02_Controller,&(loadAllCtrlPara.Ffd),LOAD_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromFfd_write load error:%d\r\n",rc);
	// 	} 
	// }
	// //Ffd ext store updated
	// if(semaphore->write_ffd_ext == 1)
	// {
	// semaphore->write_ffd_ext = 0;
	// rc = EepromFfd_write(M24C02_Controller,&(extAllCtrlPara.Ffd),EXTEN_MODE);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromFfd_write ext error:%d\r\n",rc);
	// 	} 
	// }
	// //system parameter store updated
	// if(semaphore->write_syspara == 1)
	// {
	// semaphore->write_syspara = 0;
	// rc = EepromSysPara_WriteAll(&SysPara);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromSysPara_WriteAll error:%d\r\n",rc);
	// 	} 
	// }
	// //output parameter store updated
	// if(semaphore->write_outputpara == 1)
	// {
	// semaphore->write_outputpara = 0;
	// rc = EepromOutputPara_WriteAll(&OutputPara);
	// 	if(rc != 0)
	// 	{
	// 		printf("EepromOutputPara_WriteAll error:%d\r\n",rc);
	// 	}
		
	// }
	//sendata parameter store updated
	if(semaphore->write_sendataOther == 1)
	{
		semaphore->write_sendataOther = 0;
		switch(sensorConnector)
		{
			// case ch0Pose://pose
			// 	rc = EepromSensor_WriteAll(M24C02_Controller,&SenData[ch0Pose]);
			// 		if(rc != 0)
			// 		{
			// 			printf("EepromSensor_WriteAll ch0Pose error:%d\r\n",rc);
			// 		}
			// break;
			// case ch1Bd://bigdeformation

			// break;
			case ch2Ext1://extensometer
				rc = EepromSensor_WriteAll(M24C02_EXT1,&SenData[ch2Ext1]);
					if(rc != 0)
					{
						printf("EepromSensor_WriteAll ext1 error:%d\r\n",rc);
					}
			break;
			case ch3Ext2://extend
				rc = EepromSensor_WriteAll(M24C02_EXT2,&SenData[ch3Ext2]);
					if(rc != 0)
					{
						printf("EepromSensor_WriteAll ext2 error:%d\r\n",rc);
					}
			break;
			case ch4Load://load
				rc = EepromSensor_WriteAll(M24C02_Load,&SenData[ch4Load]);
					if(rc != 0)
					{
						printf("EepromSensor_WriteAll load error:%d\r\n",rc);
					}
			break;
		}
	}
	if(semaphore->readSensorOnceExt1  == 1){
		semaphore->readSensorOnceExt1 = 0;
		rc = EepromSensor_ReadAll(EXTERN_SENSER1,&SenData[ch2Ext1]);
		if(rc != 0)
		{
			printf("EepromSensor_ReadAll ext1 error:%d\r\n",rc);
		}
		else
			senDateSync(SenData,&AL);
	}
	if(semaphore->readSensorOnceExt2  == 1){
		semaphore->readSensorOnceExt2 = 0;
		rc = EepromSensor_ReadAll(EXTERN_SENSER2,&SenData[ch3Ext2]);
		if(rc != 0)
		{
			printf("EepromSensor_ReadAll ext2 error:%d\r\n",rc);
		}
		else
			senDateSync(SenData,&AL);
	}
	if(semaphore->readSensorOnceLoad  == 1){
		semaphore->readSensorOnceLoad = 0;
		rc = EepromSensor_ReadAll(LOAD_SENSER,&SenData[ch4Load]);
		log_i("EepromSensor_ReadAll rc:%d",rc);
		if(rc != 0)
		{
			printf("EepromSensor_ReadAll load error:%d\r\n",rc);
		}
		else
			senDateSync(SenData,&AL);
	}

}

/**
 * @brief		when system init,read sensor data that has connected
 * @note		include extensometer1,extensometer2，load
 * @param[in]	x	type:SENSER_CHECK_STRUCT
 * @retval		err——number of worry read
*/
void eepromSensorReadInit(SENSER_CHECK_STRUCT* x)
{	
	uint8_t rc = 0;
	if(x->Register[0] == 1){//load
		rc = EepromSensor_ReadAll(x->Addr[0],&SenData[ch4Load]);
		if(rc != 0){
			x->Register[0] = 0;
		}
	}
	if(x->Register[1] == 1){//ext1
		rc = EepromSensor_ReadAll(x->Addr[1],&SenData[ch2Ext1]);
		if(rc != 0){
			x->Register[1] = 0;
		}
	}
	if(x->Register[2] == 1){//ext2
		rc = EepromSensor_ReadAll(x->Addr[2],&SenData[ch3Ext2]);
		if(rc != 0){
			x->Register[2] = 0;
		}
	}

}


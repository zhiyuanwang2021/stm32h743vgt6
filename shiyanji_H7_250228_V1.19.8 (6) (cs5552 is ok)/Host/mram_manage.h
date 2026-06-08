#ifndef _MRAM_MANAGE_H_
#define _MRAM_MANAGE_H_

#include "MRAM.h"
#include "gParameter.h"
#include "in_out.h"
#define MRAM_CHECK 0x55
//Eeprom On Controller
#define CHEAK_ADDR 32767	//check num address

#define distance_scale_ADDR 	0// 4Byte distance_scale
#define PosDefaultAcc_ADDR 		(distance_scale_ADDR+4)//4Byte DefaultPosAcc
#define PosDefaultDec_ADDR 		(PosDefaultAcc_ADDR+4)//4Byte DefaultPosDec
#define LoadDefaultAcc_ADDR 	(PosDefaultDec_ADDR+4)//4Byte DefaultLoadAcc
#define LoadDefaultDec_ADDR 	(LoadDefaultAcc_ADDR+4)//4Byte DefaultLoadDec
#define ExtDefaultAcc_ADDR 		(LoadDefaultDec_ADDR+4)//4Byte DefaultExtAcc
#define ExtDefaultDec_ADDR 		(ExtDefaultAcc_ADDR+4)//4Byte DefaultExtDec
#define upperlimit_phy_ADDR 	(ExtDefaultDec_ADDR+4)//4Byte upperlimit_phy
#define lowerlimit_phy_ADDR 	(upperlimit_phy_ADDR+4)//4Byte lowerlimit_phy
#define defaultspeed_ADDR 		(lowerlimit_phy_ADDR+4)//4Byte defaultspeed
#define defaultspeedlimit_ADDR 	(defaultspeed_ADDR+4)//4Byte defaultspeedlimit
#define DataTransmitPeriod_ADDR (defaultspeedlimit_ADDR+2)//2Byte DataTransmitPeriod


//All CTRL PARAM

//POS all closeloop control parameter 115Byte  400-522  reserve space:400-599
#define 	Posallctrlpara_PosPid_P_ADDR				400 									//4Byte float
#define 	Posallctrlpara_PosPid_I_ADDR				(Posallctrlpara_PosPid_P_ADDR + 4)		//4Byte float
#define 	Posallctrlpara_PosPid_D_ADDR				(Posallctrlpara_PosPid_I_ADDR + 4)		//4Byte float
#define 	Posallctrlpara_PosPid_Hp_P_ADDR				(Posallctrlpara_PosPid_D_ADDR + 4) 		//4Byte float
#define 	Posallctrlpara_PosPid_Hp_I_ADDR				(Posallctrlpara_PosPid_Hp_P_ADDR + 4) 	//4Byte float
#define 	Posallctrlpara_PosPid_Hp_D_ADDR				(Posallctrlpara_PosPid_Hp_I_ADDR + 4) 	//4Byte float
#define 	Posallctrlpara_SpeedPid_P_ADDR				(Posallctrlpara_PosPid_Hp_D_ADDR + 4) 	//4Byte float
#define 	Posallctrlpara_SpeedPid_I_ADDR				(Posallctrlpara_SpeedPid_P_ADDR + 4) 	//4Byte float
#define 	Posallctrlpara_SpeedPid_D_ADDR				(Posallctrlpara_SpeedPid_I_ADDR + 4) 	//4Byte float
#define 	Posallctrlpara_Ffd_SpeedFFP_ADDR			(Posallctrlpara_SpeedPid_D_ADDR + 4) 	//4Byte float
#define 	Posallctrlpara_Ffd_PosDelay_ADDR			(Posallctrlpara_Ffd_SpeedFFP_ADDR + 4) 	//4Byte uint32_t
#define 	Posallctrlpara_Ffd_Hp_SpeedFFP_ADDR			(Posallctrlpara_Ffd_PosDelay_ADDR + 4) 	//4Byte float
#define 	Posallctrlpara_Ffd_Hp_PosDelay_ADDR			(Posallctrlpara_Ffd_Hp_SpeedFFP_ADDR + 4) 	//4Byte uint32_t
#define 	Posallctrlpara_DefaultAcc_ADDR				(Posallctrlpara_Ffd_Hp_PosDelay_ADDR + 4) 	//4Byte float
#define 	Posallctrlpara_Speed_ADDR					(Posallctrlpara_DefaultAcc_ADDR + 4) 	//4Byte float
#define 	Posallctrlpara_Deviation_ADDR				(Posallctrlpara_Speed_ADDR + 4) 		//4Byte float
#define 	Posallctrlpara_DevReaction_ADDR				(Posallctrlpara_Deviation_ADDR + 4) 	//1Byte uint8_t
#define 	Posallctrlpara_Wnd_WndSize_ADDR				(Posallctrlpara_DevReaction_ADDR + 1) 	//4Byte float
#define 	Posallctrlpara_Wnd_WndTime_ADDR				(Posallctrlpara_Wnd_WndSize_ADDR + 4) 	//4Byte uint32_t
#define 	Posallctrlpara_Sft_UpperSft_ADDR			(Posallctrlpara_Wnd_WndTime_ADDR + 4) 	//4Byte float
#define 	Posallctrlpara_Sft_LowerStf_ADDR			(Posallctrlpara_Sft_UpperSft_ADDR + 4) 	//4Byte float
#define 	Posallctrlpara_Sft_Reaction_ADDR			(Posallctrlpara_Sft_LowerStf_ADDR + 4) 	//1Byte uint8_t
#define 	Posallctrlpara_MinAcceleration_ADDR			(Posallctrlpara_Sft_Reaction_ADDR + 1) 	//4Byte float
#define 	Posallctrlpara_MaxAcceleration_ADDR			(Posallctrlpara_MinAcceleration_ADDR + 4) 	//4Byte float
#define 	Posallctrlpara_MinDeceleration_ADDR			(Posallctrlpara_MaxAcceleration_ADDR + 4) 	//4Byte float
#define 	Posallctrlpara_MaxDeceleration_ADDR			(Posallctrlpara_MinDeceleration_ADDR + 4) 	//4Byte float
#define 	Posallctrlpara_MinSpeed_ADDR				(Posallctrlpara_MaxDeceleration_ADDR + 4) 	//4Byte float
#define 	Posallctrlpara_MaxSpeed_ADDR				(Posallctrlpara_MinSpeed_ADDR + 4) 		//4Byte float
#define 	Posallctrlpara_Deadband_Deadband_ADDR		(Posallctrlpara_MaxSpeed_ADDR + 4) 		//4Byte float
#define 	Posallctrlpara_Deadband_PercentP_ADDR		(Posallctrlpara_Deadband_Deadband_ADDR + 4) 	//1Byte uint8_t
#define 	Posallctrlpara_Dither_DitherAmplitude_ADDR	(Posallctrlpara_Deadband_PercentP_ADDR + 1) 	//4Byte float
#define 	Posallctrlpara_Dither_DitherFrequency_ADDR	(Posallctrlpara_Dither_DitherAmplitude_ADDR + 4)//4Byte float
#define 	Posallctrlpara_NominalAcc_ADDR	            (Posallctrlpara_Dither_DitherFrequency_ADDR + 4) 	//4Byte float
#define 	Posallctrlpara_NominalSpeed_ADDR	        (Posallctrlpara_NominalAcc_ADDR + 4)//4Byte float

//LOAD_MODE all closeloop control parameter 115Byte  600-722  reserve space:600-799
#define 	Loadallctrlpara_PosPid_P_ADDR				600 									//4Byte float
#define 	Loadallctrlpara_PosPid_I_ADDR				(Loadallctrlpara_PosPid_P_ADDR + 4)		//4Byte float
#define 	Loadallctrlpara_PosPid_D_ADDR				(Loadallctrlpara_PosPid_I_ADDR + 4) 		//4Byte float
#define 	Loadallctrlpara_PosPid_Hp_P_ADDR			(Loadallctrlpara_PosPid_D_ADDR + 4) 		//4Byte float
#define 	Loadallctrlpara_PosPid_Hp_I_ADDR			(Loadallctrlpara_PosPid_Hp_P_ADDR + 4) 	//4Byte float
#define 	Loadallctrlpara_PosPid_Hp_D_ADDR			(Loadallctrlpara_PosPid_Hp_I_ADDR + 4) 	//4Byte float
#define 	Loadallctrlpara_SpeedPid_P_ADDR				(Loadallctrlpara_PosPid_Hp_D_ADDR + 4) 	//4Byte float
#define 	Loadallctrlpara_SpeedPid_I_ADDR				(Loadallctrlpara_SpeedPid_P_ADDR + 4) 	//4Byte float
#define 	Loadallctrlpara_SpeedPid_D_ADDR				(Loadallctrlpara_SpeedPid_I_ADDR + 4) 	//4Byte float
#define 	Loadallctrlpara_Ffd_SpeedFFP_ADDR			(Loadallctrlpara_SpeedPid_D_ADDR + 4) 	//4Byte float
#define 	Loadallctrlpara_Ffd_PosDelay_ADDR			(Loadallctrlpara_Ffd_SpeedFFP_ADDR + 4) 	//4Byte uint32_t
#define 	Loadallctrlpara_Ffd_Hp_SpeedFFP_ADDR		(Loadallctrlpara_Ffd_PosDelay_ADDR + 4) 	//4Byte float
#define 	Loadallctrlpara_Ffd_Hp_PosDelay_ADDR		(Loadallctrlpara_Ffd_Hp_SpeedFFP_ADDR + 4) 	//4Byte uint32_t
#define 	Loadallctrlpara_DefaultAcc_ADDR				(Loadallctrlpara_Ffd_Hp_PosDelay_ADDR + 4) 	//4Byte float
#define 	Loadallctrlpara_Speed_ADDR					(Loadallctrlpara_DefaultAcc_ADDR + 4) 	//4Byte float
#define 	Loadallctrlpara_Deviation_ADDR				(Loadallctrlpara_Speed_ADDR + 4) 		//4Byte float
#define 	Loadallctrlpara_DevReaction_ADDR			(Loadallctrlpara_Deviation_ADDR + 4) 	//1Byte uint8_t
#define 	Loadallctrlpara_Wnd_WndSize_ADDR			(Loadallctrlpara_DevReaction_ADDR + 1) 	//4Byte float
#define 	Loadallctrlpara_Wnd_WndTime_ADDR			(Loadallctrlpara_Wnd_WndSize_ADDR + 4) 	//4Byte uint32_t
#define 	Loadallctrlpara_Sft_UpperSft_ADDR			(Loadallctrlpara_Wnd_WndTime_ADDR + 4) 	//4Byte float
#define 	Loadallctrlpara_Sft_LowerStf_ADDR			(Loadallctrlpara_Sft_UpperSft_ADDR + 4) 	//4Byte float
#define 	Loadallctrlpara_Sft_Reaction_ADDR			(Loadallctrlpara_Sft_LowerStf_ADDR + 4) 	//1Byte uint8_t
#define 	Loadallctrlpara_MinAcceleration_ADDR		(Loadallctrlpara_Sft_Reaction_ADDR + 1) 	//4Byte float
#define 	Loadallctrlpara_MaxAcceleration_ADDR		(Loadallctrlpara_MinAcceleration_ADDR + 4) 	//4Byte float
#define 	Loadallctrlpara_MinDeceleration_ADDR		(Loadallctrlpara_MaxAcceleration_ADDR + 4) 	//4Byte float
#define 	Loadallctrlpara_MaxDeceleration_ADDR		(Loadallctrlpara_MinDeceleration_ADDR + 4) 	//4Byte float
#define 	Loadallctrlpara_MinSpeed_ADDR				(Loadallctrlpara_MaxDeceleration_ADDR + 4) 	//4Byte float
#define 	Loadallctrlpara_MaxSpeed_ADDR				(Loadallctrlpara_MinSpeed_ADDR + 4) 		//4Byte float
#define 	Loadallctrlpara_Deadband_Deadband_ADDR		(Loadallctrlpara_MaxSpeed_ADDR + 4) 		//4Byte float
#define 	Loadallctrlpara_Deadband_PercentP_ADDR		(Loadallctrlpara_Deadband_Deadband_ADDR + 4) 	//1Byte uint8_t
#define 	Loadallctrlpara_Dither_DitherAmplitude_ADDR	(Loadallctrlpara_Deadband_PercentP_ADDR + 1) 	//4Byte float
#define 	Loadallctrlpara_Dither_DitherFrequency_ADDR	(Loadallctrlpara_Dither_DitherAmplitude_ADDR + 4)//4Byte float
#define 	Loadallctrlpara_NominalAcc_ADDR	            (Loadallctrlpara_Dither_DitherFrequency_ADDR + 4) 	//4Byte float
#define 	Loadallctrlpara_NominalSpeed_ADDR	        (Loadallctrlpara_NominalAcc_ADDR + 4)//4Byte float

//EXTEN_MODE all closeloop control parameter 115Byte  800-922  reserve space:800-999
#define 	Extallctrlpara_PosPid_P_ADDR				800 									//4Byte float
#define 	Extallctrlpara_PosPid_I_ADDR				(Extallctrlpara_PosPid_P_ADDR + 4)		//4Byte float
#define 	Extallctrlpara_PosPid_D_ADDR				(Extallctrlpara_PosPid_I_ADDR + 4) 		//4Byte float
#define 	Extallctrlpara_PosPid_Hp_P_ADDR				(Extallctrlpara_PosPid_D_ADDR + 4) 		//4Byte float
#define 	Extallctrlpara_PosPid_Hp_I_ADDR				(Extallctrlpara_PosPid_Hp_P_ADDR + 4) 	//4Byte float
#define 	Extallctrlpara_PosPid_Hp_D_ADDR				(Extallctrlpara_PosPid_Hp_I_ADDR + 4) 	//4Byte float
#define 	Extallctrlpara_SpeedPid_P_ADDR				(Extallctrlpara_PosPid_Hp_D_ADDR + 4) 	//4Byte float
#define 	Extallctrlpara_SpeedPid_I_ADDR				(Extallctrlpara_SpeedPid_P_ADDR + 4) 	//4Byte float
#define 	Extallctrlpara_SpeedPid_D_ADDR				(Extallctrlpara_SpeedPid_I_ADDR + 4) 	//4Byte float
#define 	Extallctrlpara_Ffd_SpeedFFP_ADDR			(Extallctrlpara_SpeedPid_D_ADDR + 4) 	//4Byte float
#define 	Extallctrlpara_Ffd_PosDelay_ADDR			(Extallctrlpara_Ffd_SpeedFFP_ADDR + 4) 	//4Byte uint32_t
#define 	Extallctrlpara_Ffd_Hp_SpeedFFP_ADDR			(Extallctrlpara_Ffd_PosDelay_ADDR + 4) 	//4Byte float
#define 	Extallctrlpara_Ffd_Hp_PosDelay_ADDR			(Extallctrlpara_Ffd_Hp_SpeedFFP_ADDR + 4) 	//4Byte uint32_t
#define 	Extallctrlpara_DefaultAcc_ADDR				(Extallctrlpara_Ffd_Hp_PosDelay_ADDR + 4) 	//4Byte float
#define 	Extallctrlpara_Speed_ADDR					(Extallctrlpara_DefaultAcc_ADDR + 4) 	//4Byte float
#define 	Extallctrlpara_Deviation_ADDR				(Extallctrlpara_Speed_ADDR + 4) 		//4Byte float
#define 	Extallctrlpara_DevReaction_ADDR				(Extallctrlpara_Deviation_ADDR + 4) 	//1Byte uint8_t
#define 	Extallctrlpara_Wnd_WndSize_ADDR				(Extallctrlpara_DevReaction_ADDR + 1) 	//4Byte float
#define 	Extallctrlpara_Wnd_WndTime_ADDR				(Extallctrlpara_Wnd_WndSize_ADDR + 4) 	//4Byte uint32_t
#define 	Extallctrlpara_Sft_UpperSft_ADDR			(Extallctrlpara_Wnd_WndTime_ADDR + 4) 	//4Byte float
#define 	Extallctrlpara_Sft_LowerStf_ADDR			(Extallctrlpara_Sft_UpperSft_ADDR + 4) 	//4Byte float
#define 	Extallctrlpara_Sft_Reaction_ADDR			(Extallctrlpara_Sft_LowerStf_ADDR + 4) 	//1Byte uint8_t
#define 	Extallctrlpara_MinAcceleration_ADDR			(Extallctrlpara_Sft_Reaction_ADDR + 1) 	//4Byte float
#define 	Extallctrlpara_MaxAcceleration_ADDR			(Extallctrlpara_MinAcceleration_ADDR + 4) 	//4Byte float
#define 	Extallctrlpara_MinDeceleration_ADDR			(Extallctrlpara_MaxAcceleration_ADDR + 4) 	//4Byte float
#define 	Extallctrlpara_MaxDeceleration_ADDR			(Extallctrlpara_MinDeceleration_ADDR + 4) 	//4Byte float
#define 	Extallctrlpara_MinSpeed_ADDR				(Extallctrlpara_MaxDeceleration_ADDR + 4) 	//4Byte float
#define 	Extallctrlpara_MaxSpeed_ADDR				(Extallctrlpara_MinSpeed_ADDR + 4) 		//4Byte float
#define 	Extallctrlpara_Deadband_Deadband_ADDR		(Extallctrlpara_MaxSpeed_ADDR + 4) 		//4Byte float
#define 	Extallctrlpara_Deadband_PercentP_ADDR		(Extallctrlpara_Deadband_Deadband_ADDR + 4) 	//1Byte uint8_t
#define 	Extallctrlpara_Dither_DitherAmplitude_ADDR	(Extallctrlpara_Deadband_PercentP_ADDR + 1) 	//4Byte float
#define 	Extallctrlpara_Dither_DitherFrequency_ADDR	(Extallctrlpara_Dither_DitherAmplitude_ADDR + 4)//4Byte float
#define 	Extallctrlpara_NominalAcc_ADDR	            (Extallctrlpara_Dither_DitherFrequency_ADDR + 4) 	//4Byte float
#define 	Extallctrlpara_NominalSpeed_ADDR	        (Extallctrlpara_NominalAcc_ADDR + 4)//4Byte float

//system parameter	 3Byte  5000-5002  reserve space:5000-5049
#define 	SYSPARA_TRANSMITDATAPERIOD_ADDR		        5000 	//2Byte uint16_t
#define 	SYSPARA_CONTROLLERSTRUCTURE_ADDR	        (SYSPARA_TRANSMITDATAPERIOD_ADDR + 2)//1Byte uint8_t

//servo parameter	 12Byte  5050-5061  reserve space:5050-5099
#define     SERVOPARA_ELECTRONICGEAR_ADDR               5050  //4Byte uint32_t
#define     SERVOPARA_ENCODERRESOLUTION_ADDR            (SERVOPARA_ELECTRONICGEAR_ADDR + 4) //4Byte uint32_t
#define     SERVOPARA_ANALOGGAIN_ADDR                   (SERVOPARA_ENCODERRESOLUTION_ADDR + 4) //4Byte float

//output parameter	 30Byte  5100-5129   reserve space:5100-5199
#define 	OUTPUTPARA_SIGNAL_ADDR		5100 	//1Byte uint8_t
#define 	OUTPUTPARA_SIGN_ADDR		(OUTPUTPARA_SIGNAL_ADDR + 1)//1Byte uint8_t
#define 	OUTPUTPARA_MAXVALUE_ADDR	(OUTPUTPARA_SIGN_ADDR + 1)//4Byte float
#define 	OUTPUTPARA_MINVALUE_ADDR	(OUTPUTPARA_MAXVALUE_ADDR + 4)//4Byte float
#define 	OUTPUTPARA_INITVALUE_ADDR	(OUTPUTPARA_MINVALUE_ADDR + 4)//4Byte float
#define 	OUTPUTPARA_OFFSET_ADDR		(OUTPUTPARA_INITVALUE_ADDR + 4)//4Byte float
#define 	OUTPUTPARA_SIGNALFREQUENCY_ADDR	(OUTPUTPARA_OFFSET_ADDR + 4)//4Byte float
#define 	OUTPUTPARA_MAXVOLTAGE_ADDR	(OUTPUTPARA_SIGNALFREQUENCY_ADDR + 4)//4Byte float
#define 	OUTPUTPARA_MAXCURRENT_ADDR	(OUTPUTPARA_MAXVOLTAGE_ADDR + 4)//4Byte float

//these data need to be saved realtime     space usage:5200-5211     space reservation:5200-5399
#define     POSE_ENCODER_CODE_ADDR  5200 //4Byte int32_t
#define     MRAM_LOAD_TARE_VALUE_ADDR     (POSE_ENCODER_CODE_ADDR+4) //4Byte int32_t
#define     MRAM_EXTEN_TARE_VALUE_ADDR    (MRAM_LOAD_TARE_VALUE_ADDR+4) //4Byte int32_t
#define     MRAM_EXTEN2_TARE_VALUE_ADDR   (MRAM_EXTEN_TARE_VALUE_ADDR+4) //4Byte int32_t
#define     MRAM_LOAD_TARE_CODE_ADDR      (MRAM_EXTEN2_TARE_VALUE_ADDR+4) //4Byte int32_t
#define     MRAM_EXTEN_TARE_CODE_ADDR     (MRAM_LOAD_TARE_CODE_ADDR+4) //4Byte int32_t
#define     MRAM_EXTEN2_TARE_CODE_ADDR    (MRAM_EXTEN_TARE_CODE_ADDR+4) //4Byte int32_t

//these data need to be saved when they are changed   space usage:5400-     space reservation:5400-5599
#define     SENSOR_CHECK_REGISTER0_ADDR  5400 //1Byte uint8_t
#define     SENSOR_CHECK_REGISTER1_ADDR  (SENSOR_CHECK_REGISTER0_ADDR + 1) //1Byte uint8_t
#define     SENSOR_CHECK_REGISTER2_ADDR  (SENSOR_CHECK_REGISTER1_ADDR + 1)  //1Byte uint8_t
#define     SENSOR_CHECK_MAPNUM0_ADDR     (SENSOR_CHECK_REGISTER2_ADDR + 1) //1Byte uint8_t
#define     SENSOR_CHECK_MAPNUM1_ADDR     (SENSOR_CHECK_MAPNUM0_ADDR + 1) //1Byte uint8_t
#define     SENSOR_CHECK_MAPNUM2_ADDR     (SENSOR_CHECK_MAPNUM1_ADDR + 1) //1Byte uint8_t

//Sensor Pose Content 
//400Byte 5600-5999
#define SensorType_POSE_ADDR 		5600							//1Byte uint16_t
#define Sign_POSE_ADDR 				(SensorType_POSE_ADDR 	+ 1)	//1Byte uint8_t
#define NominalValue_POSE_ADDR 		(Sign_POSE_ADDR 			+ 1)	//4Byte float
#define NominalSensitive_POSE_ADDR 	(NominalValue_POSE_ADDR 	+ 4)	//4Byte float
#define Day_POSE_ADDR 				(NominalSensitive_POSE_ADDR + 4)	//1Byte uint8_t
#define Month_POSE_ADDR 				(Day_POSE_ADDR 			+ 1)	//1Byte uint8_t
#define Year_POSE_ADDR 				(Month_POSE_ADDR			+ 1)	//2Byte uint8_t
#define LinPoint_POSE_ADDR 			(Year_POSE_ADDR			+ 2)	//1Byte uint8_t
#define LinV_POSE_ADDR 				(LinPoint_POSE_ADDR		+ 1)	//3*4*SEN_LIN_DATA_MAX Byte LinVal

//sensor intgr content
//20Byte 6000-6019
#define INTGR_POSE_ADDR              6000                               //4Byte uint32_t
#define INTGR_BIGDERFORMATION_ADDR   (INTGR_POSE_ADDR + 4)              //4Byte uint32_t
#define INTGR_EXTEN1_ADDR            (INTGR_BIGDERFORMATION_ADDR + 4)   //4Byte uint32_t
#define INTGR_EXTEN2_ADDR            (INTGR_EXTEN1_ADDR + 4)            //4Byte uint32_t
#define INTGR_LOAD_ADDR              (INTGR_EXTEN2_ADDR + 4)            //4Byte uint32_t

//sensor intgr content
// Byte 6020-6036
#define ETH_MODE_ADDR                6020                               //1Byte uint8_t
#define ETH_SERVER_IP_ADDR           (ETH_MODE_ADDR + 1)                //4Byte uint8_t[4]
#define ETH_SERVER_PORT_ADDR         (ETH_SERVER_IP_ADDR + 4)         //2Byte uint16_t
#define ETH_LOCAL_IP_ADDR            (ETH_SERVER_PORT_ADDR + 2)            //4Byte uint8_t[4]
#define ETH_LOCAL_MAC_ADDR           (ETH_LOCAL_IP_ADDR + 4)            //6Byte uint8_t[6]

//#define VERSION_ADDR  20000-
#define	PeInterface_ADDR		20000					   //锟斤拷位锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟??"xx.xx" 6*4
#define Application_ADDR 		(PeInterface_ADDR 	+ 6*4 )//锟斤拷位锟斤拷应锟矫版本锟斤拷xxxxxxxx.xxx 13*4
#define Subsy_ADDR				(Application_ADDR 	+ 13*4)///锟斤拷系统锟芥本锟斤拷xx.xx 6*4
#define SubsyCustVer_ADDR		(Subsy_ADDR			+ 6*4 )//锟斤拷系统锟矫伙拷锟芥本锟斤拷xx.xx  6*4
#define SubsyCustName_ADDR		(SubsyCustVer_ADDR	+ 6*4 )//锟斤拷系统锟矫伙拷锟斤拷锟斤拷xxxxxxxx 9*4
#define Bios_ADDR				(SubsyCustName_ADDR	+ 9*4 )//锟斤拷位锟斤拷BIOS锟芥本锟斤拷xx.xx 6*4
#define HwCtrl_ADDR				(Bios_ADDR			+ 6*4 )//锟斤拷位锟斤拷锟斤拷锟斤拷锟斤拷锟教硷拷锟脚★拷xxxx.x 7*4
#define	PeInterfacePC_ADDR		(HwCtrl_ADDR		+ 7*4 )//PC锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷姹撅拷锟絰x.xx 6*4
#define DpxVer_ADDR				(PeInterfacePC_ADDR + 6*4 )//DPX锟侥硷拷锟斤拷式锟芥本锟脚★拷xx.xx 6*4
#define	SerialNumber_ADDR		(DpxVer_ADDR		+ 6*4 )//锟斤拷位锟斤拷锟斤拷锟叫号★拷xxxxxxxxxxxxxxxx  17*4
#define	VersionEnd_ADDR			(SerialNumber_ADDR	+ 17*4)//锟斤拷\0锟斤拷1


//Sensor Content 
//400Byte 100-499
#define SensorType_ADDR 		100							//1Byte uint16_t
#define Sign_ADDR 				(SensorType_ADDR 	+ 1)	//1Byte uint8_t
#define NominalValue_ADDR 		(Sign_ADDR 			+ 1)	//4Byte float
#define NominalSensitive_ADDR 	(NominalValue_ADDR 	+ 4)	//4Byte float
#define Day_ADDR 				(NominalSensitive_ADDR + 4)	//1Byte uint8_t
#define Month_ADDR 				(Day_ADDR 			+ 1)	//1Byte uint8_t
#define Year_ADDR 				(Month_ADDR			+ 1)	//2Byte uint8_t
#define LinPoint_ADDR 			(Year_ADDR			+ 2)	//1Byte uint8_t
#define LinV_ADDR 				(LinPoint_ADDR		+ 1)	//3*4*SEN_LIN_DATA_MAX Byte LinVal

//Bigderformation Senser	19Byte 500-518	reserve space:500-599
#define SensorBdSensortype_ADDR			500									//1Byte uint8_t
#define	SensorBdSign_ADDR 				(SensorBdSensortype_ADDR 	+ 1)	//1Byte uint16_t
#define SensorBdNominalValue_ADDR 		(SensorBdSign_ADDR 			+ 1)	//4Byte float
#define SensorBdCollectType_ADDR 		(SensorBdNominalValue_ADDR	+ 4)	//1Byte uint8_t
#define SensorBdNominalSensitive1_ADDR 	(SensorBdCollectType_ADDR 	+ 1)//4Byte float
#define SensorBdNominalSensitive2_ADDR 	(SensorBdNominalSensitive1_ADDR + 4)	//4Byte float
#define SensorBdDay_ADDR				(SensorBdNominalSensitive2_ADDR + 4)	//1Byte float
#define SensorBdMonth_ADDR 				(Day_ADDR 			+ 1)	//1Byte uint8_t
#define SensorBdYear_ADDR 				(Month_ADDR			+ 1)	//2Byte uint16_t


#define Dev_Controller 				0x00
#define Dev_Sensor     				0x01
#define Dev_SensorBigDeformation	0x02

void mram_test(void);

// void 	mram_test(void);
// uint8_t mramOnController_WriteAll(void);
// uint8_t mramOnController_ReadAll(void);
// uint8_t mramAnalogueSensor_WriteAll(uint16_t Device_Addr,SENSORANALOGUEDATA* x);
// uint8_t mramAnalogueSensor_ReadAll(uint16_t Device_Addr,SENSORANALOGUEDATA* x);
// uint8_t mramSensorHeaderData_Write(uint16_t Device_Addr,SENSORHEADER* x);
// uint8_t mramSensorHeaderData_Read(uint16_t Device_Addr,SENSORHEADER* x);
// uint8_t At24cxx_Check(uint16_t Device_Addr,void* x,uint8_t DeviceType);
// //SensorData
// uint8_t mramSensor_WriteAll(uint16_t Device_Addr,sensorData_t* x);
// uint8_t mramSensor_ReadAll(uint16_t Device_Addr,sensorData_t* x);
// //Bigdeformation senser data
// uint8_t mramSensorBigDeformation_WriteAll(uint16_t Device_Addr,SENSORBIGDEFORMATIONDATA* x);
// uint8_t mramSensorBigDeformation_ReadAll(uint16_t Device_Addr,SENSORBIGDEFORMATIONDATA* x);
// //system parameter
// uint8_t mramSysPara_WriteAll(SYSPARA* x);
// uint8_t mramSysPara_ReadAll(SYSPARA* x);
// //output paramter
// uint8_t mramOutputPara_WriteAll(OUTPUTPARA* x);
// uint8_t mramOutputPara_ReadAll(OUTPUTPARA* x);

// //read sendata from mram when system init
// uint8_t mram_sendata_init_read(void);
// //According to the semaphore, write relevant data to mram
// void mram_write_monitor(mySemaphore_t semaphore);
// //Wnd write
// uint8_t mramWnd_write(uint16_t Device_Addr,WND* x,uint8_t movectrl);
// //Ffd write
// uint8_t mramFfd_write(uint16_t Device_Addr,FFD* x,uint8_t movectrl);
// uint8_t mramNominalWrite(NOMINALVALUE* x,uint8_t movectrl);

void mram_write_monitor(STORAGE_PARAM_MYSEMAPHORE* semaphore);

void mramInit(void);

#endif//_MRAM_MANAGE_H_

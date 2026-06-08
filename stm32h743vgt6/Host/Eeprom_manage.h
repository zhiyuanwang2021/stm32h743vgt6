#ifndef _EEPROM_MANAGE_H_
#define _EEPROM_MANAGE_H_

#include "EEPROM.h"
#include "gParameter.h"
#include "in_out.h"
#include "sensor.h"
#define EEPROM_CHECK 0xA6

#define M24C02_Load		  M24C02_0010
#define M24C02_EXT1       M24C02_0100
#define M24C02_EXT2       M24C02_0110

//Eeprom On Controller
#define CHEAK_ADDR 32767	//???????????????????§??????????
#define SENSOR_MAP_CHEAK_ADDR 32767	//???????????????????§??????????

#define distance_scale_ADDR 	0//??????????????????°?¤??°?mm 4Byte distance_scale
#define PosDefaultAcc_ADDR 		(distance_scale_ADDR+4)//4Byte DefaultPosAcc
#define PosDefaultDec_ADDR 		(PosDefaultAcc_ADDR+4)//4Byte DefaultPosDec
#define LoadDefaultAcc_ADDR 	(PosDefaultDec_ADDR+4)//4Byte DefaultLoadAcc
#define LoadDefaultDec_ADDR 	(LoadDefaultAcc_ADDR+4)//4Byte DefaultLoadDec
#define ExtDefaultAcc_ADDR 		(LoadDefaultDec_ADDR+4)//4Byte DefaultExtAcc
#define ExtDefaultDec_ADDR 		(ExtDefaultAcc_ADDR+4)//4ByteDefaultExtDec
#define upperlimit_phy_ADDR 	(ExtDefaultDec_ADDR+4)//4Byte upperlimit_phy
#define lowerlimit_phy_ADDR 	(upperlimit_phy_ADDR+4)//4Byte lowerlimit_phy
#define defaultspeed_ADDR 		(lowerlimit_phy_ADDR+4)//4Byte defaultspeed
#define defaultspeedlimit_ADDR 	(defaultspeed_ADDR+4)//4Byte defaultspeedlimit
#define DataTransmitPeriod_ADDR (defaultspeedlimit_ADDR+2)//2Byte DataTransmitPeriod


//All CTRL PARAM

//POS_MODE ??????é????????§????????° 115Byte  400-522  é????????é??400-599
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

//LOAD_MODE ??????é????????§????????° 115Byte  600-722  é????????é??600-799
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

//EXTEN_MODE ??????é????????§????????° 115Byte  800-922  é????????é??800-999
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

// 3Byte  5000-5002  ?¤??????????????5000-5099
#define 	SYSPARA_TRANSMITDATAPERIOD_ADDR		5000 	//2Byte uint16_t
#define 	SYSPARA_CONTROLLERSTRUCTURE_ADDR	(SYSPARA_TRANSMITDATAPERIOD_ADDR + 2)//1Byte uint8_t

//??????????????????????	 Byte  5100-5129   ?¤??????????????5100-5199
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
#define     LOAD_TARE_CODE_ADDR     (POSE_ENCODER_CODE_ADDR+4) //4Byte int32_t
#define     EXTEN_TARE_CODE_ADDR    (LOAD_TARE_CODE_ADDR+4) //4Byte int32_t


//#define VERSION_ADDR  20000-
#define	PeInterface_ADDR		20000					   //??????????????????????????????????????????"xx.xx" 6*4
#define Application_ADDR 		(PeInterface_ADDR 	+ 6*4 )//????????????????????°?????????xxxxxxxx.xxx?????? 13*4
#define Subsy_ADDR				(Application_ADDR 	+ 13*4)///??????????????????????xx.xx??????6*4
#define SubsyCustVer_ADDR		(Subsy_ADDR			+ 6*4 )//??????????????????????????????xx.xx?????? 6*4
#define SubsyCustName_ADDR		(SubsyCustVer_ADDR	+ 6*4 )//??????????????????????????????xxxxxxxx??????9*4
#define Bios_ADDR				(SubsyCustName_ADDR	+ 9*4 )//??????????????BIOS????????????xx.xx?????? 6*4
#define HwCtrl_ADDR				(Bios_ADDR			+ 6*4 )//????????????????????????????????????????????????xxxx.x??????7*4
#define	PeInterfacePC_ADDR		(HwCtrl_ADDR		+ 7*4 )//PC???????????????????????????????????????xx.xx??????6*4
#define DpxVer_ADDR				(PeInterfacePC_ADDR + 6*4 )//DPX??????????????????????????????xx.xx??????6*4
#define	SerialNumber_ADDR		(DpxVer_ADDR		+ 6*4 )//??????????????????????????????xxxxxxxxxxxxxxxx?????? 17*4
#define	VersionEnd_ADDR			(SerialNumber_ADDR	+ 17*4)//??????\0??????1

////Eeprom On Analogue Sensor Connector
////Sensor Header
//#define PartNo_ADDR 			0							//2Byte uint16_t
//#define Version_ADDR 			(PartNo_ADDR 		+ 2)	//1Byte uint8_t
//#define SerNo_ADDR 				(Version_ADDR 		+ 1)	//4Byte uint32_t
//#define Class_ADDR 				(SerNo_ADDR 		+ 4)	//2Byte uint16_t
//#define DatVersion_ADDR 		(Class_ADDR 		+ 2)	//1Byte uint8_t

////298Byte 100-397
//#define MaxExcitation_ADDR 		100							//4Byte float
//#define MinImpedance_ADDR 		(MaxExcitation_ADDR + 4)	//2Byte uint16_t
//#define NominalValue_ADDR 		(MinImpedance_ADDR 	+ 2)	//4Byte float
//#define Unit_ADDR 				(NominalValue_ADDR 	+ 4)	//2Byte uint16_t
//#define Offset_ADDR 			(Unit_ADDR 			+ 2)	//4Byte float
//#define NegLimit_ADDR 			(Offset_ADDR 		+ 4)	//2Byte uint16_t
//#define PosLimit_ADDR 			(NegLimit_ADDR 		+ 2)	//2Byte uint16_t
//#define Reference_ADDR 			(PosLimit_ADDR 		+ 2)	//4Byte float
//#define CorrReference_ADDR 		(Reference_ADDR 	+ 4)	//4Byte float
//#define Sensortype_ADDR 		(CorrReference_ADDR + 4)	//2Byte uint16_t
//#define NominalSensitive_ADDR 	(Sensortype_ADDR 	+ 2)	//4Byte float
//#define GaugeLength_ADDR		(NominalSensitive_ADDR 	+ 4)	//2Byte float
//#define Sign_ADDR 				(GaugeLength_ADDR 	+ 2)	//2Byte uint16_t
//#define Day_ADDR 				(Sign_ADDR 			+ 2)	//2Byte uint16_t
//#define Month_ADDR 				(Day_ADDR 			+ 2)	//2Byte uint16_t
//#define Year_ADDR 				(Month_ADDR 		+ 2)	//2Byte uint16_t
//#define LinPoint_ADDR 			(Year_ADDR 			+ 2)	//2Byte uint16_t
//#define LinVal_ADDR 			(LinPoint_ADDR 		+ 2)	//2*4*SEN_LIN_DATA_MAX Byte LinVal

//Eeprom On Sensor Connector

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

//typedef struct 
//{ 
// float 		NominalValue; 		/* Nominal value of the sensor [Unit] ??????????*/
// float 		NominalSensitive1; 	/* Sensitivity at Nominal value [*] ?????????1*/
// float 		NominalSensitive2; 	/* Sensitivity at Nominal value [*] ?????????2*/
// uint8_t	CollectType;		/* type of Collect [No] ????????????????*/
// uint8_t   Sensortype;			/* Sensor type [No] ??????????*/
// uint8_t 	Sign; 				/* Invert sign of channel [1/0] ??????????*/
// uint8_t 	Day; 				/* Date of last change [No] ??????????????????*/
// uint8_t 	Month; 				/* Date of last change [No]	??????????????????*/
// uint16_t 	Year; 				/* Date of last change [No]	??????????????????*/
//}SENSORBIGDEFORMATIONDATA; 

//??????????????????????????????????????????	19Byte 500-518	?¤??????????????500-599
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
void 	EEPROM_test(void);
uint8_t EepromOnController_WriteAll(void);
uint8_t EepromOnController_ReadAll(void);
uint8_t EepromAnalogueSensor_WriteAll(uint16_t Device_Addr,SENSORANALOGUEDATA* x);
uint8_t EepromAnalogueSensor_ReadAll(uint16_t Device_Addr,SENSORANALOGUEDATA* x);
uint8_t EepromSensorHeaderData_Write(uint16_t Device_Addr,SENSORHEADER* x);
uint8_t EepromSensorHeaderData_Read(uint16_t Device_Addr,SENSORHEADER* x);
uint8_t At24cxx_Check(uint16_t Device_Addr,void* x,uint8_t DeviceType);
//SensorData
uint8_t EepromSensor_WriteAll(uint16_t Device_Addr,sensorData_t* x);
uint8_t EepromSensor_ReadAll(uint16_t Device_Addr,sensorData_t* x);
//????????????????
uint8_t EepromSensorBigDeformation_WriteAll(uint16_t Device_Addr,SENSORBIGDEFORMATIONDATA* x);
uint8_t EepromSensorBigDeformation_ReadAll(uint16_t Device_Addr,SENSORBIGDEFORMATIONDATA* x);
//????????????????
uint8_t EepromSysPara_WriteAll(SYSPARA* x);
uint8_t EepromSysPara_ReadAll(SYSPARA* x);
//??????????????????????
uint8_t EepromOutputPara_WriteAll(OUTPUTPARA* x);
uint8_t EepromOutputPara_ReadAll(OUTPUTPARA* x);

//read sendata from eeprom when system init
uint8_t eeprom_sendata_init_read(void);
//According to the semaphore, write relevant data to eeprom
void eeprom_write_monitor(STORAGE_PARAM_MYSEMAPHORE* semaphore);
//Wnd write
uint8_t EepromWnd_write(uint16_t Device_Addr,WND* x,uint8_t movectrl);
//Ffd write
uint8_t EepromFfd_write(uint16_t Device_Addr,FFD* x,uint8_t movectrl);
//sensor read when system init
void eepromSensorReadInit(SENSER_CHECK_STRUCT* x);
#endif//_EEPROM_MANAGE_H_


#include "sensor.h"
#include "gParameter.h"
#include "DI.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include <elog.h>
#include "Eeprom_manage.h"
#include "adc_scale.h"

static float multipointCalibrateFunc(const int32_t _code,const uint8_t _connector,const sensorData_t *_senData,float *_orig);
static int32_t antiMultipointCalibrateFunc( const float _orig,const uint8_t _connector,const sensorData_t *_senData,int32_t *_code);
static float singlepointCalibrateFunc(const int32_t _code,const uint8_t _connector,const sensorData_t *_senData,float *_orig);
static int32_t antiSinglepointCalibrateFunc(const float _orig,const uint8_t _connector,const sensorData_t *_senData,int32_t *_code);
static void multipointZeroCodeUpdateFunc(const uint8_t _connector,const sensorData_t *_senData,int32_t *zeroCode);

SENSER_CHECK_STRUCT sensorCheck = {
	.Addr[0] = LOAD_SENSER, .Addr[1] = EXTERN_SENSER1, .Addr[2] = EXTERN_SENSER2,
	.Register[0] = 0, .Register[1] = 0, .Register[2] = 0,
	.MapNum[0] = SENSOR_NO_CHANNEL, .MapNum[1] = SENSOR_NO_CHANNEL, .MapNum[2] = SENSOR_NO_CHANNEL
};//shouled be store in mram

sensorCalibrate_t sensorCalibrate;

// void sensorTypeMapCheckIn(DB9_MYSEMAPHORE* Sem);
// void sensorTypeMapCheckOut(DB9_MYSEMAPHORE* Sem);


float pose_calibrate_process(void)
{
	return SenData[ch0Pose].NominalSensitive;
}

float load_calibrate_process(void)
{
	return SenData[ch4Load].NominalSensitive;
}

float ext1_calibrate_process(void)
{
	return SenData[ch2Ext1].NominalSensitive;
}

float ext2_calibrate_process(void)
{
	return SenData[ch3Ext2].NominalSensitive;
}

/**
 * @brief Initialize the sensor calibration functions.
 */
void sensorCalibrateFuncInit(void){
	sensorCalibrate.multipointCalibrate = multipointCalibrateFunc;
	sensorCalibrate.antiMultipointCalibrate = antiMultipointCalibrateFunc;
	sensorCalibrate.singlepointCalibrate = singlepointCalibrateFunc;
	sensorCalibrate.antiSinglepointCalibrate = antiSinglepointCalibrateFunc;
	sensorCalibrate.multipointZeroCodeUpdate = multipointZeroCodeUpdateFunc;
}

void sensorZeroCodeInit(void){
	//pose
	if(SenData[ch0Pose].LinPoint > 0){
		sensorCalibrate.multipointZeroCodeUpdate(ch0Pose,&SenData[ch0Pose],&sensorCalibrate.zeroCode[ch0Pose]);
	}else{
		sensorCalibrate.zeroCode[ch0Pose] = 0;
	}
	//bigdeformation
	if(SenData[ch1Bd].LinPoint > 0){
		sensorCalibrate.multipointZeroCodeUpdate(ch1Bd,&SenData[ch1Bd],&sensorCalibrate.zeroCode[ch1Bd]);
	}else{
		sensorCalibrate.zeroCode[ch1Bd] = 0;
	}
	//ext1
	if(SenData[ch2Ext1].LinPoint > 0){
		sensorCalibrate.multipointZeroCodeUpdate(ch2Ext1,&SenData[ch2Ext1],&sensorCalibrate.zeroCode[ch2Ext1]);
	}else{
		sensorCalibrate.zeroCode[ch2Ext1] = 0;
	}
	//ext2
	if(SenData[ch3Ext2].LinPoint > 0){
		sensorCalibrate.multipointZeroCodeUpdate(ch3Ext2,&SenData[ch3Ext2],&sensorCalibrate.zeroCode[ch3Ext2]);
	}else{
		sensorCalibrate.zeroCode[ch3Ext2] = 0;
	}
	//load
	if(SenData[ch4Load].LinPoint > 0){
		sensorCalibrate.multipointZeroCodeUpdate(ch4Load,&SenData[ch4Load],&sensorCalibrate.zeroCode[ch4Load]);
	}else{
		sensorCalibrate.zeroCode[ch4Load] = 0;
	}

}

/**
 * @brief In system init process, sync sensor data read from mram to the application layer.
 * @param[in]	in[5] 	type:sensorData_t
 * @param[out]	out 	type:APP_LAYER_VARIABLE_STRUCT*
*/
void senDateSync(sensorData_t in[5],APP_LAYER_VARIABLE_STRUCT* out){
	//pose
	if(in[ch0Pose].Sign == 0x00)
		out->posCtrl.sign = 1;
	else if(in[ch0Pose].Sign == 0x01)
		out->posCtrl.sign = -1;
	out->posCtrl.NominalSensitive = in[ch0Pose].NominalSensitive;
	//big deformation

	//ext1
	if(in[ch2Ext1].Sign == 0x00)
		out->ext1Ctrl.sign = 1;
	else if(in[ch2Ext1].Sign == 0x01){
		out->ext1Ctrl.sign = -1;
	}
	out->ext1Ctrl.NominalSensitive = in[ch2Ext1].NominalSensitive;
	out->ext1Ctrl.NominalValue = in[ch2Ext1].NominalValue;
	//ext2
	if(in[ch3Ext2].Sign == 0x00)
		out->ext2Ctrl.sign = 1;
	else if(in[ch3Ext2].Sign == 0x01)
		out->ext2Ctrl.sign = -1;
	out->ext2Ctrl.NominalSensitive = in[ch3Ext2].NominalSensitive;
	out->ext2Ctrl.NominalValue = in[ch3Ext2].NominalValue;
	//load
	if(in[ch4Load].Sign == 0x00)
		out->loadCtrl.sign = 1;
	else if(in[ch4Load].Sign == 0x01)
		out->loadCtrl.sign = -1;
	out->loadCtrl.NominalSensitive = in[ch4Load].NominalSensitive;
	out->loadCtrl.NominalValue = in[ch4Load].NominalValue;
}

/**
 * @note	Fun_163
 * @brief when mysemaphore.comm.sendataUpdate is 1,this function will be called
 * 
*/
void sendata_update(void)
{
	uint8_t j=0;
	//fucntion code 0x1b not include sensorIntgr,so sensorIntgr must be updated manually before total memcpy
	SenData[sensorConnector].sensorIntgr = senDataCommbuf[sensorConnector].sensorIntgr;
	memcpy(&SenData[sensorConnector],&senDataCommbuf[sensorConnector],sizeof(senDataCommbuf[sensorConnector]));
	switch(sensorConnector)
	{
		case 0x00://Pos
			AL.posCtrl.NominalSensitive = pose_calibrate_process();//位置校正系数更新
			if(SenData[ch0Pose].Sign == 0x01)
				AL.posCtrl.sign = -1;
			else
				AL.posCtrl.sign = 1;
			//zero code update
			if(SenData[ch0Pose].LinPoint > 0){
				sensorCalibrate.multipointZeroCodeUpdate(ch0Pose,&SenData[ch0Pose],&sensorCalibrate.zeroCode[ch0Pose]);
			}else{
				sensorCalibrate.zeroCode[ch0Pose] = 0;
			}
		break;
		case 0x01://Bigdeformation
			// AL.ext1Ctrl.NominalSensitive = ext1_calibrate_process();//位置校正系数更新
			// if(SenData[ch2Ext1].Sign == 0x01)
			// 	;
			// else
			// 	;
		break;
		case 0x02://Ext1
			AL.ext1Ctrl.NominalSensitive = ext1_calibrate_process();//位置校正系数更新
			AL.ext1Ctrl.NominalValue = SenData[ch2Ext1].NominalValue;
			if(SenData[ch2Ext1].Sign == 0x01)
				AL.ext1Ctrl.sign = -1;
			else
				AL.ext1Ctrl.sign = 1;
			
			//zero code update
			if(SenData[ch2Ext1].LinPoint > 0){
				for(;j<SenData[sensorConnector].LinPoint;j++)
				{
					SenData[sensorConnector].LinV[j].CorrectionFactor = 1.0f / SenData[sensorConnector].LinV[j].CorrectionFactor
												* ADC_FACTOR_SENSOR_FRONTEND_ROUND * SenData[sensorConnector].NominalValue;	
					//printf("Num:%d Temp:%f\r\n",j,SenData[sensorConnector].LinV[j].CorrectionFactor);												
				}
				sensorCalibrate.multipointZeroCodeUpdate(ch2Ext1,&SenData[ch2Ext1],&sensorCalibrate.zeroCode[ch2Ext1]);
			}else{
				sensorCalibrate.zeroCode[ch2Ext1] = 0;
			}
		break;
		case 0x03://Ext2
			AL.ext2Ctrl.NominalSensitive = ext2_calibrate_process();//位置校正系数更新
			AL.ext2Ctrl.NominalValue = SenData[ch3Ext2].NominalValue;
			if(SenData[ch3Ext2].Sign == 0x01)
				AL.ext2Ctrl.sign = -1;
			else
				AL.ext2Ctrl.sign = 1;
			
			//zero code update
			if(SenData[ch3Ext2].LinPoint > 0){
				for(;j<SenData[sensorConnector].LinPoint;j++)
				{
					SenData[sensorConnector].LinV[j].CorrectionFactor = 1.0f / SenData[sensorConnector].LinV[j].CorrectionFactor
												* ADC_FACTOR_SENSOR_FRONTEND_ROUND * SenData[sensorConnector].NominalSensitive;													
				}
				sensorCalibrate.multipointZeroCodeUpdate(ch3Ext2,&SenData[ch3Ext2],&sensorCalibrate.zeroCode[ch3Ext2]);
			}else{
				sensorCalibrate.zeroCode[ch3Ext2] = 0;
			}
		break;
		case 0x04://Load
			AL.loadCtrl.NominalSensitive = load_calibrate_process();//载荷校正系数更新
			AL.loadCtrl.NominalValue = SenData[ch4Load].NominalValue;
			if(SenData[ch4Load].Sign == 0x01)
				AL.loadCtrl.sign = -1;
			else
				AL.loadCtrl.sign = 1;
			//zero code update
			if(SenData[ch4Load].LinPoint > 0){
				for(;j<SenData[sensorConnector].LinPoint;j++)
				{
					SenData[sensorConnector].LinV[j].CorrectionFactor = 1.0f / SenData[sensorConnector].LinV[j].CorrectionFactor
												* ADC_FACTOR_SENSOR_FRONTEND_ROUND * SenData[sensorConnector].NominalValue * 1000.0f;	
					//printf("Num:%d Temp:%f\r\n",j,SenData[sensorConnector].LinV[j].CorrectionFactor);						
				}
				sensorCalibrate.multipointZeroCodeUpdate(ch4Load,&SenData[ch4Load],&sensorCalibrate.zeroCode[ch4Load]);
			}else{
				sensorCalibrate.zeroCode[ch4Load] = 0;
			}
		break;
	}
	
	
}

void sendata_pg_update(void* set)
{
	switch(sensorConnector)
		{
			case 0x00://pose
				memset(&pg,0,sizeof(pg));
				pg.sg = *((float*)set);
				pg.s3 = pg.sg;
				pg.st = pg.sg;
			break;
			case 0x01://big deformation

			break;
			case 0x02://ext1

			break;
			case 0x03://ext2

			break;
			case 0x04://load
				memset(&pgLoad,0,sizeof(pgLoad));
				pgLoad.sg = *((float*)set);
				pgLoad.s3 = pgLoad.sg;
				pgLoad.st = pgLoad.sg;
			break;
		}
}




#define NO_PRESS 0 		//CON no connected or ADJ no pressed
#define DELAY_PRESS 1	//eliminate dither
#define YES_PRESS 2			//CON has been connected or ADJ has been pressed
#define DELAY_UNPRESS 3			//CON has been connected or ADJ has been pressed
#if USE_DB9_CON
static DB9Typedef lastRecord = {
	.CON1.value = GPIO_PIN_RESET,
	.CON2.value = GPIO_PIN_RESET,
	.CON3.value = GPIO_PIN_RESET,
};
#endif
/**
 * @brief monitor the state of CON and ADJ of sensor DB9 connector
 * @param[in] Db9 DB9Typedef*
*/
void sensorCON_ADJ_Monitor(DB9Typedef* Db9){
#if USE_DB9_CON 
	 //CON1
	 //CON1 connect
	 if(lastRecord.CON1.value == GPIO_PIN_RESET && Db9->CON1.value == GPIO_PIN_SET){
		mySemaphore.db9.CON1 = 1;
		//log_d("mySemaphore.db9.CON1:%d",mySemaphore.db9.CON1);
	 }
	 //CON1 disconnect
	 else if(lastRecord.CON1.value == GPIO_PIN_SET && Db9->CON1.value == GPIO_PIN_RESET){
		mySemaphore.db9.CON1 = 2;
		//log_d("mySemaphore.db9.CON1:%d",mySemaphore.db9.CON1);
	 }
	 //CON1 disconnect before power on
	 else if((sensorCheck.MapNum[0] ==  SENSOR_CHANNEL_1 || sensorCheck.MapNum[1] ==  SENSOR_CHANNEL_1 || sensorCheck.MapNum[2] ==  SENSOR_CHANNEL_1)
	 		&& Db9->CON1.value == GPIO_PIN_RESET && Db9->CON1.timUnPress > 200 && mySemaphore.db9.CON1 == 0){
		mySemaphore.db9.CON1 = 2;
		//log_d("CON1 disconnect before power on, mySemaphore.db9.CON1:%d",mySemaphore.db9.CON1);
	 }
	 lastRecord.CON1.value = Db9->CON1.value;

	//CON2 connect
	 if(lastRecord.CON2.value == GPIO_PIN_RESET && Db9->CON2.value == GPIO_PIN_SET){
		mySemaphore.db9.CON2 = 1;
		//log_d("mySemaphore.db9.CON2:%d",mySemaphore.db9.CON2);
	 }
	 //CON2 disconnect
	 else if(lastRecord.CON2.value == GPIO_PIN_SET && Db9->CON2.value == GPIO_PIN_RESET){
		mySemaphore.db9.CON2 = 2;
		//log_d("mySemaphore.db9.CON2:%d",mySemaphore.db9.CON2);
	 }
	 //CON2 disconnect before power on
	 else if((sensorCheck.MapNum[0] ==  SENSOR_CHANNEL_2 || sensorCheck.MapNum[1] ==  SENSOR_CHANNEL_2 || sensorCheck.MapNum[2] ==  SENSOR_CHANNEL_2)
	 		&& Db9->CON2.value == GPIO_PIN_RESET && Db9->CON2.timUnPress > 200  && mySemaphore.db9.CON2 == 0){
		mySemaphore.db9.CON2 = 2;
		//log_d("CON2 disconnect before power on, mySemaphore.db9.CON2:%d",mySemaphore.db9.CON2);
	 }
	 lastRecord.CON2.value = Db9->CON2.value;

	//CON3 connect
	 if(lastRecord.CON3.value == GPIO_PIN_RESET && Db9->CON3.value == GPIO_PIN_SET){
		mySemaphore.db9.CON3 = 1;
		//log_d("mySemaphore.db9.CON3:%d",mySemaphore.db9.CON3);
	 }
	 //CON3 disconnect
	 else if(lastRecord.CON3.value == GPIO_PIN_SET && Db9->CON3.value == GPIO_PIN_RESET){
		mySemaphore.db9.CON3 = 2;
		//log_d("mySemaphore.db9.CON3:%d",mySemaphore.db9.CON3);
	 }
	 //CON3 disconnect before power on
	 else if((sensorCheck.MapNum[0] ==  SENSOR_CHANNEL_3 || sensorCheck.MapNum[1] ==  SENSOR_CHANNEL_3 || sensorCheck.MapNum[2] ==  SENSOR_CHANNEL_3)
	 		&& Db9->CON3.value == GPIO_PIN_RESET && Db9->CON3.timUnPress > 200  && mySemaphore.db9.CON3 == 0){
		mySemaphore.db9.CON3 = 2;
		//log_d("CON3 disconnect before power on, mySemaphore.db9.CON3:%d",mySemaphore.db9.CON3);
	 }
	 lastRecord.CON3.value = Db9->CON3.value;
#endif
}

/**
 * @brief if the semaphores of mySemaphore.db9.conx are given, this function will be called
 * @param[in] Sem DB9_MYSEMAPHORE*
*/
// void sensorTypeMap(DB9_MYSEMAPHORE* Sem){
// 	//connector connect
// 	if(Sem->CON1 == 1 || Sem->CON2 == 1 || Sem->CON3 == 1){
// 		sensorTypeMapCheckIn(Sem);
// 	}
// 	//connector disconnect
// 	if(Sem->CON1 == 2 || Sem->CON2 == 2 || Sem->CON3 == 2){
// 		sensorTypeMapCheckOut(Sem);
// 	}
// }


/**
 * @brief sensor type check by reading eeprom.
 * @note just consider one sensor at a time
 * @param[in] Sem DB9_MYSEMAPHORE*
*/
#define SENSOR_CHANNEL_NUM_SUM 3
// void sensorTypeMapCheckIn(DB9_MYSEMAPHORE* Sem){
// 	uint8_t i,err,check_num = EEPROM_CHECK;
// 	for(i = 0; i < SENSOR_CHANNEL_NUM_SUM; i++){
// 		if(sensorCheck.Register[i] != 0) 
// 		{
// 			if(sensorCheck.Register[i] != 1){
// 				sensorCheck.Register[i] = 0;
// 				sensorCheck.MapNum[i] = SENSOR_NO_CHANNEL;
// 				mySemaphore.mram.write_sensorCheck = 1;
// 				continue;
// 			}

// 			if(sensorCheck.MapNum[i] == SENSOR_CHANNEL_1 && Sem->CON1 == 1)
// 				Sem->CON1 = 0;
// 			if(sensorCheck.MapNum[i] == SENSOR_CHANNEL_2 && Sem->CON2 == 1)
// 				Sem->CON2 = 0;
// 			if(sensorCheck.MapNum[i] == SENSOR_CHANNEL_3 && Sem->CON3 == 1)
// 				Sem->CON3 = 0;
// 			//log_d("the sensor[%d] has been registered,skip",i);
// 			//valid type
// 			err = At24cxx_Write_Byte(sensorCheck.Addr[i], SENSOR_MAP_CHEAK_ADDR, &check_num);
// 			if(err) 
// 			{
// 				//log_d("write[%d] err, skip",i);
// 				continue;//write err, skip this cycle 
// 			}
// 			err = At24cxx_Read_Byte(sensorCheck.Addr[i]|1, SENSOR_MAP_CHEAK_ADDR, &check_num);
// 			if(err) 
// 			{
// 				//log_d("read err, skip");
// 				continue;//read err, skip this cycle 
// 			}
// 			if(check_num == EEPROM_CHECK){
// 				switch (sensorCheck.Addr[i])
// 				{
// 				case LOAD_SENSER:
// 					mySemaphore.eeprom.readSensorOnceLoad = 1;
// 					log_i("readSensorOnceLoad = 1");
// 					break;
// 				case EXTERN_SENSER1:
// 					mySemaphore.eeprom.readSensorOnceExt1 = 1;
// 					log_i("readSensorOnceExt1 = 1");
// 					break;
// 				case EXTERN_SENSER2:
// 					mySemaphore.eeprom.readSensorOnceExt2 = 1;
// 					log_i("readSensorOnceExt2 = 1");
// 					break;
// 				default:
// 					break;
// 				}
// 			}
// 			continue;//the sensor has been registered,skip this cycle
// 		}
// 		err = At24cxx_Write_Byte(sensorCheck.Addr[i], SENSOR_MAP_CHEAK_ADDR, &check_num);
// 		if(err) 
// 		{
// 			//log_d("write[%d] err, skip",i);
// 			continue;//write err, skip this cycle 
// 		}
// 		err = At24cxx_Read_Byte(sensorCheck.Addr[i]|1, SENSOR_MAP_CHEAK_ADDR, &check_num);
// 		if(err) 
// 		{
// 			//log_d("read err, skip");
// 			continue;//read err, skip this cycle 
// 		}
// 		//valid type
// 		if(check_num == EEPROM_CHECK){
// 			if(Sem->CON1 == 1){//channel 2
// 				sensorCheck.MapNum[i] = SENSOR_CHANNEL_1;
// 				Sem->CON1 = 0;
// 				sensorCheck.Register[i] = 1; //regiter
// 				//log_d("sensorCheckIn.MapNum[%d] == SENSOR_CHANNEL_1",i);
// 				//sensorCheck mram store semphore give
// 				mySemaphore.mram.write_sensorCheck = 1;
// 			}else if(Sem->CON2 == 1){//channel 1
// 				sensorCheck.MapNum[i] = SENSOR_CHANNEL_2;
// 				Sem->CON2 = 0;
// 				sensorCheck.Register[i] = 1; //regiter
// 				//log_d("sensorCheckIn.MapNum[%d] == SENSOR_CHANNEL_2",i);
// 				//sensorCheck mram store semphore give
// 				mySemaphore.mram.write_sensorCheck = 1;
// 			}else if(Sem->CON3 == 1){//channel 0
// 				sensorCheck.MapNum[i] = SENSOR_CHANNEL_3;
// 				Sem->CON3 = 0;
// 				sensorCheck.Register[i] = 1; //regiter
// 				//log_d("sensorCheckIn.MapNum[%d] == SENSOR_CHANNEL_3",i);
// 				//sensorCheck mram store semphore give
// 				mySemaphore.mram.write_sensorCheck = 1;
// 			}
// 			switch (sensorCheck.Addr[i])
// 			{
// 			case LOAD_SENSER:
// 				mySemaphore.eeprom.readSensorOnceLoad = 1;
// 				log_i("readSensorOnceLoad = 1");
// 				break;
// 			case EXTERN_SENSER1:
// 				mySemaphore.eeprom.readSensorOnceExt1 = 1;
// 				break;
// 			case EXTERN_SENSER2:
// 				mySemaphore.eeprom.readSensorOnceExt2 = 1;
// 				break;
// 			default:
// 				break;
// 			}

// 			break;
// 		}
// 	}
// }

// /**
//  * @brief sensor type check by reading eeprom.
//  * @note just consider one sensor at a time
//  * @param[in] Sem DB9_MYSEMAPHORE*
//  * 
// */
// void sensorTypeMapCheckOut(DB9_MYSEMAPHORE* Sem){
// 	uint8_t i;
// 	if(Sem->CON1 == 2){
// 		for(i = 0; i < SENSOR_CHANNEL_NUM_SUM; i++){
// 			if(sensorCheck.MapNum[i] == SENSOR_CHANNEL_1){
// 				sensorCheck.MapNum[i] = SENSOR_NO_CHANNEL;
// 				sensorCheck.Register[i] = 0;
// 				//log_d("sensorCheckOut.MapNum[%d] == SENSOR_CHANNEL_1",i);
// 				//sensorCheck mram store semphore give
// 				mySemaphore.mram.write_sensorCheck = 1;
// 				break;
// 			}
// 		}
// 		Sem->CON1 = 0;
// 	}else if(Sem->CON2 == 2){
// 		for(i = 0; i < SENSOR_CHANNEL_NUM_SUM; i++){
// 			if(sensorCheck.MapNum[i] == SENSOR_CHANNEL_2){
// 				sensorCheck.MapNum[i] = SENSOR_NO_CHANNEL;
// 				sensorCheck.Register[i] = 0;
// 				//log_d("sensorCheckOut.MapNum[%d] == SENSOR_CHANNEL_2",i);
// 				//sensorCheck mram store semphore give
// 				mySemaphore.mram.write_sensorCheck = 1;
// 				break;
// 			}
// 		}
// 		Sem->CON2 = 0;
// 	}else if(Sem->CON3 == 2){
// 		for(i = 0; i < SENSOR_CHANNEL_NUM_SUM; i++){
// 			if(sensorCheck.MapNum[i] == SENSOR_CHANNEL_3){
// 				sensorCheck.MapNum[i] = SENSOR_NO_CHANNEL;
// 				sensorCheck.Register[i] = 0;
// 				//log_d("sensorCheckOut.MapNum[%d] == SENSOR_CHANNEL_3",i);
// 				//sensorCheck mram store semphore give
// 				mySemaphore.mram.write_sensorCheck = 1;
// 				break;
// 			}
// 		}
// 		Sem->CON3 = 0;
// 	}
// }

void sensorConnectCheckByEeprom(HAL_DB9_STRUCT *_Db9){
	uint8_t i,err,check_num = EEPROM_CHECK,readCounter = 0;
	//
	for(i = 0; i < SENSOR_CHANNEL_NUM_SUM; i++){
		//have registered
		if(sensorCheck.Register[i] != 0) 
		{
			if(sensorCheck.Register[i] != 1){
				sensorCheck.Register[i] = 0;
				sensorCheck.MapNum[i] = SENSOR_NO_CHANNEL;
				mySemaphore.mram.write_sensorCheck = 1;
				continue;
			}


			//check whether the sensor connector have been disconnected
			//read once which have been registered in UTC controller
			for(readCounter = 0; readCounter < 4; readCounter++){
				err = At24cxx_Read_Byte(sensorCheck.Addr[i]|1, SENSOR_MAP_CHEAK_ADDR, &check_num);
				if(err == 0){
					break;
				}
			}
			if(check_num == EEPROM_CHECK && err == 0){
				//sensor register state no need to be changed
				switch (sensorCheck.Addr[i])
				{
				case LOAD_SENSER:
					_Db9->CON2.value = GPIO_PIN_SET;
					break;
				case EXTERN_SENSER1:
					_Db9->CON1.value = GPIO_PIN_SET;
					break;
				case EXTERN_SENSER2:
					_Db9->CON3.value = GPIO_PIN_SET;
					break;
				default:
					break;
				}
				continue;
			}else{
				sensorCheck.Register[i] = 0;
				sensorCheck.MapNum[i] = SENSOR_NO_CHANNEL;
				mySemaphore.mram.write_sensorCheck = 1;
				//sensor register state need to be changed, because the sensor connector have been disconnected
				switch (sensorCheck.Addr[i])
				{
				case LOAD_SENSER:
					_Db9->CON2.value = GPIO_PIN_RESET;//disconnected
					log_i("CON2 disconnected");
					break;
				case EXTERN_SENSER1:
					_Db9->CON1.value = GPIO_PIN_RESET;//disconnected
					log_i("CON1 disconnected");
					break;
				case EXTERN_SENSER2:
					_Db9->CON3.value = GPIO_PIN_RESET;//disconnected
					log_i("CON3 disconnected");
					break;
				default:
					break;
				}
			}
			continue;//the sensor has been removed,skip this cycle
		}
		//have no registered
		err = At24cxx_Write_Byte(sensorCheck.Addr[i], SENSOR_MAP_CHEAK_ADDR, &check_num);
		if(err) 
		{
			//log_d("write[%d] err, skip",i);
			//continue;//write err, skip this cycle 
		}
		check_num = 0;
		err = At24cxx_Read_Byte(sensorCheck.Addr[i]|1, SENSOR_MAP_CHEAK_ADDR, &check_num);
		if(err) 
		{
			//log_d("read err, skip");
			//continue;//read err, skip this cycle 
		}
		//valid type
		if(check_num == EEPROM_CHECK){
			log_i("check_num == EEPROM_CHECK");
			switch (sensorCheck.Addr[i])
			{
			case LOAD_SENSER:
				sensorCheck.MapNum[i] = SENSOR_CHANNEL_2;
				_Db9->CON2.value = GPIO_PIN_SET;//disconnected
				mySemaphore.eeprom.readSensorOnceLoad = 1;
				log_i("CON2 connected");
				break;
			case EXTERN_SENSER1:
				sensorCheck.MapNum[i] = SENSOR_CHANNEL_1;
				_Db9->CON1.value = GPIO_PIN_SET;//disconnected
				mySemaphore.eeprom.readSensorOnceExt1 = 1;
				log_i("CON1 connected");
				break;
			case EXTERN_SENSER2:
				sensorCheck.MapNum[i] = SENSOR_CHANNEL_3;	
				_Db9->CON3.value = GPIO_PIN_SET;//disconnected
				mySemaphore.eeprom.readSensorOnceExt2 = 1;
				log_i("CON3 connected");
				break;
			default:
				break;
			}
			sensorCheck.Register[i] = 1;
			mySemaphore.mram.write_sensorCheck = 1;
		}else if(sensorCheck.Register[i] != 0 || sensorCheck.MapNum[i] != SENSOR_NO_CHANNEL){
			sensorCheck.Register[i] = 0;
			sensorCheck.MapNum[i] = SENSOR_NO_CHANNEL;
			mySemaphore.mram.write_sensorCheck = 1;
			switch (sensorCheck.Addr[i])
			{
			case LOAD_SENSER:
				sensorCheck.MapNum[i] = SENSOR_NO_CHANNEL;
				_Db9->CON2.value = GPIO_PIN_RESET;//disconnected
				break;
			case EXTERN_SENSER1:
				sensorCheck.MapNum[i] = SENSOR_NO_CHANNEL;
				_Db9->CON1.value = GPIO_PIN_RESET;//disconnected
				break;
			case EXTERN_SENSER2:
				sensorCheck.MapNum[i] = SENSOR_NO_CHANNEL;	
				_Db9->CON3.value = GPIO_PIN_RESET;//disconnected
				break;
			default:
				break;
			}
		}
	}
}

/**
 * @brief Performs multi-point calibration based on the given ADC code and sensor data.
 * 
 * This function takes an ADC code, a pointer to a sensor data structure, and a pointer to the original value.
 * It calculates the calibrated original value based on the sensor data's linearization points.
 * 
 * @param[in] _code The ADC code to be calibrated.
 * @param[in] _connector The connector type.
 * @param[in] _senData A pointer to the sensor data structure containing calibration parameters.
 * @param[in,out] _orig A pointer to the variable storing the calibrated original value.
 * 
 * @return The calibrated original value.
 */
static float multipointCalibrateFunc( 	const int32_t _code,
										const uint8_t _connector,
										const sensorData_t *_senData,
										float *_orig){
	uint8_t index;
	int8_t signFactor = (_senData->LinV[0].CorrectionFactor < 0.0f ? 1 : 0);//0 represent positive,1 represent negative
	int32_t m;
	float n,k;//(m,n),k
	if(signFactor == 0){
		for(index = 0;index<_senData->LinPoint;index++){
			if(_code <= _senData->LinV[index].ADCCode){
				m = _senData->LinV[index].ADCCode;//x-coordinate
				n = _senData->LinV[index].RefValue;//y-coordinate
				k = _senData->LinV[index].CorrectionFactor;//slope
				break;
			}
		}
		if(index >= _senData->LinPoint){
			m = _senData->LinV[_senData->LinPoint-1].ADCCode;//x-coordinate
			n = _senData->LinV[_senData->LinPoint-1].RefValue;//y-coordinate
			k = _senData->LinV[_senData->LinPoint-1].CorrectionFactor;//slope
		}
	}else{
		for(index = 0;index<_senData->LinPoint;index++){
			if(_code > _senData->LinV[index].ADCCode){
				m = _senData->LinV[index].ADCCode;//x-coordinate
				n = _senData->LinV[index].RefValue;//y-coordinate
				k = _senData->LinV[index].CorrectionFactor;//slope
				break;
			}
		}
		if(index >= _senData->LinPoint){
			m = _senData->LinV[0].ADCCode;//x-coordinate
			n = _senData->LinV[0].RefValue;//y-coordinate
			k = _senData->LinV[0].CorrectionFactor;//slope
		}
	}

	*_orig = (float)(_code - m)*k + n;

	switch (_connector)
	{
	case ch0Pose:
		*_orig = AL.posCtrl.sign * *_orig;
		break;
	case ch1Bd:
		
		break;
	case ch2Ext1:
		strain1.origTare = AL.ext1Ctrl.sign * *_orig;
		*_orig = AL.ext1Ctrl.sign * *_orig - AL.tare.fValue[ch2Ext1];
		break;
	case ch3Ext2:
		strain2.origTare = AL.ext2Ctrl.sign * *_orig;
		*_orig = AL.ext2Ctrl.sign * *_orig - AL.tare.fValue[ch3Ext2];
		break;
	case ch4Load:
		force.origTare = AL.loadCtrl.sign * *_orig;
		*_orig = AL.loadCtrl.sign * *_orig - AL.tare.fValue[ch4Load];
		break;
	
	default:
		break;
	}

	if(_senData->Sensortype == 0x00){
		test.monitor1 = n;
		test.monitor2 = k;
		test.monitor3 = *_orig;
		test.monitorCode1 = _code;
		test.monitorCode2 = m;
		test.index = index;
	}

	return (*_orig);
}

/**
 * @brief Performs anti-multi-point calibration based on the given original value and sensor data.
 * 
 * This function takes an original value, a pointer to a sensor data structure, and a pointer to the ADC code.
 * It calculates the original ADC code based on the sensor data's linearization points.
 * 
 * @param[in] _orig The original value to be calibrated.
 * @param[in] _connector The connector type.
 * @param[in] _senData A pointer to the sensor data structure containing calibration parameters.
 * @param[in,out] _code A pointer to the variable storing the calculated ADC code.
 * 
 * @return The calculated ADC code.
 */
static int32_t antiMultipointCalibrateFunc( const float _orig,
											const uint8_t _connector,
                                        	const sensorData_t *_senData,
                                        	int32_t *_code){
	uint8_t index;
	int32_t m;
	float n,k;//(m,n),k
	float _origOffsetTare;

	switch (_connector)
	{
	case ch0Pose:
		_origOffsetTare = AL.posCtrl.sign * _orig;
		break;
	case ch1Bd:
		_origOffsetTare = _orig;
		break;
	case ch2Ext1:
		_origOffsetTare = (_orig + AL.tare.fValue[ch2Ext1]) * AL.ext1Ctrl.sign;
		break;
	case ch3Ext2:
		_origOffsetTare = (_orig + AL.tare.fValue[ch3Ext2]) * AL.ext2Ctrl.sign;
		break;
	case ch4Load:
		_origOffsetTare = (_orig + AL.tare.fValue[ch4Load]) * AL.loadCtrl.sign;
		break;
	
	default:
		break;
	}

	for(index = 0;index<_senData->LinPoint;index++){
		if(_origOffsetTare <= _senData->LinV[index].RefValue){
			m = _senData->LinV[index].ADCCode;//x-coordinate
			n = _senData->LinV[index].RefValue;//y-coordinate
			k = _senData->LinV[index].CorrectionFactor;//slope
			break;
		}
	}
	if(index >= _senData->LinPoint){
		m = _senData->LinV[_senData->LinPoint-1].ADCCode;//x-coordinate
		n = _senData->LinV[_senData->LinPoint-1].RefValue;//y-coordinate
		k = _senData->LinV[_senData->LinPoint-1].CorrectionFactor;//slope
	}
	*_code = (int32_t)((_origOffsetTare - n)/k + m);
	return (*_code);
}

/**
 * @brief Performs single point calibration based on the given ADC code and connector type.
 * 
 * This function takes an ADC code, a connector type, a pointer to a sensor data structure, and a pointer to the original value.
 * It calculates the calibrated original value based on the connector type and the calibration parameters in the sensor data structure.
 * 
 * @param[in] _code The ADC code to be calibrated.
 * @param[in] _connector The connector type.
 * @param[in] _senData A pointer to the sensor data structure containing calibration parameters.
 * @param[in,out] _orig A pointer to the variable storing the calibrated original value.
 * 
 * @return The calibrated original value.
 */
static float singlepointCalibrateFunc(  const int32_t _code,
                                    	const uint8_t _connector,
										const sensorData_t *_senData,
										float *_orig){
	switch (_connector)
	{
	case ch0Pose:
		*_orig = AL.posCtrl.sign * _code * AL.posCtrl.NominalSensitive;
		break;
	case ch1Bd:
		//wait for adding
		*_orig = 7777;
		break;
	case ch2Ext1:
		strain1.origTare = AL.ext1Ctrl.sign * (ADC_FACTOR_SENSOR_FRONTEND_ROUND * _code
				/ AL.ext1Ctrl.NominalSensitive * AL.ext1Ctrl.NominalValue);
		*_orig = strain1.origTare - AL.tare.fValue[ch2Ext1];
		break;
	case ch3Ext2:
		strain2.origTare = AL.ext2Ctrl.sign * (ADC_FACTOR_SENSOR_FRONTEND_ROUND * _code
				/ AL.ext2Ctrl.NominalSensitive * AL.ext2Ctrl.NominalValue);
		*_orig = strain2.origTare - AL.tare.fValue[ch3Ext2];
		break;
	case ch4Load:
		force.origTare = AL.loadCtrl.sign * ((ADC_FACTOR_SENSOR_FRONTEND_ROUND * 1000.0f)  * _code
				/ AL.loadCtrl.NominalSensitive * AL.loadCtrl.NominalValue);
		*_orig = force.origTare - AL.tare.fValue[ch4Load];
		break;
	default:
		break;
	}
	return (*_orig);
}

/**
 * @brief Performs anti-single point calibration based on the given original value and connector type.
 * 
 * This function takes an original value, a connector type, a pointer to a sensor data structure, and a pointer to the ADC code.
 * It calculates the original ADC code based on the connector type and the calibration parameters in the sensor data structure.
 * 
 * @param[in] _orig The original value to be calibrated.
 * @param[in] _connector The connector type.
 * @param[in] _senData A pointer to the sensor data structure containing calibration parameters.
 * @param[in,out] _code A pointer to the variable storing the calculated ADC code.
 * 
 * @return The calculated ADC code.
 */
static int32_t antiSinglepointCalibrateFunc(const float _orig,
                                        	const uint8_t _connector,   
                                       		const sensorData_t *_senData,
                                        	int32_t *_code){
	switch (_connector)
	{
	case ch0Pose:
		*_code = (int32_t)(_orig / AL.posCtrl.NominalSensitive / AL.posCtrl.sign);
		break;
	case ch1Bd:
		//wait for adding
		*_code = 0x7777;
		break;
	case ch2Ext1:
		*_code = (int32_t)((_orig + AL.tare.fValue[ch2Ext1]) * AL.ext1Ctrl.NominalSensitive * AL.ext1Ctrl.sign / AL.ext1Ctrl.NominalValue * 1.0f / ADC_FACTOR_SENSOR_FRONTEND_ROUND);
		break;
	case ch3Ext2:
		*_code = (int32_t)((_orig + AL.tare.fValue[ch3Ext2]) * AL.ext2Ctrl.NominalSensitive * AL.ext2Ctrl.sign / AL.ext2Ctrl.NominalValue * 1.0f / ADC_FACTOR_SENSOR_FRONTEND_ROUND);
		break;
	case ch4Load:
		*_code = (int32_t)((_orig + AL.tare.fValue[ch4Load]) * AL.loadCtrl.NominalSensitive * AL.loadCtrl.sign / AL.loadCtrl.NominalValue * 1.0f / (ADC_FACTOR_SENSOR_FRONTEND_ROUND * 1000.0f));
		break;
	default:
		break;
	}
	return (*_code);
}

/**
 * @brief Update the zero code for multi-point calibration.
 * 
 * This function updates the zero code for multi-point calibration based on the given sensor data.
 * It calls the anti-multi-point calibration function to calculate the zero code.
 * 
 * @param[in] _senData A pointer to the sensor data structure containing calibration parameters.
 * @param[in] _connector The connector type.
 * @param[in,out] zeroCode A pointer to the variable storing the updated zero code.
 */
static void multipointZeroCodeUpdateFunc(const uint8_t _connector,const sensorData_t *_senData,int32_t *zeroCode){
	sensorCalibrate.antiMultipointCalibrate(0.0f,_connector,_senData,zeroCode);
}

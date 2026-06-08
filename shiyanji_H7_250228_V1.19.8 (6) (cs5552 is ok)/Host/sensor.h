#ifndef _SENSOR_H_
#define	_SENSOR_H_

#include "gParameter.h"
#include "DI.h"
#include "EEPROM.h"

typedef enum{
 	SENSOR_CHANNEL_1       = 2, 
    SENSOR_CHANNEL_2       = 1, 
    SENSOR_CHANNEL_3       = 0, 
    SENSOR_NO_CHANNEL      = 255,
} SENSOR_CHANNEL_ENUM;

typedef enum{
    LOAD_SENSER             = M24C02_0010, //Load
    EXTERN_SENSER1          = M24C02_0100, //Extensometer1
    EXTERN_SENSER2          = M24C02_0110, //Extensometer2
} SENSOR_CHEACK_ENUM;

typedef struct{
    uint8_t Register[3];//Load ext1 ext2
	uint8_t MapNum[3];//Load ext1 ext2
	SENSOR_CHEACK_ENUM Addr[3];//Load ext1 ext2
}SENSER_CHECK_STRUCT;
extern SENSER_CHECK_STRUCT sensorCheck;

typedef struct{
    //Multipoint calibration table lookup function
    float (*multipointCalibrate)(   const int32_t _code,
                                    const uint8_t _connector,
                                    const sensorData_t *_senData,
                                    float *_orig);

    //Multipoint calibration reverse table lookup function
    int32_t (*antiMultipointCalibrate)( const float _orig,
                                        const uint8_t _connector,
                                        const sensorData_t *_senData,
                                        int32_t *_code);
    
    //Singlepoint calibration calculate function
    float (*singlepointCalibrate)(  const int32_t _code,
                                    const uint8_t _connector,
                                    const sensorData_t *_senData,
                                    float *_orig);
    
    //Singlepoint calibration reverse calculate function
    int32_t (*antiSinglepointCalibrate)(const float _orig,
                                        const uint8_t _connector,   
                                        const sensorData_t *_senData,
                                        int32_t *_code);
    void (*multipointZeroCodeUpdate)(   const uint8_t _connector,
                                        const sensorData_t *_senData,
                                        int32_t *zeroCode);
    int32_t zeroCode[SENSOR_CH_NUM];
}sensorCalibrate_t;
extern sensorCalibrate_t sensorCalibrate;

float pose_calibrate_process(void);//位移标定
float load_calibrate_process(void);//载荷标定
float ext1_calibrate_process(void);//应变1标定
float ext2_calibrate_process(void);//应变2标定

void sensorCalibrateFuncInit(void);
void sensorZeroCodeInit(void);

void senDateSync(sensorData_t in[5],APP_LAYER_VARIABLE_STRUCT* out);
void sendata_update(void);
void sendata_pg_update(void* set);
void sensorCON_ADJ_Monitor(DB9Typedef* x);

void sensorTypeMap(DB9_MYSEMAPHORE* Sem);
void sensorConnectCheckByEeprom(HAL_DB9_STRUCT *_Db9);
#endif


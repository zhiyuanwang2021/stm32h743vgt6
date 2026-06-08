#include "RS485.h"
#include "usart.h"
#include "CRC16.h"
#include <elog.h>
#include "in_out.h"

#ifndef sign
	#define sign(x) ((x)>0?1:((x)<0?-1:1))
#endif

uint8_t rs485_rxbuf[rs485_rxbuf_size]={0};//娑撴彃褰涢崡鏇炵摟閼哄倹甯撮弨鏈电瑐娴ｅ秵婧€Modbus閹稿洣鎶ょ紓鎾冲暱閿燂拷?
uint8_t rs485_txbuf[rs485_txbuf_size]={0};//娑撴彃褰涢崡鏇炵摟閼哄倸褰傞柅浣风瑐娴ｅ秵婧€Modbus閹稿洣鎶ょ紓鎾冲暱閿燂拷?
uint8_t buf_485[3]={0};

manualBox485_t       manualBox485={
	.mainBoardAddr = 0x01,
	.manualBoxAddr = 0x02,
	.reflag = 0,
	.recount = 0,
};
manualBoxHoldreg_t   manualBoxHoldreg;
manualBoxCoil_t      manualBoxCoil={
	.word = 0xFFFF,
};



void RS485_init(UART_HandleTypeDef *huartx)
{
	HAL_UARTEx_ReceiveToIdle_IT(huartx, rs485_rxbuf, rs485_rxbuf_size);//485閹恒儲鏁归崚婵嗩潗閿燂拷?
	RS485_EN_RESET;//婢惰精鍏楨N 485鏉╂稑鍙嗛幒銉︽暪濡€崇础
}

void RS485_test()
{
		//printf("rxbuf:%x\r\n",buf[0]);
		RS485_EN_SET;//
		manualBox485Func10Request(&manualBox485,&manualBoxHoldreg,pose.orig,speedPose.filter,force.filterTrans,strain1.filterTrans);
		RS485_EN_RESET;//
}

uint8_t manualBoxCoilUpdate(const uint8_t _coilNum,const bool _state)
{
	uint8_t err = 0;
	switch((_coilNum+1))
	{
		case 1:
			if(_state == true){
				mySemaphore.manualBox485.up = 1;
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word &= 0xFFFD;//DI2
				//printf("manualBox485 Up");
			}else{
				mySemaphore.manualBox485.halt = 1;
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word |= 0x003F;//DI1-DI6 set 1
				//printf("manualBox485 Up halt");
			}
			break;
		case 2:
			if(_state == true){
				mySemaphore.manualBox485.rapidUp = 1;
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word &= 0xFFFB;//DI3
				//printf("manualBox485 rapidUp");
			}else{
				mySemaphore.manualBox485.halt = 1;
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word |= 0x003F;//DI1-DI6 set 1
				//printf("manualBox485 rapidUp halt");
			}
			break;
		case 3:
			if(_state == true){
				mySemaphore.manualBox485.down = 1;
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word &= 0xFFF7;//DI4
				//printf("manualBox485 Down");
			}else{
				mySemaphore.manualBox485.halt = 1;
				if(manualBox.type == manualBox485_Type)	
					manualBoxCoil.word |= 0x003F;//DI1-DI6 set 1
				//printf("manualBox485 Down halt");
			}
			break;
		case 4:
			if(_state == true){
				mySemaphore.manualBox485.rapidDown = 1;
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word &= 0xFFEF;//DI5
				//printf("manualBox485 rapidDown");
			}else{
				mySemaphore.manualBox485.halt = 1;
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word |= 0x003F;//DI1-DI6 set 1
				//printf("manualBox485 rapidDown halt");
			}
			break;
		case 5:
			if(_state == true){
				mySemaphore.manualBox485.testStart = 1;
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word &= 0xFFFE;//DI1
				//printf("manualBox485 testStart");
			}else{
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word |= 0x003F;//DI1-DI6 set 1
				
			}
			break;
		case 6:
			if(_state == true){
				mySemaphore.manualBox485.reset = 1;
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word &= 0xFFDF;//DI6
				//printf("manualBox485 reset");
			}else{
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word |= 0x003F;//DI1-DI6 set 1
			}
			break;
		case 7:
			if(_state == true){
				mySemaphore.manualBox485.clampUpperFixture = 1;
				//printf("manualBox485 clampUpperFixture");
			}
			break;
		case 8:
			if(_state == true){
				mySemaphore.manualBox485.releaseUpperFixture = 1;
				//printf("manualBox485 releaseUpperFixture");
			}
			break;
		case 9:
			if(_state == true){
				mySemaphore.manualBox485.clampLowerFixture = 1;
				//printf("manualBox485 clampLowerFixture");
			}
			break;
		case 10:
			if(_state == true){
				mySemaphore.manualBox485.releaseLowerFixture = 1;
				//printf("manualBox485 releaseLowerFixture");
			}
			break;
		default:
			printf("unnamed coil num-address:0x%2X",_coilNum);
			err = 1;
			break;
	}
	return err;
}

/**
 * @brief send the callBack of the response of writing multi holdregs
 * @param[in out] _mb485
 * @param[out] _mbHoldreg
 * @param[in] _pose
 * @param[in] _poseSpeed
 * @param[in] _load
 * @param[in] _strain
 */
void manualBox485Func03ResponseCallBack(manualBox485_t *_mb485,manualBoxHoldreg_t *_mbHoldreg,
		const float _pose,const float _poseSpeed,const float _load,const float _strain){
	uint16_t regAdd,regLen,crc;//鐎靛嫬鐡ㄩ崳銊ㄦ崳婵婀撮崸鈧�,鐎靛嫬鐡ㄩ崳銊ュ礋閸忓啴鏆辨惔锔肩礉閺佺増宓侀敍瀹憆c閺嶏繝鐛欓敓锟�?
	uint32_t poseInt32,poseSpeedInt32,loadInt32,strainInt32;
	uint8_t sizeByte,i=0,j=0;//鐎涙濡敓锟�?
	int32_t poseSign,poseSpeedSign,loadSign,strainSign;
	uint16_t* p=(uint16_t*)_mbHoldreg;//閹稿洤鎮滄穱婵囧瘮鐎靛嫬鐡ㄩ崳銊х波閺嬪嫪缍嬫＃鏍ф勾閸р偓
	regAdd = MB485_regAdd;//鐠囪褰囩€靛嫬鐡ㄩ崳銊ㄦ崳婵婀撮崸鈧�
	regLen = MB485_regLen;//鐎靛嫬鐡ㄩ崳銊ュ礋閸忓啴鏆遍敓锟�?
	sizeByte = regLen*2;//鐎涙濡敓锟�?

	poseSign = sign(_pose);
	poseSpeedSign = sign(_poseSpeed);
	loadSign = sign(_load);
	strainSign = sign(_strain);

	poseInt32 		= (uint32_t)((_pose * 1e6f) * poseSign);
	if(poseSign == -1){
		poseInt32 = poseInt32 | 0x80000000;
	}
	poseSpeedInt32 	= (uint32_t)((_poseSpeed * 1e6f) * poseSpeedSign);
	if(poseSpeedSign == -1){
		poseSpeedInt32 = poseSpeedInt32 | 0x80000000;
	}
	loadInt32 		= (uint32_t)((_load * 1e3f) * loadSign);
	if(loadSign == -1){
		loadInt32 = loadInt32 | 0x80000000;
	}
	strainInt32 	= (uint32_t)((_strain * 1e6f) * strainSign);
	if(strainSign == -1){
		strainInt32 = strainInt32 | 0x80000000;
	}

	_mbHoldreg->poseH16 		= (uint16_t)((poseInt32 >> 16)&0xFFFF);
	_mbHoldreg->poseL16 		= (uint16_t)((poseInt32)&0xFFFF);
	_mbHoldreg->poseSpeedH16 	= (uint16_t)((poseSpeedInt32 >> 16)&0xFFFF);
	_mbHoldreg->poseSpeedL16 	= (uint16_t)((poseSpeedInt32)&0xFFFF);
	_mbHoldreg->loadH16 		= (uint16_t)((loadInt32 >> 16)&0xFFFF);
	_mbHoldreg->loadL16 		= (uint16_t)((loadInt32)&0xFFFF);
	_mbHoldreg->strainH16 		= (uint16_t)((strainInt32 >> 16)&0xFFFF);
	_mbHoldreg->strainL16 		= (uint16_t)((strainInt32)&0xFFFF);

	//閸欐垿鈧浇顕Ч鍌涙殶閹诡喖瀵�
	_mb485->sendbuf[i++]=_mb485->manualBoxAddr;
	_mb485->sendbuf[i++]=0x10;//function code
	_mb485->sendbuf[i++]=(uint8_t)(regAdd>>8);
	_mb485->sendbuf[i++]=(uint8_t)regAdd;//鐎靛嫬鐡ㄩ崳銊ㄦ崳婵婀撮崸鈧敓锟�?
	_mb485->sendbuf[i++]=(uint8_t)(regLen>>8);//鐎靛嫬鐡ㄩ崳銊ュ礋閸忓啴鏆辨惔锕傜彯
	_mb485->sendbuf[i++]=(uint8_t)regLen;//鐎靛嫬鐡ㄩ崳銊ュ礋閸忓啴鏆辨惔锔跨秵
	_mb485->sendbuf[i++]=sizeByte;//鐎涙濡敓锟�?
	for(j=0;j<regLen;j++){
		_mb485->sendbuf[i++]=(uint8_t)(*(p+j+1)>>8)&0xFF;
		_mb485->sendbuf[i++]=(uint8_t)(*(p+j+1))&0xFF;
	}

	crc = CRC16(_mb485->sendbuf,i);    //鐠侊紕鐣荤憰浣界箲閸ョ偞鏆熼幑顔炬畱CRC
	_mb485->sendbuf[i++]=(uint8_t)(crc>>8);//CRC16閿燂拷?
	_mb485->sendbuf[i++]=(uint8_t)crc;//CRC16閿燂拷?
	//閸欐垿鈧焦鏆熼幑顔煎瘶 
	//HAL_UART_Transmit_DMA(&huart2,_mb485->sendbuf,i);
	HAL_UART_Transmit(&huart2, _mb485->sendbuf, i,2);//
}

/**
 * @brief send the request of writing multi holdregs
 * @param[in out] _mb485
 * @param[out] _mbHoldreg
 * @param[in] _pose
 * @param[in] _poseSpeed
 * @param[in] _load
 * @param[in] _strain
 */
void manualBox485Func10Request(manualBox485_t *_mb485,manualBoxHoldreg_t *_mbHoldreg,
		const float _pose,const float _poseSpeed,const float _load,const float _strain){
	uint16_t regAdd,regLen,crc;//鐎靛嫬鐡ㄩ崳銊ㄦ崳婵婀撮崸鈧�,鐎靛嫬鐡ㄩ崳銊ュ礋閸忓啴鏆辨惔锔肩礉閺佺増宓侀敍瀹憆c閺嶏繝鐛欓敓锟�?
	uint32_t poseInt32,poseSpeedInt32,loadInt32,strainInt32;
	uint8_t sizeByte,i=0,j=0;//鐎涙濡敓锟�?
	int32_t poseSign,poseSpeedSign,loadSign,strainSign;
	uint16_t* p=(uint16_t*)_mbHoldreg;//閹稿洤鎮滄穱婵囧瘮鐎靛嫬鐡ㄩ崳銊х波閺嬪嫪缍嬫＃鏍ф勾閸р偓
	regAdd = MB485_regAdd;//鐠囪褰囩€靛嫬鐡ㄩ崳銊ㄦ崳婵婀撮崸鈧�
	regLen = MB485_regLen;//鐎靛嫬鐡ㄩ崳銊ュ礋閸忓啴鏆遍敓锟�?
	sizeByte = regLen*2;//鐎涙濡敓锟�?

	poseSign = sign(_pose);
	poseSpeedSign = sign(_poseSpeed);
	loadSign = sign(_load);
	strainSign = sign(_strain);

	poseInt32 		= (uint32_t)((_pose * 1e6f) * poseSign);
	if(poseSign == -1){
		poseInt32 = poseInt32 | 0x80000000;
	}
	poseSpeedInt32 	= (uint32_t)((_poseSpeed * 1e6f) * poseSpeedSign);
	if(poseSpeedSign == -1){
		poseSpeedInt32 = poseSpeedInt32 | 0x80000000;
	}
	loadInt32 		= (uint32_t)((_load * 1e3f) * loadSign);
	if(loadSign == -1){
		loadInt32 = loadInt32 | 0x80000000;
	}
	strainInt32 	= (uint32_t)((_strain * 1e6f) * strainSign);
	if(strainSign == -1){
		strainInt32 = strainInt32 | 0x80000000;
	}

	_mbHoldreg->poseH16 		= (uint16_t)((poseInt32 >> 16)&0xFFFF);
	_mbHoldreg->poseL16 		= (uint16_t)((poseInt32)&0xFFFF);
	_mbHoldreg->poseSpeedH16 	= (uint16_t)((poseSpeedInt32 >> 16)&0xFFFF);
	_mbHoldreg->poseSpeedL16 	= (uint16_t)((poseSpeedInt32)&0xFFFF);
	_mbHoldreg->loadH16 		= (uint16_t)((loadInt32 >> 16)&0xFFFF);
	_mbHoldreg->loadL16 		= (uint16_t)((loadInt32)&0xFFFF);
	_mbHoldreg->strainH16 		= (uint16_t)((strainInt32 >> 16)&0xFFFF);
	_mbHoldreg->strainL16 		= (uint16_t)((strainInt32)&0xFFFF);

	//閸欐垿鈧浇顕Ч鍌涙殶閹诡喖瀵�
	_mb485->sendbuf[i++]=_mb485->manualBoxAddr;
	_mb485->sendbuf[i++]=0x10;//function code
	_mb485->sendbuf[i++]=(uint8_t)(regAdd>>8);
	_mb485->sendbuf[i++]=(uint8_t)regAdd;//鐎靛嫬鐡ㄩ崳銊ㄦ崳婵婀撮崸鈧敓锟�?
	_mb485->sendbuf[i++]=(uint8_t)(regLen>>8);//鐎靛嫬鐡ㄩ崳銊ュ礋閸忓啴鏆辨惔锕傜彯
	_mb485->sendbuf[i++]=(uint8_t)regLen;//鐎靛嫬鐡ㄩ崳銊ュ礋閸忓啴鏆辨惔锔跨秵
	_mb485->sendbuf[i++]=sizeByte;//鐎涙濡敓锟�?
	for(j=0;j<regLen;j++){
		_mb485->sendbuf[i++]=(uint8_t)(*(p+j+1)>>8)&0xFF;
		_mb485->sendbuf[i++]=(uint8_t)(*(p+j+1))&0xFF;
	}

	crc = CRC16(_mb485->sendbuf,i);    //鐠侊紕鐣荤憰浣界箲閸ョ偞鏆熼幑顔炬畱CRC
	_mb485->sendbuf[i++]=(uint8_t)(crc>>8);//CRC16閿燂拷?
	_mb485->sendbuf[i++]=(uint8_t)crc;//CRC16閿燂拷?
	//閸欐垿鈧焦鏆熼幑顔煎瘶 
	//HAL_UART_Transmit_DMA(&huart2,_mb485->sendbuf,i);
	HAL_UART_Transmit(&huart2, _mb485->sendbuf, i,2);//
}

/**
 * @brief receive the response of writing multi holdregs
 * @param[in out] _mb485
 */
manualBox485_e manualBox485Func10ResponseCallBack(manualBox485_t *_mb485){
	uint16_t regAdd,regLen;//鐎靛嫬鐡ㄩ崳銊ㄦ崳婵婀撮崸鈧�,鐎靛嫬鐡ㄩ崳銊ュ礋閸忓啴鏆辨惔锔肩礉鐎涙濡弫甯礉閺佺増宓侀敍瀹憆c閺嶏繝鐛欓敓锟�?
	// uint8_t j=0;
	regAdd = (_mb485->rcbuf[2]<<8)+_mb485->rcbuf[3];//鐠囪褰囩€靛嫬鐡ㄩ崳銊ㄦ崳婵婀撮崸鈧�
	regLen = (_mb485->rcbuf[4]<<8)+_mb485->rcbuf[5];//鐎靛嫬鐡ㄩ崳銊ュ礋閸忓啴鏆遍敓锟�?
	if(regAdd != MB485_regAdd || regLen != MB485_regLen){
		printf("manual box func10 request err!");
		return MB485_errFunc10;
	} 
	return MB485_noErr;
}


/**
 * @brief receive the request of writing single coil
 * @param[in out] _mb485
 * @param[out] _mbCoil
 */
manualBox485_e manualBox485Func05Response(manualBox485_t *_mb485,manualBoxCoil_t *_mbCoil){
	uint16_t regAddr,coilState,crc;//鐎靛嫬鐡ㄩ崳銊ㄦ崳婵婀撮崸鈧敍宀€鍤庨崷鍫㈠Ц閿燂拷?crc閺嶏繝鐛欓敓锟�?
	uint8_t i=0;
	bool state;
	regAddr = (_mb485->rcbuf[2]<<8)+_mb485->rcbuf[3];//鐠囪褰囩€靛嫬鐡ㄩ崳銊ㄦ崳婵婀撮崸鈧�,閸楀磭鍤庨崷鍫濇勾閸р偓
	coilState = (_mb485->rcbuf[4]<<8)+_mb485->rcbuf[5];//鐠囪褰囩痪鍨箑閻樿鎷�?
	state = (coilState == 0xFF00 ? true:false);//鐠佸墽鐤嗙€电懓绨插鏇″壖閻樿鎷�?
	//printf("regAddr:%d,state:%d\r\n",regAddr,state);
	manualBoxCoilUpdate((uint8_t)regAddr,state);
	//閸欐垿鈧礁娲栨惔鏃€鏆熼幑顔煎瘶
	_mb485->sendbuf[i++]=_mb485->mainBoardAddr;//ID閸欏嚖绱伴崣鎴︹偓浣规拱閺堥缚顔曟径鍥ф勾閸р偓
	_mb485->sendbuf[i++]=0x05;//閸旂喕鍏橀敓锟�?
	_mb485->sendbuf[i++]=(uint8_t)(regAddr>>8);//缁惧灝婀€閸︽澘娼冮敓锟�?
	_mb485->sendbuf[i++]=(uint8_t)regAddr;//缁惧灝婀€閸︽澘娼冮敓锟�?
	_mb485->sendbuf[i++]=(uint8_t)(coilState>>8);//缁惧灝婀€閻樿埖鈧線鐝�
	_mb485->sendbuf[i++]=(uint8_t)coilState;//缁惧灝婀€閻樿埖鈧椒缍�
	crc = CRC16(_mb485->sendbuf,i);    //鐠侊紕鐣荤憰浣界箲閸ョ偞鏆熼幑顔炬畱CRC
	_mb485->sendbuf[i++]=(uint8_t)(crc>>8);//CRC16閿燂拷?
	_mb485->sendbuf[i++]=(uint8_t)crc;//CRC16閿燂拷?

	//閸欐垿鈧焦鏆熼幑顔煎瘶 
	HAL_UART_Transmit(&huart2,_mb485->sendbuf,i,1);
	return MB485_noErr;
}


 

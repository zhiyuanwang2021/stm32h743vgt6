/*
鏁版嵁甯у垎涓?涓儴鍒嗭細璧峰鏍囧織浣?甯уご锛屽姛鑳界爜锛屾湁鏁堟暟鎹暱搴︼紝鏈夋晥杞借嵎鏁版嵁锛孋RC鏍￠獙鐮併€?
鏁版嵁甯х粨鏋勫涓嬭〃鎵€绀猴細
											Name	甯уご		鍔熻兘鐮?	鏁版嵁闀垮害		鏈夋晥杞借嵎鏁版嵁Payload		鏍￠獙鐮?
											Size	2Byte		1Byte			2Byte					N Byte					2 Bytes
								澶囨敞锛氳秴杩?瀛楄妭闀垮害鐨勬暟鎹兘浠ラ珮浣嶅湪鍓嶃€佷綆浣嶅湪鍚庛€?
											(1)璧峰鏍囧織浣?甯уご锛圚eader锛?
											鏁版嵁甯т互鈥?xAA 0xAA鈥濅綔涓鸿捣濮嬫爣蹇椾綅锛?
											(2)鍔熻兘鐮侊紙FunCode锛?
											涓嶅悓鍔熻兘鐨勬暟鎹紝鏈夋晥杞借嵎鏁版嵁鐨勫畾涔変笉鍚岋紝閫氳繃鍔熻兘鐮佹潵鍖哄垎銆傚姛鑳界爜瑙侀檮浠?銆?
											(3)鏁版嵁闀垮害锛圠ength锛?
											瀹氫箟杩欎釜鏁版嵁鍖呬腑鏈夋晥杞借嵎鏁版嵁鐨勫瓧鑺傛暟銆?
											(4)鏈夋晥杞借嵎鏁版嵁锛圥ayload锛?
											涓嶅悓鍔熻兘鐨勬暟鎹紝鏈夋晥杞借嵎鏁版嵁鐨勫畾涔変笉鍚屻€傝瑙侀檮浠?銆?
											(5)甯у熬锛圱ail锛?
											甯у熬鍥哄畾浣嶁€?x55 0x55鈥濅綔涓虹粨鏉熸爣蹇椾綅銆?
											
*/

#include "EthProtocol.h"
#include "usart.h"
#include "myFifo.h"
#include "elog.h"

//#define fifo_debug_rev

#define TAIL_CHECK_NUM1 0x55
#define TAIL_CHECK_NUM2 0x55

/*
		涓婁綅鏈虹粰涓嬩綅鏈哄彂閫佹暟鎹鐞?
		鍗充笅浣嶆満鏀?
		鏁版嵁甯т互鈥?xAA 0xAA鈥濅綔涓鸿捣濮嬫爣蹇椾綅
*/
#define REV_CHECK_NUM1 0xAA 
#define REV_CHECK_NUM2 0xAA
uint8_t buf_eth[DATA_BUF_SIZE];

uint8_t UTC_revbuf_process(UTC_ETH* EthStruct)
{
	uint16_t i=0,j=0,k=0;
	uint8_t buf_last=0,buf_now=0,i_last=0;
	uint8_t* p;
	uint16_t DataLen=0;//甯ф湁鏁堣浇鑽锋暟鎹暱搴?
	p = EthStruct->Revbuf;
	memset(buf_eth,0,sizeof(buf_eth));//娓呯┖buf
	//鎷嗗寘
	//log_i("size %d",EthStruct->revsize);
	for(i=0;i<EthStruct->revsize;i++)
	{
			buf_now = p[i];
			i_last = i;
			if( buf_last == REV_CHECK_NUM1 && buf_now == REV_CHECK_NUM2 ) //鎵惧埌甯уご
			{
					j=0;
					buf_eth[j++]= REV_CHECK_NUM1;
					buf_eth[j++]= REV_CHECK_NUM2;
					buf_eth[j++] = p[++i];//鍔熻兘鐮?
					buf_eth[j++] = p[++i];//鏁版嵁闀垮害H
					buf_eth[j++] = p[++i];//鏁版嵁闀垮害L
				  DataLen = buf_eth[j-2]*256+buf_eth[j-1];
				  for(k=0;k<DataLen;k++)
					{
						buf_eth[j++] = p[++i];//鏈夋晥杞借嵎鏁版嵁 Payload
					}
					// 璇诲彇鏁版嵁甯у熬
					buf_eth[j++] = p[++i];//甯у熬H
					buf_eth[j++] = p[++i];//甯у熬L						
					if(buf_eth[j-2] == TAIL_CHECK_NUM1 && buf_eth[j-1] == TAIL_CHECK_NUM2)//甯у熬姝ｇ‘
					{
#ifdef fifo_debug_rev
						printf("DataLen:%d\r\n",DataLen);
						printf("FuncNum:0x%2x\r\n",FuncNum);
						printf("j:%d\r\n",j);
						printf("buf_eth:");
						for(k=0;k<j-2;k++)
						printf("%2x ",buf_eth[k]);
						printf("\r\n");
#endif
						buf_now	= buf_eth[j-1];	
						//澶勭悊姝ゅ抚buf_eth  鍘嬪叆寰幆闃熷垪
						rev_ring_fifo_push(j-2,buf_eth);
#ifdef fifo_debug_rev1
							for(k=0;k<REV_FIFO_SIZE;k++)
									{
										printf("Fifo%d  ",k);
											for(l=0;l<50;l++)
											printf("%2x",rev_fifo_data[k][l]);
										printf("\r\n");
									}
									printf("\r\n \r\n");
						
#endif
					}
					else//甯у熬涓嶆纭紝缁х画瀵绘壘涓嬩竴甯э紝浠庝笂涓€娆℃壘鍒?xAA 0xAA 鍐嶉噸鏂板紑濮嬪鎵?
					{
						i = i_last;
						buf_now	= p[i];	
					}
			}
			buf_last = buf_now;
	}
		return 0;
}

/*
		涓嬩綅鏈虹粰涓婁綅鏈哄彂閫佹暟鎹鐞?
		鍗充笅浣嶆満鍙?
		鏁版嵁甯т互鈥?xAA 0xAA鈥濅綔涓鸿捣濮嬫爣蹇椾綅
*/
#define SEND_CHECK_NUM1 0xAA 
#define SEND_CHECK_NUM2 0xAA
uint8_t UTC_sendbuf_process(uint8_t* Data,UTC_ETH* EthStruct)
{
	uint16_t i=0,j=0,DataLen=0;
	uint8_t ret=0;
	uint8_t* p;
	p = EthStruct->Sendbuf;
	memset(buf_eth,0,sizeof(buf_eth));//娓呯┖buf
	//甯уご
	buf_eth[j++] = SEND_CHECK_NUM1;
	buf_eth[j++] = SEND_CHECK_NUM2;
	//鍔熻兘鐮?
	buf_eth[j++]  = Data[0];
	//鏁版嵁闀垮害
	buf_eth[j++]  = Data[1];
	buf_eth[j++]  = Data[2];
	DataLen = (Data[1]<<8)+ Data[2];
	//鏈夋晥杞借嵎鏁版嵁Payload
	for(i=0;i<DataLen;i++)
	buf_eth[j++]  = Data[i+3];
	
	if((EthStruct->sendsize+5+DataLen) <= DATA_BUF_SIZE)//瓒呰繃缂撳啿鍖猴紝鍒欐甯ф暟鎹涪澶?
	{
		if(p[EthStruct->sendsize] != 0x00)//闃叉鏁版嵁瑕嗙洊
			EthStruct->sendsize ++;
		
		for(i=0;i<j;i++)
		p[EthStruct->sendsize++]=buf_eth[i];
		
		p[EthStruct->sendsize++] = TAIL_CHECK_NUM1;
		p[EthStruct->sendsize++] = TAIL_CHECK_NUM2;
		
		//璁板緱娓呯┖Data鐩稿叧绌洪棿 浠ラ槻涓嬫缁х画鍙戦€佸悓鏍风殑鍊?
		ret = 0;
	}
	else
	{
			ret = 1;//缂撳啿鍖烘弧锛屽垯姝ゅ抚鏁版嵁涓㈠け  鏍规嵁ret鍒ゆ柇鏄惁闇€瑕佷笅涓€鍛ㄦ湡閲嶅彂
	}
	return ret;
}

utcSetOnPosMsg_t utcSetOnPosMsg={
	.Control = POS_MODE,
	.Position = 0,
	.DControl = POS_MODE,
	.Destination = 0,
	.Reached = 0,
	.Time = 0,
	.DoPError = 0,
	.usTAN = 0,
	.timeoutAllowed = 10.0f,
};
utcSetOnTPosMsg_t utcSetOnTPosMsg={
	.Control = POS_MODE,
	.Position = 0,
	.DControl = POS_MODE,
	.Destination = 0,
	.Reached = 0,
	.Time = 0,
	.DoPError = 0,
	.usTAN = 0,
};
utcSetOnLPosMsg_t utcSetOnLPosMsg={
	.Control = POS_MODE,
	.Position = 0,
	.DControl = POS_MODE,
	.Destination = 0,
	.Reached = 0,
	.Time = 0,
	.DoPError = 0,
	.usTAN = 0,
};
utcSetOnSftMsg_t utcSetOnSftMsg;
utcSetOnRuntimeError_t utcSetOnRuntimeError;
utcSetOnSystemMsg_t utcSetOnSystemMsg;
utcSetOnDebugMsg_t utcSetOnDebugMsg;

/**
 * @brief Update the system message content and set the corresponding semaphore.
 * @param _msgContent A pointer to the message content to be copied.
 * @param _utcSetOnSystemMsg A pointer to the system message structure.
 */
void utcSetOnSystemMsgUpdate(char* _msgContent,utcSetOnSystemMsg_t* _utcSetOnSystemMsg){
	if (_msgContent == NULL || _utcSetOnSystemMsg == NULL) {
        return; 
    }
	strncpy(_utcSetOnSystemMsg->Text, _msgContent, SYSTEM_MSG_TEXT_LEN);
	mySemaphore.utcSetOn.SetOnSystem = 1;
}

/**
 * @brief Update the debug message content and set the corresponding semaphore.
 * @param _msgContent A pointer to the message content to be copied.
 * @param _utcSetOnDebugMsg A pointer to the debug message structure.
 */
void utcSetOnDebugMsgUpdate(char* _msgContent,utcSetOnDebugMsg_t* _utcSetOnDebugMsg){
	if (_msgContent == NULL || _utcSetOnDebugMsg == NULL) {
        return; 
    }
	strncpy(_utcSetOnDebugMsg->Text, _msgContent, DEBUG_MSG_TEXT_LEN);
	mySemaphore.utcSetOn.SetOnDebug = 1;
}









#include "communicate.h"
#include "gParameter.h"
#include "posGenerator.h"
#include "Servo_driver.h"
#include "usart.h"
#include <string.h>
#include "freertos.h"
#include "adc_scale.h"
#include "DI.h"
#include "DO.h"
#include "in_out.h"
#include "CS5552.h"
#include "pid.h"
#include <elog.h>

//#define CurrentData_debug

#define FLOAT_TYPE 0
#define UINT32_TYPE 1
#define UINT16_TYPE 2
#define UINT8_TYPE 3
#define INT32_TYPE 4
#define CHAR_TYPE 5


void LINK_ACK_Process(void);
void DOPE_OPEN_CLOSE_LINK_Process(void);
void DOPEON_OFF_Process(void);
void DOPESETCTRL_Process(void);
void DOPETRANSMITDATA_Process(void);
void DOPEDEFAULTACC_Process(void);
void DOPERDNOMINALACCSPEED_Process(void);
void DOPESETNOMINALACCSPEED_Process(void);
void DOPESETDATATRANSMISSIONRATE_Process(void);
void DOPERDCTRLPARAMETER_Process(void);
void DOPEDEADBANDCTRL_Process(void);
void DOPESPEEDLIMIT_Process(void);
void DOPEINTGR_Process(void);
void DOPEDESTWND_Process(void);
void DOPESFT_Process(void);
void DOPEPOSPID_Process(void);
void DOPERDPOSPID_Process(void);
void DOPEWRPOSPID_Process(void);
void DOPESPEEDPID_Process(void);
void DOPERDSPEEDPID_Process(void);
void DOPEWRSPEEDPID_Process(void);
void DOPEFEEDFORWARD_Process(void);
void DOPERDFEEDFORWARD_Process(void);
void DOPEWRFEEDFORWARD_Process(void);
void SETTARE_Process(void);
void WRSERVOPARA_Process(void);
void RDSERVOPARA_Process(void);
void DOPESETSENSORCORRECTION_Process(void);

void DOPEMOVE_Process(void);
void DOPEMOVE_A_Process(void);
void DOPEPOS_Process(void);
void DOPEPOS_A_Process(void);
void DOPEHALT_Process(void);
void DOPEHALT_A_Process(void);
void DOPESHALT_Process(void);
void DOPETRIG_Process(void);
void DOPETRIG_A_Process(void);
void DOPEPOSEXT_Process(void);
void DOPEPOSEXT_A_Process(void);
void DOPEBLOCKHEADER_Process(void);
void DOPEBLOCKEXECUTE_Process(void);
void DOPECYCLE_Process(void);
void DOPEDYNCYCLE_Process(void);
void RDOUTPUTPARA_Process(void);//璇诲彇杈撳嚭鍙傛暟
void WROUTPUTPARA_Process(OUTPUTPARA *x);//鍐欏叆杈撳嚭鍙傛暟
void RDETHPARA_Process(void);//璇讳互澶綉鍙傛暟
void WRETHPARA_Process(void);//涓嬪彂浠ュお缃戝弬鏁?
void RDSYSPARA_Process(void);//璇荤郴缁熷弬鏁?
void WRSYSPARA_Process(void);//涓嬪彂绯荤粺鍙傛暟
void DOPESETOPENLOOPCOMMAND_Process(void);//鎵嬪姩娴嬭瘯鎸囦护

//浼犳劅鍣ㄧ浉鍏抽€氫俊澶勭悊鍑芥暟
void RDSENSORDATA_Process(void);//璇诲彇浼犳劅鍣ㄦ暟鎹?
void WRSENSORDATA_Process(void);//鍐欏叆浼犳劅鍣ㄦ暟鎹?
void RDSENSORBIGDEFORMATIONDATA_Process(void);//璇诲彇澶у彉褰紶鎰熷櫒鏁版嵁
void WRSENSORBIGDEFORMATIONDATA_Process(void);//鍐欏叆澶у彉褰紶鎰熷櫒鏁版嵁
//void SENSORZEROSET_Process(void);
//void SETSENSORCALIBRATION_Process(void);
//void WRITESENSORPARS_Process(void);

void combinedMoveDWND_Process(void);
void sendUtcSetOn(void);

void Variable_Type_Conversion(uint8_t type,uint32_t x,uint8_t* buf,uint16_t* bufnum);

uint8_t buf_com[500]={0};
/* Fun_183 */
void ETH_Communicate_Process(void)
{
	uint16_t _lpusTAN;
	extern osMessageQId lpusTANQueueHandle;
	//lpusTAN push in send ring fifo push
	while(uxQueueMessagesWaiting(lpusTANQueueHandle) != 0){
		if(xQueueReceive(lpusTANQueueHandle,&(_lpusTAN),0) == pdPASS){
			buf_com[0] = _lpusTAN >> 8;
			buf_com[1] = _lpusTAN;
			send_ring_fifo_push(2,buf_com,LPUSTAN_REPORT);
            /* xRxedStructure now contains a copy of xMessage. */
        }
	}
	//send hldr of utcSetOn
	sendUtcSetOn();


	//鎺ユ敹鏁版嵁鍖呭鐞?
	while(rev_fifo.state != FIFO_EMPTY)
	{
		rev_ring_fifo_pop(buf_data_save);//鎺ユ敹闃熷垪pop涓€鏉℃暟鎹抚
		while(mySemaphore.paraconfig.write == 1 || mySemaphore.paraconfig.set == 1){
			osDelay(1);
		}	
			if(buf_data_save[0] == 0xAA && buf_data_save[1] == 0xAA) //鍐嶆纭甯уご 骞?闃叉闃熷垪绌哄拰閿欒甯ф儏鍐?
			{ 
				//in assembly move instruction
				//case 1:CMD_DWND,record all movement command (Max load value is 10) 
				if((combinedMove.blockHeader.ModeFlags & BHM_CMD_DWND) == BHM_CMD_DWND
					&& buf_data_save[2] >= DOPEPOS && buf_data_save[2] <= DOPEBLOCKEXECUTE){
						combinedMoveDWND_Process();
				}else{
					switch(buf_data_save[2])//鍔熻兘鐮?
					{
						case LINK_ACK://瀵?xA0鍔熻兘鐮佸簲绛旓紝鐢ㄤ簬鍒ゆ柇缃戠粶鐘舵€?
								LINK_ACK_Process();
							break;
						case DOPE_OPEN_CLOSE_LINK://寤虹珛鎴栨柇寮€杩炴帴 鏁版嵁闀垮害 00 01  鏁版嵁鍐呭 01   //01 On   00 Off 
								DOPE_OPEN_CLOSE_LINK_Process();
							break;
						case DOPEON_OFF://婵€娲?鍋滅敤鎺у埗鍣?鏁版嵁闀垮害 00 01  鏁版嵁鍐呭 01   //01 On   00 Off 
								DOPEON_OFF_Process();
							break;
						case DOPESETCTRL://浣胯兘/涓嶄娇鑳介棴鐜帶鍒?鏁版嵁闀垮害 00 01  鏁版嵁鍐呭 01   //01 On   00 Off 
								DOPESETCTRL_Process();
							break; 
						case DOPETRANSMITDATA://婵€娲?鍋滅敤娴嬮噺鏁版嵁浼犺緭
								DOPETRANSMITDATA_Process();
							break;
						case DOPERDNOMINALACCSPEED://0x07 璇诲彇浣嶇疆鐢熸垚鍣ㄦ爣绉板€?*****
								DOPERDNOMINALACCSPEED_Process();
						break;
						case DOPESETNOMINALACCSPEED://0x08 璇诲彇浣嶇疆鐢熸垚鍣ㄦ爣绉板€?*****
								DOPESETNOMINALACCSPEED_Process();
						break;
						case DOPERDCTRLPARAMETER://璇诲彇鎵€鏈夐棴鐜帶鍒跺弬鏁?**
								DOPERDCTRLPARAMETER_Process();
							break;
						case DOPEDEADBANDCTRL://璁惧畾璇樊姝诲尯鎺у埗鍙傛暟*
								DOPEDEADBANDCTRL_Process();
							break;
						case DOPEDEFAULTACC://璁惧畾榛樿鍔犻€熷害*
								DOPEDEFAULTACC_Process();
							break;
						case DOPESPEEDLIMIT://璁惧畾鏈€澶ч檺瀹氶€熷害*
								DOPESPEEDLIMIT_Process();
							break;
						case DOPESETDATATRANSMISSIONRATE://璁惧畾鏁版嵁鍙戦€佸懆鏈?
								DOPESETDATATRANSMISSIONRATE_Process();
							break;
						case DOPEINTGR://璁惧畾浼犳劅鍣ㄦ护娉㈡椂闂?
								DOPEINTGR_Process();
							break;
						case DOPEDESTWND://璁惧畾鐩爣璇樊/鏃堕棿绐楀彛*
								DOPEDESTWND_Process();
							break;
						case DOPESFT://璁惧畾杞檺浣?
								DOPESFT_Process();
							break;
						case DOPEPOSPID://0x11  璁剧疆浣嶇疆闂幆鎺у埗鍙傛暟****** 
								DOPEPOSPID_Process();
							break;
						case DOPERDPOSPID://璇诲彇浣嶇疆闂幆鎺у埗鍙傛暟*
								DOPERDPOSPID_Process();
							break;
						case DOPEWRPOSPID://鍐欏叆浣嶇疆闂幆鎺у埗鍙傛暟*
								DOPEWRPOSPID_Process();
							break;
						case DOPESPEEDPID://0x14  璁剧疆閫熷害闂幆鎺у埗鍙傛暟****** 
								DOPESPEEDPID_Process();
						break;
						case DOPERDSPEEDPID://0x15  璇诲彇閫熷害闂幆鎺у埗鍙傛暟****** 
								DOPERDSPEEDPID_Process();
						break;
						case DOPEWRSPEEDPID://0x16  鍐欏叆閫熷害闂幆鎺у埗鍙傛暟****** 
								DOPEWRSPEEDPID_Process();
						break;
						case DOPEFEEDFORWARD://0x17 璁剧疆閫熷害鍓嶉鍙傛暟****** 
								DOPEFEEDFORWARD_Process();
						break;
						case DOPERDFEEDFORWARD://璇诲彇閫熷害鍓嶉鍙傛暟*
								DOPERDFEEDFORWARD_Process(); 
							break;
						case DOPEWRFEEDFORWARD://鍐欏叆閫熷害鍓嶉鍙傛暟*
								DOPEWRFEEDFORWARD_Process();
							break;
						case DOPESETSENSORCORRECTION://璁惧畾浼犳劅鍣ㄦ牎姝ｈ〃鎸囦护
								//DOPESETSENSORCORRECTION_Process();
							break;
						case RDOUTPUTPARA://璇诲彇杈撳嚭鍙傛暟
								RDOUTPUTPARA_Process();
							break;
						case WROUTPUTPARA://鍐欏叆杈撳嚭鍙傛暟
								WROUTPUTPARA_Process(&OutputPara_combuf);
							break;
						case RDETHPARA://璇讳互澶綉鍙傛暟*
								RDETHPARA_Process();
							break;
						case WRETHPARA://涓嬪彂浠ュお缃戝弬鏁?涓嬩綅鏈烘敹鍒板悗閲嶅惎*
								WRETHPARA_Process();
							break;				
						case RDSYSPARA://璇荤郴缁熷弬鏁?
								RDSYSPARA_Process();
							break;
						case WRSYSPARA://涓嬪彂绯荤粺鍙傛暟
								WRSYSPARA_Process();
							break;				
						case DOPEWRDVERSION://璇荤増鏈俊鎭?
							/* Fun_18335 */
							/* Fun_18415 */
							break;
						case SETTARE://娴嬮噺閫氶亾鏁板€兼竻闆?
							SETTARE_Process();
							break;
						case WRSERVOPARA://鍐欏叆浼烘湇鍙傛暟
							WRSERVOPARA_Process();
							break;
						case RDSERVOPARA://璇讳己鏈嶅弬鏁?
							RDSERVOPARA_Process();
							break;
						case DOPESETOPENLOOPCOMMAND://寮€鐜祴璇曟寚浠?
							DOPESETOPENLOOPCOMMAND_Process();
							break;				
						case DOPEMOVE://浠ラ粯璁ゅ姞閫熷害涓婁笅绉诲姩
							DOPEMOVE_Process();
							break;
						case DOPEMOVE_A://浠ユ寚瀹氬姞閫熷害涓婁笅绉诲姩
							DOPEMOVE_A_Process();
							break;				
						case DOPEPOS://浠ラ粯璁ゅ姞閫熷害绉诲姩鑷虫寚瀹氫綅缃?0x23
							DOPEPOS_Process();
							break;
						case DOPEPOS_A://浠ユ寚瀹氬姞/鍑忛€熷害绉诲姩鑷虫寚瀹氫綅缃?0x24
							DOPEPOS_A_Process();
							break;
						case DOPEPOSEXT:
						/*鍦ㄦ寚瀹氱殑鎺у埗妯″紡涓嬶紝浠ラ粯璁ゅ姞閫熷害绉诲姩鑷虫寚瀹氫綅缃紝鑻ヨ揪鍒伴檺浣嶄綅缃紝鍒欎互榛樿鍑忛€熷害鍋滄锛?
						鑻ュ嵆灏嗚揪鍒版寚瀹氫綅缃紝鍒欐牴鎹弬鏁扳€淒estinationMode鈥濈‘瀹氬埌杈炬寚瀹氫綅缃殑鎺у埗鏂瑰紡浠ュ強鍒拌揪鎸囧畾浣嶇疆鍚庣殑鎿嶄綔*/
							DOPEPOSEXT_Process();
						break;
						case DOPEPOSEXT_A:
						/* Fun_18343 */
						/*鍚孌oPEPosExt锛屼絾鍦ㄥ姞鍑忛€熻繃绋嬩腑锛岄渶瑕佹寚瀹氬姞閫熷害锛岄檺浣嶅噺閫熷害锛岄潬杩戠洰鏍囦綅缃悗鐨勫噺閫熷害*/
							DOPEPOSEXT_A_Process();
						break;
						case DOPEHALT://浠ラ粯璁ゅ噺閫熷害浠庣幇Command閫熷害鍑忛€熻嚦0
							DOPEHALT_Process();
						break;
						case DOPEHALT_A://浠ユ寚瀹氬噺閫熷害浠庣幇Command閫熷害鍑忛€熻嚦0
							DOPEHALT_A_Process();
						break;
						case DOPESHALT://浠ユ爣绉板噺閫熷害浠庡綋鍓嶄綅缃噺閫熻嚦0锛屾病鏈塎oveCtrl閫夋嫨锛屽彧鏈変綅缃ā寮?
							DOPESHALT_Process();
						break;
						case DOPETRIG:
						/*浠ユ寚瀹氶€熷害绉诲姩鍒伴檺鍒朵綅缃?
						濡傛灉鍒拌揪瑙﹀彂鏉′欢浣嶇疆锛屽皢鍙戦€佷竴鏉℃秷鎭紙濡傛灉鍦ㄧ粍鍚堢Щ鍔ㄥ簭鍒椾腑浣跨敤锛屽垯婵€娲讳笅涓€涓懡浠わ級*/
							DOPETRIG_Process();
						break;
						case DOPETRIG_A:
						/*鍚孌oPETrig锛屼絾鍦ㄥ姞鍑忛€熻繃绋嬩腑锛岄渶瑕佹寚瀹氬姞閫熷害锛岄檺浣嶅噺閫熷害*/
							DOPETRIG_A_Process();
						break;
						case DOPEBLOCKHEADER://鎵€鏈夌畝鍗曞懡浠ょ粍鍚堢Щ鍔ㄧ殑鎸囦护搴忓垪澶?
							DOPEBLOCKHEADER_Process();
						break;
						case DOPEBLOCKEXECUTE://鎵€鏈夌畝鍗曞懡浠ょ粍鍚堢Щ鍔ㄧ殑鎵ц鍛戒护(寮€濮?缁撴潫)
							DOPEBLOCKEXECUTE_Process();
						break;
						case DOPECYCLE://鐢ㄦ枩鍧″嚱鏁版墽琛屽懆鏈熻繍鍔?
							DOPECYCLE_Process();
						break;
						case DOPEDYNCYCLE://鍔ㄦ€佸惊鐜寚浠?**
							DOPEDYNCYCLE_Process();
						break;
						//浼犳劅鍣ㄧ浉鍏?
						case RDSENSORDATA://璇诲彇浼犳劅鍣ㄦ暟鎹?
							RDSENSORDATA_Process();
						break;
						case WRSENSORDATA://鍐欏叆浼犳劅鍣ㄦ暟鎹?
							WRSENSORDATA_Process();
						break;
						case RDSENSORBIGDEFORMATIONDATA://璇诲彇浼犳劅鍣ㄦ暟鎹?
							RDSENSORBIGDEFORMATIONDATA_Process();
						break;
						case WRSENSORBIGDEFORMATIONDATA://鍐欏叆浼犳劅鍣ㄦ暟鎹?
							WRSENSORBIGDEFORMATIONDATA_Process();
						break;
						default:
							log_e("undefine function code:0x%2X!",buf_data_save[2]);
						break;
					
					}
				}
			}
			
	memset(buf_data_save,0,sizeof(buf_data_save));
	}
}

/**
 *@brief recording combinedMove command in CMD_DWND Modeflag 
 */
void combinedMoveDWND_Process(void){
	if(combinedMove.counter < COMBINED_MOVE_MAX_LEN){
		switch(buf_data_save[2])//鍔熻兘鐮?
		{
			case DOPEPOS://浠ラ粯璁ゅ姞閫熷害绉诲姩鑷虫寚瀹氫綅缃?0x23
				DOPEPOS_Process();
				break;
			case DOPEPOS_A://浠ユ寚瀹氬姞/鍑忛€熷害绉诲姩鑷虫寚瀹氫綅缃?0x24
				DOPEPOS_A_Process();
				break;
			case DOPEPOSEXT:
			/*鍦ㄦ寚瀹氱殑鎺у埗妯″紡涓嬶紝浠ラ粯璁ゅ姞閫熷害绉诲姩鑷虫寚瀹氫綅缃紝鑻ヨ揪鍒伴檺浣嶄綅缃紝鍒欎互榛樿鍑忛€熷害鍋滄锛?
			鑻ュ嵆灏嗚揪鍒版寚瀹氫綅缃紝鍒欐牴鎹弬鏁扳€淒estinationMode鈥濈‘瀹氬埌杈炬寚瀹氫綅缃殑鎺у埗鏂瑰紡浠ュ強鍒拌揪鎸囧畾浣嶇疆鍚庣殑鎿嶄綔*/
				DOPEPOSEXT_Process();
			break;
			case DOPEPOSEXT_A:
			/* Fun_18343 */
			/*鍚孌oPEPosExt锛屼絾鍦ㄥ姞鍑忛€熻繃绋嬩腑锛岄渶瑕佹寚瀹氬姞閫熷害锛岄檺浣嶅噺閫熷害锛岄潬杩戠洰鏍囦綅缃悗鐨勫噺閫熷害*/
			break;
			case DOPEHALT://浠ラ粯璁ゅ噺閫熷害浠庣幇Command閫熷害鍑忛€熻嚦0
				DOPEHALT_Process();
			break;
			case DOPEHALT_A://浠ユ寚瀹氬噺閫熷害浠庣幇Command閫熷害鍑忛€熻嚦0
				DOPEHALT_A_Process();
			break;
			case DOPESHALT://浠ユ爣绉板噺閫熷害浠庡綋鍓嶄綅缃噺閫熻嚦0锛屾病鏈塎oveCtrl閫夋嫨锛屽彧鏈変綅缃ā寮?
				DOPESHALT_Process();
			break;
			case DOPETRIG:
			/*浠ユ寚瀹氶€熷害绉诲姩鍒伴檺鍒朵綅缃?
			濡傛灉鍒拌揪瑙﹀彂鏉′欢浣嶇疆锛屽皢鍙戦€佷竴鏉℃秷鎭紙濡傛灉鍦ㄧ粍鍚堢Щ鍔ㄥ簭鍒椾腑浣跨敤锛屽垯婵€娲讳笅涓€涓懡浠わ級*/
				DOPETRIG_Process();
			break;
			case DOPETRIG_A:
			/*鍚孌oPETrig锛屼絾鍦ㄥ姞鍑忛€熻繃绋嬩腑锛岄渶瑕佹寚瀹氬姞閫熷害锛岄檺浣嶅噺閫熷害*/
				DOPETRIG_A_Process();
			break;
			case DOPEBLOCKHEADER://鎵€鏈夌畝鍗曞懡浠ょ粍鍚堢Щ鍔ㄧ殑鎸囦护搴忓垪澶?
				DOPEBLOCKHEADER_Process();
			break;
			case DOPEBLOCKEXECUTE://鎵€鏈夌畝鍗曞懡浠ょ粍鍚堢Щ鍔ㄧ殑鎵ц鍛戒护(寮€濮?缁撴潫)
				DOPEBLOCKEXECUTE_Process();
			break;
			default:
				log_e("undefine function code:0x%2X!",buf_data_save[2]);
			break;
		}
	}else{
		combinedMove.counter = COMBINED_MOVE_MAX_LEN;
		switch(buf_data_save[2])//鍔熻兘鐮?
		{
			case DOPEBLOCKHEADER://鎵€鏈夌畝鍗曞懡浠ょ粍鍚堢Щ鍔ㄧ殑鎸囦护搴忓垪澶?
				DOPEBLOCKHEADER_Process();
			break;
			case DOPEBLOCKEXECUTE://鎵€鏈夌畝鍗曞懡浠ょ粍鍚堢Щ鍔ㄧ殑鎵ц鍛戒护(寮€濮?缁撴潫)
				DOPEBLOCKEXECUTE_Process();
			break;
			default:
				log_e("movement command overflow > COMBINED_MOVE_MAX_LEN");
			break;
		}
	}

}

void LINK_ACK_Process(void)
{
	stateFlag.PC_ack_state = 1;//鏈塧ck淇″彿锛岀綉缁滆繛鎺ユ甯?
	//log_i("stateFlag.PC_ack_state:%d",stateFlag.PC_ack_state);
}

/* Fun_1831 */
void DOPE_OPEN_CLOSE_LINK_Process(void)
{
	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	if(buf_data_save[5] == Device_Open)
	{commBuf.OpenDevice_state = Device_Open;	buf_com[0]=Device_Open;}
	else
	{commBuf.OpenDevice_state = Device_Off;	buf_com[0]=Device_Off;}
	
	//瑕佸姞鍏ヤ换鍔￠€氱煡妫€娴?鏄惁鍑轰簬绱ф€ョ姸鎬?濡傛灉鍑轰簬绱ф€ョ姸鎬?闇€瑕佺瓑寰呯郴缁熷畬鎴愮揣鎬ュ鐞嗗悗鍐嶅洖娑堟伅 11.11
	/* Fun_1841 */
	send_ring_fifo_push(1,buf_com,DOPEOPENDEVICEID);//涓嬩綅鏈轰細杩斿洖0x80鍔熻兘鐮?
	memset(buf_com,0,sizeof(buf_com));
}

/* Fun_1832 */
void DOPEON_OFF_Process(void)
{
	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	if(buf_data_save[5] == DoPE_On)
	{commBuf.DoPE_state = DoPE_On;	buf_com[0]=DoPE_On;}
	else
	{commBuf.DoPE_state = DoPE_Off;	buf_com[0]=DoPE_Off;}
	AL.command = DOPEON_OFF;
	/* Fun_1842 */
	send_ring_fifo_push(1,buf_com,UTCON_OFF);//涓嬩綅鏈轰細杩斿洖0x81鍔熻兘鐮?
	memset(buf_com,0,sizeof(buf_com));
}

/* Fun_1833 */
void DOPESETCTRL_Process(void)
{
	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	if(buf_data_save[5] == CtrlTRUE)
	{commBuf.DoPESetCtrl = CtrlTRUE;buf_com[0]=CtrlTRUE;}
	else
	{commBuf.DoPESetCtrl = CtrlFALSE;buf_com[0]=CtrlFALSE;}
	commBuf.lpusTAN = (buf_data_save[6]<<8) +(buf_data_save[7]);//lpusTAN 鎸囦护缂栧彿
	AL.command = DOPESETCTRL;
	/* Fun_1843 */
	send_ring_fifo_push(1,buf_com,UTCSETCTRL);//涓嬩綅鏈轰細杩斿洖0x83鍔熻兘鐮?
	memset(buf_com,0,sizeof(buf_com));
}

/* Fun_1834 */
void DOPETRANSMITDATA_Process(void)
{
	if(buf_data_save[5] == CtrlTRUE)
	{stateFlag.DoPETransmitData = TRUE;}
	else
	{stateFlag.DoPETransmitData = FALSE;}
	AL.command = DOPETRANSMITDATA;
	commBuf.lpusTAN = (buf_data_save[10]<<8) +(buf_data_save[11]);//lpusTAN 鎸囦护缂栧彿
}
/*
DOPERDCTRLPARAMETER_Process 
璇诲彇闂幆鎺у埗鍙傛暟鎸囦护,瀵瑰簲涓嬩綅鏈鸿繑鍥炴寚浠?x87
*/
void DOPERDCTRLPARAMETER_Process_LoadingData(uint8_t movectrl)
{
	switch(movectrl)
	{
		case POS_MODE:
			posAllCtrlPara.PosPid.P = 1;
			posAllCtrlPara.PosPid.I = 1;
			posAllCtrlPara.PosPid.D = 1;
			posAllCtrlPara.SpeedPid.P = 1;
			posAllCtrlPara.SpeedPid.I = 1;
			posAllCtrlPara.SpeedPid.D = 1;
			posAllCtrlPara.Ffd.SpeedFFP = 1;
			posAllCtrlPara.Ffd.PosDelay = 1;
			posAllCtrlPara.DefaultAcc = 1;
			posAllCtrlPara.Speed = 1;
			posAllCtrlPara.Deviation = 1;
			posAllCtrlPara.DevReaction = 1;
			posAllCtrlPara.Wnd.WndSize = 1;
			posAllCtrlPara.Wnd.WndTime = 1;
			posAllCtrlPara.Sft.UpperSft = 1;
			posAllCtrlPara.Sft.LowerSft = 1;
			posAllCtrlPara.Sft.Reaction = 1;
			posAllCtrlPara.MinAcceleration = 1;
			posAllCtrlPara.MaxAcceleration = 1;
			posAllCtrlPara.MinDeceleration = 1;
			posAllCtrlPara.MaxDeceleration = 1;
			posAllCtrlPara.MinSpeed = 1;
			posAllCtrlPara.MaxSpeed = 1;
			posAllCtrlPara.Deadband.Deadband = 1;
			posAllCtrlPara.Deadband.PercentP = 1;
			posAllCtrlPara.Dither.DitherFrequency = 1;
			posAllCtrlPara.Dither.DitherAmplitude = 1;
		break;
		case LOAD_MODE:
			loadAllCtrlPara.PosPid.P = 2;
			loadAllCtrlPara.PosPid.I = 2;
			loadAllCtrlPara.PosPid.D = 2;
			loadAllCtrlPara.SpeedPid.P = 2;
			loadAllCtrlPara.SpeedPid.I = 2;
			loadAllCtrlPara.SpeedPid.D = 2;
			loadAllCtrlPara.Ffd.SpeedFFP = 2;
			loadAllCtrlPara.Ffd.PosDelay = 2;
			loadAllCtrlPara.DefaultAcc = 2;
			loadAllCtrlPara.Speed = 2;
			loadAllCtrlPara.Deviation = 2;
			loadAllCtrlPara.DevReaction = 2;
			loadAllCtrlPara.Wnd.WndSize = 2;
			loadAllCtrlPara.Wnd.WndTime = 2;
			loadAllCtrlPara.Sft.UpperSft = 2;
			loadAllCtrlPara.Sft.LowerSft = 2;
			loadAllCtrlPara.Sft.Reaction = 2;
			loadAllCtrlPara.MinAcceleration = 2;
			loadAllCtrlPara.MaxAcceleration = 2;
			loadAllCtrlPara.MinDeceleration = 2;
			loadAllCtrlPara.MaxDeceleration = 2;
			loadAllCtrlPara.MinSpeed = 2;
			loadAllCtrlPara.MaxSpeed = 2;
			loadAllCtrlPara.Deadband.Deadband = 2;
			loadAllCtrlPara.Deadband.PercentP = 2;
			loadAllCtrlPara.Dither.DitherFrequency = 2;
			loadAllCtrlPara.Dither.DitherAmplitude = 2;
		break;
		case EXTEN_MODE:
			extAllCtrlPara.PosPid.P = 3;
			extAllCtrlPara.PosPid.I = 3;
			extAllCtrlPara.PosPid.D = 3;
			extAllCtrlPara.SpeedPid.P = 3;
			extAllCtrlPara.SpeedPid.I = 3;
			extAllCtrlPara.SpeedPid.D = 3;
			extAllCtrlPara.Ffd.SpeedFFP = 3;
			extAllCtrlPara.Ffd.PosDelay = 3;
			extAllCtrlPara.DefaultAcc = 3;
			extAllCtrlPara.Speed = 3;
			extAllCtrlPara.Deviation = 3;
			extAllCtrlPara.DevReaction = 3;
			extAllCtrlPara.Wnd.WndSize = 3;
			extAllCtrlPara.Wnd.WndTime = 3;
			extAllCtrlPara.Sft.UpperSft = 3;
			extAllCtrlPara.Sft.LowerSft = 3;
			extAllCtrlPara.Sft.Reaction = 3;
			extAllCtrlPara.MinAcceleration = 3;
			extAllCtrlPara.MaxAcceleration = 3;
			extAllCtrlPara.MinDeceleration = 3;
			extAllCtrlPara.MaxDeceleration = 3;
			extAllCtrlPara.MinSpeed = 3;
			extAllCtrlPara.MaxSpeed = 3;
			extAllCtrlPara.Deadband.Deadband = 3;
			extAllCtrlPara.Deadband.PercentP = 3;
			extAllCtrlPara.Dither.DitherFrequency = 3;
			extAllCtrlPara.Dither.DitherAmplitude = 3;
		break;
		default:break;
	}
	
}
void DOPERDCTRLPARAMETER_Process_Conversion(ALLCTRLPARA x,uint16_t* i)
{
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.PosPid.P),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.PosPid.I),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.PosPid.D),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.SpeedPid.P),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.SpeedPid.I),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.SpeedPid.D),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.Ffd.SpeedFFP),buf_com,i);		
	Variable_Type_Conversion(UINT32_TYPE,*(uint32_t*)(&x.Ffd.PosDelay),buf_com,i);		
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.DefaultAcc),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.Speed),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.Deviation),buf_com,i);
	Variable_Type_Conversion(UINT8_TYPE,*(uint32_t*)(&x.DevReaction),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.Wnd.WndSize),buf_com,i);
	Variable_Type_Conversion(UINT32_TYPE,*(uint32_t*)(&x.Wnd.WndTime),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.Sft.UpperSft),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.Sft.LowerSft),buf_com,i);
	Variable_Type_Conversion(UINT8_TYPE,*(uint32_t*)(&x.Sft.Reaction),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.MinAcceleration),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.MaxAcceleration),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.MinDeceleration),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.MaxDeceleration),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.MinSpeed),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.MaxSpeed),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.Deadband.Deadband),buf_com,i);
	Variable_Type_Conversion(UINT8_TYPE,*(uint32_t*)(&x.Deadband.PercentP),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.Dither.DitherFrequency),buf_com,i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&x.Dither.DitherAmplitude),buf_com,i);	
}

/* Fun_1837 */
void DOPERDCTRLPARAMETER_Process(void)//璇诲彇闂幆鎺у埗鍙傛暟鎸囦护,瀵瑰簲涓嬩綅鏈鸿繑鍥炴寚浠?
{
	uint8_t movectrl=0;
	uint16_t i=0;
	movectrl = buf_data_save[5];
	/* Fun_1845 */
	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	Variable_Type_Conversion(UINT8_TYPE,*(uint32_t*)(&movectrl),buf_com,&i);
	//DOPERDCTRLPARAMETER_Process_LoadingData(movectrl);//鏇存柊鎵€鏈夎涓婂彂鐨勬帶鍒跺弬鏁?
	switch(movectrl)
	{
		case POS_MODE:
				DOPERDCTRLPARAMETER_Process_Conversion(posAllCtrlPara,&i);			
		break;
		case LOAD_MODE: 
				DOPERDCTRLPARAMETER_Process_Conversion(loadAllCtrlPara,&i);
		break;
		case EXTEN_MODE:	
				DOPERDCTRLPARAMETER_Process_Conversion(extAllCtrlPara,&i);
		break;
		default:break;
	}
	send_ring_fifo_push(i,buf_com,0x84);//涓嬩綅鏈轰細杩斿洖0x84鍔熻兘鐮?
}

/**
 * @note Fun_1838
DOPEDEADBANDCTRL_Process 
璁惧畾璇樊姝诲尯鎺у埗鍙傛暟鎸囦护澶勭悊鍑芥暟
*/
void DOPEDEADBANDCTRL_Process(void)//璁惧畾璇樊姝诲尯鎺у埗鍙傛暟鎸囦护澶勭悊鍑芥暟
{
	uint8_t movectrl=0,temp1=0;
	uint32_t temp=0;
	commBuf.Command = DOPEDEADBANDCTRL;
	movectrl = buf_data_save[5]; 
	temp = (buf_data_save[6]<<24) + (buf_data_save[7]<<16) +(buf_data_save[8]<<8) +(buf_data_save[9]);
	temp1 = buf_data_save[10];
	switch(movectrl)
	{
		case POS_MODE:	commBuf.Pos.Deadband.Deadband = *((float*)(&temp));
					commBuf.Pos.Deadband.PercentP = temp1;
		printf("Pos Deadband:%f PercentP:%d%%\r\n",commBuf.Pos.Deadband.Deadband,commBuf.Pos.Deadband.PercentP);
		break;
		case LOAD_MODE:	commBuf.Load.Deadband.Deadband = *((float*)(&temp));
					commBuf.Load.Deadband.PercentP = temp1;
		printf("Load Deadband:%f PercentP:%d%%\r\n",commBuf.Load.Deadband.Deadband,commBuf.Load.Deadband.PercentP);
		break;
		case EXTEN_MODE:	commBuf.Ext.Deadband.Deadband = *((float*)(&temp));
					commBuf.Ext.Deadband.PercentP = temp1;
		printf("Ext Deadband:%f PercentP:%d%%\r\n",commBuf.Ext.Deadband.Deadband,commBuf.Ext.Deadband.PercentP);
		break;
		default:break;
	}
	mySemaphore.paraconfig.write = 1;
}

/**
 * @note Fun_1839
DOPEDEADBANDCTRL_Process 
璁剧疆榛樿鍔犻€熷害
鍚屾椂鍑忛€熷害涔熶細琚缃垚-Acc
*/
void DOPEDEFAULTACC_Process(void)//璁剧疆榛樿鍔犻€熷害
{
	uint32_t temp=0;
	commBuf.MoveCtrl = buf_data_save[5]; 
	temp = (buf_data_save[6]<<24) + (buf_data_save[7]<<16) +(buf_data_save[8]<<8) +(buf_data_save[9]);
	switch(commBuf.MoveCtrl)
	{
		case POS_MODE:	commBuf.Pos.DefaultAcc = *((float*)(&temp));
					commBuf.Pos.DefaultDec = -commBuf.Pos.DefaultAcc;
					//leave it there for now ,and then change it to loopctrl
					posAllCtrlPara.DefaultAcc = commBuf.Pos.DefaultAcc;
					mySemaphore.mram.write_defalutacc_pos = 1;
		printf("Pos DefaultAcc:%f Dec:%f\r\n",commBuf.Pos.DefaultAcc,commBuf.Pos.DefaultDec);
		break;
		case LOAD_MODE:	commBuf.Load.DefaultAcc = *((float*)(&temp));
					commBuf.Load.DefaultDec = -commBuf.Load.DefaultAcc;
					//leave it there for now ,and then change it to loopctrl
					loadAllCtrlPara.DefaultAcc = commBuf.Load.DefaultAcc;
					mySemaphore.mram.write_defalutacc_load = 1;
		printf("Load DefaultAcc:%f Dec:%f\r\n",commBuf.Load.DefaultAcc,commBuf.Load.DefaultDec);
		break;
		case EXTEN_MODE:	commBuf.Ext.DefaultAcc = *((float*)(&temp));
					commBuf.Ext.DefaultDec = -commBuf.Ext.DefaultAcc;
					//leave it there for now ,and then change it to loopctrl
					extAllCtrlPara.DefaultAcc = commBuf.Ext.DefaultAcc;
					mySemaphore.mram.write_defalutacc_ext = 1;
		printf("Ext DefaultAcc:%f Dec:%f\r\n",commBuf.Ext.DefaultAcc,commBuf.Ext.DefaultDec);
		break;
		default:break;
	}
	mySemaphore.paraconfig.write = 1;
}

/* Fun_18310 */
void DOPESPEEDLIMIT_Process(void)//璁剧疆榛樿鏈€澶ч檺瀹氶€熷害
{
	uint32_t temp=0;
	commBuf.MoveCtrl = buf_data_save[5]; 
	temp = (buf_data_save[6]<<24) + (buf_data_save[7]<<16) +(buf_data_save[8]<<8) +(buf_data_save[9]);
	switch(commBuf.MoveCtrl)
	{
		case POS_MODE:	commBuf.Pos.DefaultSpeed = *((float*)(&temp));
					//leave it there for now ,and then change it to loopctrl
					posAllCtrlPara.Speed = commBuf.Pos.DefaultSpeed;
					mySemaphore.mram.write_speed_pos = 1;
		printf("Pos DefaultSpeed:%f\r\n",commBuf.Pos.DefaultSpeed);
		break;
		case LOAD_MODE:	commBuf.Load.DefaultSpeed = *((float*)(&temp));
					//leave it there for now ,and then change it to loopctrl
					loadAllCtrlPara.Speed = commBuf.Load.DefaultSpeed;
					mySemaphore.mram.write_speed_load = 1;
		printf("Load DefaultSpeed:%f\r\n",commBuf.Load.DefaultSpeed);
		break;
		case EXTEN_MODE:	commBuf.Ext.DefaultSpeed = *((float*)(&temp));
					//leave it there for now ,and then change it to loopctrl
					extAllCtrlPara.Speed = commBuf.Ext.DefaultSpeed;
					mySemaphore.mram.write_speed_ext = 1;
		printf("Ext DefaultSpeed:%f\r\n",commBuf.Ext.DefaultSpeed);
		break;
		default:break;
	}
	mySemaphore.paraconfig.write = 1;
}

/* Fun_18311 */
void DOPESETDATATRANSMISSIONRATE_Process(void)//璁惧畾鏁版嵁鍙戦€佸懆鏈?
{
	commBuf.SysPara.TransmitDataPeriod = (buf_data_save[5]<<24) + (buf_data_save[6]<<16) +(buf_data_save[7]<<8) +(buf_data_save[8]);
	//leave it there for now ,and then change it to loopctrl
	mySemaphore.comm.transperiodUpdate = 1;
	commBuf.lpusTAN = (buf_data_save[9]<<8) +(buf_data_save[10]);//lpusTAN 鎸囦护缂栧彿
	printf("TransmitDataPeriod:%d\r\n",commBuf.SysPara.TransmitDataPeriod);
}

/* Fun_18312 */
void DOPEINTGR_Process(void)//璁惧畾浼犳劅鍣ㄦ护娉㈡椂闂?
{
	commBuf.Command = DOPEINTGR;
	sensorConnector = (sensorConnector_e)buf_data_save[5]; //传感器编号
	senDataCommbuf[sensorConnector].sensorIntgr = (buf_data_save[6]<<24) + (buf_data_save[7]<<16) +(buf_data_save[8]<<8) +(buf_data_save[9]);//积分时间
	senDataCommbuf[sensorConnector].sensorIntgr /=1000;
	commBuf.lpusTAN = (buf_data_save[10]<<8) +(buf_data_save[11]);//lpusTAN 鎸囦护缂栧彿
	mySemaphore.paraconfig.write = 1;
	log_d("SensorNo:%d,Intgr:%d\r\n",sensorConnector,senDataCommbuf[sensorConnector].sensorIntgr);
}

/* Fun_18313 */
void DOPEDESTWND_Process(void)//璁惧畾鐩爣璇樊/鏃堕棿绐楀彛
{
	uint32_t temp1=0,temp2=0;
	commBuf.Command = DOPEDESTWND;
	log_d("recive DOPEDESTWND command");
	commBuf.MoveCtrl = buf_data_save[5]; 
	temp1 = (buf_data_save[6]<<24) + (buf_data_save[7]<<16) +(buf_data_save[8]<<8) +(buf_data_save[9]);
	temp2 = (buf_data_save[10]<<24) + (buf_data_save[11]<<16) +(buf_data_save[12]<<8) +(buf_data_save[13]);
	switch(commBuf.MoveCtrl)
	{
		case POS_MODE:	
		commBuf.Pos.Wnd.WndSize = *((float*)(&temp1));
		commBuf.Pos.Wnd.WndTime = temp2;
		break;
		case LOAD_MODE:	
		commBuf.Load.Wnd.WndSize = *((float*)(&temp1));
		commBuf.Load.Wnd.WndTime = temp2;
		break;
		case EXTEN_MODE:	
		commBuf.Ext.Wnd.WndSize = *((float*)(&temp1));
		commBuf.Ext.Wnd.WndTime = temp2;
		break;
		default:break;
	}
	mySemaphore.paraconfig.write = 1;
}

/* Fun_18314 */
void DOPESFT_Process(void)//璁惧畾杞檺浣?
{
	uint8_t reaction=0;
	uint32_t temp1=0,temp2=0;
	commBuf.Command = DOPESFT;
	commBuf.MoveCtrl = buf_data_save[5]; 
	temp1 = (buf_data_save[6]<<24) + (buf_data_save[7]<<16) +(buf_data_save[8]<<8) +(buf_data_save[9]);
	temp2 = (buf_data_save[10]<<24) + (buf_data_save[11]<<16) +(buf_data_save[12]<<8) +(buf_data_save[13]);
	reaction = buf_data_save[14];
	switch(commBuf.MoveCtrl)
	{
		case POS_MODE:	commBuf.Pos.Sft.UpperSft = *((float*)(&temp1));
					commBuf.Pos.Sft.LowerSft = *((float*)(&temp2));
					commBuf.Pos.Sft.Reaction = reaction;
					break;
		case LOAD_MODE:	commBuf.Load.Sft.UpperSft = *((float*)(&temp1));
					commBuf.Load.Sft.LowerSft = *((float*)(&temp2));
					commBuf.Load.Sft.Reaction = reaction;
					break;
		case EXTEN_MODE:	commBuf.Ext.Sft.UpperSft = *((float*)(&temp1));
					commBuf.Ext.Sft.LowerSft = *((float*)(&temp2));
					commBuf.Ext.Sft.Reaction = reaction;
					break;
		default:break;
	}
	mySemaphore.paraconfig.write = 1;
}

/* Fun_18316 */
void DOPERDPOSPID_Process(void)//璇诲彇浣嶇疆闂幆鎺у埗鍙傛暟
{
	uint8_t movectrl=0,highpressure=0;
	uint16_t i=0;
	movectrl = buf_data_save[5];
	highpressure = buf_data_save[6];
	/* Fun_1846 */
	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	Variable_Type_Conversion(UINT8_TYPE,*(uint32_t*)(&movectrl),buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,*(uint32_t*)(&highpressure),buf_com,&i);
	switch(movectrl)
	{
		case POS_MODE:	
					if(highpressure == 0)
					{
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&posAllCtrlPara.PosPid.P),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&posAllCtrlPara.PosPid.I),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&posAllCtrlPara.PosPid.D),buf_com,&i);
					}
					else
					{
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&posAllCtrlPara.PosPid_Hp.P),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&posAllCtrlPara.PosPid_Hp.I),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&posAllCtrlPara.PosPid_Hp.D),buf_com,&i);
					}
		break;
		case LOAD_MODE: 
					if(highpressure == 0)
					{
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&loadAllCtrlPara.PosPid.P),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&loadAllCtrlPara.PosPid.I),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&loadAllCtrlPara.PosPid.D),buf_com,&i);
					}
					else
					{
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&loadAllCtrlPara.PosPid_Hp.P),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&loadAllCtrlPara.PosPid_Hp.I),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&loadAllCtrlPara.PosPid_Hp.D),buf_com,&i);
					}
		break;
		case EXTEN_MODE:	
					if(highpressure == 0)
					{
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&extAllCtrlPara.PosPid.P),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&extAllCtrlPara.PosPid.I),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&extAllCtrlPara.PosPid.D),buf_com,&i);
					}
					else
					{
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&extAllCtrlPara.PosPid_Hp.P),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&extAllCtrlPara.PosPid_Hp.I),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&extAllCtrlPara.PosPid_Hp.D),buf_com,&i);
					}
		break;
		default:break;
	}
	log_i("DOPERDPOSPID MODE:%d,P:%f I:%f D:%f\r\n",movectrl,extAllCtrlPara.PosPid.P,extAllCtrlPara.PosPid.I,extAllCtrlPara.PosPid.D);
	send_ring_fifo_push(i,buf_com,0x85);//涓嬩綅鏈轰細杩斿洖0x85鍔熻兘鐮?
}

/* Fun_18317 */
void DOPEWRPOSPID_Process(void)//鍐欏叆闂幆鎺у埗鍙傛暟
{
	uint32_t temp=0; 
	uint16_t i=5;
	float P,I,D;
	commBuf.Command = DOPEWRPOSPID;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.MoveCtrl = buf_data_save[i++];
	commBuf.HighPressure = buf_data_save[i++];
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	P = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	I = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	D = *((float*)(&temp));temp=0;
	
	switch(commBuf.MoveCtrl)
	{
		case POS_MODE:
			if(commBuf.HighPressure == 0){
			commBuf.Pos.Pid.P = P;
			commBuf.Pos.Pid.I = I;
			commBuf.Pos.Pid.D = D;
			}
			else{
			commBuf.Pos.Pid_Hp.P = P;
			commBuf.Pos.Pid_Hp.I = I;
			commBuf.Pos.Pid_Hp.D = D;
			}
			break;
		case LOAD_MODE:
			if(commBuf.HighPressure == 0){
			commBuf.Load.Pid.P = P;
			commBuf.Load.Pid.I = I;
			commBuf.Load.Pid.D = D;
			}
			else{
			commBuf.Load.Pid_Hp.P = P;
			commBuf.Load.Pid_Hp.I = I;
			commBuf.Load.Pid_Hp.D = D;
			}
			break;
		case EXTEN_MODE:
			if(commBuf.HighPressure == 0){
			commBuf.Ext.Pid.P = P;
			commBuf.Ext.Pid.I = I;
			commBuf.Ext.Pid.D = D;
			}
			else{
			commBuf.Ext.Pid_Hp.P = P;
			commBuf.Ext.Pid_Hp.I = I;
			commBuf.Ext.Pid_Hp.D = D;
			}
			break;
		default:break;
	}
	mySemaphore.paraconfig.write = 1;
	
	printf("Command=0x%2x DOPEWRPOSPID\t\r\n",commBuf.Command);
	printf("MoveCtrl=%d\t\r\n",commBuf.MoveCtrl);
	printf("Highpressure=%d\t\r\n",commBuf.HighPressure);
	printf("P=%f\t\r\n",P);
	printf("I=%f\t\r\n",I);
	printf("D=%f\t\r\n",D);
}

/* Fun_18322 */
void DOPERDFEEDFORWARD_Process(void)//璇诲彇閫熷害鍓嶉鍙傛暟
{
	uint8_t movectrl=0,highpressure=0;
	uint16_t i=0;
	movectrl = buf_data_save[5];
	highpressure = buf_data_save[6];
	/* Fun_1849 */
	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	Variable_Type_Conversion(UINT8_TYPE,*(uint32_t*)(&movectrl),buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,*(uint32_t*)(&highpressure),buf_com,&i);
	switch(movectrl)
	{
		case POS_MODE:	
					if(highpressure == 0)
					{
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&posAllCtrlPara.Ffd.SpeedFFP),buf_com,&i);
						Variable_Type_Conversion(UINT32_TYPE,*(uint32_t*)(&posAllCtrlPara.Ffd.PosDelay),buf_com,&i);
						printf("DOPERDFEEDFORWARD PosDelay:%d\r\n",posAllCtrlPara.Ffd.PosDelay);
					}
					else
					{
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&posAllCtrlPara.Ffd_Hp.SpeedFFP),buf_com,&i);
						Variable_Type_Conversion(UINT32_TYPE,*(uint32_t*)(&posAllCtrlPara.Ffd_Hp.PosDelay),buf_com,&i);
					}
		break;
		case LOAD_MODE: 
					if(highpressure == 0)
					{
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&loadAllCtrlPara.Ffd.SpeedFFP),buf_com,&i);
						Variable_Type_Conversion(UINT32_TYPE,*(uint32_t*)(&loadAllCtrlPara.Ffd.PosDelay),buf_com,&i);
						printf("DOPERDFEEDFORWARD PosDelay:%d\r\n",loadAllCtrlPara.Ffd.PosDelay);
					}
					else
					{
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&loadAllCtrlPara.Ffd_Hp.SpeedFFP),buf_com,&i);
						Variable_Type_Conversion(UINT32_TYPE,*(uint32_t*)(&loadAllCtrlPara.Ffd_Hp.PosDelay),buf_com,&i);
					}
		break;
		case EXTEN_MODE:	
					if(highpressure == 0)
					{
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&extAllCtrlPara.Ffd.SpeedFFP),buf_com,&i);
						Variable_Type_Conversion(UINT32_TYPE,*(uint32_t*)(&extAllCtrlPara.Ffd.PosDelay),buf_com,&i);
						printf("DOPERDFEEDFORWARD PosDelay:%d\r\n",extAllCtrlPara.Ffd.PosDelay);
					}
					else
					{
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&extAllCtrlPara.Ffd_Hp.SpeedFFP),buf_com,&i);
						Variable_Type_Conversion(UINT32_TYPE,*(uint32_t*)(&extAllCtrlPara.Ffd_Hp.PosDelay),buf_com,&i);
					}
		break;
		default:break;
	}
	send_ring_fifo_push(i,buf_com,0x87);//涓嬩綅鏈轰細杩斿洖0x89鍔熻兘鐮?
}

/* Fun_18323 */
void DOPEWRFEEDFORWARD_Process(void)//璁惧畾閫熷害鍓嶉鍙傛暟
{
	uint32_t tempFFP=0,tempDelay=0;
	commBuf.Command = DOPEWRFEEDFORWARD;
	commBuf.MoveCtrl = buf_data_save[5];
	commBuf.HighPressure = buf_data_save[6];
	tempFFP = (buf_data_save[7]<<24) + (buf_data_save[8]<<16) +(buf_data_save[9]<<8) +(buf_data_save[10]);
	tempDelay = (buf_data_save[11]<<24) + (buf_data_save[12]<<16) +(buf_data_save[13]<<8) +(buf_data_save[14]);
	switch(commBuf.MoveCtrl)
	{
		case POS_MODE:	
			commBuf.Pos.HighPressure = commBuf.HighPressure;
			if(commBuf.Pos.HighPressure == 0)
			{
			commBuf.Pos.Ffd.SpeedFFP = *((float*)(&tempFFP));
			commBuf.Pos.Ffd.PosDelay = tempDelay;
			}
			else
			{
			commBuf.Pos.Ffd_Hp.SpeedFFP = *((float*)(&tempFFP));
			commBuf.Pos.Ffd_Hp.PosDelay = tempDelay;
			}
		break;
		case LOAD_MODE:  
			commBuf.Load.HighPressure = commBuf.HighPressure;
			if(commBuf.Load.HighPressure == 0)
			{
			commBuf.Load.Ffd.SpeedFFP = *((float*)(&tempFFP));
			commBuf.Load.Ffd.PosDelay = tempDelay;
			}
			else
			{
			commBuf.Load.Ffd_Hp.SpeedFFP = *((float*)(&tempFFP));
			commBuf.Load.Ffd_Hp.PosDelay = tempDelay;
			}
		break;
		case EXTEN_MODE:	
			commBuf.Ext.HighPressure = commBuf.HighPressure;
			if(commBuf.Ext.HighPressure == 0)
			{
			commBuf.Ext.Ffd.SpeedFFP = *((float*)(&tempFFP));
			commBuf.Ext.Ffd.PosDelay = tempDelay;
			}
			else
			{
			commBuf.Ext.Ffd_Hp.SpeedFFP = *((float*)(&tempFFP));
			commBuf.Ext.Ffd_Hp.PosDelay = tempDelay;
			}
		break;
		default:break;
	}
	mySemaphore.paraconfig.write = 1;

	printf("Command: %2x movectrl:%d\r\n",commBuf.Command,commBuf.MoveCtrl);
	printf("SpeedFFP = %f\r\n",*((float*)(&tempFFP)));
	printf("PosDelay = %d\r\n",tempDelay);

}

/* Fun_18324 */
void DOPESETSENSORCORRECTION_Process(void)//璁惧畾浼犳劅鍣ㄦ牎姝ｈ〃鎸囦护
{
	uint32_t temp=0;
	uint16_t i=5,j=0;
	commBuf.sensorset.Correct.CalculatedSensor = buf_data_save[i++]; 
	commBuf.sensorset.Correct.SENSOR_CORR_MAX = buf_data_save[i++]; 
	commBuf.sensorset.Correct.CorrNo = buf_data_save[i++]; 
	for(j=0;j<commBuf.sensorset.Correct.CorrNo;j++)//瀛楽1鐨勬牎姝ｅ€?
	{
		temp += (buf_data_save[i++]<<24);
		temp += (buf_data_save[i++]<<16);
		temp += (buf_data_save[i++]<<8);
		temp += (buf_data_save[i++]);
		commBuf.sensorset.Correct.S1Correction[j] = *((float*)(&temp));
		temp=0;
	}
	for(j=0;j<commBuf.sensorset.Correct.CorrNo;j++)//瀛楽2鐨凙DC鐮佸€?
	{
		temp += (buf_data_save[i++]<<24);
		temp += (buf_data_save[i++]<<16);
		temp += (buf_data_save[i++]<<8);
		temp += (buf_data_save[i++]);
		commBuf.sensorset.Correct.S2Value[j] = temp;
		temp=0;
	}
	
	printf("CalculatedSensor=%d\r\n",commBuf.sensorset.Correct.CalculatedSensor);
	printf("SENSOR_CORR_MAX=%d\r\n",commBuf.sensorset.Correct.SENSOR_CORR_MAX);
	printf("CorrNo=%d\r\n",commBuf.sensorset.Correct.CorrNo);
	for(j=0;j<commBuf.sensorset.Correct.CorrNo;j++)//瀛楽1鐨勬牎姝ｅ€?
		printf("S1Correction[%d] = %.6f \tS1Correction[%d] = %d\r\n",
				j,commBuf.sensorset.Correct.S1Correction[j],
				j,commBuf.sensorset.Correct.S2Value[j]);
}

/* Fun_18337 */
void DOPESETOPENLOOPCOMMAND_Process(void)//鎵嬪姩娴嬭瘯鎸囦护
{
	uint32_t temp=0;
	uint8_t i=5;
	commBuf.Command = DOPESETOPENLOOPCOMMAND;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.openloopFrq = temp;temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.openloopAO = *((float*)(&temp));temp=0;
	commBuf.DO = buf_data_save[i++]; 
	
	if(commBuf.Command == AL.command)//鍒ゆ柇涓婃鎸囦护鍜屾湰娆℃寚浠ゆ槸鍚﹂噸澶?
		if(	memcmp(&commBuf,&commBufLast,sizeof(commBuf)) == 0)	//鍒ゆ柇鏁版嵁鍏蜂綋鍙傛暟鏄惁涓€鑷?
					return; //濡傛灉鎸囦护閲嶅鍒欒繑鍥?涓嶅仛鎿嶄綔
	

	printf("Command=0x%2x  DOPESETOPENLOOPCOMMAND\r\n",commBuf.Command);
	printf("openloopAO=%.2f\r\n",commBuf.openloopAO);
	printf("openloopFrq=%5d\r\n",commBuf.openloopFrq);
	printf("DO=%1x\r\n",commBuf.DO);

	commBuf.OpenLoopflag = 1;
}

/* Fun_18338 */
void DOPEMOVE_Process(void)//涓婁笅绉诲姩鎸囦护 榛樿鍔犻€熷害
{
	uint32_t temp=0;
	commBuf.Command = DOPEMOVE;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.Direction = buf_data_save[5]; 
	commBuf.MoveCtrl = buf_data_save[6]; 
	temp = (buf_data_save[7]<<24) + (buf_data_save[8]<<16) +(buf_data_save[9]<<8) +(buf_data_save[10]);
	commBuf.Speed = *((float*)(&temp));
	commBuf.lpusTAN = (buf_data_save[11]<<8) +(buf_data_save[12]);//lpusTAN 鎸囦护缂栧彿
	
	if(commBuf.Command == AL.command)//鍒ゆ柇涓婃鎸囦护鍜屾湰娆℃寚浠ゆ槸鍚﹂噸澶?
		if(	memcmp(&commBuf,&commBufLast,sizeof(commBuf)) == 0)	//鍒ゆ柇鏁版嵁鍏蜂綋鍙傛暟鏄惁涓€鑷?
					return; //濡傛灉鎸囦护閲嶅鍒欒繑鍥?涓嶅仛鎿嶄綔
	
	switch (commBuf.MoveCtrl)
	{
	case POS_MODE:
		commBuf.Acc = posAllCtrlPara.DefaultAcc;
		break;
	case LOAD_MODE:
		commBuf.Acc = loadAllCtrlPara.DefaultAcc;
		break;
	case EXTEN_MODE:
		commBuf.Acc = extAllCtrlPara.DefaultAcc;
		break;
	default:
		break;
	}
	commBuf.Dec = -commBuf.Acc;
	
	log_i("DOPEMOVE movectrl=%d speed=%f dir=%d defalutAcc=%f defaultDec=%f\r\n",commBuf.MoveCtrl,commBuf.Speed,commBuf.Direction,commBuf.Acc,commBuf.Dec);
	
	mySemaphore.move.pgInit = 1;

}

/* Fun_18339 */
void DOPEMOVE_A_Process(void)
{
	uint32_t temp=0;
	commBuf.Command = DOPEMOVE_A;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.Direction = buf_data_save[5]; 
	commBuf.MoveCtrl = buf_data_save[6]; 
	temp = (buf_data_save[7]<<24) + (buf_data_save[8]<<16) +(buf_data_save[9]<<8) +(buf_data_save[10]);
	commBuf.Acc = *((float*)(&temp));
	temp = (buf_data_save[11]<<24) + (buf_data_save[12]<<16) +(buf_data_save[13]<<8) +(buf_data_save[14]);
	commBuf.Speed = *((float*)(&temp));
	commBuf.lpusTAN = (buf_data_save[15]<<8) +(buf_data_save[16]);//lpusTAN 鎸囦护缂栧彿
	
	if(commBuf.Command == AL.command)//鍒ゆ柇涓婃鎸囦护鍜屾湰娆℃寚浠ゆ槸鍚﹂噸澶?
		if(	memcmp(&commBuf,&commBufLast,sizeof(commBuf)) == 0)	//鍒ゆ柇鏁版嵁鍏蜂綋鍙傛暟鏄惁涓€鑷?
					return; //濡傛灉鎸囦护閲嶅鍒欒繑鍥?涓嶅仛鎿嶄綔

	commBuf.Dec = -commBuf.Acc;

	
	
	printf("speed=%f\r\n",commBuf.Speed);
	printf("acc=%f\r\n",commBuf.Acc);
	printf("dir=%d\r\n",commBuf.Direction);
	
	mySemaphore.move.pgInit = 1;
}

/*
 *@note Fun_18340
Byte0			MoveCtrl 绉诲姩鎺у埗妯″紡	0x00 POS_MODE; 0x01 LOAD_MODE;0x02 Extension
Byte1-4		Speed    閫熷害	
Byte5-8		Destination
Byte9-10	lpusTAN 鎸囦护缂栧彿	
*/
void DOPEPOS_Process(void)//鏁版嵁闀垮害 00 0B  鏁版嵁鍐呭 01   //01 On   00 Off 
{
	uint32_t temp=0;
	commBuf.Command = DOPEPOS;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.MoveCtrl = buf_data_save[5]; 
	temp = (buf_data_save[6]<<24) + (buf_data_save[7]<<16) +(buf_data_save[8]<<8) +(buf_data_save[9]);
	commBuf.Speed = *((float*)(&temp));
	temp= (buf_data_save[10]<<24) + (buf_data_save[11]<<16) +(buf_data_save[12]<<8) +(buf_data_save[13]);
	commBuf.Destination =  *((float*)(&temp));
	commBuf.lpusTAN = (buf_data_save[14]<<8) +(buf_data_save[15]);//lpusTAN 鎸囦护缂栧彿
	
	if(commBuf.Command == AL.command)//鍒ゆ柇涓婃鎸囦护鍜屾湰娆℃寚浠ゆ槸鍚﹂噸澶?
		if(	memcmp(&commBuf,&commBufLast,sizeof(commBuf)) == 0)	//鍒ゆ柇鏁版嵁鍏蜂綋鍙傛暟鏄惁涓€鑷?
					return; //濡傛灉鎸囦护閲嶅鍒欒繑鍥?涓嶅仛鎿嶄綔
	
	switch (commBuf.MoveCtrl)
	{
	case POS_MODE:
		commBuf.Acc = posAllCtrlPara.DefaultAcc;
		commBuf.Dec = -posAllCtrlPara.DefaultAcc;
		if(posAllCtrlPara.Speed < commBuf.Speed) 
			commBuf.Speed = posAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
		break;
	case LOAD_MODE:
		commBuf.Acc = loadAllCtrlPara.DefaultAcc;
		commBuf.Dec = -loadAllCtrlPara.DefaultAcc;
		if(loadAllCtrlPara.Speed < commBuf.Speed) 
			commBuf.Speed = loadAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
		break;
	case EXTEN_MODE:
		commBuf.Acc = extAllCtrlPara.DefaultAcc;
		commBuf.Dec = -extAllCtrlPara.DefaultAcc;
		if(extAllCtrlPara.Speed < commBuf.Speed) 
			commBuf.Speed = extAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
		break;
	default:
		break;
	}
	log_i("speed=%f",commBuf.Speed);
	log_i("dest=%f",commBuf.Destination);
	log_i("acc=%f",commBuf.Acc);
	log_i("dec=%f",commBuf.Dec);
	
	if(combinedMove.blockHeader.Cycles >= 1 && (combinedMove.blockHeader.ModeFlags & BHM_CMD_DWND) == BHM_CMD_DWND){
		combinedMove.command[combinedMove.counter] = commBuf.Command;
		combinedMove.MoveCtrl[combinedMove.counter] = commBuf.MoveCtrl;
		combinedMove.Speed[combinedMove.counter] = commBuf.Speed;
		combinedMove.Acc[combinedMove.counter] = commBuf.Acc;
		combinedMove.Dec[combinedMove.counter] = commBuf.Dec;
		combinedMove.Destination[combinedMove.counter] = commBuf.Destination;
		combinedMove.lpusTAN[combinedMove.counter] = commBuf.lpusTAN;
		combinedMove.counter++;
	}else{
		mySemaphore.move.pgInit = 1;
	}
}

/* Fun_18341 */
void DOPEPOS_A_Process(void)//鏁版嵁闀垮害 00 10  鏁版嵁鍐呭
{
	uint32_t temp=0;
	commBuf.Command = DOPEPOS_A;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.MoveCtrl = buf_data_save[5]; 
	temp = (buf_data_save[6]<<24) + (buf_data_save[7]<<16) +(buf_data_save[8]<<8) +(buf_data_save[9]);
	commBuf.Acc = *((float*)(&temp));
	temp = (buf_data_save[10]<<24) + (buf_data_save[11]<<16) +(buf_data_save[12]<<8) +(buf_data_save[13]);
	commBuf.Speed = *((float*)(&temp));
	temp = (buf_data_save[14]<<24) + (buf_data_save[15]<<16) +(buf_data_save[16]<<8) +(buf_data_save[17]);
	commBuf.Dec =  - *((float*)(&temp));
	temp = (buf_data_save[18]<<24) + (buf_data_save[19]<<16) +(buf_data_save[20]<<8) +(buf_data_save[21]);
	commBuf.Destination =  *((float*)(&temp));
	commBuf.lpusTAN = (buf_data_save[22]<<8) +(buf_data_save[23]);//lpusTAN 鎸囦护缂栧彿
	
	if(commBuf.Command == AL.command)//鍒ゆ柇涓婃鎸囦护鍜屾湰娆℃寚浠ゆ槸鍚﹂噸澶?
		if(	memcmp(&commBuf,&commBufLast,sizeof(commBuf)) == 0)	//鍒ゆ柇鏁版嵁鍏蜂綋鍙傛暟鏄惁涓€鑷?
					return; //濡傛灉鎸囦护閲嶅鍒欒繑鍥?涓嶅仛鎿嶄綔
	
	switch (commBuf.MoveCtrl)
	{
	case POS_MODE:
		if(posAllCtrlPara.Speed < commBuf.Speed) 
		commBuf.Speed = posAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
		break;
	case LOAD_MODE:
		if(loadAllCtrlPara.Speed < commBuf.Speed) 
		commBuf.Speed = loadAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
		break;
	default:
		break;
	}
	
		printf("Acc=%f\r\n",commBuf.Acc);
		printf("Speed=%f\r\n",commBuf.Speed);
		printf("Dec=%f\r\n",commBuf.Dec);
	
	if(combinedMove.blockHeader.Cycles >= 1 && (combinedMove.blockHeader.ModeFlags & BHM_CMD_DWND) == BHM_CMD_DWND){
		combinedMove.command[combinedMove.counter] = commBuf.Command;
		combinedMove.MoveCtrl[combinedMove.counter] = commBuf.MoveCtrl;
		combinedMove.Speed[combinedMove.counter] = commBuf.Speed;
		combinedMove.Acc[combinedMove.counter] = commBuf.Acc;
		combinedMove.Dec[combinedMove.counter] = commBuf.Dec;
		combinedMove.Destination[combinedMove.counter] = commBuf.Destination;
		combinedMove.lpusTAN[combinedMove.counter] = commBuf.lpusTAN;
		combinedMove.counter++;
	}else{
		mySemaphore.move.pgInit = 1;
	}
}

/* Fun_18344 */
void DOPEHALT_Process(void)////鏁版嵁闀垮害 00 03  鏁版嵁鍐呭
{
	commBuf.Command = DOPEHALT;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.MoveCtrl = buf_data_save[5]; 
	commBuf.lpusTAN = (buf_data_save[6]<<8) +(buf_data_save[7]);//lpusTAN 鎸囦护缂栧彿
	
	// if(commBuf.Command == AL.command)//鍒ゆ柇涓婃鎸囦护鍜屾湰娆℃寚浠ゆ槸鍚﹂噸澶?
	// 	if(	memcmp(&commBuf,&commBufLast,sizeof(commBuf)) == 0)	//鍒ゆ柇鏁版嵁鍏蜂綋鍙傛暟鏄惁涓€鑷?
	// 				return; //濡傛灉鎸囦护閲嶅鍒欒繑鍥?涓嶅仛鎿嶄綔
	
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
	
	log_i("DOPEHALT movectrl=%d defaultAcc=%f defaultDec=%f",commBuf.MoveCtrl,commBuf.Acc,commBuf.Dec );
	if(combinedMove.blockHeader.Cycles >= 1 && (combinedMove.blockHeader.ModeFlags & BHM_CMD_DWND) == BHM_CMD_DWND){
		combinedMove.command[combinedMove.counter] = commBuf.Command;
		combinedMove.MoveCtrl[combinedMove.counter] = commBuf.MoveCtrl;
		combinedMove.Acc[combinedMove.counter] = commBuf.Acc;
		combinedMove.Dec[combinedMove.counter] = commBuf.Dec;
		combinedMove.lpusTAN[combinedMove.counter] = commBuf.lpusTAN;
		combinedMove.counter++;
	}else{
		mySemaphore.move.pgInit = 1;
	}
}

/* Fun_18345 */
void DOPEHALT_A_Process(void)////鏁版嵁闀垮害 00 03  鏁版嵁鍐呭
{
	uint16_t i = 5;
	uint32_t temp = 0;
	commBuf.Command = DOPEHALT_A;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.MoveCtrl = buf_data_save[i++]; 
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.Acc = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.lpusTAN = temp;temp=0;//lpusTAN 鎸囦护缂栧彿
	commBuf.Dec = -commBuf.Acc;

	// if(commBuf.Command == AL.command)//鍒ゆ柇涓婃鎸囦护鍜屾湰娆℃寚浠ゆ槸鍚﹂噸澶?
	// 	if(	memcmp(&commBuf,&commBufLast,sizeof(commBuf)) == 0)	//鍒ゆ柇鏁版嵁鍏蜂綋鍙傛暟鏄惁涓€鑷?
	// 				return; //濡傛灉鎸囦护閲嶅鍒欒繑鍥?涓嶅仛鎿嶄綔
	log_i("DOPEHALT_A movectrl=%d Acc=%f Dec=%f",commBuf.MoveCtrl,commBuf.Acc,commBuf.Dec );
	if(combinedMove.blockHeader.Cycles >= 1 && (combinedMove.blockHeader.ModeFlags & BHM_CMD_DWND) == BHM_CMD_DWND){
		combinedMove.command[combinedMove.counter] = commBuf.Command;
		combinedMove.MoveCtrl[combinedMove.counter] = commBuf.MoveCtrl;
		combinedMove.Acc[combinedMove.counter] = commBuf.Acc;
		combinedMove.Dec[combinedMove.counter] = commBuf.Dec;
		combinedMove.lpusTAN[combinedMove.counter] = commBuf.lpusTAN;
		combinedMove.counter++;
	}else{
		mySemaphore.move.pgInit = 1;
	}
}

/* Fun_18346 */
void DOPESHALT_Process(void){
	uint16_t i = 5;
	uint32_t temp = 0;
	commBuf.Command = DOPESHALT;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.MoveCtrl = POS_MODE; 
	temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.lpusTAN = temp;temp=0;//lpusTAN 鎸囦护缂栧彿
	commBuf.Acc = posAllCtrlPara.Nominal.Acc;
	commBuf.Dec = -commBuf.Acc;
	// if(commBuf.Command == AL.command)//鍒ゆ柇涓婃鎸囦护鍜屾湰娆℃寚浠ゆ槸鍚﹂噸澶?
	// 	if(	memcmp(&commBuf,&commBufLast,sizeof(commBuf)) == 0)	//鍒ゆ柇鏁版嵁鍏蜂綋鍙傛暟鏄惁涓€鑷?
	// 				return; //濡傛灉鎸囦护閲嶅鍒欒繑鍥?涓嶅仛鎿嶄綔
	
	log_i("DOPESHALT movectrl=%d Acc=%f Dec=%f",commBuf.MoveCtrl,commBuf.Acc,commBuf.Dec );
	if(combinedMove.blockHeader.Cycles >= 1 && (combinedMove.blockHeader.ModeFlags & BHM_CMD_DWND) == BHM_CMD_DWND){
		combinedMove.command[combinedMove.counter] = commBuf.Command;
		combinedMove.MoveCtrl[combinedMove.counter] = commBuf.MoveCtrl;
		combinedMove.Acc[combinedMove.counter] = commBuf.Acc;
		combinedMove.Dec[combinedMove.counter] = commBuf.Dec;
		combinedMove.lpusTAN[combinedMove.counter] = commBuf.lpusTAN;
		combinedMove.counter++;
	}else{
		mySemaphore.move.pgInit = 1;
	}
}

/* Fun_18351 */
void DOPETRIG_Process(void){
	uint16_t i = 5;
	uint32_t temp = 0;
	commBuf.Command = DOPETRIG;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.MoveCtrl = buf_data_save[i++]; 
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.Speed = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.trig.Limit = *((float*)(&temp));temp=0;
	commBuf.trig.TriggerCtrl = buf_data_save[i++]; 
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.trig.Trigger= *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.lpusTAN = temp;temp=0;//lpusTAN 鎸囦护缂栧彿

	switch (commBuf.MoveCtrl)
	{
	case POS_MODE:
		commBuf.Acc = posAllCtrlPara.DefaultAcc;
		commBuf.Dec = -posAllCtrlPara.DefaultAcc;
		if(posAllCtrlPara.Speed < commBuf.Speed) 
			commBuf.Speed = posAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
		break;
	case LOAD_MODE:
		commBuf.Acc = loadAllCtrlPara.DefaultAcc;
		commBuf.Dec = -loadAllCtrlPara.DefaultAcc;
		if(loadAllCtrlPara.Speed < commBuf.Speed) 
			commBuf.Speed = loadAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
		break;
	case EXTEN_MODE:
		commBuf.Acc = extAllCtrlPara.DefaultAcc;
		commBuf.Dec = -extAllCtrlPara.DefaultAcc;
		if(extAllCtrlPara.Speed < commBuf.Speed) 
			commBuf.Speed = extAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
		break;
	default:
		break;
	}

	log_i("DOPETRIG movectrl=0x%2X speed=%f limit=%f triggerCtrl=0x%2X trigger=%f lpusTAN=%d",
					commBuf.MoveCtrl,commBuf.Speed,commBuf.trig.Limit,commBuf.trig.TriggerCtrl,commBuf.trig.Trigger,commBuf.lpusTAN);

	if(combinedMove.blockHeader.Cycles >= 1 && (combinedMove.blockHeader.ModeFlags & BHM_CMD_DWND) == BHM_CMD_DWND){
		combinedMove.command[combinedMove.counter] = commBuf.Command;
		combinedMove.MoveCtrl[combinedMove.counter] = commBuf.MoveCtrl;
		combinedMove.Speed[combinedMove.counter] = commBuf.Speed;
		combinedMove.Acc[combinedMove.counter] = commBuf.Acc;
		combinedMove.Dec[combinedMove.counter] = commBuf.Dec;
		combinedMove.Limit[combinedMove.counter] = commBuf.trig.Limit;
		combinedMove.TriggerCtrl[combinedMove.counter] = commBuf.trig.TriggerCtrl;
		combinedMove.Trigger[combinedMove.counter] = commBuf.trig.Trigger;
		combinedMove.lpusTAN[combinedMove.counter] = commBuf.lpusTAN;
		combinedMove.counter++;
	}else{
		mySemaphore.move.pgInit = 1;
	}
}

/* Fun_18352 */
void DOPETRIG_A_Process(void){
	uint16_t i = 5;
	uint32_t temp = 0;
	commBuf.Command = DOPETRIG;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.MoveCtrl = buf_data_save[i++]; 
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.Acc = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.Speed = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.Dec = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.trig.Limit = *((float*)(&temp));temp=0;
	commBuf.trig.TriggerCtrl = buf_data_save[i++]; 
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.trig.Trigger= *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.lpusTAN = temp;temp=0;//lpusTAN 鎸囦护缂栧彿

	switch (commBuf.MoveCtrl)
	{
	case POS_MODE:
		if(posAllCtrlPara.Speed < commBuf.Speed) 
			commBuf.Speed = posAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
		break;
	case LOAD_MODE:
		if(loadAllCtrlPara.Speed < commBuf.Speed) 
			commBuf.Speed = loadAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
		break;
	case EXTEN_MODE:
		if(extAllCtrlPara.Speed < commBuf.Speed) 
			commBuf.Speed = extAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
		break;
	default:
		break;
	}

	log_i("DOPETRIG movectrl=0x%2X acc=%f speed=%f Dec=%f limit=%f triggerCtrl=0x%2X trigger=%f lpusTAN=%d",
					commBuf.MoveCtrl,commBuf.Acc,commBuf.Speed,commBuf.Dec,commBuf.trig.Limit,commBuf.trig.TriggerCtrl,commBuf.trig.Trigger,commBuf.lpusTAN);
	
	if(combinedMove.blockHeader.Cycles >= 1 && (combinedMove.blockHeader.ModeFlags & BHM_CMD_DWND) == BHM_CMD_DWND){
		combinedMove.command[combinedMove.counter] = commBuf.Command;
		combinedMove.MoveCtrl[combinedMove.counter] = commBuf.MoveCtrl;
		combinedMove.Speed[combinedMove.counter] = commBuf.Speed;
		combinedMove.Acc[combinedMove.counter] = commBuf.Acc;
		combinedMove.Dec[combinedMove.counter] = commBuf.Dec;
		combinedMove.Limit[combinedMove.counter] = commBuf.trig.Limit;
		combinedMove.TriggerCtrl[combinedMove.counter] = commBuf.trig.TriggerCtrl;
		combinedMove.Trigger[combinedMove.counter] = commBuf.trig.Trigger;
		combinedMove.lpusTAN[combinedMove.counter] = commBuf.lpusTAN;
		combinedMove.counter++;
	}else{
		mySemaphore.move.pgInit = 1;
	}
}

/* Fun_18353 */
void DOPEBLOCKHEADER_Process(void){
	uint16_t i = 5;
	commBuf.Command = DOPEBLOCKHEADER;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.blockHeader.Cycles = buf_data_save[i++]; 
	commBuf.blockHeader.ModeFlags = buf_data_save[i++]; 
	log_i("DOPEBLOCKHEADER Cycles=%d ModeFlags=0x%2X",commBuf.blockHeader.Cycles,commBuf.blockHeader.ModeFlags);
	memset(&combinedMove,0,sizeof(combinedMove));
	combinedMove.blockHeader.Cycles = commBuf.blockHeader.Cycles;
	combinedMove.blockHeader.ModeFlags = commBuf.blockHeader.ModeFlags;
}

/* Fun_18354 */
void DOPEBLOCKEXECUTE_Process(void){
	uint16_t i = 5;
	uint32_t temp = 0;
	commBuf.Command = DOPEBLOCKEXECUTE;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.blockExecute.Operation = buf_data_save[i++]; 
	temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.lpusTAN = temp;temp=0;//lpusTAN 鎸囦护缂栧彿
	log_i("DOPEBLOCKEXECUTE Operation=0x%2X lpusTAN=%d",commBuf.blockExecute.Operation,commBuf.lpusTAN);
	if(commBuf.blockExecute.Operation == BEM_CMD_START){
		mySemaphore.move.combinedMoveExecute = BEM_CMD_START;
		mySemaphore.move.pgInit = 1;
	}else if(commBuf.blockExecute.Operation == BEM_CMD_DISCARD){
		mySemaphore.move.combinedMoveExecute = BEM_CMD_DISCARD;
		mySemaphore.move.pgInit = 1;
	}
}

/* Fun_18342 */
void DOPEPOSEXT_Process(void){
	uint32_t temp=0;
	uint8_t i=5;
	commBuf.Command = DOPEPOSEXT;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.MoveCtrl = buf_data_save[i++]; 
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.Speed = *((float*)(&temp));temp=0;
	commBuf.LimitMode = buf_data_save[i++];
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.Limit = *((float*)(&temp));temp=0;
	commBuf.DestinationCtrl = buf_data_save[i++];
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.Destination = *((float*)(&temp));temp=0;
	commBuf.DestinationMode = buf_data_save[i++];
	temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.lpusTAN = temp;temp=0;//lpusTAN 鎸囦护缂栧彿

	if(commBuf.Command == AL.command)//鍒ゆ柇涓婃鎸囦护鍜屾湰娆℃寚浠ゆ槸鍚﹂噸澶?
		if(	memcmp(&commBuf,&commBufLast,sizeof(commBuf)) == 0)	//鍒ゆ柇鏁版嵁鍏蜂綋鍙傛暟鏄惁涓€鑷?
					return; //濡傛灉鎸囦护閲嶅鍒欒繑鍥?涓嶅仛鎿嶄綔
	switch(commBuf.MoveCtrl)
	{
		case POS_MODE:
			commBuf.Acc = posAllCtrlPara.DefaultAcc;
			commBuf.Dec = -posAllCtrlPara.DefaultAcc;
			if(posAllCtrlPara.Speed < commBuf.Speed) 
				commBuf.Speed = posAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
			commBuf.DecLimit = -posAllCtrlPara.DefaultAcc;
		break;
		case LOAD_MODE:
			commBuf.Acc = loadAllCtrlPara.DefaultAcc;
			commBuf.Dec = -loadAllCtrlPara.DefaultAcc;
			if(loadAllCtrlPara.Speed < commBuf.Speed) 
				commBuf.Speed = loadAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
			commBuf.DecLimit = -loadAllCtrlPara.DefaultAcc;
		break;
		case EXTEN_MODE:
			commBuf.Acc = extAllCtrlPara.DefaultAcc;
			commBuf.Dec = -extAllCtrlPara.DefaultAcc;
			if(extAllCtrlPara.Speed < commBuf.Speed) 
				commBuf.Speed = extAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
			commBuf.DecLimit = -extAllCtrlPara.DefaultAcc;
		break;
		default:break;
	}
	switch(commBuf.DestinationCtrl)
	{
		case POS_MODE:
			commBuf.DecDestination = -posAllCtrlPara.DefaultAcc;
		break;
		case LOAD_MODE:
			commBuf.DecDestination = -loadAllCtrlPara.DefaultAcc;
		break;
		case EXTEN_MODE:
			commBuf.DecDestination = -extAllCtrlPara.DefaultAcc;
		break;
		default:break;
	}
	
	
		printf("Command=0x%2x\r\n",commBuf.Command);
		printf("MoveCtrl=0x%2x\r\n",commBuf.MoveCtrl);
		printf("Speed=%f\r\n",commBuf.Speed);
		printf("LimitMode=%d\r\n",commBuf.LimitMode);
		printf("Limit=%f\r\n",commBuf.Limit);
		printf("DestinationCtrl=%d\r\n",commBuf.DestinationCtrl);
		printf("Destination=%f\r\n",commBuf.Destination);
		printf("DecDestination=%f\r\n",commBuf.DecDestination);
		printf("DestinationMode=%d\r\n",commBuf.DestinationMode);
		printf("lpuTAN=%d\r\n",commBuf.lpusTAN);
	
	if(combinedMove.blockHeader.Cycles >= 1 && (combinedMove.blockHeader.ModeFlags & BHM_CMD_DWND) == BHM_CMD_DWND){
		combinedMove.command[combinedMove.counter] = commBuf.Command;
		combinedMove.MoveCtrl[combinedMove.counter] = commBuf.MoveCtrl;
		combinedMove.Speed[combinedMove.counter] = commBuf.Speed;
		combinedMove.Acc[combinedMove.counter] = commBuf.Acc;
		combinedMove.Dec[combinedMove.counter] = commBuf.Dec;
		combinedMove.LimitMode[combinedMove.counter] = commBuf.Limit;
		combinedMove.Limit[combinedMove.counter] = commBuf.DestinationCtrl;
		combinedMove.Destination[combinedMove.counter] = commBuf.Destination;
		combinedMove.DecDestination[combinedMove.counter] = commBuf.DecDestination;
		combinedMove.DestinationMode[combinedMove.counter] = commBuf.DestinationMode;
		combinedMove.lpusTAN[combinedMove.counter] = commBuf.lpusTAN;
		combinedMove.counter++;
	}else{
		stateFlag.posExtDestReach = reachNone;
		mySemaphore.move.pgInit = 1;
	}
}

/* Fun_18343 */
void DOPEPOSEXT_A_Process(void){
	uint32_t temp=0;
	uint8_t i=5;
	commBuf.Command = DOPEPOSEXT_A;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.MoveCtrl = buf_data_save[i++]; 
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.Acc = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.Speed = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.DecLimit = *((float*)(&temp));temp=0;
	commBuf.LimitMode = buf_data_save[i++];
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.Limit = *((float*)(&temp));temp=0;
	commBuf.DestinationCtrl = buf_data_save[i++];
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.DecDestination = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.Destination = *((float*)(&temp));temp=0;
	commBuf.DestinationMode = buf_data_save[i++];
	temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.lpusTAN = temp;temp=0;//lpusTAN 鎸囦护缂栧彿

	if(commBuf.Command == AL.command)//鍒ゆ柇涓婃鎸囦护鍜屾湰娆℃寚浠ゆ槸鍚﹂噸澶?
		if(	memcmp(&commBuf,&commBufLast,sizeof(commBuf)) == 0)	//鍒ゆ柇鏁版嵁鍏蜂綋鍙傛暟鏄惁涓€鑷?
					return; //濡傛灉鎸囦护閲嶅鍒欒繑鍥?涓嶅仛鎿嶄綔
	
	commBuf.Dec = commBuf.DecLimit;

		printf("Command=0x%2x\r\n",commBuf.Command);
		printf("MoveCtrl=0x%2x\r\n",commBuf.MoveCtrl);
		printf("Acc=%f\r\n",commBuf.Acc);
		printf("Speed=%f\r\n",commBuf.Speed);
		printf("DecLimit=%f\r\n",commBuf.DecLimit);
		printf("LimitMode=%d\r\n",commBuf.LimitMode);
		printf("Limit=%f\r\n",commBuf.Limit);
		printf("DestinationCtrl=%d\r\n",commBuf.DestinationCtrl);
		printf("DecDestination=%f\r\n",commBuf.DecDestination);
		printf("Destination=%f\r\n",commBuf.Destination);
		printf("DestinationMode=%d\r\n",commBuf.DestinationMode);
		printf("lpuTAN=%d\r\n",commBuf.lpusTAN);
	
	if(combinedMove.blockHeader.Cycles >= 1 && (combinedMove.blockHeader.ModeFlags & BHM_CMD_DWND) == BHM_CMD_DWND){
		combinedMove.command[combinedMove.counter] = commBuf.Command;
		combinedMove.MoveCtrl[combinedMove.counter] = commBuf.MoveCtrl;
		combinedMove.Speed[combinedMove.counter] = commBuf.Speed;
		combinedMove.Acc[combinedMove.counter] = commBuf.Acc;
		combinedMove.Dec[combinedMove.counter] = commBuf.Dec;
		combinedMove.LimitMode[combinedMove.counter] = commBuf.Limit;
		combinedMove.Limit[combinedMove.counter] = commBuf.DestinationCtrl;
		combinedMove.Destination[combinedMove.counter] = commBuf.Destination;
		combinedMove.DestinationMode[combinedMove.counter] = commBuf.DestinationMode;
		combinedMove.lpusTAN[combinedMove.counter] = commBuf.lpusTAN;
		combinedMove.counter++;
	}else{
		stateFlag.posExtDestReach = reachNone;
		mySemaphore.move.pgInit = 1;
	}
}

/* Fun_18355 */
void DOPECYCLE_Process(void) //0x40
{
	uint32_t temp=0;
	commBuf.Command = DOPECYCLE;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.MoveCtrl = buf_data_save[5]; 
	temp = (buf_data_save[6]<<24) + (buf_data_save[7]<<16) +(buf_data_save[8]<<8) +(buf_data_save[9]);
	commBuf.Speed1 = *((float*)(&temp));
	temp = (buf_data_save[10]<<24) + (buf_data_save[11]<<16) +(buf_data_save[12]<<8) +(buf_data_save[13]);
	commBuf.Destination1 =  *((float*)(&temp));
	temp = (buf_data_save[14]<<24) + (buf_data_save[15]<<16) +(buf_data_save[16]<<8) +(buf_data_save[17]);
	commBuf.Halt1 =  *((float*)(&temp));
	temp = (buf_data_save[18]<<24) + (buf_data_save[19]<<16) +(buf_data_save[20]<<8) +(buf_data_save[21]);
	commBuf.Speed2 = *((float*)(&temp));
	temp = (buf_data_save[22]<<24) + (buf_data_save[23]<<16) +(buf_data_save[24]<<8) +(buf_data_save[25]);
	commBuf.Destination2 =  *((float*)(&temp));
	temp = (buf_data_save[26]<<24) + (buf_data_save[27]<<16) +(buf_data_save[28]<<8) +(buf_data_save[29]);
	commBuf.Halt2 =  *((float*)(&temp));
	commBuf.Cycles = buf_data_save[30];	
	temp = (buf_data_save[31]<<24) + (buf_data_save[32]<<16) +(buf_data_save[33]<<8) +(buf_data_save[34]);
	commBuf.Speed = *((float*)(&temp));
	temp = (buf_data_save[35]<<24) + (buf_data_save[36]<<16) +(buf_data_save[37]<<8) +(buf_data_save[38]);
	commBuf.Destination =  *((float*)(&temp));
	commBuf.lpusTAN = (buf_data_save[39]<<8) +(buf_data_save[40]);//lpusTAN 鎸囦护缂栧彿

	if(commBuf.Command == AL.command)//鍒ゆ柇涓婃鎸囦护鍜屾湰娆℃寚浠ゆ槸鍚﹂噸澶?
		if(	memcmp(&commBuf,&commBufLast,sizeof(commBuf)) == 0)	//鍒ゆ柇鏁版嵁鍏蜂綋鍙傛暟鏄惁涓€鑷?
					return; //濡傛灉鎸囦护閲嶅鍒欒繑鍥?涓嶅仛鎿嶄綔

	switch(commBuf.MoveCtrl)
	{
		case POS_MODE:
			commBuf.Acc = posAllCtrlPara.DefaultAcc;
			commBuf.Dec = -posAllCtrlPara.DefaultAcc;
			if(posAllCtrlPara.Speed < commBuf.Speed) 
				commBuf.Speed = posAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
			//閫熷害闄愬埗骞呭害
			if(posAllCtrlPara.Speed < commBuf.Speed)  commBuf.Speed	= posAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
			if(posAllCtrlPara.Speed < commBuf.Speed1) commBuf.Speed1	= posAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
			if(posAllCtrlPara.Speed < commBuf.Speed2) commBuf.Speed2	= posAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
		break;
		case LOAD_MODE:
			commBuf.Acc = loadAllCtrlPara.DefaultAcc;
			commBuf.Dec = -loadAllCtrlPara.DefaultAcc;
			if(loadAllCtrlPara.Speed < commBuf.Speed) 
				commBuf.Speed = loadAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
			//閫熷害闄愬埗骞呭害
			if(loadAllCtrlPara.Speed < commBuf.Speed)  commBuf.Speed	= loadAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
			if(loadAllCtrlPara.Speed < commBuf.Speed1) commBuf.Speed1	= loadAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
			if(loadAllCtrlPara.Speed < commBuf.Speed2) commBuf.Speed2	= loadAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
		break;
		case EXTEN_MODE:
			if(extAllCtrlPara.Speed < commBuf.Speed) 
				commBuf.Speed = extAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
			//閫熷害闄愬埗骞呭害
			if(extAllCtrlPara.Speed < commBuf.Speed)  commBuf.Speed	= extAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
			if(extAllCtrlPara.Speed < commBuf.Speed1) commBuf.Speed1	= extAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
			if(extAllCtrlPara.Speed < commBuf.Speed2) commBuf.Speed2	= extAllCtrlPara.Speed;	//鏈€澶ч€熷害榛樿鍊紆nit/s
		break;
		default:break;
	}

		
	pgcycle.cycles = commBuf.Cycles;
	pgcycle.destination = commBuf.Destination;
	pgcycle.destination1 = commBuf.Destination1;
	pgcycle.destination2 = commBuf.Destination2;
	pgcycle.speed = commBuf.Speed;
	pgcycle.speed1 = commBuf.Speed1;		
	pgcycle.speed2 = commBuf.Speed2;		
	pgcycle.halt1 = commBuf.Halt1;
	pgcycle.halt2 = commBuf.Halt2;		
	pgcycle.count_step = 0;
	pgcycle.count_cycle=0;

	
		printf("Speed1=%f\r\n",commBuf.Speed1);
		printf("Destination1=%f\r\n",commBuf.Destination1);
		printf("Halt1=%f\r\n",commBuf.Halt1);	
		printf("Speed2=%f\r\n",commBuf.Speed2);
		printf("Destination2=%f\r\n",commBuf.Destination2);
		printf("Halt2=%f\r\n",commBuf.Halt2);	
		printf("Cycles=%d\r\n",commBuf.Cycles);
		printf("Speed=%f\r\n",commBuf.Speed);
		printf("Destination=%f\r\n",commBuf.Destination);

	mySemaphore.move.pgInit = 1;
}

void DOPEDYNCYCLE_Process(void)
{
	uint32_t temp=0;
	uint16_t i=5;
	commBuf.Command = DOPEDYNCYCLE;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.pgdyn.waveform = buf_data_save[i++]; 
	commBuf.pgdyn.modify = buf_data_save[i++]; 
	commBuf.pgdyn.peakctrl = buf_data_save[i++]; 
	commBuf.pgdyn.movectrl = buf_data_save[i++]; 
	commBuf.pgdyn.relativedestination = buf_data_save[i++]; temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.pgdyn.speedtostart = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.pgdyn.offset = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.pgdyn.amplitude = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.pgdyn.haltatplusamplitude = temp;temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.pgdyn.haltatminusamplitude = temp;temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.pgdyn.frequency = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.pgdyn.halfcycles = temp;temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.pgdyn.speedtodestination = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.pgdyn.destination = *((float*)(&temp));temp=0;
	//鎵寰呮坊鍔?
	
	
	commBuf.MoveCtrl = commBuf.pgdyn.movectrl; 

	if(commBuf.Command == AL.command)//鍒ゆ柇涓婃鎸囦护鍜屾湰娆℃寚浠ゆ槸鍚﹂噸澶?
		if(	memcmp(&commBuf,&commBufLast,sizeof(commBuf)) == 0)	//鍒ゆ柇鏁版嵁鍏蜂綋鍙傛暟鏄惁涓€鑷?
					return; //濡傛灉鎸囦护閲嶅鍒欒繑鍥?涓嶅仛鎿嶄綔
	
	
	
		printf("commBuf.pgdyn:\r\n");
		printf("waveform=0x%2x\r\n",commBuf.pgdyn.waveform);
		printf("modify=%d\r\n",commBuf.pgdyn.modify);
		printf("peakctrl=%d\r\n",commBuf.pgdyn.peakctrl);
		printf("movectrl=0x%2x\r\n",commBuf.pgdyn.movectrl);
		printf("relativedestination=%d\r\n",commBuf.pgdyn.relativedestination);
		printf("speedtostart=%f\r\n",commBuf.pgdyn.speedtostart);
		printf("offset=%f\r\n",commBuf.pgdyn.offset);
		printf("amplitude=%f\r\n",commBuf.pgdyn.amplitude);
		printf("haltatplusamplitude=%f\r\n",commBuf.pgdyn.haltatplusamplitude);
		printf("haltatminusamplitude=%f\r\n",commBuf.pgdyn.haltatminusamplitude);
		printf("frequency=%f\r\n",commBuf.pgdyn.frequency);
		printf("halfcycles=%d\r\n",commBuf.pgdyn.halfcycles);
		printf("speedtodestination=%f\r\n",commBuf.pgdyn.speedtodestination);
		printf("destination=%f\r\n",commBuf.pgdyn.destination);	
	
		stateFlag.DynCycle_OffsetReach = 0;
		mySemaphore.move.pgInit = 1;
}

/* Fun_18329 */
void RDOUTPUTPARA_Process(void)//璇诲彇杈撳嚭鍙傛暟
{
	uint16_t i=5;
	commBuf.Command = RDOUTPUTPARA;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	/* Fun_18412 */
	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	i=0;
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)OutputPara.Signal,buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)OutputPara.Sign,buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&OutputPara.MaxValue),buf_com,&i);//DataTransmitPeriod
	printf("OutputPara.MaxValue:%f\r\n",OutputPara.MaxValue);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&OutputPara.MinValue),buf_com,&i);//DataTransmitPeriod
	printf("OutputPara.MinValue:%f\r\n",OutputPara.MinValue);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&OutputPara.InitValue),buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&OutputPara.Offset),buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&OutputPara.SignalFrequency),buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&OutputPara.MaxVoltage),buf_com,&i);//DataTransmitPeriod
	printf("OutputPara.MaxVoltage:%f\r\n",OutputPara.MaxVoltage);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&OutputPara.MaxCurrent),buf_com,&i);//DataTransmitPeriod

	send_ring_fifo_push(i,buf_com,0x8A);//涓嬩綅鏈轰細杩斿洖0x8A鍔熻兘鐮?
}

/* Fun_18330 */
void WROUTPUTPARA_Process(OUTPUTPARA *x)//鍐欏叆杈撳嚭鍙傛暟
{
	uint32_t temp=0; 
	uint16_t i=5;
	// uint8_t  j=0;
	commBuf.Command = WROUTPUTPARA;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	x->Signal = buf_data_save[i++];
	x->Sign = buf_data_save[i++];
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	x->MaxValue = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	x->MinValue = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	x->InitValue = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	x->Offset = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	x->SignalFrequency = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	x->MaxVoltage = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	x->MaxCurrent = *((float*)(&temp));temp=0;
	
	mySemaphore.paraconfig.write = 1;
	printf("Command=0x%2x WROUTPUTPARA\t\r\n",commBuf.Command);
	printf("Signal=%d\t\r\n",x->Signal);
	printf("Sign=%d\t\r\n",x->Sign);
	printf("MaxValue=%f\t\r\n",x->MaxValue);
	printf("MinValue=%f\t\r\n",x->MinValue);
	printf("InitValue=%f\t\r\n",x->InitValue);
	printf("Offset=%f\t\r\n",x->Offset);
	printf("SignalFrequency=%f\t\r\n",x->SignalFrequency);
	printf("MaxVoltage=%f\t\r\n",x->MaxVoltage);
	printf("MaxCurrent=%f\t\r\n",x->MaxCurrent);
}

/* Fun_18331 */
void RDETHPARA_Process(void)//璇讳互澶綉鍙傛暟
{
	//杩斿洖0x8A鎸囦护 涓婂彂浠ュお缃戝弬鏁?
	uint16_t i=0,j=0;
	/* Fun_18413 */
	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	buf_com[i++] = utc_eth.Mode;//浠ュお缃戝崗璁?
	for(j=0;j<4;j++) buf_com[i++]=dest_ip[j];//鏈嶅姟鍣↖P鍦板潃
	Variable_Type_Conversion(UINT16_TYPE,*(uint32_t*)(&dest_port),buf_com,&i);//鏈嶅姟鍣ㄧ鍙ｅ彿
	for(j=0;j<4;j++) buf_com[i++]=gWIZNETINFO.ip[j];//鏈湴IP鍦板潃
	for(j=0;j<6;j++) buf_com[i++]=gWIZNETINFO.mac[j];//鏈湴MAC鍦板潃
	send_ring_fifo_push(i,buf_com,0x8A);//涓嬩綅鏈轰細杩斿洖0x8A鍔熻兘鐮?
}

/* Fun_18332 */
void WRETHPARA_Process(void)//涓嬪彂浠ュお缃戝弬鏁?
{
	uint8_t i=0;
	commBuf.Command = WRETHPARA;
	ethConfig.EthMode = buf_data_save[5]; 
	for(i=0;i<4;i++) ethConfig.ServerIp[i]= buf_data_save[6+i];
	ethConfig.ServerPort = (buf_data_save[10]<<8) + buf_data_save[11];
	for(i=0;i<4;i++) ethConfig.LocalIp[i]= buf_data_save[12+i];
	for(i=0;i<6;i++) ethConfig.LocalMac[i]= buf_data_save[16+i];
	//rationality of param check
	log_i("Command=0x%2x WRETHPARA\t\r\n",commBuf.Command);
	log_i("ETH_Mode:0x%2x\r\n",ethConfig.EthMode);
	log_i("ServerIp:%d,%d,%d,%d\r\n",ethConfig.ServerIp[0],ethConfig.ServerIp[1],ethConfig.ServerIp[2],ethConfig.ServerIp[3]);
	log_i("ServerPort:%d\r\n",ethConfig.ServerPort);
	log_i("LocalIp:%d,%d,%d,%d\r\n",ethConfig.LocalIp[0],ethConfig.LocalIp[1],ethConfig.LocalIp[2],ethConfig.LocalIp[3]);
	log_i("LocalMac:0x%2x,0x%2x,0x%2x,0x%2x,0x%2x,0x%2x\r\n",ethConfig.LocalMac[0],ethConfig.LocalMac[1],ethConfig.LocalMac[2],ethConfig.LocalMac[3],ethConfig.LocalMac[4],ethConfig.LocalMac[5]);
	
	/*寰呭姞鍏?
	  瀛樺叆鎺у埗鍣∕RAM  閲嶅惎绯荤粺 閲嶆柊閰嶇疆浠ュお缃戝弬鏁?
	*/	
	mySemaphore.paraconfig.write = 1;
}

/* Fun_18333 */
void RDSYSPARA_Process(void)//璇诲彇绯荤粺鍙傛暟
{
	uint16_t i=0;
	commBuf.Command = RDSYSPARA;
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	/* Fun_18414 */
	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	Variable_Type_Conversion(UINT16_TYPE,*(uint32_t*)(&SysPara.TransmitDataPeriod),buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(UINT8_TYPE,*(uint32_t*)(&SysPara.ControllerStructure),buf_com,&i);//DataTransmitPeriod
	send_ring_fifo_push(i,buf_com,UTCRDSYSPARA);//涓嬩綅鏈轰細杩斿洖0x8C鍔熻兘鐮?
}

/* Fun_18334 */
void WRSYSPARA_Process(void)//鍐欏叆绯荤粺鍙傛暟
{
	uint32_t temp=0;
	uint16_t i=5;
	commBuf.Command = WRSYSPARA;
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.SysPara.TransmitDataPeriod = temp;
	commBuf.SysPara.ControllerStructure = buf_data_save[i++];
	
	mySemaphore.comm.transperiodUpdate = 1;
	mySemaphore.comm.ctrlstructUpdate = 1;

	mySemaphore.paraconfig.write = 1;
	
	//杩斿洖0x8C
	i=0;
	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	Variable_Type_Conversion(UINT16_TYPE,*(uint32_t*)(&commBuf.SysPara.TransmitDataPeriod),buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(UINT8_TYPE,*(uint32_t*)(&commBuf.SysPara.ControllerStructure),buf_com,&i);//DataTransmitPeriod
	send_ring_fifo_push(i,buf_com,0x8C);//涓嬩綅鏈轰細杩斿洖0x8C鍔熻兘鐮?

	printf("Command=0x%2x RDSYSPARA\t\r\n",commBuf.Command);
	printf("DataTransmitPeriod=%dms\t\r\n",commBuf.SysPara.TransmitDataPeriod);
	printf("ControllerStructure=%d\r\n",commBuf.SysPara.ControllerStructure);
}

/* Fun_18336 */
void SETTARE_Process(void)
{
	uint16_t i=5;
	commBuf.Command = SETTARE;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	commBuf.Connector = buf_data_save[i++];
	//瀵瑰簲鎿嶄綔锛屾垨鑰呯疆鏍囧織浣嶆垨浜掓枼淇″彿閲?
	AL.tare.flag = 1;
	AL.tare.connector = commBuf.Connector;
	mySemaphore.paraconfig.write = 1;
	
	printf("Command=0x%2x SETTARE\t\r\n",commBuf.Command);
	printf("Connector=%d\t\r\n",commBuf.Connector);
}

void WRSERVOPARA_Process(void){
	uint32_t temp=0; 
	uint16_t i=5;
	// uint8_t  j=0;
	commBuf.Command = WRSERVOPARA;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲

	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.servoParam.electronicGear = temp;temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.servoParam.encoderResolution = temp;temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	commBuf.servoParam.analogGain = *((float*)(&temp));temp=0;
	
	mySemaphore.paraconfig.write = 1;
	log_i("Command=0x%2x WRSERVOPARA\t",commBuf.Command);
	log_i("electronicGear=%d",commBuf.servoParam.electronicGear);
	log_i("encoderResolution=%d\t",commBuf.servoParam.encoderResolution);
	log_i("analogGain=%f\t",commBuf.servoParam.analogGain);
}

void RDSERVOPARA_Process(void){
	uint16_t i=0;
	commBuf.Command = RDSERVOPARA;
	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	Variable_Type_Conversion(UINT32_TYPE,servoParam.electronicGear,buf_com,&i);//electronicGear
	Variable_Type_Conversion(UINT32_TYPE,servoParam.encoderResolution,buf_com,&i);//encoderResolution
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&servoParam.analogGain),buf_com,&i);//DataTransmitPeriod
	send_ring_fifo_push(i,buf_com,UTCRDSERVOPARA);//涓嬩綅鏈轰細杩斿洖UTCRDSERVOPARA鍔熻兘鐮?
}

/* Fun_133 */

void CurrentData_Fifo_Push(void)
{
	uint32_t temp=0;
	uint16_t i=0;
	cdatatrans.Position=pose.orig;
	cdatatrans.Load	= cs5552_compat_ctx.Value[CS5552_COMPAT_CHANNEL_FORCE];
	// cdatatrans.Load	= force.filterTrans;
	cdatatrans.Extensometer1 = strain1.filterTrans;
	// strain1Lsm.filter*1000;
	cdatatrans.Extensometer2 = strain2.orig;//if the orig of strain2 is fine,then we should change the strain2.orig to strain2.filterTrans
	cdatatrans.BigDeformation = 0;
	cdatatrans.ManualBox = (DI.ALL & 0x003F);
	cdatatrans.InSignals = (DI.ALL >> 6) & 0x00FF;
	temp = (DO.DO5_RL<<6) + (DO.DO4_RL<<5) + (DO.DO3<<4) + (DO.DO2<<3) + (DO.DO1<<2) 
			+ ((DO.DIR)<<1) 
			+ (DO.SON);
	cdatatrans.OutSignals= (uint16_t)temp;temp=0;
	cdatatrans.CtrlState1= 0;
	cdatatrans.CtrlState2= 0;
	cdatatrans.UpperLimits_state=stateFlag.UpperLimits;
	cdatatrans.LowerLimits_state=stateFlag.LowerLimits;
	cdatatrans.SensorConnected=(((DB9.CON3.value<<3) + (DB9.CON2.value<<2) + (DB9.CON1.value<<1)))^0x0f;//Byte27.0: POS_MODE Byte27.1: LOAD_MODE Byte27.2: EXTENSION1 Byte27.3: EXTENSION2
	cdatatrans.SensorKeyPressed=((DB9.ADJ3.value<<3) + (DB9.ADJ2.value<<2) + (DB9.ADJ1.value<<1))^0x0f;//Byte28.0: POS_MODE Byte28.1: LOAD_MODE Byte28.2: EXTENSION1 Byte28.3: EXTENSION2
	cdatatrans.ModuleError=0;
	cdatatrans.PosSpeed=speedPose.filter;
//	printf("speedPose.filter:%f\r\n",speedPose.filter);
	cdatatrans.LoadSpeed=0;
	cdatatrans.Extensometer1Speed=0;
	cdatatrans.Extensometer2Speed=0;
	cdatatrans.BigDeformationSpeed = 0;
	cdatatrans.PosEncoderCode = pose.code;
	cdatatrans.LoadADCCode = force.code;
	cdatatrans.Extension1ADCCode = strain1.code;
	cdatatrans.Extension2ADCCode = strain2.code;
	cdatatrans.BigDeformationCode = 0;
	//cdatatrans.pgst	= pg.st;
	cdatatrans.pgst	= pg.st_last;
	cdatatrans.pgLoadst	= pgLoad.st_last;
	cdatatrans.pgExtst = pgExt.st_last;
	cdatatrans.outputFrequency = (hal_output.DO.DIR == 1 ? hal_output.PWM:(hal_output.PWM*-1));
	cdatatrans.outputDAC = (float)hal_output.AO/1000.0f;

	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&cdatatrans.Position),cdatabuf,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&cdatatrans.Load),cdatabuf,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&cdatatrans.Extensometer1),cdatabuf,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&cdatatrans.Extensometer2),cdatabuf,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&cdatatrans.BigDeformation),cdatabuf,&i);
	Variable_Type_Conversion(UINT16_TYPE,(uint32_t)cdatatrans.ManualBox,cdatabuf,&i);
	Variable_Type_Conversion(UINT16_TYPE,(uint32_t)cdatatrans.InSignals,cdatabuf,&i);
	Variable_Type_Conversion(UINT16_TYPE,(uint32_t)cdatatrans.OutSignals,cdatabuf,&i);
	Variable_Type_Conversion(UINT16_TYPE,(uint32_t)cdatatrans.CtrlState1,cdatabuf,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)cdatatrans.CtrlState2,cdatabuf,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)cdatatrans.UpperLimits_state,cdatabuf,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)cdatatrans.LowerLimits_state,cdatabuf,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)cdatatrans.SensorConnected,cdatabuf,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)cdatatrans.SensorKeyPressed,cdatabuf,&i);
	Variable_Type_Conversion(UINT32_TYPE,cdatatrans.ModuleError,cdatabuf,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&cdatatrans.PosSpeed),cdatabuf,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&cdatatrans.LoadSpeed),cdatabuf,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&cdatatrans.Extensometer1Speed),cdatabuf,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&cdatatrans.Extensometer2Speed),cdatabuf,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&cdatatrans.BigDeformationSpeed),cdatabuf,&i);
	Variable_Type_Conversion(UINT32_TYPE,cdatatrans.PosEncoderCode,cdatabuf,&i);
	Variable_Type_Conversion(UINT32_TYPE,cdatatrans.LoadADCCode,cdatabuf,&i);
	Variable_Type_Conversion(UINT32_TYPE,cdatatrans.Extension1ADCCode,cdatabuf,&i);
	Variable_Type_Conversion(UINT32_TYPE,cdatatrans.Extension2ADCCode,cdatabuf,&i);
	Variable_Type_Conversion(UINT32_TYPE,cdatatrans.BigDeformationCode,cdatabuf,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&cdatatrans.pgst),cdatabuf,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&cdatatrans.pgLoadst),cdatabuf,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&cdatatrans.pgExtst),cdatabuf,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&cdatatrans.outputFrequency),cdatabuf,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&cdatatrans.outputDAC),cdatabuf,&i);
	/* Fun_18416 */
	  send_ring_fifo_push(i,cdatabuf,DOPECURRENTDATA);//涓嬩綅鏈轰細杩斿洖0xA0鍔熻兘鐮?
	
#ifdef CurrentData_debug
		for(k=0;k<SEND_FIFO_SIZE;k++)
				{
					printf("SEND Fifo%d  ",k);
						for(l=0;l<70;l++)
						printf("%2x",send_fifo_data[k][l]);
					printf("\r\n");
				}
				printf("\r\n \r\n");
#endif
	
}

void Variable_Type_Conversion(uint8_t type,uint32_t x,uint8_t* buf,uint16_t* bufnum)
{
	switch(type)
	{
		case UINT8_TYPE:
			  buf[(*bufnum)++]=(uint8_t)x;
			break;
		case UINT16_TYPE:
			  buf[(*bufnum)++]=(uint8_t)(x>>8);
			  buf[(*bufnum)++]=(uint8_t)x;
			break;	
		case FLOAT_TYPE:
		case UINT32_TYPE:
		case INT32_TYPE:
			  buf[(*bufnum)++]=(uint8_t)(x>>24);
			  buf[(*bufnum)++]=(uint8_t)(x>>16);
			  buf[(*bufnum)++]=(uint8_t)(x>>8);
			  buf[(*bufnum)++]=(uint8_t)x;
			break;
		default:
			break;
	}
}

/* Fun_18325 */
void RDSENSORDATA_Process(void)//璇诲彇浼犳劅鍣ㄦ暟鎹?
{
	// uint32_t temp=0;
	uint16_t i=5;
	uint8_t j=0;
	float sensorCorrectionFactorTemp;
	commBuf.Command = RDSENSORDATA;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	sensorConnector = (sensorConnector_e)buf_data_save[i++];
	log_i("RDSENSORDATA Connector=%d\t",sensorConnector);
	/* Fun_18410 */
	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	i=0;
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)sensorConnector,buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)SenData[sensorConnector].Sensortype,buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)SenData[sensorConnector].Sign,buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&SenData[sensorConnector].NominalValue),buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&SenData[sensorConnector].NominalSensitive),buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)SenData[sensorConnector].Day,buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)SenData[sensorConnector].Month,buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(UINT16_TYPE,(uint32_t)SenData[sensorConnector].Year,buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)SenData[sensorConnector].LinPoint,buf_com,&i);//DataTransmitPeriod
	
	if(SenData[sensorConnector].LinPoint > 0)
	{
		for(;j<SenData[sensorConnector].LinPoint;j++)
		{
			Variable_Type_Conversion(INT32_TYPE,*(uint32_t*)(&SenData[sensorConnector].LinV[j].ADCCode),buf_com,&i);//DataTransmitPeriod
			Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&SenData[sensorConnector].LinV[j].RefValue),buf_com,&i);//DataTransmitPeriod
			if(sensorConnector != ch0Pose){
				sensorCorrectionFactorTemp = 1/SenData[sensorConnector].LinV[j].CorrectionFactor * ADC_FACTOR_SENSOR_FRONTEND_ROUND * SenData[sensorConnector].NominalValue;
				if(sensorConnector == ch4Load)
					sensorCorrectionFactorTemp *= 1000.0f;
			}
			printf("Num:%d Temp:%f,%f\r\n",j,sensorCorrectionFactorTemp,SenData[sensorConnector].LinV[j].CorrectionFactor);
			Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&sensorCorrectionFactorTemp),buf_com,&i);//DataTransmitPeriod
		}
	}
	send_ring_fifo_push(i,buf_com,0x88);//涓嬩綅鏈轰細杩斿洖0x88鍔熻兘鐮?
}

/* Fun_18326 */
void WRSENSORDATA_Process(void)//鍐欏叆浼犳劅鍣ㄦ暟鎹?
{
	uint32_t temp=0; 
	uint16_t i=5;
	uint8_t j=0;
	commBuf.Command = WRSENSORDATA;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	sensorConnector = (sensorConnector_e)buf_data_save[i++];
	senDataCommbuf[sensorConnector].Sensortype = buf_data_save[i++];
	senDataCommbuf[sensorConnector].Sign = buf_data_save[i++];
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	senDataCommbuf[sensorConnector].NominalValue= *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	senDataCommbuf[sensorConnector].NominalSensitive= *((float*)(&temp));temp=0;
	senDataCommbuf[sensorConnector].Day = buf_data_save[i++];
	senDataCommbuf[sensorConnector].Month = buf_data_save[i++];
	temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	senDataCommbuf[sensorConnector].Year= temp;temp=0;
	senDataCommbuf[sensorConnector].LinPoint= buf_data_save[i++];
	
	if(senDataCommbuf[sensorConnector].LinPoint > 0)
	{
		for(;j<senDataCommbuf[sensorConnector].LinPoint;j++)
		{
			temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
			senDataCommbuf[sensorConnector].LinV[j].ADCCode = temp;temp=0;
			temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
			senDataCommbuf[sensorConnector].LinV[j].RefValue = *((float*)(&temp));temp=0;
			temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
			senDataCommbuf[sensorConnector].LinV[j].CorrectionFactor = *((float*)(&temp));temp=0;
		}
	}
	mySemaphore.calibrate.write = 1;
	mySemaphore.comm.sendataUpdate = 1;
	
	log_i("COMMAND:WRSENSORDATA\t\r\n");
	log_i("Connector=%d\t\r\n",sensorConnector);
	log_i("Sensortype=%2x\t\r\n",senDataCommbuf[sensorConnector].Sensortype);
	log_i("Sign=%d\t\r\n",senDataCommbuf[sensorConnector].Sign);
	log_i("NominalValue=%f\t\r\n",senDataCommbuf[sensorConnector].NominalValue);
	log_i("NominalSensitive=%f\t\r\n",senDataCommbuf[sensorConnector].NominalSensitive);
	log_i("Last Modified Time:%d\t%d\t%d\t\r\n",senDataCommbuf[sensorConnector].Year,senDataCommbuf[sensorConnector].Month,senDataCommbuf[sensorConnector].Day);
	log_i("LinPoint=%d\t\r\n",senDataCommbuf[sensorConnector].LinPoint);
	if(senDataCommbuf[sensorConnector].LinPoint > 0)
	{
		for(j=0;j<senDataCommbuf[sensorConnector].LinPoint;j++)
		{
			log_i("LinV[%d].ADCCode=%d\t\r\n",j,senDataCommbuf[sensorConnector].LinV[j].ADCCode);
			log_i("LinV[%d].RefValue=%f\t\r\n",j,senDataCommbuf[sensorConnector].LinV[j].RefValue);
			log_i("LinV[%d].CorrectionFactor=%f\t\r\n",j,senDataCommbuf[sensorConnector].LinV[j].CorrectionFactor);
		}
	}
}

/* Fun_18327 */
void RDSENSORBIGDEFORMATIONDATA_Process(void)//璇诲彇澶у彉褰紶鎰熷櫒鏁版嵁
{
	// uint32_t temp=0;
	uint16_t i=5;
	// uint8_t j=0;
	commBuf.Command = RDSENSORBIGDEFORMATIONDATA;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	sensorConnector = (sensorConnector_e)buf_data_save[i++];
	printf("RDSENSORDATA Connector=%d\t\r\n",sensorConnector);
	/* Fun_18411 */
	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	i=0;
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)sensorConnector,buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)SensorBigDeformationData[sensorConnector].Sensortype,buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)SensorBigDeformationData[sensorConnector].Sign,buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&SensorBigDeformationData[sensorConnector].NominalValue),buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(UINT8_TYPE,*(uint32_t*)(&SensorBigDeformationData[sensorConnector].CollectType),buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&SensorBigDeformationData[sensorConnector].NominalSensitive1),buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&SensorBigDeformationData[sensorConnector].NominalSensitive2),buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)SensorBigDeformationData[sensorConnector].Day,buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)SensorBigDeformationData[sensorConnector].Month,buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(UINT16_TYPE,(uint32_t)SensorBigDeformationData[sensorConnector].Year,buf_com,&i);//DataTransmitPeriod

	send_ring_fifo_push(i,buf_com,0x89);//涓嬩綅鏈轰細杩斿洖0x88鍔熻兘鐮?
}

/* Fun_18328 */
void WRSENSORBIGDEFORMATIONDATA_Process(void)//鍐欏叆澶у彉褰紶鎰熷櫒鏁版嵁
{
	uint32_t temp=0; 
	uint16_t i=5;
	commBuf.Command = WRSENSORBIGDEFORMATIONDATA;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	sensorConnector = (sensorConnector_e)buf_data_save[i++];
	SensorBigDeformationData[sensorConnector].Sensortype = buf_data_save[i++];
	SensorBigDeformationData[sensorConnector].Sign = buf_data_save[i++];
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	SensorBigDeformationData[sensorConnector].NominalValue= *((float*)(&temp));temp=0;
	SensorBigDeformationData[sensorConnector].CollectType = buf_data_save[i++];
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	SensorBigDeformationData[sensorConnector].NominalSensitive1= *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	SensorBigDeformationData[sensorConnector].NominalSensitive2= *((float*)(&temp));temp=0;
	SensorBigDeformationData[sensorConnector].Day = buf_data_save[i++];
	SensorBigDeformationData[sensorConnector].Month = buf_data_save[i++];
	temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	SensorBigDeformationData[sensorConnector].Year= temp;temp=0;
	
	
//	if(commBuf.Command == AL.command)//鍒ゆ柇涓婃鎸囦护鍜屾湰娆℃寚浠ゆ槸鍚﹂噸澶?
//		if(	memcmp(&commBuf,&commBufLast,sizeof(commBuf)) == 0)	//鍒ゆ柇鏁版嵁鍏蜂綋鍙傛暟鏄惁涓€鑷?
//					return; //濡傛灉鎸囦护閲嶅鍒欒繑鍥?涓嶅仛鎿嶄綔
	
	
	printf("Connector=%d\t\r\n",sensorConnector);
	printf("Sensortype=%d\t\r\n",SensorBigDeformationData[sensorConnector].Sensortype);
	printf("Sign=%d\t\r\n",SensorBigDeformationData[sensorConnector].Sign);
	printf("NominalValue=%f\t\r\n",SensorBigDeformationData[sensorConnector].NominalValue);
	printf("CollectType=%d\t\r\n",SensorBigDeformationData[sensorConnector].CollectType);
	printf("NominalSensitive1=%f\t\r\n",SensorBigDeformationData[sensorConnector].NominalSensitive1);
	printf("NominalSensitive2=%f\t\r\n",SensorBigDeformationData[sensorConnector].NominalSensitive2);
	printf("Last Modified Time:%d\t%d\t%d\t\r\n",SensorBigDeformationData[sensorConnector].Year,
												 SensorBigDeformationData[sensorConnector].Month,
												 SensorBigDeformationData[sensorConnector].Day);
}

/* Fun_1835 */
void DOPERDNOMINALACCSPEED_Process(void)//璇诲彇浣嶇疆鐢熸垚鍣ㄦ爣绉板€?
{
	// uint32_t temp=0;
	uint16_t i=5;
	// uint8_t j=0;
	ALLCTRLPARA *p=0;
	commBuf.Command = DOPERDNOMINALACCSPEED;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.MoveCtrl = buf_data_save[i++];
	printf("Command=0x%2x DOPERDNOMINALACCSPEED\t\r\n",commBuf.Command);
	printf("MoveCtrl=%d\t\r\n",commBuf.MoveCtrl);
	
	switch(commBuf.MoveCtrl)
	{
		case POS_MODE:
			p = &posAllCtrlPara;
			break;
		case LOAD_MODE:
			p = &loadAllCtrlPara;
			break;
		case EXTEN_MODE:
			p = &extAllCtrlPara;
			break;
		default:break;
	}
		
	/* Fun_1844 */
	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	i=0;
	
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)commBuf.MoveCtrl,buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&p->Nominal.Acc),buf_com,&i);//DataTransmitPeriod
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&p->Nominal.Speed),buf_com,&i);//DataTransmitPeriod

	send_ring_fifo_push(i,buf_com,0x83);//涓嬩綅鏈轰細杩斿洖0x83鍔熻兘鐮?
	
}

/* Fun_1836 */
void DOPESETNOMINALACCSPEED_Process(void)//璁剧疆浣嶇疆鐢熸垚鍣ㄦ爣绉板€?
{
	uint32_t temp=0; 
	uint16_t i=5;
	float Acc,Speed;
	commBuf.Command = DOPESETNOMINALACCSPEED;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.MoveCtrl = buf_data_save[i++];
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	Acc = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	Speed = *((float*)(&temp));temp=0;

	switch(commBuf.MoveCtrl){
		case POS_MODE:
			commBuf.Pos.Nominal.Acc = Acc;
			commBuf.Pos.Nominal.Speed = Speed;
		break;
		case LOAD_MODE:
			commBuf.Load.Nominal.Acc = Acc;
			commBuf.Load.Nominal.Speed = Speed;
		break;
		case EXTEN_MODE:
			commBuf.Ext.Nominal.Acc = Acc;
			commBuf.Ext.Nominal.Speed = Speed;
		break;
		default:break;
	}
	mySemaphore.paraconfig.write = 1;
	
	printf("Command=0x%2x DOPESETNOMINALACCSPEED\t\r\n",commBuf.Command);
	printf("MoveCtrl=%d\t\r\n",commBuf.MoveCtrl);
	printf("NominalAcc=%f\t\r\n",Acc);
	printf("NominalSpeed=%f\t\r\n",Speed);
	
}

/* Fun_18315 */
void DOPEPOSPID_Process(void)//璁惧畾浣嶇疆PID鍙傛暟
{
	uint32_t temp=0; 
	uint16_t i=5;
	float P,I,D;
	commBuf.Command = DOPEPOSPID;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.MoveCtrl = buf_data_save[i++];
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	P = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	I = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	D = *((float*)(&temp));temp=0;
	switch(commBuf.MoveCtrl)
	{
		case POS_MODE:
			commBuf.Pos.Pid.P = P;
			commBuf.Pos.Pid.I = I;
			commBuf.Pos.Pid.D = D;
			break;
		case LOAD_MODE:
			commBuf.Load.Pid.P = P;
			commBuf.Load.Pid.I = I;
			commBuf.Load.Pid.D = D;
			break;
		case EXTEN_MODE:
			commBuf.Ext.Pid.P = P;
			commBuf.Ext.Pid.I = I;
			commBuf.Ext.Pid.D = D;
			break;
		default:break;
	}
	mySemaphore.paraconfig.set = 1;
	
	printf("Command=0x%2x DOPEPOSPID\t\r\n",commBuf.Command);
	printf("MoveCtrl=%d\t\r\n",commBuf.MoveCtrl);
	printf("P=%f\t\r\n",P);
	printf("I=%f\t\r\n",I);
	printf("D=%f\t\r\n",D);
}

/* Fun_18318 */
void DOPESPEEDPID_Process(void)//璁剧疆閫熷害PID鍙傛暟
{
	uint32_t temp=0; 
	uint16_t i=5;
	float P,I,D;
	commBuf.Command = DOPESPEEDPID;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.MoveCtrl = buf_data_save[i++];
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	P = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	I = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	D = *((float*)(&temp));temp=0;
	
	switch(commBuf.MoveCtrl)
	{
		case POS_MODE:
			commBuf.Pos.PidSpeed.P = P;
			commBuf.Pos.PidSpeed.I = I;
			commBuf.Pos.PidSpeed.D = D;
			break;
		case LOAD_MODE:
			commBuf.Load.PidSpeed.P = P;
			commBuf.Load.PidSpeed.I = I;
			commBuf.Load.PidSpeed.D = D;
			break;
		case EXTEN_MODE:
			commBuf.Ext.PidSpeed.P = P;
			commBuf.Ext.PidSpeed.I = I;
			commBuf.Ext.PidSpeed.D = D;
			break;
		default:break;
	}
	mySemaphore.paraconfig.set = 1;
	
	printf("Command=0x%2x DOPESPEEDPID\t\r\n",commBuf.Command);
	printf("MoveCtrl=%d\t\r\n",commBuf.MoveCtrl);
	printf("P=%f\t\r\n",P);
	printf("I=%f\t\r\n",I);
	printf("D=%f\t\r\n",D);
}

/* Fun_18319 */
void DOPERDSPEEDPID_Process(void)//璇诲彇閫熷害PID鍙傛暟
{
	uint8_t movectrl=0,highpressure=0;
	uint16_t i=5;
	movectrl = buf_data_save[i++];
	highpressure = buf_data_save[i++];
	/* Fun_1848 */
	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	i=0;
	Variable_Type_Conversion(UINT8_TYPE,*(uint32_t*)(&movectrl),buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,*(uint32_t*)(&highpressure),buf_com,&i);
	switch(movectrl)
	{
		case POS_MODE:	
					if(highpressure == 0)
					{
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&posAllCtrlPara.SpeedPid.P),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&posAllCtrlPara.SpeedPid.I),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&posAllCtrlPara.SpeedPid.D),buf_com,&i);
					}
					else
					{
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&posAllCtrlPara.SpeedPid_Hp.P),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&posAllCtrlPara.SpeedPid_Hp.I),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&posAllCtrlPara.SpeedPid_Hp.D),buf_com,&i);
					}
		break;
		case LOAD_MODE: 
					if(highpressure == 0)
					{
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&loadAllCtrlPara.SpeedPid.P),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&loadAllCtrlPara.SpeedPid.I),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&loadAllCtrlPara.SpeedPid.D),buf_com,&i);
					}
					else
					{
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&loadAllCtrlPara.SpeedPid_Hp.P),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&loadAllCtrlPara.SpeedPid_Hp.I),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&loadAllCtrlPara.SpeedPid_Hp.D),buf_com,&i);
					}
		break;
		case EXTEN_MODE:	
					if(highpressure == 0)
					{
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&extAllCtrlPara.SpeedPid.P),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&extAllCtrlPara.SpeedPid.I),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&extAllCtrlPara.SpeedPid.D),buf_com,&i);
					}
					else
					{
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&extAllCtrlPara.SpeedPid_Hp.P),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&extAllCtrlPara.SpeedPid_Hp.I),buf_com,&i);
						Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&extAllCtrlPara.SpeedPid_Hp.D),buf_com,&i);
					}
		break;
		default:break;
	}
	send_ring_fifo_push(i,buf_com,0x86);//涓嬩綅鏈轰細杩斿洖0x88鍔熻兘鐮?
}

/* Fun_18320 */
void DOPEWRSPEEDPID_Process(void)//鍐欏叆閫熷害PID鍙傛暟
{
	uint32_t temp=0; 
	uint16_t i=5;
	float P,I,D;
	commBuf.Command = DOPEWRSPEEDPID;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	//杩愬姩妯″紡鍙戠敓鏀瑰彉
	commBuf.MoveCtrl = buf_data_save[i++];
	commBuf.HighPressure = buf_data_save[i++];
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	P = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	I = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	D = *((float*)(&temp));temp=0;
	
	switch(commBuf.MoveCtrl)
	{
		case POS_MODE:
			if(commBuf.HighPressure == 0){
			commBuf.Pos.PidSpeed.P = P;
			commBuf.Pos.PidSpeed.I = I;
			commBuf.Pos.PidSpeed.D = D;
			}
			else{
			commBuf.Pos.PidSpeed_Hp.P = P;
			commBuf.Pos.PidSpeed_Hp.I = I;
			commBuf.Pos.PidSpeed_Hp.D = D;
			}
			break;
		case LOAD_MODE:
			if(commBuf.HighPressure == 0){
			commBuf.Load.PidSpeed.P = P;
			commBuf.Load.PidSpeed.I = I;
			commBuf.Load.PidSpeed.D = D;
			}
			else{
			commBuf.Load.PidSpeed_Hp.P = P;
			commBuf.Load.PidSpeed_Hp.I = I;
			commBuf.Load.PidSpeed_Hp.D = D;
			}
			break;
		case EXTEN_MODE:
			if(commBuf.HighPressure == 0){
			commBuf.Ext.PidSpeed.P = P;
			commBuf.Ext.PidSpeed.I = I;
			commBuf.Ext.PidSpeed.D = D;
			}
			else{
			commBuf.Ext.PidSpeed_Hp.P = P;
			commBuf.Ext.PidSpeed_Hp.I = I;
			commBuf.Ext.PidSpeed_Hp.D = D;
			}
			break;
		default:break;
	}
	mySemaphore.paraconfig.write = 1;

	printf("Command=0x%2x DOPEWRSPEEDPID\t\r\n",commBuf.Command);
	printf("MoveCtrl=%d\t\r\n",commBuf.MoveCtrl);
	printf("Highpressure=%d\t\r\n",commBuf.HighPressure);
	printf("P=%f\t\r\n",P);
	printf("I=%f\t\r\n",I);
	printf("D=%f\t\r\n",D);
}

/* Fun_18321 */
void DOPEFEEDFORWARD_Process(void)//璁剧疆閫熷害鍓嶉鍙傛暟鎸囦护
{
	uint8_t movectrl=0,i=5;
	float FFP=0;
	uint32_t Delay=0,temp=0;
	commBuf.Command = DOPEFEEDFORWARD;//鐢ㄤ簬鍦╨oopcontrol涓綔妯″紡鍒囨崲
	commBuf.MoveCtrl = buf_data_save[i++];
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	FFP = *((float*)(&temp));temp=0;
	temp += (buf_data_save[i++]<<24);temp += (buf_data_save[i++]<<16);temp += (buf_data_save[i++]<<8);temp += (buf_data_save[i++]);
	Delay = temp;temp=0;
	switch(movectrl)
	{
		case POS_MODE:	
					commBuf.Pos.Ffd.SpeedFFP = FFP;
					commBuf.Pos.Ffd.PosDelay = Delay;
		break;
		case LOAD_MODE:  
					commBuf.Load.Ffd.SpeedFFP = FFP;
					commBuf.Load.Ffd.PosDelay = Delay;
		break;
		case EXTEN_MODE:	
					commBuf.Ext.Ffd.SpeedFFP = FFP;
					commBuf.Ext.Ffd.PosDelay = Delay;
		break;
		default:break;
	}
	mySemaphore.paraconfig.set = 1;
	
	printf("Command=0x%2x DOPEFEEDFORWARD\t\r\n",commBuf.Command);
	printf("MoveCtrl=%d\t\r\n",commBuf.MoveCtrl);
	printf("FFP=%f\t\r\n",FFP);
	printf("Delay=%d\t\r\n",Delay);
}

/* Fun_ */
void UTC_SET_ON_POS_MSG_Process(utcSetOnPosMsg_t *_utcSetOnPosMsg)//绉诲姩鎸囦护瀹屾垚鍚?涓嬩綅鏈哄彂閫侀€氱煡PC
{
	uint16_t i=0;
	_utcSetOnPosMsg->DoPError = 0;
	_utcSetOnPosMsg->usTAN = ++AL.usTAN;

	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	Variable_Type_Conversion(UINT16_TYPE,(uint32_t)_utcSetOnPosMsg->DoPError,buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)_utcSetOnPosMsg->Reached,buf_com,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&_utcSetOnPosMsg->Time),buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)_utcSetOnPosMsg->Control,buf_com,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&_utcSetOnPosMsg->Position),buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)_utcSetOnPosMsg->DControl,buf_com,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&_utcSetOnPosMsg->Destination),buf_com,&i);
	Variable_Type_Conversion(UINT16_TYPE,(uint32_t)_utcSetOnPosMsg->usTAN,buf_com,&i);

	send_ring_fifo_push(i,buf_com,UTC_SET_ON_POS_MSG);//涓嬩綅鏈轰細杩斿洖0x88鍔熻兘鐮?
}

/* Fun_ */
void UTC_SET_ON_TPOS_MSG_Process(utcSetOnTPosMsg_t *_utcSetOnTPosMsg)//绉诲姩鎸囦护瀹屾垚鍚?涓嬩綅鏈哄彂閫侀€氱煡PC
{
	uint16_t i=0;
	_utcSetOnTPosMsg->DoPError = 0;
	_utcSetOnTPosMsg->usTAN = ++AL.usTAN;

	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	Variable_Type_Conversion(UINT16_TYPE,(uint32_t)_utcSetOnTPosMsg->DoPError,buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)_utcSetOnTPosMsg->Reached,buf_com,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&_utcSetOnTPosMsg->Time),buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)_utcSetOnTPosMsg->Control,buf_com,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&_utcSetOnTPosMsg->Position),buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)_utcSetOnTPosMsg->DControl,buf_com,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&_utcSetOnTPosMsg->Destination),buf_com,&i);
	Variable_Type_Conversion(UINT16_TYPE,(uint32_t)_utcSetOnTPosMsg->usTAN,buf_com,&i);

	send_ring_fifo_push(i,buf_com,UTC_SET_ON_TPOS_MSG);//涓嬩綅鏈轰細杩斿洖0x88鍔熻兘鐮?
}

/* Fun_ */
void UTC_SET_ON_LPOS_MSG_Process(utcSetOnLPosMsg_t *_utcSetOnLPosMsg)//绉诲姩鎸囦护瀹屾垚鍚?涓嬩綅鏈哄彂閫侀€氱煡PC
{
	uint16_t i=0;
	_utcSetOnLPosMsg->DoPError = 0;
	_utcSetOnLPosMsg->usTAN = ++AL.usTAN;

	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	Variable_Type_Conversion(UINT16_TYPE,(uint32_t)_utcSetOnLPosMsg->DoPError,buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)_utcSetOnLPosMsg->Reached,buf_com,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&_utcSetOnLPosMsg->Time),buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)_utcSetOnLPosMsg->Control,buf_com,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&_utcSetOnLPosMsg->Position),buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)_utcSetOnLPosMsg->DControl,buf_com,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&_utcSetOnLPosMsg->Destination),buf_com,&i);
	Variable_Type_Conversion(UINT16_TYPE,(uint32_t)_utcSetOnLPosMsg->usTAN,buf_com,&i);

	send_ring_fifo_push(i,buf_com,UTC_SET_ON_LPOS_MSG);//涓嬩綅鏈轰細杩斿洖0x88鍔熻兘鐮?
}

/* Fun_ */
void UTC_SET_ON_SFT_MSG_Process(utcSetOnSftMsg_t *_utcSetOnSftMsg)//绉诲姩鎸囦护瀹屾垚鍚?涓嬩綅鏈哄彂閫侀€氱煡PC
{
	uint16_t i=0;
	_utcSetOnSftMsg->DoPError = 0;
	_utcSetOnSftMsg->Upper = 0;
	_utcSetOnSftMsg->Time = 0;
	_utcSetOnSftMsg->Control = 0;
	_utcSetOnSftMsg->Position = 0;
	_utcSetOnSftMsg->usTAN = ++AL.usTAN;

	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	Variable_Type_Conversion(UINT16_TYPE,(uint32_t)_utcSetOnSftMsg->DoPError,buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)_utcSetOnSftMsg->Upper,buf_com,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&_utcSetOnSftMsg->Time),buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)_utcSetOnSftMsg->Control,buf_com,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&_utcSetOnSftMsg->Position),buf_com,&i);
	Variable_Type_Conversion(UINT16_TYPE,(uint32_t)_utcSetOnSftMsg->usTAN,buf_com,&i);

	send_ring_fifo_push(i,buf_com,UTC_SET_ON_SFT_MSG);//涓嬩綅鏈轰細杩斿洖0x88鍔熻兘鐮?
}

/* Fun_ */
void UTC_SET_ON_RUNTIME_MSG_Process(utcSetOnRuntimeError_t *_utcSetOnRuntimeError)//绉诲姩鎸囦护瀹屾垚鍚?涓嬩綅鏈哄彂閫侀€氱煡PC
{
	uint16_t i=0;
	_utcSetOnRuntimeError->DoPError = 0;
	_utcSetOnRuntimeError->ErrorNumber = 0;
	_utcSetOnRuntimeError->Time = 0;
	_utcSetOnRuntimeError->Device = 0;
	_utcSetOnRuntimeError->Bits = 0;
	_utcSetOnRuntimeError->usTAN = ++AL.usTAN;

	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	Variable_Type_Conversion(UINT16_TYPE,(uint32_t)_utcSetOnRuntimeError->DoPError,buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)_utcSetOnRuntimeError->ErrorNumber,buf_com,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&_utcSetOnRuntimeError->Time),buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)_utcSetOnRuntimeError->Device,buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)_utcSetOnRuntimeError->Bits,buf_com,&i);
	Variable_Type_Conversion(UINT16_TYPE,(uint32_t)_utcSetOnRuntimeError->usTAN,buf_com,&i);

	send_ring_fifo_push(i,buf_com,UTC_SET_ON_RUNTIME_MSG);//涓嬩綅鏈轰細杩斿洖0x88鍔熻兘鐮?
}

/* Fun_ */
void UTC_SET_ON_SYSTEM_MSG_Process(utcSetOnSystemMsg_t *_utcSetOnSystemMsg)//绉诲姩鎸囦护瀹屾垚鍚?涓嬩綅鏈哄彂閫侀€氱煡PC
{
	uint16_t i=0,j=0;
	char * testString = "test of system msg";
	_utcSetOnSystemMsg->DoPError = 0;
	_utcSetOnSystemMsg->MsgNumber	= 0;
	_utcSetOnSystemMsg->Time = 0;
	strncpy(_utcSetOnSystemMsg->Text, testString, SYSTEM_MSG_TEXT_LEN);
	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	Variable_Type_Conversion(UINT16_TYPE,(uint32_t)_utcSetOnSystemMsg->DoPError,buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)_utcSetOnSystemMsg->MsgNumber,buf_com,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&_utcSetOnSystemMsg->Time),buf_com,&i);
	for(j = 0;j < SYSTEM_MSG_TEXT_LEN;j++){
		Variable_Type_Conversion(UINT8_TYPE,(uint32_t)_utcSetOnSystemMsg->Text[j],buf_com,&i);
	}

	send_ring_fifo_push(i,buf_com,UTC_SET_ON_SYSTEM_MSG);//涓嬩綅鏈轰細杩斿洖0x88鍔熻兘鐮?
}

/* Fun_ */
void UTC_SET_ON_DEBUG_MSG_Process(utcSetOnDebugMsg_t *_utcSetOnDebugMsg)//绉诲姩鎸囦护瀹屾垚鍚?涓嬩綅鏈哄彂閫侀€氱煡PC
{
	uint16_t i=0,j=0;
	char * testString = "test of debug msg";
	_utcSetOnDebugMsg->DoPError = 0;
	_utcSetOnDebugMsg->MsgType = 0;
	_utcSetOnDebugMsg->Time = 0;
	strncpy(_utcSetOnDebugMsg->Text, testString, strlen(testString));
	memset(buf_com,0,sizeof(buf_com));//buf娓呯┖
	Variable_Type_Conversion(UINT16_TYPE,(uint32_t)_utcSetOnDebugMsg->DoPError,buf_com,&i);
	Variable_Type_Conversion(UINT8_TYPE,(uint32_t)_utcSetOnDebugMsg->MsgType,buf_com,&i);
	Variable_Type_Conversion(FLOAT_TYPE,*(uint32_t*)(&_utcSetOnDebugMsg->Time),buf_com,&i);
	for(j = 0;j < DEBUG_MSG_TEXT_LEN;j++){
		Variable_Type_Conversion(UINT8_TYPE,(uint32_t)_utcSetOnDebugMsg->Text[j],buf_com,&i);
	}

	send_ring_fifo_push(i,buf_com,UTC_SET_ON_DEBUG_MSG);//涓嬩綅鏈轰細杩斿洖0x88鍔熻兘鐮?
}

void sendUtcSetOn(void){
	if(mySemaphore.utcSetOn.SetOnPos == 1){
		mySemaphore.utcSetOn.SetOnPos = 0;
		UTC_SET_ON_POS_MSG_Process(&utcSetOnPosMsg);
	}
	if(mySemaphore.utcSetOn.SetOnTPos == 1){
		mySemaphore.utcSetOn.SetOnTPos = 0;
		UTC_SET_ON_TPOS_MSG_Process(&utcSetOnTPosMsg);
	}
	if (mySemaphore.utcSetOn.SetOnLPos == 1){
		mySemaphore.utcSetOn.SetOnLPos = 0;
		UTC_SET_ON_LPOS_MSG_Process(&utcSetOnLPosMsg);
	}
	if (mySemaphore.utcSetOn.SetOnSft == 1){
		mySemaphore.utcSetOn.SetOnSft = 0;
		UTC_SET_ON_SFT_MSG_Process(&utcSetOnSftMsg);
	}
	if (mySemaphore.utcSetOn.SetOnRuntime == 1){
		mySemaphore.utcSetOn.SetOnRuntime = 0;
		UTC_SET_ON_RUNTIME_MSG_Process(&utcSetOnRuntimeError);
	}
	if (mySemaphore.utcSetOn.SetOnSystem == 1){
		mySemaphore.utcSetOn.SetOnSystem = 0;
		UTC_SET_ON_SYSTEM_MSG_Process(&utcSetOnSystemMsg);//trigger this function after utcSetOnSystemMsgUpdate is called.
	}
	if (mySemaphore.utcSetOn.SetOnDebug == 1){
		mySemaphore.utcSetOn.SetOnDebug = 0;
		UTC_SET_ON_DEBUG_MSG_Process(&utcSetOnDebugMsg);//trigger this function after utcSetOnDebugMsgUpdate is called.
	}
}


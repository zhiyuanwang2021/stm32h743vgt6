#include "control.h"
#include "cmsis_os.h"
#include "posGenerator.h"
#include "gParameter.h"
#include "Servo_driver.h"
#include "usart.h"
#include "communicate.h"
#include "Encoder.h"
#include "tim.h"
#include "lsm.h"
#include "in_out.h"
#include "pid.h"
#include "adrc.h"
#include "sensor.h"
#include <elog.h>

#ifndef sign
	#define sign(x) ((x)>0?1:((x)<0?-1:1))
#endif

uint8_t pg_init_err=0;
 float MAINTANCE_out=0;
 float MAINTANCE_err_last = 0;

void CommandUpdate(void);
void closeloopParaUpdate(void);
void POS_Process(void);
void LOAD_Process(void);
void EXTEN_Process(void);
void CommandCompletionMonitor(void);
void TrackSwitch(void);
void halt_destination_caclu(void);
uint8_t DestWndMonitor(float setpos,float nowpos,WND* wnd,uint32_t* timer);
float lsm_predicte_process(LSM_PREDICTOR_STRUCT* pre,LS_SLIDE_STRUCT* ls,float x,float y_expect);
uint16_t periodCalcu(float speed,double sensitive,ctrlPeriod_t* period);
float posExtLimitCalculate(const float _limit,const uint8_t _limitMode,const float _sftLimitMax,const float _sftLimitMin,
						const float _limitPos,const float _desPos,const float _destiantion);
void combinedMoveCommandCompletionMonitor(MOVE_MYSEMAPHORE* _move,combinedMove_t* _combinedMove,const uint16_t* _lpusTAN);

void controlLoop()
{	
	//控制指令完成度检测
		CommandCompletionMonitor();
	//指令更新
		CommandUpdate();
	//闭环参数更新
		closeloopParaUpdate();
	//轨迹规划  模式切换
		TrackSwitch();
	//执行具体闭环控制
		if(AL.complete_state.flag == 0)
		{
			/* Fun_176 */
			switch(AL.movectrl)
			{
				case POS_MODE:
						POS_Process();
				break;
				case LOAD_MODE:
						LOAD_Process();
				break;
				case EXTEN_MODE:
						EXTEN_Process();
				break;
				default:break;
			}
		}
		else
		{
			switch(AL.movectrl)
			{
				case POS_MODE:
						//AL.posCtrl.posLoopCtrlOut = 0;
						svPWM.pos = 0;
						svAO.pos = 0;
				break;
				case LOAD_MODE:
						//AL.loadCtrl.posLoopCtrlOut = 0;
						//AL.loadCtrl.speedLoopCtrlOut =	0;
						//AL.posCtrl.posLoopCtrlOut = 0;
						svPWM.load = 0;
						svAO.load = 0;
				break;
				case EXTEN_MODE:
						
				break;
				default:break;
			}

		}
}

void posExtParamInit(uint8_t _command);
/*
函数名：CommandUpdate
功能：如果接收到新运动指令mySemaphore.move.pgInit会被置1，此时要进行MoveCtrl，Command指令更新
*/
void CommandUpdate(void)
{
	if(mySemaphore.move.pgInit == 1)
	{
		if(mySemaphore.move.combinedMoveExecute == BEM_CMD_IDLE){
			AL.movectrl_last = AL.movectrl;
			AL.movectrl = commBuf.MoveCtrl;
			AL.command_last = AL.command;
			AL.command = commBuf.Command;
			AL.lpusTAN = commBuf.lpusTAN;
			AL.complete_state.flag =0;
			AL.halt_flag = 0;
			AL.wnd_timer=0;
			AL.wnd_timerPos=0;
			AL.wnd_timerLoad=0;
			AL.wnd_timerExt=0;
			MAINTANCE_test = 0;
			MAINTANCE_out = 0;
		}else if(mySemaphore.move.combinedMoveExecute == BEM_CMD_START){
			if((combinedMove.blockHeader.ModeFlags & BHM_CMD_DWND) == BHM_CMD_DWND){
				AL.movectrl_last = AL.movectrl;
				AL.movectrl = combinedMove.MoveCtrl[combinedMove.executeStep];
				AL.command_last = AL.command;
				AL.command = combinedMove.command[combinedMove.executeStep];
				//set move command param
				commBuf.LimitMode = combinedMove.LimitMode[combinedMove.executeStep];
				commBuf.DestinationCtrl = combinedMove.DestinationCtrl[combinedMove.executeStep];
				commBuf.DestinationMode = combinedMove.DestinationMode[combinedMove.executeStep];
				commBuf.lpusTAN = combinedMove.lpusTAN[combinedMove.executeStep];
				commBuf.trig.TriggerCtrl = combinedMove.TriggerCtrl[combinedMove.executeStep];
				commBuf.haltDelay = combinedMove.Delay[combinedMove.executeStep];
				commBuf.Speed = combinedMove.Speed[combinedMove.executeStep];
				commBuf.Acc = combinedMove.Acc[combinedMove.executeStep];
				commBuf.Dec = combinedMove.Dec[combinedMove.executeStep];
				commBuf.Destination = combinedMove.Destination[combinedMove.executeStep];
				commBuf.DecDestination = combinedMove.DecDestination[combinedMove.executeStep];
				commBuf.DecLimit = combinedMove.DecLimit[combinedMove.executeStep];
				commBuf.Limit = combinedMove.Limit[combinedMove.executeStep];
				commBuf.trig.Limit = combinedMove.Limit[combinedMove.executeStep];
				commBuf.DecLimit = combinedMove.DecLimit[combinedMove.executeStep];
				commBuf.trig.Trigger = combinedMove.Trigger[combinedMove.executeStep];
				commBuf.lpusTAN = combinedMove.lpusTAN[combinedMove.executeStep];
				combinedMove.executeStep++;
				//move ctrl related state clear
				AL.lpusTAN = commBuf.lpusTAN;
				AL.complete_state.flag =0;
				AL.halt_flag = 0;
				AL.wnd_timer=0;
				AL.wnd_timerPos=0;
				AL.wnd_timerLoad=0;
				AL.wnd_timerExt=0;
				MAINTANCE_test = 0;
				MAINTANCE_out = 0;
			}else if(combinedMove.blockHeader.ModeFlags == BHM_CMD_MESSAGE){
				//may some other operation will be needed to add here.
				AL.movectrl_last = AL.movectrl;
				AL.movectrl = commBuf.MoveCtrl;
				AL.command_last = AL.command;
				AL.command = commBuf.Command;
				AL.lpusTAN = commBuf.lpusTAN;
				AL.complete_state.flag =0;
				AL.halt_flag = 0;
				AL.wnd_timer=0;
				AL.wnd_timerPos=0;
				AL.wnd_timerLoad=0;
				AL.wnd_timerExt=0;
				MAINTANCE_test = 0;
				MAINTANCE_out = 0;
			}
		}else if(mySemaphore.move.combinedMoveExecute == BEM_CMD_DISCARD){
			//keep now movectrl 
			AL.movectrl_last = AL.movectrl;
			AL.movectrl = commBuf.MoveCtrl;
			//call halt
			AL.command_last = AL.command;
			AL.command = DOPEHALT;
			AL.lpusTAN = commBuf.lpusTAN;
		}
		//PosExt Param Init or clear
		posExtParamInit(AL.command);
	}
}

/**
 * @note  Fun161
 * @todo  add strain parameter
 * @brief update close loop parameters,
 * 		  like pid paramters,speed pid parameters,which involve in real-time calculations
*/
void closeloopParaUpdate(void){
	if(mySemaphore.pid.pos == 1){
		pidParaUpdatePos();
		mySemaphore.pid.pos = 0;
	}
	if(mySemaphore.pid.load == 1){
		pidParaUpdateLoad();
		mySemaphore.pid.load = 0;
	}
	if(mySemaphore.pid.ext == 1){
		pidParaUpdateExt();
		mySemaphore.pid.ext = 0;
	}
}


/*
函数名：CommandCompletionMonitor
功能：	完成度检测函数，判断当前运动指令的完成度
*/
void CommandCompletionMonitor_POS(void);//POS_MODE完成度检测
void CommandCompletionMonitor_LOAD(void);//LOAD_MODE完成度检测
void CommandCompletionMonitor_EXTEN(void);//EXT完成度检测
void posExtCompletionMonitorProcess(const ALLCTRLPARA* _allCtrlPara);
extern osMessageQId lpusTANQueueHandle;
/* Fun_12 */
void CommandCompletionMonitor(void)//完成度检测
{
	uint8_t _moveCtrl;
	if(stateFlag.posExtDestReach == reachNone){
		_moveCtrl = AL.movectrl;
	}else if(stateFlag.posExtDestReach == reachPosition){
		_moveCtrl = posext.moveCtrlOrig;
	}
	switch(_moveCtrl)
	{
		case POS_MODE:CommandCompletionMonitor_POS();
			break;
		case LOAD_MODE:CommandCompletionMonitor_LOAD();
			break;
		case EXTEN_MODE:CommandCompletionMonitor_EXTEN();
			break;
		default:break;
	}
}
/* Fun_1211 */
void CommandCompletionMonitor_POS(void)//POS_MODE完成度检测
{
		switch(AL.command)
		{
			case DOPESETOPENLOOPCOMMAND://开环测试指令

				break;
			case DOPEMOVE://会不停向上/向下移动 直至到达限位 没有完成指令
			case DOPEMOVE_A:

				break;
			case DOPEPOS:
			case DOPEPOS_A:
				if(mySemaphore.move.pgInit == 0){
					//目标设定位置和当前实际位置<0.001f 当前规划位置 == 设定目标位置 
					if(DestWndMonitor(AL.posCtrl.setPos,pose.orig,&posAllCtrlPara.Wnd,&AL.wnd_timer)
						&& pg.t <= (uint32_t)((float)pg.t3 * (1.0f+utcSetOnPosMsg.timeoutAllowed))+ posAllCtrlPara.Wnd.WndTime	//最大控制允许时间
						)
					{
						if(stateFlag.DynCycle_OffsetReach == 1)
						{
							stateFlag.DynCycle_OffsetReach = 2;//进行dyncycle
							mySemaphore.move.pgInit = 1;		//重新规划允许位
							AL.command = DOPEDYNCYCLE;			//dyncycle
							log_i("stateFlag.DynCycle_OffsetReach = 2\r\n");
						}
						else{
							AL.complete_state.flag =1;
							utcSetOnPosMsg.Reached = 1;//time not out,and now position in the wnd
							utcSetOnPosMsg.Time = AL.utcTime;//unit:s
							utcSetOnPosMsg.Control = AL.movectrl;
							utcSetOnPosMsg.Position = AL.posCtrl.setPos;
							utcSetOnPosMsg.DControl = AL.movectrl;
							utcSetOnPosMsg.Destination = AL.posCtrl.setPos;
							mySemaphore.utcSetOn.SetOnPos = 1;
						}
					}else if(pg.t > (uint32_t)((float)pg.t3 * (1.0f+utcSetOnPosMsg.timeoutAllowed))+ posAllCtrlPara.Wnd.WndTime	//最大控制允许时间
					&& mySemaphore.move.pgInit == 0){
						AL.complete_state.flag = 1;
						utcSetOnPosMsg.Reached = 0;//timeout and position not in the wnd
						utcSetOnPosMsg.Time = AL.utcTime;//unit:s
						utcSetOnPosMsg.Control = AL.movectrl;
						utcSetOnPosMsg.Position = pose.orig;
						utcSetOnPosMsg.DControl = AL.movectrl;
						utcSetOnPosMsg.Destination = AL.posCtrl.setPos;
						mySemaphore.utcSetOn.SetOnPos = 1;
					}
				}
				break;
			case DOPEHALT:
			case DOPEHALT_A:
				//目标设定位置和当前实际位置<0.001f 当前规划位置 == 设定目标位置 
				if(DestWndMonitor(AL.posCtrl.setPos,pose.orig,&wndHaltPos,&AL.wnd_timer)
					&& mySemaphore.move.pgInit == 0)
				{
					if(stateFlag.posExtDestReach == 1 || stateFlag.posExtDestReach == 3){
						log_i("posExtDestReach Complete !\r\n");
						AL.complete_state.flag =1;
					}
					else
						AL.complete_state.flag =1;
					
					if(AL.emergency_state.processing == 1)
					{
						AL.emergency_state.complete=1;
						log_i("emergency_state.complete == 1\r\n");
					}
				}
				break;
			case DOPESHALT:
				//目标设定位置和当前实际位置<WndSize/2 当前规划位置 == 设定目标位置 
				if(DestWndMonitor(AL.posCtrl.setPos,pose.orig,&wndHaltPos,&AL.wnd_timer)
					&& mySemaphore.move.pgInit == 0)
				{
					AL.complete_state.flag =1;
					
					if(AL.emergency_state.processing == 1)
					{
						AL.emergency_state.complete=1;
						log_e("emergency_state.complete == 1\r\n");
					}
				}
				break;
			case DOPETRIG:
			case DOPETRIG_A:
				//trigger monitor
				if( mySemaphore.move.pgInit == 0){
					if(utcTrig.TriggerCtrl == POS_MODE){
						if(sign(utcTrig.Trigger)*(utcTrig.Trigger-pose.orig)<=0){
							AL.trigState.flag = successTrigger;
							AL.complete_state.flag = 1;
						}
					}else if(utcTrig.TriggerCtrl == LOAD_MODE){
						if(sign(utcTrig.Trigger)*(utcTrig.Trigger-force.filter)<=0){
							AL.trigState.flag = successTrigger;
							AL.complete_state.flag = 1;
						}
					}else if(utcTrig.TriggerCtrl == EXTEN_MODE){
						if(sign(utcTrig.Trigger)*(utcTrig.Trigger-strain1.filter)<=0){
							AL.trigState.flag = successTrigger;
							AL.complete_state.flag = 1;
						}
					}
					if(AL.trigState.flag == successTrigger){
						utcSetOnTPosMsg.Reached = 1;
						utcSetOnTPosMsg.Time = AL.utcTime;//unit:s
						utcSetOnTPosMsg.Control = AL.movectrl;
						utcSetOnTPosMsg.Position = utcTrig.Limit;
						utcSetOnTPosMsg.DControl = utcTrig.TriggerCtrl;
						utcSetOnTPosMsg.Destination = utcTrig.Trigger;
						mySemaphore.utcSetOn.SetOnTPos = 1;
					}
					//Limit arrive monitor
					if(DestWndMonitor(AL.posCtrl.setPos,pose.orig,&posAllCtrlPara.Wnd,&AL.wnd_timer)){
						log_e("DOPETrig POS limit arrive");
						AL.trigState.flag = limitArrive;
						AL.complete_state.flag = 1;
					}
				}
				break;
			case DOPEPOSEXT:
				//according to the destinationCtrl,the destination is being monitored.
				if(stateFlag.posExtDestReach == reachNone){
					switch(posext.destinationctrl){
						case POS_MODE:
							if(DestWndMonitor(posext.Destination,pose.orig,&wndPosExtPos,&AL.wnd_timer)
							&& mySemaphore.move.pgInit == 0){
								posExtCompletionMonitorProcess(&posAllCtrlPara);
							}
						break;
						case LOAD_MODE:
							if(DestWndMonitor(posext.Destination,force.filter,&wndPosExtLoad,&AL.wnd_timer)
							//if((sign(posext.Destination)*(posext.Destination - force.filter) <= 0)
							&& mySemaphore.move.pgInit == 0){
								posExtCompletionMonitorProcess(&loadAllCtrlPara);
							}
						break;
						case EXTEN_MODE:
							if(DestWndMonitor(posext.Destination,strain1.filter,&wndPosExtExt,&AL.wnd_timer)
							&& mySemaphore.move.pgInit == 0){
								posExtCompletionMonitorProcess(&extAllCtrlPara);
							}
						break;
					}
				}
				if(DestWndMonitor(posext.limit,pose.filter,&wndPosExtPos,&AL.wnd_timer)
					&& mySemaphore.move.pgInit == 0){
					log_i("DoPEPosExt moveCtrl:0x%2X destinationCtrl:0x%2X,limit arrive!",AL.movectrl,posext.destinationctrl);
					utcSetOnLPosMsg.Reached = 1;
					utcSetOnLPosMsg.Time = AL.utcTime;//unit:s
					utcSetOnLPosMsg.Control = AL.movectrl;
					utcSetOnLPosMsg.Position = posext.limit;
					utcSetOnLPosMsg.DControl = posext.destinationctrl;
					utcSetOnLPosMsg.Destination = posext.Destination;
					mySemaphore.utcSetOn.SetOnLPos = 1;
					AL.complete_state.flag = 1;
				}
				break;
			case DOPECYCLE:
				//目标设定位置和当前实际位置<0.001f 当前规划位置 == 设定目标位置 
				if(pg.st == AL.posCtrl.setPos && mySemaphore.move.pgInit == 0)
				{
					//mySemaphore.move.pgInit = 1;
					
					if(pgcycle.count_cycle >= pgcycle.cycles)
					{
						if(DestWndMonitor(AL.posCtrl.setPos,pose.orig,&posAllCtrlPara.Wnd,&AL.wnd_timer))
							AL.complete_state.flag =1;
					}
					else
					AL.halt_flag = 1;
				}
				if(AL.halt_flag == 1)
				{
					AL.halt++;
					switch(pgcycle.count_step)
					{
						case 1:
							if(AL.halt >= pgcycle.halt1*1000)
							{
							mySemaphore.move.pgInit = 1;
							AL.halt_flag = 0;
							AL.halt=0;
							}
							break;
						case 2:
							if(AL.halt >= pgcycle.halt2*1000)
							{
							mySemaphore.move.pgInit = 1;
							AL.halt_flag = 0;
							AL.halt=0;
							}
							break;
						default:
							break;
					}
				}
				break;
			case DOPEDYNCYCLE:
				if(pgdyn.modify != 0 && pg.t >= dyncycle_modify.t5)//表示轨迹生成已全部完成
				{
					//判断当前位置与Destination位置是否一致  后面替换成WND ***
					if(DestWndMonitor(AL.posCtrl.setPos,pose.orig,&posAllCtrlPara.Wnd,&AL.wnd_timer)
					&& mySemaphore.move.pgInit == 0)
						{
							AL.complete_state.flag =1;
							stateFlag.DynCycle_OffsetReach = 0;
							memset(&pgdyn,0,sizeof(pgdyn));
						}
				}
				else if(pgdyn.modify == 0 && pg.t >= dyncycle_normal.t3)
				{
					//判断当前位置与Destination位置是否一致  后面替换成WND ***
					if(DestWndMonitor(AL.posCtrl.setPos,pose.orig,&posAllCtrlPara.Wnd,&AL.wnd_timer)
					&& mySemaphore.move.pgInit == 0)
						{
							AL.complete_state.flag =1;
							stateFlag.DynCycle_OffsetReach = 0;
							memset(&pgdyn,0,sizeof(pgdyn));
						}
				}
				break;
			default:
				break;
		}
		
		if(AL.complete_state.flag == 1 && AL.command != 0) //上一指令完成 指令归位0x00
		{
			pidInitPos();
			pid_pos.calcuRun = 0;
			log_i("POS_MODE 0x%2xCommand Complete",AL.command);		
			AL.command = 0x00;
			// combinedMove command completion monitor
			combinedMoveCommandCompletionMonitor(&mySemaphore.move,&combinedMove,&AL.lpusTAN);
		}
}

/* Fun_1212 */
void CommandCompletionMonitor_LOAD(void)//LOAD_MODE完成度检测***
{
		switch(AL.command)
		{
			case DOPEMOVE://会不停向上/向下移动 直至到达限位 没有完成指令
			case DOPEMOVE_A:

				break;
			case DOPEPOS:
			case DOPEPOS_A:
				//目标设定位置和当前实际位置<0.001f 当前规划位置 == 设定目标位置 
				if(DestWndMonitor(AL.loadCtrl.setPos,force.filter,&loadAllCtrlPara.Wnd,&AL.wnd_timer)
					&& pgLoad.t <= (uint32_t)((float)pgLoad.t3 * (1.0f+utcSetOnPosMsg.timeoutAllowed))+ loadAllCtrlPara.Wnd.WndTime	//最大控制允许时间
					&& mySemaphore.move.pgInit == 0)
				{
					if(stateFlag.DynCycle_OffsetReach == 1)
					{
						stateFlag.DynCycle_OffsetReach = 2;//进行dyncycle
						mySemaphore.move.pgInit = 1;		//重新规划允许位
						AL.command = DOPEDYNCYCLE;			//dyncycle
						log_i("stateFlag.DynCycle_OffsetReach = 2\r\n");
					}
					else{
						AL.complete_state.flag =1;
						utcSetOnPosMsg.Reached = 1;//time not out,and now position in the wnd
						utcSetOnPosMsg.Time = AL.utcTime;//unit:s
						utcSetOnPosMsg.Control = AL.movectrl;
						utcSetOnPosMsg.Position = AL.loadCtrl.setPos;
						utcSetOnPosMsg.DControl = AL.movectrl;
						utcSetOnPosMsg.Destination = AL.loadCtrl.setPos;
						mySemaphore.utcSetOn.SetOnPos = 1;
					}
				}else if(pgLoad.t > (uint32_t)((float)pgLoad.t3 * (1.0f+utcSetOnPosMsg.timeoutAllowed))+ loadAllCtrlPara.Wnd.WndTime	//最大控制允许时间
				&& mySemaphore.move.pgInit == 0){
					AL.complete_state.flag =1;
					utcSetOnPosMsg.Reached = 0;//timeout and position not in the wnd
					utcSetOnPosMsg.Time = AL.utcTime;//unit:s
					utcSetOnPosMsg.Control = AL.movectrl;
					utcSetOnPosMsg.Position = force.filter;
					utcSetOnPosMsg.DControl = AL.movectrl;
					utcSetOnPosMsg.Destination = AL.loadCtrl.setPos;
					mySemaphore.utcSetOn.SetOnPos = 1;
				}
				break;
			case DOPEHALT:
				//目标设定位置和当前实际位置<0.001f 当前规划位置 == 设定目标位置 
				if(DestWndMonitor(AL.loadCtrl.setPos,force.filter,&wndHaltLoad,&AL.wnd_timer)
					&& mySemaphore.move.pgInit == 0)				
					{
						AL.complete_state.flag =1;
						if(AL.emergency_state.processing == 1)
						{
							AL.emergency_state.complete=1;
							log_e("emergency_state.complete == 1\r\n");
						}
					}
				break;
			case DOPETRIG:
			case DOPETRIG_A:
				//trigger monitor
				if( mySemaphore.move.pgInit == 0){
					if(utcTrig.TriggerCtrl == POS_MODE){
						if(sign(utcTrig.Trigger)*(utcTrig.Trigger-pose.orig)<=0){
							AL.trigState.flag = successTrigger;
							AL.complete_state.flag = 1;
						}
					}else if(utcTrig.TriggerCtrl == LOAD_MODE){
						if(sign(utcTrig.Trigger)*(utcTrig.Trigger-force.filter)<=0){
							AL.trigState.flag = successTrigger;
							AL.complete_state.flag = 1;
						}
					}else if(utcTrig.TriggerCtrl == EXTEN_MODE){
						if(sign(utcTrig.Trigger)*(utcTrig.Trigger-strain1.filter)<=0){
							AL.trigState.flag = successTrigger;
							AL.complete_state.flag = 1;
						}
					}
					if(AL.trigState.flag == successTrigger){
						utcSetOnTPosMsg.Reached = 1;
						utcSetOnTPosMsg.Time = AL.utcTime;//unit:s
						utcSetOnTPosMsg.Control = AL.movectrl;
						utcSetOnTPosMsg.Position = utcTrig.Limit;
						utcSetOnTPosMsg.DControl = utcTrig.TriggerCtrl;
						utcSetOnTPosMsg.Destination = utcTrig.Trigger;
						mySemaphore.utcSetOn.SetOnTPos = 1;
					}
					//Limit arrive monitor
					if(DestWndMonitor(AL.loadCtrl.setPos,force.filter,&loadAllCtrlPara.Wnd,&AL.wnd_timer)){
						log_e("DOPETrig LOAD limit arrive");
						AL.trigState.flag = limitArrive;
						AL.complete_state.flag = 1;
					}
				}
				break;
			case DOPEPOSEXT:
				//according to the destinationCtrl,the destination is being monitored.
				if(stateFlag.posExtDestReach == reachNone){
					switch(posext.destinationctrl){
						case POS_MODE:
							if(DestWndMonitor(posext.Destination,pose.orig,&wndPosExtPos,&AL.wnd_timer)
							&& mySemaphore.move.pgInit == 0){
								posExtCompletionMonitorProcess(&posAllCtrlPara);
							}
						break;
						case LOAD_MODE:
							if(DestWndMonitor(posext.Destination,force.filter,&wndPosExtLoad,&AL.wnd_timer)
							&& mySemaphore.move.pgInit == 0){
								posExtCompletionMonitorProcess(&loadAllCtrlPara);
							}
						break;
						case EXTEN_MODE:
							if(DestWndMonitor(posext.Destination,strain1.filter,&wndPosExtExt,&AL.wnd_timer)
							&& mySemaphore.move.pgInit == 0){
								posExtCompletionMonitorProcess(&extAllCtrlPara);
							}
						break;
					}
				}
				if(DestWndMonitor(posext.limit,force.filter,&wndPosExtLoad,&AL.wnd_timer)
					&& mySemaphore.move.pgInit == 0){
					log_i("DoPEPosExt moveCtrl:0x%2X destinationCtrl:0x%2X,limit arrive!",AL.movectrl,posext.destinationctrl);
					utcSetOnLPosMsg.Reached = 1;
					utcSetOnLPosMsg.Time = AL.utcTime;//unit:s
					utcSetOnLPosMsg.Control = AL.movectrl;
					utcSetOnLPosMsg.Position = posext.limit;
					utcSetOnLPosMsg.DControl = posext.destinationctrl;
					utcSetOnLPosMsg.Destination = posext.Destination;
					mySemaphore.utcSetOn.SetOnLPos = 1;
					AL.complete_state.flag = 1;
				}
				break;
			case DOPECYCLE:
				//目标设定位置和当前实际位置<0.001f 当前规划位置 == 设定目标位置 
				if(DestWndMonitor(AL.loadCtrl.setPos,force.filter,&loadAllCtrlPara.Wnd,&AL.wnd_timer)
					&& mySemaphore.move.pgInit == 0)
				{
					//mySemaphore.move.pgInit = 1;
					if(pgcycle.count_cycle >= pgcycle.cycles)
					AL.complete_state.flag =1;
					else
					AL.halt_flag = 1;
				}
				if(AL.halt_flag == 1)
				{
					AL.halt++;
					switch(pgcycle.count_step)
					{
						case 1:
							if(AL.halt >= pgcycle.halt1*1000)
							{
							mySemaphore.move.pgInit = 1;
							AL.halt_flag = 0;
							AL.halt=0;
							}
							break;
						case 2:
							if(AL.halt >= pgcycle.halt2*1000)
							{
							mySemaphore.move.pgInit = 1;
							AL.halt_flag = 0;
							AL.halt=0;
							}
							break;
						default:
							break;
					}
				}
				break;
			case DOPEDYNCYCLE://涉及到现场保护的内容还需要调试***
				if(pgdyn.modify != 0 && pg.t >= dyncycle_modify.t5)//表示轨迹生成已全部完成
				{
					//判断当前位置与Destination位置是否一致  后面替换成WND ***
					if(DestWndMonitor(AL.loadCtrl.setPos,force.filter,&loadAllCtrlPara.Wnd,&AL.wnd_timer)
					&& mySemaphore.move.pgInit == 0)
						{
							AL.complete_state.flag =1;
							stateFlag.DynCycle_OffsetReach = 0;
							memset(&pgdyn,0,sizeof(pgdyn));
						}
				}
				else if(pgdyn.modify == 0 && pg.t >= dyncycle_normal.t3)
				{
					//判断当前位置与Destination位置是否一致  后面替换成WND ***
					if(DestWndMonitor(AL.loadCtrl.setPos,force.filter,&loadAllCtrlPara.Wnd,&AL.wnd_timer)
					&& mySemaphore.move.pgInit == 0)
						{
							AL.complete_state.flag =1;
							stateFlag.DynCycle_OffsetReach = 0;
							memset(&pgdyn,0,sizeof(pgdyn));
						}
				}
				break;
			default:
				break;
		}

		if(AL.complete_state.flag == 1 && AL.command != 0) //上一指令完成 指令归位0x00
		{
			pidInitLoad();
			pid_load.calcuRun = 0;
			log_i("LOAD_MODE 0x%2xCommand Complete",AL.command);		
			AL.command = 0x00;
			// combinedMove command completion monitor
			combinedMoveCommandCompletionMonitor(&mySemaphore.move,&combinedMove,&AL.lpusTAN);
		}
}

/* Fun_1213 */
void CommandCompletionMonitor_EXTEN(void)//EXT完成度检测***
{
		switch(AL.command)
		{
			case DOPEMOVE://会不停向上/向下移动 直至到达限位 没有完成指令
			case DOPEMOVE_A:

				break;
			case DOPEPOS:
			case DOPEPOS_A:
				//目标设定位置和当前实际位置<0.001f 当前规划位置 == 设定目标位置 
				if(DestWndMonitor(AL.ext1Ctrl.setPos,strain1.filter,&extAllCtrlPara.Wnd,&AL.wnd_timer)
					&& pgExt.t <= (uint32_t)((float)pgExt.t3 * (1.0f+utcSetOnPosMsg.timeoutAllowed))+ extAllCtrlPara.Wnd.WndTime	//最大控制允许时间
					&& mySemaphore.move.pgInit == 0)
				{
					if(stateFlag.DynCycle_OffsetReach == 1)
					{
						stateFlag.DynCycle_OffsetReach = 2;//进行dyncycle
						mySemaphore.move.pgInit = 1;		//重新规划允许位
						AL.command = DOPEDYNCYCLE;			//dyncycle
						log_i("stateFlag.DynCycle_OffsetReach = 2\r\n");
					}
					else{
						AL.complete_state.flag =1;
						utcSetOnPosMsg.Reached = 1;//time not out,and now position in the wnd
						utcSetOnPosMsg.Time = AL.utcTime;//unit:s
						utcSetOnPosMsg.Control = AL.movectrl;
						utcSetOnPosMsg.Position = AL.ext1Ctrl.setPos;
						utcSetOnPosMsg.DControl = AL.movectrl;
						utcSetOnPosMsg.Destination = AL.ext1Ctrl.setPos;
						mySemaphore.utcSetOn.SetOnPos = 1;
					}
				}else if(pgExt.t > (uint32_t)((float)pgExt.t3 * (1.0f+utcSetOnPosMsg.timeoutAllowed))+ extAllCtrlPara.Wnd.WndTime	//最大控制允许时间
				&& mySemaphore.move.pgInit == 0){
					AL.complete_state.flag =1;
					utcSetOnPosMsg.Reached = 0;//timeout and position not in the wnd
					utcSetOnPosMsg.Time = AL.utcTime;//unit:s
					utcSetOnPosMsg.Control = AL.movectrl;
					utcSetOnPosMsg.Position = strain1.filter;
					utcSetOnPosMsg.DControl = AL.movectrl;
					utcSetOnPosMsg.Destination = AL.ext1Ctrl.setPos;
					mySemaphore.utcSetOn.SetOnPos = 1;
				}
				break;
			case DOPEHALT:
				//目标设定位置和当前实际位置<0.001f 当前规划位置 == 设定目标位置 
				if(pg.st == AL.posCtrl.setPos && (fabs(AL.posCtrl.setPos - pose.orig) <0.001f) && mySemaphore.move.pgInit == 0)
				{
					AL.complete_state.flag =1;
					if(AL.emergency_state.processing == 1)
					{
						AL.emergency_state.complete=1;
						log_e("emergency_state.complete == 1\r\n");
					}
				}
				break;
			case DOPETRIG:
			case DOPETRIG_A:
				//trigger monitor
				if( mySemaphore.move.pgInit == 0){
					if(utcTrig.TriggerCtrl == POS_MODE){
						if(sign(utcTrig.Trigger)*(utcTrig.Trigger-pose.orig)<=0){
							AL.trigState.flag = successTrigger;
							AL.complete_state.flag = 1;
						}
					}else if(utcTrig.TriggerCtrl == LOAD_MODE){
						if(sign(utcTrig.Trigger)*(utcTrig.Trigger-force.filter)<=0){
							AL.trigState.flag = successTrigger;
							AL.complete_state.flag = 1;
						}
					}else if(utcTrig.TriggerCtrl == EXTEN_MODE){
						if(sign(utcTrig.Trigger)*(utcTrig.Trigger-strain1.filter)<=0){
							AL.trigState.flag = successTrigger;
							AL.complete_state.flag = 1;
						}
					}
					if(AL.trigState.flag == successTrigger){
						utcSetOnTPosMsg.Reached = 1;
						utcSetOnTPosMsg.Time = AL.utcTime;//unit:s
						utcSetOnTPosMsg.Control = AL.movectrl;
						utcSetOnTPosMsg.Position = utcTrig.Limit;
						utcSetOnTPosMsg.DControl = utcTrig.TriggerCtrl;
						utcSetOnTPosMsg.Destination = utcTrig.Trigger;
						mySemaphore.utcSetOn.SetOnTPos = 1;
					}
					//Limit arrive monitor
					if(DestWndMonitor(AL.ext1Ctrl.setPos,strain1.filter,&extAllCtrlPara.Wnd,&AL.wnd_timer)){
						log_e("DOPETrig EXTEN limit arrive");
						AL.trigState.flag = limitArrive;
						AL.complete_state.flag = 1;
					}
				}
				break;
			case DOPEPOSEXT:
				//according to the destinationCtrl,the destination is being monitored.
				if(stateFlag.posExtDestReach == reachNone){
					switch(posext.destinationctrl){
						case POS_MODE:
							if(DestWndMonitor(posext.Destination,pose.orig,&wndPosExtPos,&AL.wnd_timer)
							&& mySemaphore.move.pgInit == 0){
								posExtCompletionMonitorProcess(&posAllCtrlPara);
							}
						break;
						case LOAD_MODE:
							if(DestWndMonitor(posext.Destination,force.filter,&wndPosExtLoad,&AL.wnd_timer)
							&& mySemaphore.move.pgInit == 0){
								posExtCompletionMonitorProcess(&loadAllCtrlPara);
							}
						break;
						case EXTEN_MODE:
							if(DestWndMonitor(posext.Destination,strain1.filter,&wndPosExtExt,&AL.wnd_timer)
							&& mySemaphore.move.pgInit == 0){
								posExtCompletionMonitorProcess(&extAllCtrlPara);
							}
						break;
					}
				}
				if(DestWndMonitor(posext.limit,strain1.filter,&wndPosExtExt,&AL.wnd_timer)
					&& mySemaphore.move.pgInit == 0){
					log_i("DoPEPosExt moveCtrl:0x%2X destinationCtrl:0x%2X,limit arrive!",AL.movectrl,posext.destinationctrl);
					utcSetOnLPosMsg.Reached = 1;
					utcSetOnLPosMsg.Time = AL.utcTime;//unit:s
					utcSetOnLPosMsg.Control = AL.movectrl;
					utcSetOnLPosMsg.Position = posext.limit;
					utcSetOnLPosMsg.DControl = posext.destinationctrl;
					utcSetOnLPosMsg.Destination = posext.Destination;
					mySemaphore.utcSetOn.SetOnLPos = 1;
					AL.complete_state.flag = 1;
				}
				break;
			case DOPECYCLE:
				//目标设定位置和当前实际位置<0.001f 当前规划位置 == 设定目标位置 
				if(pg.st == AL.posCtrl.setPos && (fabs(AL.posCtrl.setPos - pose.orig) <0.001f) && mySemaphore.move.pgInit == 0)
				{
					//mySemaphore.move.pgInit = 1;
					if(pgcycle.count_cycle >= pgcycle.cycles)
					AL.complete_state.flag =1;
					else
					AL.halt_flag = 1;
				}
				if(AL.halt_flag == 1)
				{
					AL.halt++;
					switch(pgcycle.count_step)
					{
						case 1:
							if(AL.halt >= pgcycle.halt1*1000)
							{
							mySemaphore.move.pgInit = 1;
							AL.halt_flag = 0;
							AL.halt=0;
							}
							break;
						case 2:
							if(AL.halt >= pgcycle.halt2*1000)
							{
							mySemaphore.move.pgInit = 1;
							AL.halt_flag = 0;
							AL.halt=0;
							}
							break;
						default:
							break;
					}
				}
				break;
			case DOPEDYNCYCLE:
				
				break;
			default:
				break;
		}
		if(AL.complete_state.flag == 1 && AL.command != 0) //上一指令完成 指令归位0x00
		{
			//pidInitExt();
			//pid_ext.calcuRun = 0;
			log_i("EXT_MODE 0x%2xCommand Complete",AL.command);		
			AL.command = 0x00;
			// combinedMove command completion monitor
			combinedMoveCommandCompletionMonitor(&mySemaphore.move,&combinedMove,&AL.lpusTAN);
		}

		
}

void DOPEDYNCYCLE_StartBefore_Process(void);

void trigPgParamInit(void);
void normalPosGeneratorInit(uint8_t movectrl);

/* Fun_14 */
void TrackSwitch(void)
{
	if(mySemaphore.move.pgInit == 1)
	{	
		//PID run enable
		switch (AL.movectrl)
		{
		case POS_MODE:
			pid_pos.calcuRun = 1;
			break;
		case LOAD_MODE:
			pid_load.calcuRun = 1;
			pid_pos.calcuRun = 1;
			break;
		case EXTEN_MODE:
			pid_ext.calcuRun = 1;
			pid_pos.calcuRun = 1;
			break;
		default:
			break;
		}
		//PID reinit
		if(AL.movectrl != AL.movectrl_last)
		{
			switch (AL.movectrl)
			{
			case POS_MODE:
				pidInitPos();
				//adrcInit(&leso,pose.orig);
				break;
			case LOAD_MODE:
				pidInitLoad();
				//adrcInit(&leso,pose.orig);
				break;
			case EXTEN_MODE:
				pidInitExt();
				break;
			default:
				break;
			}
		}else{
			switch (AL.movectrl)
			{
			case POS_MODE:
				pid_pos.integral=0.0;
				break;
			case LOAD_MODE:
				pid_load.integral=0.0;
				break;
			case EXTEN_MODE:
				pid_ext.integral=0.0;
				break;
			default:
				break;
			}
		}
		//posGenerator param init
		switch(AL.command)
		{
			/* Fun_1711 */
			case DOPEMOVE:
			case DOPEMOVE_A:
				switch(AL.movectrl)
				{
					case POS_MODE:
						if(commBuf.Direction == DIRECTION_UP)
							pg.sg = AL.upperlimit_phy;
						else if(commBuf.Direction == DIRECTION_DOWN)
							pg.sg = AL.lowerlimit_phy;
						pg.vmax = commBuf.Speed;
						pg.a	= commBuf.Acc;//加速度unit/s2
						pg.d	= commBuf.Dec;//减速度 unit/s2	
						AL.posCtrl.setPos = pg.sg;
						break;
					case LOAD_MODE:
						if(commBuf.Direction == DIRECTION_UP)
							pgLoad.sg = 4000;//need to be assigned to a variable
						else if(commBuf.Direction == DIRECTION_DOWN)
							pgLoad.sg = -4000;//need to be assigned to a variable
						pgLoad.vmax = commBuf.Speed;
						pgLoad.a	= commBuf.Acc;//加速度unit/s2
						pgLoad.d	= commBuf.Dec;//减速度 unit/s2
						AL.loadCtrl.setPos = pgLoad.sg;
						break;
					case EXTEN_MODE:
						if(commBuf.Direction == DIRECTION_UP)
							pgExt.sg = 5;//need to be assigned to a variable
						else if(commBuf.Direction == DIRECTION_DOWN)
							pgExt.sg = -5;//need to be assigned to a variable
						pgExt.vmax = commBuf.Speed;
						pgExt.a		= commBuf.Acc;//加速度unit/s2
						pgExt.d		= commBuf.Dec;//减速度 unit/s2
						AL.ext1Ctrl.setPos = pgExt.sg;
						break;
				}
				break;
			/* Fun_1712 */
			case DOPEPOS:
			case DOPEPOS_A:
				switch(AL.movectrl)
				{
					case POS_MODE:
						pg.sg	= commBuf.Destination;
						pg.vmax = commBuf.Speed;
						pg.a	= commBuf.Acc;//加速度unit/s2
						pg.d	= commBuf.Dec;//减速度 unit/s2
						AL.posCtrl.setPos = pg.sg;
					break;
					case LOAD_MODE:
						pgLoad.sg	= commBuf.Destination;
						pgLoad.vmax = commBuf.Speed;
						pgLoad.a	= commBuf.Acc;//加速度unit/s2
						pgLoad.d	= commBuf.Dec;//减速度 unit/s2
						AL.loadCtrl.setPos = pgLoad.sg;
						//log_i("pgLoad.vmax:%f",pgLoad.vmax);
					break;
					case EXTEN_MODE:				
						pgExt.sg	= commBuf.Destination;
						pgExt.vmax 	= commBuf.Speed;
						pgExt.a		= commBuf.Acc;//加速度unit/s2
						pgExt.d		= commBuf.Dec;//减速度 unit/s2
						AL.ext1Ctrl.setPos = pgExt.sg;
						//log_i("pgExt.vmax:%f",pgExt.vmax);
					break;
				}
				break;
			/* Fun_1713 */
			case DOPEHALT:
			case DOPEHALT_A:
				switch(AL.movectrl)
				{
					case POS_MODE:
						pg.a	= commBuf.Acc;//加速度unit/s2
						pg.d	= commBuf.Dec;//减速度unit/s2
						halt_destination_caclu();//生成停止目标位置
						AL.posCtrl.setPos = pg.sg;	
					break;
					case LOAD_MODE:
						pgLoad.a	= commBuf.Acc;//加速度unit/s2
						pgLoad.d	= commBuf.Dec;//减速度unit/s2
						halt_destination_caclu();//生成停止目标位置
						AL.loadCtrl.setPos = pgLoad.sg;	
					break;
					case EXTEN_MODE:					
						pgExt.a	= commBuf.Acc;//加速度unit/s2
						pgExt.d	= commBuf.Dec;//减速度unit/s2
						halt_destination_caclu();//生成停止目标位置
						AL.ext1Ctrl.setPos = pgExt.sg;
					break;
				}
				break;
			/* Fun_1714 */
			case DOPESHALT:
				//DoPESHALT is always executed in POS_MODE 
					pg.a	= commBuf.Acc;//加速度unit/s2
					pg.d	= commBuf.Dec;//减速度unit/s2
					halt_destination_caclu();//生成停止目标位置
					AL.posCtrl.setPos = pg.sg;	

				break;
			/* Fun_1716 */
			case DOPEPOSEXT://调用DOPEMOVE 然后在目标阈值处进行判断 *** ①没有写完 ②pg 和 pgLoad也没有分清
					//应该是调用DOPEPOS到limit处，然后在目标阈值处进行判断，关键点在目标阈值前多少来切换成目标控制方式来保持
					//需要添加一个拉压同向、拉压反向？
					//先暂定位移和力始终同向！2024-12-8 21:57:23
					//2024-12-12 15:25:55 让横梁先向limit上移动，然后在完成度检测中不断提前判断posext.Destination是否到达
					if(stateFlag.posExtDestReach == reachNone){
						posext.moveCtrlOrig = commBuf.MoveCtrl;
						posext.limitmode = commBuf.LimitMode;
						posext.destinationctrl = commBuf.DestinationCtrl;
						posext.destinationmode = commBuf.DestinationMode;
						posext.limit = commBuf.Limit;
						posext.declimit = commBuf.DecLimit;
						posext.decdestination = commBuf.DecDestination;
						posext.Destination = commBuf.Destination;
						switch(AL.movectrl)
						{
							case POS_MODE:
								pg.vmax = commBuf.Speed;
								pg.a	= commBuf.Acc;//加速度unit/s2
								pg.d	= commBuf.Dec;//减速度 unit/s2
								switch(posext.destinationctrl)
								{
									case POS_MODE:				//运动方式POS 目标运动方式POS
										AL.posCtrl.setPos = posExtLimitCalculate(posext.limit,posext.limitmode,posAllCtrlPara.Sft.UpperSft,posAllCtrlPara.Sft.LowerSft,
															pose.orig,pose.orig,posext.Destination);
									break;
									case LOAD_MODE:				//运动方式POS 目标运动方式LOAD
										AL.posCtrl.setPos = posExtLimitCalculate(posext.limit,posext.limitmode,loadAllCtrlPara.Sft.UpperSft,loadAllCtrlPara.Sft.LowerSft,
															pose.orig,force.filter,posext.Destination);
										//AL.loadCtrl.setPos = posext.Destination;
									break;
									case EXTEN_MODE:				//运动方式POS 目标运动方式EXTEN	
										AL.posCtrl.setPos = posExtLimitCalculate(posext.limit,posext.limitmode,extAllCtrlPara.Sft.UpperSft,extAllCtrlPara.Sft.LowerSft,
															pose.orig,strain1.filter,posext.Destination);
										//AL.ext1Ctrl.setPos = posext.Destination;
									break;
								}
								pg.sg = AL.posCtrl.setPos;
							break;
							case LOAD_MODE:
								pgLoad.vmax = commBuf.Speed;
								pgLoad.a	= commBuf.Acc;//加速度unit/s2
								pgLoad.d	= commBuf.Dec;//减速度 unit/s2
								switch(posext.destinationctrl)
								{
									case POS_MODE:				//运动方式LOAD 目标运动方式POS
										AL.loadCtrl.setPos = posExtLimitCalculate(posext.limit,posext.limitmode,posAllCtrlPara.Sft.UpperSft,posAllCtrlPara.Sft.LowerSft,
															force.filter,pose.orig,posext.Destination);
										//AL.posCtrl.setPos = posext.Destination;
									break;
									case LOAD_MODE:				//运动方式LOAD 目标运动方式LOAD
										AL.loadCtrl.setPos = posExtLimitCalculate(posext.limit,posext.limitmode,loadAllCtrlPara.Sft.UpperSft,loadAllCtrlPara.Sft.LowerSft,
															force.filter,force.filter,posext.Destination);
									break;
									case EXTEN_MODE:			//运动方式LOAD 目标运动方式EXTEN	
										AL.loadCtrl.setPos = posExtLimitCalculate(posext.limit,posext.limitmode,extAllCtrlPara.Sft.UpperSft,extAllCtrlPara.Sft.LowerSft,
															pose.orig,strain1.filter,posext.Destination);
										//AL.ext1Ctrl.setPos = posext.Destination;
									break;
								}
								pgLoad.sg = AL.loadCtrl.setPos;	
							break;
							case EXTEN_MODE:
								pgExt.vmax = commBuf.Speed;
								pgExt.a	= commBuf.Acc;//加速度unit/s2
								pgExt.d	= commBuf.Dec;//减速度 unit/s2
								switch(posext.destinationctrl)
								{
									case POS_MODE:				//运动方式LOAD 目标运动方式POS
										AL.ext1Ctrl.setPos = posExtLimitCalculate(posext.limit,posext.limitmode,posAllCtrlPara.Sft.UpperSft,posAllCtrlPara.Sft.LowerSft,
															strain1.filter,pose.orig,posext.Destination);
										//AL.posCtrl.setPos = posext.Destination;
									break;
									case LOAD_MODE:				//运动方式LOAD 目标运动方式LOAD
										AL.ext1Ctrl.setPos = posExtLimitCalculate(posext.limit,posext.limitmode,loadAllCtrlPara.Sft.UpperSft,loadAllCtrlPara.Sft.LowerSft,
															strain1.filter,force.filter,posext.Destination);
										//AL.loadCtrl.setPos = posext.Destination;
									break;
									case EXTEN_MODE:			//运动方式LOAD 目标运动方式EXTEN	
										AL.ext1Ctrl.setPos = posExtLimitCalculate(posext.limit,posext.limitmode,extAllCtrlPara.Sft.UpperSft,extAllCtrlPara.Sft.LowerSft,
															strain1.filter,strain1.filter,posext.Destination);
									break;
								}
								pgExt.sg = AL.ext1Ctrl.setPos;	
							break;
						}
					}else if(stateFlag.posExtDestReach == reachPosition){
						switch(posext.destinationctrl){
							case POS_MODE:
								log_i("POS reachPosition");
								pg.vmax = speedPose.filter;
								pg.a	= -posext.decdestination;//加速度unit/s2
								pg.d	= posext.decdestination;//减速度 unit/s2
								pg.sg	= posext.Destination;
							break;
							case LOAD_MODE:
								log_i("LOAD reachPosition");
								pgLoad.vmax = speedForce.filter;
								pgLoad.a	= -posext.decdestination;//加速度unit/s2
								pgLoad.d	= posext.decdestination;//减速度 unit/s2
								pgLoad.sg	= posext.Destination;
							break;
							case EXTEN_MODE:
								log_i("EXTEN reachPosition");
								pgExt.vmax = speedStrain.filter;
								pgExt.a	= -posext.decdestination;//加速度unit/s2
								pgExt.d	= posext.decdestination;//减速度 unit/s2
								pgExt.sg	= posext.Destination;
							break;
						}
						//在这种模式下 完成度检测就不进行检测 永远不完成，直到有下一个指令重新进行规划轨迹
					}
				break;
			/* Fun_1715 */
			case DOPECYCLE:
				switch(AL.movectrl)
				{
					case POS_MODE:
						pgcycle_init(&pgcycle,&pg);//循环运动指令轨迹规划初始化 
						pg.a	= commBuf.Acc;//加速度unit/s2
						pg.d	= commBuf.Dec;//减速度unit/s2
						AL.posCtrl.setPos = pg.sg;		
					break;
					case LOAD_MODE:
						pgcycle_init(&pgcycle,&pgLoad);//循环运动指令轨迹规划初始化 
						pgLoad.a	= commBuf.Acc;//加速度unit/s2
						pgLoad.d	= commBuf.Dec;//减速度unit/s2
						AL.loadCtrl.setPos = pgLoad.sg;	
					break;
					case EXTEN_MODE:						
						pgcycle_init(&pgcycle,&pgExt);//循环运动指令轨迹规划初始化 
						pgExt.a	= commBuf.Acc;//加速度unit/s2
						pgExt.d	= commBuf.Dec;//减速度unit/s2
						AL.ext1Ctrl.setPos = pgExt.sg;	
					break;
				}
				break;
			/* Fun_1717 */
			case DOPETRIG:
			case DOPETRIG_A:
				trigPgParamInit();
				break;
			/* Fun_1721 */
			case DOPEDYNCYCLE://取循环运动指令  在进行此指令前应该将位置 或者 力 归为至0或offset处***
				DOPEDYNCYCLE_StartBefore_Process();//轨迹归offset处理 动态指令开始前的准备过程
				if(stateFlag.DynCycle_OffsetReach == 2)//已经到达offset位置 可以开始进行DynCycle
				{
					memcpy(&pgdyn,&commBuf.pgdyn,sizeof(commBuf.pgdyn));//拷贝pgdyn	
					switch(AL.movectrl)
					{
						case POS_MODE:AL.posCtrl.setPos = pgdyn.destination;	break;
						case LOAD_MODE:AL.loadCtrl.setPos = pgdyn.destination;	break;
						case EXTEN_MODE:AL.ext1Ctrl.setPos = pgdyn.destination;	break;
					}
				}
				break;
			default:
				break;
		}
		//将当前规划位置和速度 设置为 初始位置和初始速度
		if(stateFlag.posExtDestReach == reachNone){
			if(AL.command != DOPEDYNCYCLE)
			{
				normalPosGeneratorInit(AL.movectrl);
				mySemaphore.move.pgInit = 0;
			}
			else
			{
				DynCycle_planinit();//动态循环运动指令轨迹规划初始化
				mySemaphore.move.pgInit = 0;
			}
		}else if(stateFlag.posExtDestReach == reachPosition){
			switch(posext.destinationctrl){
				case POS_MODE:
				//好像 sg没有被赋值 找一下原因
					normalPosGeneratorInit(posext.destinationctrl);
				break;
				case EXTEN_MODE:
					normalPosGeneratorInit(posext.destinationctrl);
				break;
				case LOAD_MODE:
					//normalPosGeneratorInit(posext.destinationctrl);
				break;
				default:
				break;
			}
			mySemaphore.move.pgInit = 0;
		}
		//规划错误判断 恢复前一指令现场
		if(pg_init_err == 0)	
			commBufSave(&commBuf,&commBufLast);//保存commBuf现场
		else
		{		
			if((AL.command == DOPEHALT || AL.command == DOPEHALT_A || AL.command == DOPESHALT) 
				&& pg_init_err == 1){
				AL.complete_state.flag = 1;
				pid_pos.calcuRun = 0;
					switch(AL.movectrl)
					{
						case POS_MODE:
							memset(&pg,0,sizeof(pg));
							pg.sg = pose.orig;
							pg.s3 = pg.sg;
							pg.st = pg.sg;
						break;
						case LOAD_MODE:
							memset(&pgLoad,0,sizeof(pgLoad));
							pgLoad.sg = force.filter;
							pgLoad.s3 = pgLoad.sg;
							pgLoad.st = pgLoad.sg;
						break;
						case EXTEN_MODE:
							memset(&pgExt,0,sizeof(pgExt));
							pgExt.sg = strain1.filter;
							pgExt.s3 = pgExt.sg;
							pgExt.st = pgExt.sg;
						break;			
					}
				log_i("DOPEHALT:0x%2X COMPLETED!!\r\n",AL.command);
				commBufSave(&commBuf,&commBufLast);//保存commBuf现场
			}
			else{
				log_i("pg err resume keep last command!\r\n");
				pgResume(&pg,&pg_last);//恢复posGenerator现场
				pgResume(&pgLoad,&pgLoad_last);//恢复posGenerator现场
				pgResume(&pgExt,&pgExt_last);//恢复posGenerator现场
				commBufResume(&commBuf,&commBufLast);//恢复commBuf现场
			}
		}
	}	
}

/**
 * @brief normal pos generator init
 */
void normalPosGeneratorInit(uint8_t movectrl){
	switch(movectrl)
	{
		case POS_MODE:
			if(AL.movectrl_last != POS_MODE)
			{
				pg.st = pose.orig;
				pg.vt = speedPose.filter;
			}
			if(fabs(pg.st-pose.orig)>0.05f && pg.vt < 0.1f)
			{
				pg.st = pose.orig + speedPose.filter * 0.001f;
				pg.vt = speedPose.filter;	
			}
			if(AL.command_last == 0x00){
				pg.vt = 0;
			}
			pg.s0 = pg.st;
			pg.v0 = pg.vt;	
			pg_init_err = pgNormalInit(&pg,movectrl);
			pid_pos.endValue = AL.posCtrl.setPos;
		break;
		case LOAD_MODE:
			if(AL.movectrl_last != LOAD_MODE)
			{
				pgLoad.st = force.filter;
				pgLoad.vt = 0;
			}
			if(fabs(pgLoad.st-force.filter)>0.5f && pgLoad.vt < 5.0f)
			{
				pgLoad.st = force.filter;
				pgLoad.vt = speedForce.filter;	
			}
			pgLoad.s0 = pgLoad.st;
			pgLoad.v0 = pgLoad.vt;	
			pg_init_err = pgNormalInit(&pgLoad,movectrl);
			//log_i("pgLoad vamx=%f loadAllCtrlPara.Nominal.Speed= %f",pgLoad.vmax,loadAllCtrlPara.Nominal.Speed);
			pid_load.endValue = AL.loadCtrl.setPos;
		break;
		case EXTEN_MODE:
			if(AL.movectrl_last != EXTEN_MODE)
			{
				pgExt.st = strain1.filter;
				pgExt.vt = 0;
			}
			if(fabs(pgExt.st-force.filter)>0.005f && pgExt.vt < 0.01f)
			{
				pgExt.st = strain1.filter;
				pgExt.vt = speedStrain.filter;	
			}
			pgExt.s0 = pgExt.st;
			pgExt.v0 = pgExt.vt;	
			pg_init_err = pgNormalInit(&pgExt,movectrl);
			pid_ext.endValue = AL.ext1Ctrl.setPos;
		break;			
	}
}

/*
函数名：halt_destination_caclu
功能：是HALT运动切换中的子函数，用于计算停止的目标位置
*/
void halt_destination_caclu(){
	float t_halt=0;
	switch(AL.movectrl){
		case POS_MODE:
				if(AL.command == DOPESHALT){
					//according to real poseSpeed and real poseOrig,calculate pg.sg
					if(speedPose.filter >0 ){
						t_halt = speedPose.filter/-pg.d;
						pg.sg = pose.orig + 0.5f*(-pg.d)*t_halt*t_halt+0.0001f;//根据当前期望位置 + 减速段
					}
					else if(speedPose.filter <0){
						t_halt = speedPose.filter/pg.d;
						pg.sg = pose.orig + 0.5f*pg.d*t_halt*t_halt-0.0001f;
					}
					else{ //速度为0 不需要执行HALT指令 会返回pg err1错误
						pg.sg = pose.orig;
					}
					log_i("DoPESHALT POS_MODE pg.st=%f,pg.sg=%f,t_halt=%f\n",pg.st,pg.sg,t_halt);
				}else{
					//according to pgSpeed and pgSt,calculate pg.sg
					if(AL.posCtrl.pgSpeed >0 ){
						t_halt = AL.posCtrl.pgSpeed/-pg.d;
						pg.sg = pg.st + 0.5f*(-pg.d)*t_halt*t_halt+0.0001f;//根据当前期望位置 + 减速段
					}
					else if(AL.posCtrl.pgSpeed <0){
						t_halt = AL.posCtrl.pgSpeed/pg.d;
						pg.sg = pg.st + 0.5f*pg.d*t_halt*t_halt-0.0001f;
					}
					else{ //速度为0 不需要执行HALT指令 会返回pg err1错误
						pg.sg = pg.st;
					}
					log_i("POS_MODE pg.st=%f,pg.sg=%f,t_halt=%f\n",pg.st,pg.sg,t_halt);
				}
		break;
		case LOAD_MODE:
				if(AL.loadCtrl.pgSpeed >0 )
				{
						t_halt = AL.loadCtrl.pgSpeed/-pgLoad.d;
						pgLoad.sg = pgLoad.st + 0.5f*(-pgLoad.d)*t_halt*t_halt+0.0001f;//根据当前期望位置 + 减速段
				}
				else if(AL.loadCtrl.pgSpeed <0)
				{
						t_halt = AL.loadCtrl.pgSpeed/pgLoad.d;
						pgLoad.sg = pgLoad.st + 0.5f*pgLoad.d*t_halt*t_halt-0.0001f;
				}
				else //速度为0 不需要执行HALT指令 会返回pg err1错误
				{
						pgLoad.sg = pgLoad.st;
				}
				log_i("LOAD_MODE pgLoad.st=%f,pgLoad.sg=%f,t_halt=%f\n",pgLoad.st,pgLoad.sg,t_halt);
		break;
		case EXTEN_MODE:
				if(AL.ext1Ctrl.pgSpeed >0 )
				{
						t_halt = AL.ext1Ctrl.pgSpeed/-pgExt.d;
						pgExt.sg = pgExt.st + 0.5f*(-pgExt.d)*t_halt*t_halt+0.0001f;//根据当前期望位置 + 减速段
				}
				else if(AL.ext1Ctrl.pgSpeed <0)
				{
						t_halt = AL.ext1Ctrl.pgSpeed/pgExt.d;
						pgExt.sg = pgExt.st + 0.5f*pgExt.d*t_halt*t_halt-0.0001f;
				}
				else //速度为0 不需要执行HALT指令 会返回pg err1错误
				{
						pgExt.sg = pgExt.st;
				}
				log_i("EXTEN_MODE pgExt.st=%f,pgExt.sg=%f,t_halt=%f\n",pgExt.st,pgExt.sg,t_halt);
		break;
		default:break;
	}
}
/*
函数名：DOPEDYNCYCLE_StartBefore_Process
功能：轨迹归offser处理 动态指令开始前的准备过程
	  1.检测当前位置或者力是否在offset位置
	  2.如果不在offset位置 先调用DOPEPOS功能，使当前位置或力移动至offset处 再进行DOPEDYNCYCLE循环
*/
void DOPEDYNCYCLE_StartBefore_Process(void)
{	
	if( stateFlag.DynCycle_OffsetReach == 0)
	{
		log_i("stateFlag.DynCycle_OffsetReach = 0\r\n");
		switch(AL.movectrl)
		{
			case POS_MODE:
					if(fabs(commBuf.pgdyn.offset - pg.st) < 0.001f && fabs(pg.st - pose.orig) < 0.01f)//当前位置在offset处
					{
						stateFlag.DynCycle_OffsetReach = 2;
						AL.posCtrl.setPos = commBuf.pgdyn.destination;
						pg.sg = commBuf.pgdyn.destination;
						log_i("stateFlag.DynCycle_OffsetReach = 2\r\n");
					}
					else//不在offset位置
					{
						stateFlag.DynCycle_OffsetReach = 1;
						log_i("stateFlag.DynCycle_OffsetReach = 1\r\n");
						//设定DoPEPOS参数
						AL.command = DOPEPOS;	//进行位置控制指令至offset处
						AL.posCtrl.setPos = commBuf.pgdyn.offset;
						pg.sg = commBuf.pgdyn.offset;
						pg.vmax = AL.Pos.DefaultSpeed;//默认速度
						pg.a	= AL.Pos.DefaultAcc;//默认加速度
						pg.d	= AL.Pos.DefaultDec;//默认减速度
					}
			break;
			case LOAD_MODE:
					if(fabs(commBuf.pgdyn.offset - pgLoad.st) < 0.001f && fabs(pgLoad.st - force.filter) < 0.5f)//当前位置在offset处
					{
						stateFlag.DynCycle_OffsetReach = 2;
						AL.loadCtrl.setPos = commBuf.pgdyn.destination;
						pgLoad.sg = commBuf.pgdyn.destination;
						log_i("stateFlag.DynCycle_OffsetReach = 2\r\n");
					}
					else//不在offset位置
					{
						stateFlag.DynCycle_OffsetReach = 1;
						log_i("stateFlag.DynCycle_OffsetReach = 1\r\n");
						//设定DoPEPOS参数
						AL.command = DOPEPOS;	//进行位置控制指令至offset处
						AL.loadCtrl.setPos = commBuf.pgdyn.offset;
						pgLoad.sg = commBuf.pgdyn.offset;
						pgLoad.vmax = AL.Load.DefaultSpeed;//默认速度
						pgLoad.a	= AL.Load.DefaultAcc;//默认加速度
						pgLoad.d	= AL.Load.DefaultDec;//默认减速度
					}
			break;
			case EXTEN_MODE:
					if(fabs(commBuf.pgdyn.offset - pgExt.st) < 0.001f && fabs(pgExt.st - strain1.filter) < 0.5f)//当前位置在offset处
					{
						stateFlag.DynCycle_OffsetReach = 2;
						AL.ext1Ctrl.setPos = commBuf.pgdyn.destination;
						pgExt.sg = commBuf.pgdyn.destination;
						log_i("stateFlag.DynCycle_OffsetReach = 2\r\n");
					}
					else//不在offset位置
					{
						stateFlag.DynCycle_OffsetReach = 1;
						log_i("stateFlag.DynCycle_OffsetReach = 1\r\n");
						//设定DoPEPOS参数
						AL.command = DOPEPOS;	//进行位置控制指令至offset处
						AL.ext1Ctrl.setPos = commBuf.pgdyn.offset;
						pgExt.sg = commBuf.pgdyn.offset;
						pgExt.vmax = AL.Ext.DefaultSpeed;//默认速度
						pgExt.a	= AL.Ext.DefaultAcc;//默认加速度
						pgExt.d	= AL.Ext.DefaultDec;//默认减速度
					}
			break;
			default:break;
		}
	}
}

float DOPEPOSEXT_MAINTANCE(float expect,float real,uint8_t movectrl)
{
	float err=0,outP=0,outI=0;
	static float errLast = 0;
	err = (expect - real);
	if(movectrl == POS_MODE){
	outP = posAllCtrlPara.PosExtPid.P * (err - errLast);
	outI = posAllCtrlPara.PosExtPid.I * err;
	}
	else if(movectrl == LOAD_MODE){
	outP = loadAllCtrlPara.PosExtPid.P * (err - errLast);
	outI = loadAllCtrlPara.PosExtPid.I * err;
	}
	else if(movectrl == EXTEN_MODE){
	outP = extAllCtrlPara.PosExtPid.P * (err - errLast);
	outI = extAllCtrlPara.PosExtPid.I * err;
	}
	errLast = err;
	MAINTANCE_out += (outP+outI);
	MAINTANCE_out = constraint(MAINTANCE_out,20,-20);
	return MAINTANCE_out;
}

//#define ADRC_CTRL

/**
 * @brief		POS_MODE闭环控制处理函数
*/
float PosPos_LoopCtrl(float setpos,float nowpos);
float PosSpeed_LoopCtrl(float setspeed,float nowspeed);
float PosFFD(float setspeed);
void AOPosCalcu(void);
uint16_t loopTimeGet(TIM_HandleTypeDef *_htim,uint16_t *_time);
void pwmPosLsmCalcu(void);
float PosPos_LoopCtrl_I(float setpos,float nowpos);
float posTargetCalcu(void);
float posPID(const float _target,const float _real,float *_out,pid_t *_pidPos,ctrlPeriod_t *_ctrlPeriod);
float posAdrc(PosGenerator *_pg,const float _target,const float _real,const float _speed,float *_out,ctrlPeriod_t *_ctrlPeriod,pid_t *_pidPos);
float posPwmCalcu(const float _input,float *_out,const uint16_t _time);

float PosSpeedOut=0;
float MAINTANCE_test=0;

#define ADRC_CTRL_POS
void POS_Process(void)
{
	// float vt=0;
	portTASK_USES_FLOATING_POINT();
	//calculate the target pg pos
	posTargetCalcu();
	//calculate the speed of pg
	AL.posCtrl.pgSpeed = (pg.st - pg.st_last)/0.001f;
	//calculate the period of loopctrol, accoding to the posgeneration speed
	//periodCalcu(pg.vm,AL.posCtrl.NominalSensitive*servoParam.ResolutionGearRate,&posCtrlPeriod);
	posCtrlPeriod.real = 1;
	//保存posGenerator现场
	pg_save(&pg,&pg_last);
	//loop ctrl 
//pid loop ctrl
#ifndef ADRC_CTRL_POS
	posPID(pg.st,pose.orig,&AL.posCtrl.posLoopCtrlOut,&pid_pos,&posCtrlPeriod);
#endif
//adrc loop ctrl
#ifdef ADRC_CTRL_POS
	posAdrc(&pg,pg.st,pose.orig,speedPose.filter,&AL.posCtrl.posLoopCtrlOut,&posCtrlPeriod,&pid_pos);
#endif
	//time compensation
	loopTimeGet(&htim6,&tim6us);
	//pwm Calcu
	posPwmCalcu(AL.posCtrl.posLoopCtrlOut,&svPWM.pos,tim6us);
	//AOPosCalcu();		
}

float posPID(const float _target,const float _real,float *_out,pid_t *_pidPos,ctrlPeriod_t *_ctrlPeriod){
	if(++_ctrlPeriod->count >= _ctrlPeriod->real){
		_ctrlPeriod->count = 0;
		*_out = pidCalPos(_target,_real,_pidPos);
	} 
	return *_out;
}

float posAdrc(PosGenerator *_pg,const float _target,const float _real,const float _speed,float *_out,ctrlPeriod_t *_ctrlPeriod,pid_t *_pidPos){
	//eso calcu
	//adrcLESO(AL.lesoU/(_ctrlPeriod->real * 1.0),_real,1,0.001,&leso);
	//out calcu
	if(++_ctrlPeriod->count >= _ctrlPeriod->real){
		_ctrlPeriod->count = 0;
		adrcControlLawPoswithCreep(_pg->sg,_target,_real,_pg->vm,_pg->vt,_speed);
		// if(AL.movectrl == LOAD_MODE)
			//*_out = -adrcFhan(&fhan);
		// else if(AL.movectrl == POS_MODE)
			*_out = _pg->vt;
		// if(AL.creepFlag == 1)
		// 	*_out += pidCalPos(_target,_real,_pidPos)*0.1f;
		// else
			*_out += pidCalPos(_target,_real,_pidPos);
		// if(_pidPos->calcuRun == 0)
		// 	*_out = 0;
	}
	return *_out;
}

float posTargetCalcu(void){
	pg.st_last = pg.st;
	//no need to maintain
	if(stateFlag.posExtDestReach == reachNone){
		if(AL.command != DOPEDYNCYCLE)
			pg.st = pos_generator(++pg.t,&pg);//生成位置
		else
			pg.st = DynCycle_posgenerator(&pgdyn,++pg.t);//生成位置
	}
	//need to maintain in posExt-position mode
	if(stateFlag.posExtDestReach == reachPosition){
		pg.st = pg.sg;
	}
	//need to maintain in loadExt-position mode 待测试
	//DOPEPOSEXT MAINTIANCE模式对pg.st的补偿
	if(stateFlag.posExtDestReach == reachMaintain)
	{
		MAINTANCE_test = DOPEPOSEXT_MAINTANCE(posext.Destination,force.filter,LOAD_MODE);
		pg.st += MAINTANCE_test;
	}
	return pg.st;
}

uint16_t loopTimeGet(TIM_HandleTypeDef *_htim,uint16_t *_time){
	*_time = __HAL_TIM_GET_COUNTER(_htim);
	*_time = constraintINT(*_time,2000,850);
	__HAL_TIM_SET_COUNTER(_htim,0);
	return *_time;
}

float PosPos_LoopCtrl(float setpos,float nowpos)
{
	float outP=0,outD=0,out,err;
	static float outI=0;
	static float lasterr=0;
	err = setpos - nowpos;
	outP  = posAllCtrlPara.PosPid.P * err;
	// if(fabs(err) <= 0.1)
	// 	outI += AL.Pos.Pid.I * err;
	// else
	// 	outI *= 0.2f;
	// outI = constraint(outI,0.05,-0.05);
	outI += posAllCtrlPara.PosPid.I * err;
	outI = constraint(outI,100,-100);
	outD = posAllCtrlPara.PosPid.D * (err-lasterr);
	lasterr = err;
	out = (outP + outI + outD);
	out = constraint(out,50,-50);
	return out;
}

float PosPos_LoopCtrl_I(float setpos,float nowpos)
{
	float outP=0,outD=0,out,err;
	static float outI=0;
	static float lasterr=0;
	uint8_t key=1;
	err = setpos - nowpos;
	outP  = posAllCtrlPara.PosPid.P * err;
	// if(fabs(err) <= 0.1)
	// 	outI += AL.Pos.Pid.I * err;
	// else
	// 	outI *= 0.2f;
	// outI = constraint(outI,0.05,-0.05);
	// if(fabs(err) <= 1) key=1;
	// else	key=0;
	outI += key*posAllCtrlPara.PosPid.I * err;
	// outI = constraint(outI,0.01,-0.01);
	outD = posAllCtrlPara.PosPid.D * (err-lasterr);
	lasterr = err;
	out = (outP + outI + outD);
	//out = constraint(out,0.02,-0.02);
	return out;
}

float PosSpeed_LoopCtrl(float setspeed,float nowspeed)
{
	float outP=0,outD=0,err,out;
	static float outI=0;
	static float lasterr=0;
	err = setspeed - nowspeed;
	outP = posAllCtrlPara.SpeedPid.P *err;
	outI += posAllCtrlPara.SpeedPid.I *err;
	outD = posAllCtrlPara.SpeedPid.D *(err-lasterr);
	out = outP + outI + outD;

	lasterr = err;

	return out;
}

// float PosSpeed_LoopCtrl(float setspeed,float nowspeed)
// {
// 	float outP=0,outI=0,err,out;
// 	static float lastspeed=0,lasterr=0;
// 	err = setspeed - nowspeed;
// 	outP  = AL.Pos.PidSpeed.P *(err - lasterr);
// 	outI = AL.Pos.PidSpeed.I * err;
// 	out = outP + outI;

// 	lastspeed = speedPose.filter;
// 	lasterr = err;

// 	return out;
// }

float PosFFD(float setspeed)
{
	static float setspeed_last=0;
	static uint8_t flag=0;
	float out,T=5;
	
	if(flag == 0){
	setspeed_last = setspeed;
	flag = 1;
	}
	out = posAllCtrlPara.Ffd.SpeedFFP /100.0f*(setspeed + T*(setspeed - setspeed_last));
	setspeed_last = setspeed;
	
	return out;
}

float posPwmCalcu(const float _input,float *_out,const uint16_t _time){
	float _compensateTime = 1000.0f / _time;
	int32_t servoCode;
	int64_t z2;
	if(SenData[ch0Pose].LinPoint == 0)
		//*_out = (int64_t)(_input * 100000000.0 / (AL.posCtrl.NominalSensitive * AL.posCtrl.sign * servoParam.ResolutionGearRate) * _compensateTime);
		*_out = (int64_t)(_input * 100000000.0f / (AL.posCtrl.NominalSensitive * servoParam.ResolutionGearRate) * _compensateTime);
	else{
		sensorCalibrate.antiMultipointCalibrate(_input,ch0Pose,&SenData[ch0Pose],&servoCode);
		// *_out = (int64_t)(servoCode*100000000.0 / (AL.posCtrl.sign * servoParam.ResolutionGearRate) * _compensateTime);
		*_out = (int64_t)(servoCode * 100000000.0f / (servoParam.ResolutionGearRate) * _compensateTime);
	}
#ifdef ADRC_CTRL_POS
	//  if(fabs(pg.vm) >= 0.005 && AL.creepFlag == 0)
	// 	z2 = (int64_t)(((leso.z2 * 100000000.0)/(sensitive * servoParam.ResolutionGearRate)) * _compensateTime);
	// else
		z2 = 0;
#endif
	*_out = (*_out - z2) / 100000000.0f;
	//AL.lesoU = *_out * (sensitive * servoParam.ResolutionGearRate);
	test.fs = *_out;
	//禁止输出
	if(AL.complete_state.flag == 1|| stateFlag.DoPESetCtrl == CtrlFALSE){
		*_out=0;
		//AL.lesoU = 0;
	}//代表完成此次运动或闭环禁止,禁止输出

	return *_out;
}

void pwmPosLsmCalcu(void)
{
	int64_t servoOutput,z2;
	int32_t servoCode;

	//AL.lesoU = ((double)AL.posCtrl.posLoopCtrlOut - leso.z2) * 1000.0 / (tim6us * 1.0);
	if(SenData[ch0Pose].LinPoint == 0)
		servoOutput = (int64_t)(AL.posCtrl.posLoopCtrlOut * 100000000.0f / (AL.posCtrl.NominalSensitive * AL.posCtrl.sign * servoParam.ResolutionGearRate) * 1000.0f / tim6us);
	else{
		sensorCalibrate.antiMultipointCalibrate(AL.posCtrl.posLoopCtrlOut,ch0Pose,&SenData[ch0Pose],&servoCode);
		servoOutput = (int64_t)(servoCode * 100000000.0f / (AL.posCtrl.sign * servoParam.ResolutionGearRate) * 1000.0f / tim6us);
	}
#ifdef ADRC_CTRL_POS
	//  if(fabs(pg.vm) >= 0.005 && AL.creepFlag == 0)
	// 	z2 = (int64_t)(((leso.z2 * 100000000.0)/(AL.posCtrl.NominalSensitive * AL.posCtrl.sign * 0.4)) * 1000.0 / tim6us);
	// else
		z2 = 0;
#endif
	svPWM.pos = (servoOutput - z2)/100000000.0;
	//AL.lesoU = svPWM.pos * (AL.posCtrl.NominalSensitive * AL.posCtrl.sign * 0.4);
	test.fs = svPWM.pos;
	//禁止输出
	if(AL.complete_state.flag == 1|| stateFlag.DoPESetCtrl == CtrlFALSE){
		svPWM.pos=0;
		//AL.lesoU = 0;
	}//代表完成此次运动或闭环禁止,禁止输出
}

void AOPosCalcu(void)
{
	if(AL.posCtrl.NominalSensitive * AL.posCtrl.sign < 0)
		AL.servoOutput *=-1;
	svAO.pos = AL.servoOutput *2000.0f;
	svAO.pos = constraint(svAO.pos,100000,-100000);//输出限幅
	//禁止输出
	if(AL.complete_state.flag == 1|| stateFlag.DoPESetCtrl == CtrlFALSE) svAO.pos=0;//代表完成此次运动或闭环禁止,禁止输出
}

/**
 * @brief		LOAD_MODE闭环控制处理函数
*/
float Load_LoopCtrl(void);
float LoadPos_LoopCtrl(float setpos,float nowpos);
void AOLoadCalcu(void);
float LoadFFD(float setspeed);
float LoadSpeed_LoopCtrl(float setspeed,float nowspeed);
void pwmLoadLsmCalcu(void);
float loadPID(const float _target,const float _real,const float _maxRange,float *_vm,
	float *_out,pid_t *_pidLoad,uint8_t *_count);
float loadTargetCalcu(void);
float loadAdrc(const float _target,const float _real,const float _speed,float *_vm,float *_out,ctrlPeriod_t *_ctrlPeriod,pid_t *_pidPos);
float loadPwmCalcu(const float _input,float *_out,const uint16_t _time);

#define ADRC_CTRL_LOAD

void LOAD_Process(void)
{
	static float vm;
	//calculate the target pg pos
	loadTargetCalcu();
	//calculate the speed of pgLoad
	AL.loadCtrl.pgSpeed = (pgLoad.st - pgLoad.st_last)/0.001f;

#ifdef ADRC_CTRL_LOAD
	//loop ctrl 
	//Load pid calcu
	loadPID(pgLoad.st,force.filter,AL.loadCtrl.NominalValue*1000.0f,&vm,&AL.loadCtrl.posLoopCtrlOut,&pid_load,&AL.cycleT);
	//calculate the period of loopctrol, accoding to the posgeneration speed
	//periodCalcu(vm,AL.posCtrl.NominalSensitive*servoParam.ResolutionGearRate,&posCtrlPeriod);
	posCtrlPeriod.real = 1;
	loadAdrc(AL.loadCtrl.posLoopCtrlOut,pose.orig,speedPose.filter,&vm,&AL.loadCtrl.speedLoopCtrlOut,&posCtrlPeriod,&pid_pos);
#endif
	pg_save(&pgLoad,&pgLoad_last);//保存posGenerator现现场
	//time compensation
	loopTimeGet(&htim6,&tim6us);
	//pwm Calcu
	posPwmCalcu(AL.loadCtrl.speedLoopCtrlOut,&svPWM.load,tim6us);
	//AOLoadCalcu();
}

float loadAdrc(const float _target,const float _real,const float _speed,float *_vm,float *_out,ctrlPeriod_t *_ctrlPeriod,pid_t *_pidPos){
	//leso.uPos
	//adrcLESO(AL.lesoU/(posCtrlPeriod.real * 1.0),_real,1,0.001,&leso);
	if(++_ctrlPeriod->count >= _ctrlPeriod->real){
		_ctrlPeriod->count = 0;
		//adrcControlLawLoadInterlPos(_target,_real,*_vm,_speed);
		//*_out = -adrcFhan(&fhan);
		*_out = *_vm;
		//AL.loadCtrl.speedLoopCtrlOut = vm;
		*_out += pidCalPos(_target,_real,_pidPos);
	}
	return *_out;
}

float loadPID(const float _target,const float _real,const float _maxRange,float *_vm,
	float *_out,pid_t *_pidLoad,uint8_t *_count){
	static float _loadRecord = 0;
	if((*_count)++ >= 20){
	*_count = 0;
	//PI Load loop control
	AL.loadCtrl.detaLoopCtrlOut = pidCalLoad(_target,_real,_pidLoad);
	AL.loadCtrl.detaLoopCtrlOut /= (_maxRange*1000);//以20ms为单位，刚度参考值为1000N/mm,并且参数放大1000倍
	if(fabsf(_target - pgLoad.s0) > fabsf(AL.loadCtrl.setPos - pgLoad.s0) * 0.2f && fabsf(_real - AL.loadCtrl.setPos) > fabsf(AL.loadCtrl.setPos - pgLoad.s0) * 0.2f
		&& fabsf(AL.loadCtrl.setPos - pgLoad.s0) > AL.loadCtrl.NominalValue * 1000.0f * 0.1f)
		AL.loadCtrl.detaLoopCtrlOut += pgLoad.vt * 0.02f / lsRigidity.kFilter;//lsRigidity.kFilter
	//limit
	AL.loadCtrl.detaLoopCtrlOut = constraint(AL.loadCtrl.detaLoopCtrlOut,10,-10);
	//_loadRecord = pose.orig;
	_loadRecord = *_out;
	*_vm = AL.loadCtrl.detaLoopCtrlOut / 0.02f;
	}
	*_out = _loadRecord + AL.loadCtrl.detaLoopCtrlOut * ((1 + *_count) / 20.0);
	return *_out;
}

float loadTargetCalcu(void){
	pgLoad.st_last = pgLoad.st;
	//no need to maintain
	if(stateFlag.posExtDestReach == reachNone){
		if(AL.command != DOPEDYNCYCLE)
			pgLoad.st = pos_generator(++pgLoad.t,&pgLoad);//生成位置
		else
			pgLoad.st = DynCycle_posgenerator(&pgdyn,++pgLoad.t);//生成位置
	}
	//need to maintain in posExt-position mode
	if(stateFlag.posExtDestReach == reachPosition){
		pgLoad.st = pgLoad.sg;
	}
	//need to maintain in loadExt-position mode 待测试
	//DOPEPOSEXT MAINTIANCE模式对pg.st的补偿
	if(stateFlag.posExtDestReach == reachMaintain)
		pgLoad.st += DOPEPOSEXT_MAINTANCE(AL.posCtrl.setPos,pose.orig,LOAD_MODE);	
	return pgLoad.st;
}



float lsm_predicte_process(LSM_PREDICTOR_STRUCT* pre,LS_SLIDE_STRUCT* ls,float x,float y_expect)
{
	double den;
	den = pose.orig - pose.orig_last;
	// if(den < 1e-5 && den>0) den = 1e-5;
	// if(den > -1e-5 && den<0) den = -1e-5;

	if(fabs(den) < 1e-5) 
	{
		if( AL.loadCtrl.setPos > pre->y_pre) pre->delta_x = 1e-5;
		if( AL.loadCtrl.setPos < pre->y_pre) pre->delta_x = -1e-5;
	}
	else
	{
	pre->k = (pre->y_pre - pre->y_pre_last)/(pose.orig - pose.orig_last);
	pre->y_expect = y_expect;
	pre->delta_x = (pre->y_expect - pre->y_pre)/pre->k;
	}

	// if( pre->delta_x < 0 && AL.loadCtrl.setPos > pre->y_pre) pre->delta_x = 0;
	// else if( pre->delta_x > 0 && AL.loadCtrl.setPos < pre->y_pre ) pre->delta_x = 0;
	return pre->delta_x;
}

float LoadPos_LoopCtrl(float setpos,float nowpos)
{
	float outP=0,outI=0,outD=0,out,err;
	err = setpos - nowpos;
	outP  = loadAllCtrlPara.PosPid.P * err;
	if(fabs(err) <= 0.1)
		outI += loadAllCtrlPara.PosPid.I * err;
	else
		outI *= 0.2f;
	outI = constraint(outI,0.05,-0.05);
	//outD = AL.Pos.Pid.D * (err-lasterr);
	out = (outP + outI + outD);
	out = constraint(out,5,-5);
	return out;
}

float LoadFFD(float setspeed)
{
	static float setspeed_last=0;
	static uint8_t flag=0;
	float out,T=5;
	
	if(flag == 0){
	setspeed_last = setspeed;
	flag = 1;
	}
	out =  loadAllCtrlPara.Ffd.SpeedFFP/100.0f*(setspeed + T*(setspeed - setspeed_last));
	setspeed_last = setspeed;
	
	return out;
}

float LoadSpeed_LoopCtrl(float setspeed,float nowspeed)
{
	float outP=0,outD=0,err,out;
	static float outI=0;
	static float lasterr=0;
	err = setspeed - nowspeed;
	outP = loadAllCtrlPara.SpeedPid.P *err;
	outI += loadAllCtrlPara.SpeedPid.I *err;
	outD = loadAllCtrlPara.SpeedPid.D *(err-lasterr);
	out = outP + outI + outD;

	lasterr = err;

	return out;
}

void AOLoadCalcu(void)
{
	if(AL.loadCtrl.NominalSensitive * AL.loadCtrl.sign < 0)
		AL.servoOutput *=-1;
	svAO.load = AL.servoOutput *2000.0f;
	svAO.load = constraint(svAO.load,10000,-10000);//输出限幅
	//禁止输出
	if(AL.complete_state.flag == 1|| stateFlag.DoPESetCtrl == CtrlFALSE) svAO.load=0;//代表完成此次运动或闭环禁止,禁止输出
}

//float LoadSpeed_LoopCtrl(float setspeed,float nowspeed)
//{
//	float outP=0,outI=0,err,out;
//	static float lastspeed=0,lasterr=0;
//	err = setspeed - nowspeed;
//	outP  = AL.Load.PidSpeed.P *(err - lasterr); 
//	outI = AL.Load.PidSpeed.I * err;
//	out = outP + outI;

//	lastspeed = speedPose.filter;
//	lasterr = err;

//	return out;
//}

void pwmLoadLsmCalcu(void)
{
	int64_t servoOutput,z2;
	int32_t servoCode;

	if(SenData[ch0Pose].LinPoint == 0)
		servoOutput = (int64_t)(AL.loadCtrl.speedLoopCtrlOut * 100000000.0f / (AL.posCtrl.NominalSensitive * AL.posCtrl.sign * servoParam.ResolutionGearRate) * 1000.0f / tim6us);
	else{
		sensorCalibrate.antiMultipointCalibrate(AL.loadCtrl.speedLoopCtrlOut,ch4Load,&SenData[ch4Load],&servoCode);
		servoOutput = (int64_t)(servoCode * 100000000.0f / (AL.posCtrl.sign * servoParam.ResolutionGearRate) * 1000.0f / tim6us);
	}

#ifdef ADRC_CTRL_LOAD
	// if(fabs(pg.vm) >= 0.005 && fabs(pgLoad.sg - pgLoad.st) > 10 )
	// 	z2 = (int64_t)(((leso.z2 * 100000000.0)/(AL.posCtrl.NominalSensitive * AL.posCtrl.sign * 0.4)) * 1000.0 / tim6us);
	// else
		z2 = 0;
#endif
	svPWM.load = (servoOutput - z2)/100000000.0;
	AL.lesoU = svPWM.load * (AL.posCtrl.NominalSensitive * AL.posCtrl.sign * servoParam.ResolutionGearRate);

	//禁止输出
	if(AL.complete_state.flag == 1|| stateFlag.DoPESetCtrl == CtrlFALSE){
		svPWM.load=0;
		//AL.lesoU = 0;
	}//代表完成此次运动或闭环禁止,禁止输出
}

void pwmExtenCalcu(void);

/*
void LOAD_Process(void)
{
	static float loadRecord = 0,vm;
	//calculate the target pg pos
	loadTargetCalcu();
	//calculate the speed of pgLoad
	AL.loadCtrl.pgSpeed = (pgLoad.st - pgLoad.st_last)/0.001f;

void LOAD_Process(void)
{
	static float loadRecord = 0,vm;
	//calculate the target pg pos
	loadTargetCalcu();
	//calculate the speed of pgLoad
	AL.loadCtrl.pgSpeed = (pgLoad.st - pgLoad.st_last)/0.001f;

#ifdef ADRC_CTRL_LOAD
	//loop ctrl 
	//Load pid calcu
	loadPID(pgLoad.st,force.filter,&vm,&AL.loadCtrl.posLoopCtrlOut,&pid_load,&AL.cycleT);
	//calculate the period of loopctrol, accoding to the posgeneration speed
	periodCalcu(vm,AL.posCtrl.NominalSensitive*servoParam.ResolutionGearRate,&posCtrlPeriod);
	loadAdrc(AL.loadCtrl.posLoopCtrlOut,pose.orig,speedPose.filter,&vm,&AL.loadCtrl.speedLoopCtrlOut,&posCtrlPeriod,&pid_pos);
#endif
	pg_save(&pgLoad,&pgLoad_last);//保存posGenerator现现场
	//time compensation
	loopTimeGet(&htim6,&tim6us);
	//pwm Calcu
	posPwmCalcu(AL.loadCtrl.speedLoopCtrlOut,&svPWM.load,tim6us);
	//AOLoadCalcu();
}
	pg_save(&pgLoad,&pgLoad_last);//保存posGenerator现现场
	//time compensation
	loopTimeGet(&htim6,&tim6us);
	//pwm Calcu
	posPwmCalcu(AL.loadCtrl.speedLoopCtrlOut,&svPWM.load,tim6us);
	//AOLoadCalcu();
}
*/
float extenTargetCalcu(void);
float extenPID(const float _target,const float _real,float *_vm,
	float *_out,pid_t *_pidExten,uint8_t *_count);
	float extenAdrc(const float _target,const float _real,const float _speed,float *_vm,float *_out,ctrlPeriod_t *_ctrlPeriod,pid_t *_pidPos);

#define ADRC_CTRL_EXT
void EXTEN_Process(void)
{
	static float vm;
	//calculate the target pg pos
	extenTargetCalcu();
	//calculate the speed of pgLoad
	AL.ext1Ctrl.pgSpeed = (pgExt.st - pgExt.st_last)/0.001f;
#ifdef ADRC_CTRL_EXT
	//loop ctrl 
	//exten pid calcu
	extenPID(pgExt.st,strain1.filter,&vm,&AL.ext1Ctrl.posLoopCtrlOut,&pid_ext,&AL.cycleT);
	//calculate the period of loopctrol, accoding to the posgeneration speed
	//periodCalcu(vm,AL.posCtrl.NominalSensitive*servoParam.ResolutionGearRate,&posCtrlPeriod);
	posCtrlPeriod.real = 1;
	extenAdrc(AL.ext1Ctrl.posLoopCtrlOut,pose.orig,speedPose.filter,&vm,&AL.ext1Ctrl.speedLoopCtrlOut,&posCtrlPeriod,&pid_pos);
#endif
	pg_save(&pgExt,&pgExt_last);//保存posGenerator现现场
	//time compensation
	loopTimeGet(&htim6,&tim6us);
	//pwm Calcu
	posPwmCalcu(AL.ext1Ctrl.speedLoopCtrlOut,&svPWM.ext,tim6us);
	//AOExtenCalcu();
}

float extenAdrc(const float _target,const float _real,const float _speed,float *_vm,float *_out,ctrlPeriod_t *_ctrlPeriod,pid_t *_pidPos){
	//leso.uPos //time record：2025-3-12 11:58:16
	adrcLESO(AL.lesoU/(posCtrlPeriod.real * 1.0),_real,1,0.001,&leso);
	if(++_ctrlPeriod->count >= _ctrlPeriod->real){
		_ctrlPeriod->count = 0;
		//adrcControlLawLoadInterlPos(_target,_real,*_vm,_speed);
		//*_out = -adrcFhan(&fhan);
		*_out = *_vm;
		//AL.loadCtrl.speedLoopCtrlOut = vm;
		*_out += pidCalPos(_target,_real,_pidPos);
	}
	return *_out;
}

void pwmExtenCalcu(void)
{
	int64_t servoOutput,z2;
	int32_t servoCode;

	if(SenData[ch0Pose].LinPoint == 0)
		servoOutput = (int64_t)(AL.ext1Ctrl.speedLoopCtrlOut * 100000000.0f / (AL.posCtrl.NominalSensitive * AL.posCtrl.sign * 0.4f) * 1000.0f / tim6us);
	else{
		sensorCalibrate.antiMultipointCalibrate(AL.ext1Ctrl.speedLoopCtrlOut,ch2Ext1,&SenData[ch2Ext1],&servoCode);
	}
#ifdef ADRC_CTRL_EXT
	// if(fabs(pg.vm) >= 0.005 && fabs(pgLoad.sg - pgLoad.st) > 10 )
	// 	z2 = (int64_t)(((leso.z2 * 100000000.0)/(AL.posCtrl.NominalSensitive * AL.posCtrl.sign * 0.4)) * 1000.0 / tim6us);
	// else
		z2 = 0;
#endif
	svPWM.ext = (servoOutput - z2) / 100000000.0f;
	AL.lesoU = svPWM.ext * (AL.posCtrl.NominalSensitive * AL.posCtrl.sign * 0.4f);

	//禁止输出
	if(AL.complete_state.flag == 1|| stateFlag.DoPESetCtrl == CtrlFALSE){
		svPWM.ext=0;
		//AL.lesoU = 0;
	}//代表完成此次运动或闭环禁止,禁止输出
}

float extenTargetCalcu(void){
	pgExt.st_last = pgExt.st;
	//no need to maintain
	if(stateFlag.posExtDestReach == reachNone){
		if(AL.command != DOPEDYNCYCLE)
			pgExt.st = pos_generator(++pgExt.t,&pgExt);//生成位置
		else
			pgExt.st = DynCycle_posgenerator(&pgdyn,++pgExt.t);//生成位置
	}
	//need to maintain in posExt-position mode
	if(stateFlag.posExtDestReach == reachPosition){
		pgExt.st = pgExt.sg;
	}
	//need to maintain in loadExt-position mode 待测试
	//DOPEPOSEXT MAINTIANCE模式对pg.st的补偿
	if(stateFlag.posExtDestReach == reachMaintain){
		//waiting for design
		//pgLoad.st += DOPEPOSEXT_MAINTANCE(AL.posCtrl.setPos,pose.orig,LOAD_MODE);	
	}
	return pgExt.st;
}

float extenPID(const float _target,const float _real,float *_vm,
	float *_out,pid_t *_pidExten,uint8_t *_count){
	static float _extenRecord = 0;
	if((*_count)++ >= 20){
	*_count = 0;
	//PI Load loop control
	AL.ext1Ctrl.detaLoopCtrlOut = pidCalExt(_target,_real,_pidExten);
	//limit
	AL.ext1Ctrl.detaLoopCtrlOut = constraint(AL.ext1Ctrl.detaLoopCtrlOut,0.05,-0.05);
	_extenRecord = *_out;
	*_vm = AL.ext1Ctrl.detaLoopCtrlOut / 0.02f;
	}
	*_out = _extenRecord + AL.ext1Ctrl.detaLoopCtrlOut * ((1 + *_count) / 20.0);
	return *_out;
}

/*
函数名：DestWndMonitor  ***
功能：误差窗口检测
参数：	setpos 设定位置
		nowpos 当前位置
		wnd	   误差窗口结构体
		time   当前时间
返回值：
*/
uint8_t DestWndMonitor(float setpos,float nowpos,WND* wnd,uint32_t* timer)
{
	if(fabs(setpos-nowpos) <=  wnd->WndSize/2.0f)
		(*timer)++;
	else
		(*timer) = 0;
	if(*timer >= wnd->WndTime)
	{
		*timer = 0;
		return 1;//目标位置到达 任务完成
	}
	else
		return 0;//目标位置未到达 任务未完成 继续执行当前任务
			
}

void EnableState_Update(void)
{
	if(AL.emergency_state.complete == 1 || AL.emergency_state.processing == 0)
	{
		stateFlag.OpenDevice_state = commBuf.OpenDevice_state;
		stateFlag.DoPE_state		= commBuf.DoPE_state;
		stateFlag.DoPESetCtrl		= commBuf.DoPESetCtrl;
	}

}

void ETH_HeartbeatWatchdog(uint8_t* flag,uint16_t threshold){
	static uint16_t timcount = 0;
	++timcount;
	if(*flag == 1){
		*flag = 0;
		timcount = 0;
		stateFlag.lostEthHeartbeat = ETH_HEARTBEAT_GOT;
	}
	if(timcount > threshold && stateFlag.lostEthHeartbeat == ETH_HEARTBEAT_GOT){
		stateFlag.lostEthHeartbeat = ETH_HEARTBEAT_LOST;
	}
}



/**
 * @brief calculate the period of loopctrol, accoding to the posgeneration speed
 * @param[in] speed posgeneration speed,unit:mm/s
 * @param[in] sensitive	output pulse sensitive,uint:pulse/s
 * @param	  period	struct of period 
*/
uint16_t periodCalcu(float speed,double sensitive,ctrlPeriod_t* period){
	float fclkFactor = servoParam.ResolutionGearRate;
	speed = fabsf(speed);
	if(speed <= 16E-6f){
		speed = 16E-6f;
	}
	period->fclkExp = fabs(speed) / fabs(sensitive);
	period->expect = (uint16_t)(1600.0 / period->fclkExp / fclkFactor + 0.5);
	period->real = period->expect;
	if(period->expect >= period->max)
		period->real = period->max;
	if(period->expect <= period->min)
		period->real = period->min;

	return period->real;
}

/**
 * @brief combinedMove command completion monitor 
 * @param[in out] &mySemaphore.move
 * @param[in out] &combinedMove
 * @param[in] _lpusTAN
*/
void combinedMoveCommandCompletionMonitor(MOVE_MYSEMAPHORE* _move,combinedMove_t* _combinedMove,const uint16_t* _lpusTAN){
	if(_move->combinedMoveExecute == BEM_CMD_START){
		if(_combinedMove->blockHeader.ModeFlags == BHM_CMD_DWND){
			//continue to exectue following command 
			_move->pgInit = 1;
			if(_combinedMove->executeStep >= _combinedMove->counter){
				_combinedMove->executeStep = 0;
				//cycles finish monitor
				if(++_combinedMove->executeCycleCounter >=  _combinedMove->blockHeader.Cycles){
					_move->pgInit = 0;
				}
			}
			//trig err
			if(AL.trigState.flag == limitArrive){
				_move->pgInit = 0;
				log_e("combinedMove trig limitArrive, stop combinedMove!");
			}
		}else if(_combinedMove->blockHeader.ModeFlags == BHM_CMD_MESSAGE){
			//send lpusTAN to PC: set the semaphore and passes the lpusTAN value.
			xQueueSendToBack(lpusTANQueueHandle,_lpusTAN,0);
		}else if(_combinedMove->blockHeader.ModeFlags == (BHM_CMD_DWND | BHM_CMD_MESSAGE)){
			//continue to exectue following command 
			_move->pgInit = 1;
			if(_combinedMove->executeStep >= _combinedMove->counter){
				_combinedMove->executeStep = 0;
				//cycles finish monitor
				if(++_combinedMove->executeCycleCounter >= _combinedMove->blockHeader.Cycles){
					_move->pgInit = 0;
				}
			}
			//trig err
			if(AL.trigState.flag == limitArrive){
				_move->pgInit = 0;
				log_e("combinedMove trig limitArrive, stop combinedMove!");
			}
			//send lpusTAN to PC: set the semaphore and passes the lpusTAN value.
			xQueueSendToBack(lpusTANQueueHandle,_lpusTAN,0);
		}
	}else if(_move->combinedMoveExecute == BEM_CMD_DISCARD){
		_move->combinedMoveExecute = BEM_CMD_IDLE;
	}
	log_i("combinedMove executeStep:%d,executeCycleCounter:%d,blockHeader.Cycles:%d",_combinedMove->executeStep,
		_combinedMove->executeCycleCounter,_combinedMove->blockHeader.Cycles);	
}

/**
 * @brief According to LimitMode, calculate the real limit 
 * @param[in] _limit 
 * @param[in] _limitMode
 * @param[in] _sftLimitMax
 * @param[in] _sftLimitMin
 * @param[in] _limitPos
 * @param[in] _desPos
 * @param[in] _destiantion
*/
float posExtLimitCalculate(const float _limit,const uint8_t _limitMode,const float _sftLimitMax,const float _sftLimitMin,
						const float _limitPos,const float _desPos,const float _destiantion){
	float realLimit = 0;
	switch (_limitMode)
	{
	case LIMIT_ABSOLUTE:
		realLimit = _limit;
		break;
	case LIMIT_RELATIVE:
		realLimit = _limitPos + _limit;
		break;
	default:
		log_e("err posExt limitMode:0x%2x",_limitMode);
	case LIMIT_NOT_ACTIVE:
	//provisionally, pose and force are always in the same direction.
		if(_destiantion - _desPos >= 0){
			//up
			realLimit = _sftLimitMax;
		}else{
			//down
			realLimit = _sftLimitMin;
		}
		break;
	}
	return realLimit;
}

/**
 * @brief posExt command completion monitor process
 * @param _allCtrlPara type:ALLCTRLPARA*
*/
void posExtCompletionMonitorProcess(const ALLCTRLPARA* _allCtrlPara){
	log_i("DoPEPosExt moveCtrl:0x%2X destinationCtrl:0x%2X,destination arrive.",AL.movectrl,posext.destinationctrl);
	switch(posext.destinationmode){
		case DEST_APPROACH:
			stateFlag.posExtDestReach = reachApproach;
			AL.command = DOPEHALT;
			commBuf.Acc = _allCtrlPara->Nominal.Acc;
			commBuf.Dec = -_allCtrlPara->Nominal.Acc;
			commBuf.Speed = _allCtrlPara->Nominal.Speed;
			mySemaphore.move.pgInit = 1;
		break; 
		case DEST_POSITION:
			stateFlag.posExtDestReach = reachPosition;
			AL.movectrl = posext.destinationctrl;
			mySemaphore.move.pgInit = 1;
			log_i("stateFlag.posExtDestReach = 2\r\n");
		break;
		case DEST_MAINTAIN:
		//2024-12-12 15:54:26 this pattern is not implmenting for now.process it like mode APPROACH
			stateFlag.posExtDestReach = reachMaintain;
			AL.command = DOPEHALT;
			commBuf.Acc = _allCtrlPara->Nominal.Acc;
			commBuf.Dec = -_allCtrlPara->Nominal.Acc;
			commBuf.Speed = _allCtrlPara->Nominal.Speed;
			mySemaphore.move.pgInit = 1;
		//直接在闭环中对pg.st作修改
		break;
	}

}

/*********************pg param init function************************* */
/**
 * @brief the pg param init of trig or trig_A 
 * @note Fun_1717
 */
void trigPgParamInit(void){
	utcTrig.Limit = commBuf.trig.Limit;
	utcTrig.Trigger = commBuf.trig.Trigger;
	utcTrig.TriggerCtrl = commBuf.trig.TriggerCtrl;
	switch(AL.movectrl)
	{
		case POS_MODE:
			pg.sg	= utcTrig.Limit;
			pg.vmax = commBuf.Speed;
			pg.a	= commBuf.Acc;//加速度unit/s2
			pg.d	= commBuf.Dec;//减速度 unit/s2
			AL.posCtrl.setPos = pg.sg;
		break;
		case LOAD_MODE:
			pgLoad.sg	= utcTrig.Limit;
			pgLoad.vmax = commBuf.Speed;
			pgLoad.a	= commBuf.Acc;//加速度unit/s2
			pgLoad.d	= commBuf.Dec;//减速度 unit/s2
			AL.loadCtrl.setPos = pgLoad.sg;
			//log_i("pgLoad.vmax:%f",pgLoad.vmax);
		break;
		case EXTEN_MODE:				
			pgExt.sg	= utcTrig.Limit;
			pgExt.vmax = commBuf.Speed;
			pgExt.a	= commBuf.Acc;//加速度unit/s2
			pgExt.d	= commBuf.Dec;//减速度 unit/s2
			AL.ext1Ctrl.setPos = pgExt.sg;//wait for modification
		break;
	}
}

/*********************pg param init function************************* */

/**
 * @brief init the param of posExt when command is posExt,and clear the relvant param when command is others
 * @param _command move command
 */
void posExtParamInit(uint8_t _command){
	static uint8_t posExtInitFlag = 0;//0 represent unInit，1 represent has been inited
	if(_command != DOPEPOSEXT && _command != DOPEPOSEXT_A && posExtInitFlag == 1){
		posExtInitFlag = 0;
		stateFlag.posExtDestReach = reachNone;
		memset(&posext,0,sizeof(posext));
	}else if((_command == DOPEPOSEXT || _command == DOPEPOSEXT_A) && posExtInitFlag == 0){
		posExtInitFlag = 1;
		//some posExt init operation can be added here

	}else if((_command == DOPEPOSEXT || _command == DOPEPOSEXT_A) && posExtInitFlag == 1){
		//when reach the destination
		if(stateFlag.posExtDestReach != reachNone){
			AL.movectrl_last = AL.movectrl;
			AL.movectrl = posext.destinationctrl;
		}
	}
}


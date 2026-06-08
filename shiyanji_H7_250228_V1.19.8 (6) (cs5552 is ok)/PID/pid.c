#include "pid.h"
#include "gParameter.h"
#include "math.h"
#include "in_out.h"

pid_t pid_pos,pid_load,pid_ext;
static double pidLimit(double in,double upperlimit,double lowerlimit);

void pidInitPos(void)
{
    pid_pos.endValue=AL.posCtrl.setPos;
    pid_pos.setValue=0.0;
    pid_pos.actualValue=0.0;
    pid_pos.err=0.0;
    pid_pos.errLast=0.0;
    pid_pos.out=0.0;
    pid_pos.integral=0.0;
    pid_pos.outdeta=0.0;
    pid_pos.kp=posAllCtrlPara.PosPid.P;
    pid_pos.ki=posAllCtrlPara.PosPid.I;         //这里加大了积分环节的值
    pid_pos.kd=posAllCtrlPara.PosPid.D;
    pid_pos.uMAx=400000;
    pid_pos.uMin=0;
    pid_pos.outMAx = 25;
    pid_pos.outMin = -25;
	pid_pos.iThreshold = 0.1;
}

void pidInitLoad(void)
{
    pid_load.endValue=AL.loadCtrl.setPos;
    pid_load.setValue=0.0;
    pid_load.actualValue=0.0;
    pid_load.err=0.0;
    pid_load.errLast=0.0;
    pid_load.out=0.0;
    pid_load.integral=0.0;
    pid_load.outdeta=0.0;
    pid_load.kp=loadAllCtrlPara.PosPid.P;
    pid_load.ki=loadAllCtrlPara.PosPid.I;         //这里加大了积分环节的值
    pid_load.kd=loadAllCtrlPara.PosPid.D;
    pid_load.uMAx=400000;
    pid_load.uMin=0;
    pid_load.outMAx = 5000000;
    pid_load.outMin = -5000000;
	pid_load.iThreshold = 5;
    pid_load.outIntegral = 0;
    pid_load.outIntegralRecord = 0;
    AL.loadCtrl.posLoopCtrlOut = pose.orig;
    AL.cycleT = 20;
}

void pidInitExt(void){
    pid_ext.endValue=AL.ext1Ctrl.setPos;
    pid_ext.setValue=0.0;
    pid_ext.actualValue=0.0;
    pid_ext.err=0.0;
    pid_ext.errLast=0.0;
    pid_ext.out=0.0;
    pid_ext.integral=0.0;
    pid_ext.outdeta=0.0;
    pid_ext.kp=extAllCtrlPara.PosPid.P;
    pid_ext.ki=extAllCtrlPara.PosPid.I;         //这里加大了积分环节的值
    pid_ext.kd=extAllCtrlPara.PosPid.D;
    pid_ext.uMAx=400000;
    pid_ext.uMin=0;
    pid_ext.outMAx = 1;
    pid_ext.outMin = -1;
	pid_ext.iThreshold = 5; 
    AL.ext1Ctrl.posLoopCtrlOut = pose.orig;
    AL.cycleT = 20;
}

void pidParaUpdatePos(void){
    pid_pos.kp=posAllCtrlPara.PosPid.P;
    pid_pos.ki=posAllCtrlPara.PosPid.I;         //这里加大了积分环节的值
    pid_pos.kd=posAllCtrlPara.PosPid.D;
    pidInitPos();
}

void pidParaUpdateLoad(void){
    pid_load.kp=loadAllCtrlPara.PosPid.P;
    pid_load.ki=loadAllCtrlPara.PosPid.I;         //这里加大了积分环节的值
    pid_load.kd=loadAllCtrlPara.PosPid.D;
    pidInitLoad();
}

void pidParaUpdateExt(void){
    pid_ext.kp=extAllCtrlPara.PosPid.P;
    pid_ext.ki=extAllCtrlPara.PosPid.I;         //这里加大了积分环节的值
    pid_ext.kd=extAllCtrlPara.PosPid.D;
    pidInitExt();
}

float pidCalPos(float setValue,float realValue,pid_t* pid){
	uint8_t index = 1;
    // float K = 0;
	float preOut;
    pid->setValue = setValue;
    pid->actualValue = realValue;
    pid->err = setValue - realValue;

    if(pid->calcuRun == 1)
    {
        index = 1;
        pid->integral += pid->ki* pid->err;
        pid->out = pid->kp * pid->err + index * pid->integral + pid->kd * (pid->err - pid->errLast);
        preOut = pid->out;
        pid->out = pidLimit(pid->out,pid->outMAx,pid->outMin);
        pid->outdeta = preOut - pid->out;
    }
    pid->errLast = pid->err;
    return pid->out;
}


float pidCalLoad(double setValue,double realValue,pid_t* pid){
	int index = 1;
    pid->setValue=setValue;
    pid->err=setValue-realValue;

    if(pid->calcuRun == 1)
    {
        index = 1;
        // 1.积分分离 
        // 在误差小的时候比例小、积分大 
        // 误差大的时候比例大、积分小
        // if(fabs(setValue-AL.loadCtrl.setPos) <= fabs(AL.loadCtrl.setPos)*0.01){
        //     pid->kiSeparate = pid->kp*0.2;
        //     pid->kpSeparate = pid->kp*0.2;
        //     if(1){
        //       pid->integral = 0;
        //       ;
        //     } 
        // }else{
            pid->kiSeparate = pid->ki;
            pid->kpSeparate = pid->kp;
        //}

        //2.目标点处结合死区控制
        //pid->integral += pid->ki*(pid->err - pid->outdeta);
        pid->integral += pid->kiSeparate*(pid->err - pid->outdeta);
        pid->index = index;
        pid->out=pid->kpSeparate*pid->err+index*pid->integral + pid->kd * (pid->err - pid->errLast);
        pid->out = pidLimit(pid->out,pid->outMAx,pid->outMin);
        pid->outdeta = 0;
    }
    pid->errLast=pid->err;
    return pid->out;
}

float pidCalExt(double setValue,double realValue,pid_t* pid){
	int index = 1;
	float preOut;
    pid->setValue=setValue;
    pid->err=setValue-realValue;

    // if(pid->calcuRun == 1)
    // {
        index = 1;
        pid->integral += pid->ki*(pid->err - pid->outdeta);
        pid->index = index;
        pid->out=pid->kp*pid->err+index*pid->integral;
        preOut = pid->out;
        pid->out = pidLimit(pid->out,pid->outMAx,pid->outMin);
        pid->outdeta = preOut - pid->out;
    // }
    pid->errLast=pid->err;
    return pid->out;
}

static double pidLimit(double in,double upperlimit,double lowerlimit)
{	
	double out;
	out = in;
	if(out > upperlimit) out = upperlimit;
	if(out < lowerlimit) out = lowerlimit;
	return out;
}







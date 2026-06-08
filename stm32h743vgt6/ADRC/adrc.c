#include "adrc.h"
#include "math.h"
#include "gParameter.h"
#include "pid.h"

#define W0 50

FHAN_STRUCT fhan;

leso_t leso={
    .beta1 = 2*W0,
    .beta2 = W0*W0
};

static int8_t sign(double e){
    if(e > 1E-8)
        return 1;
    else if(e < -1E-8)
        return -1;
    else
        return 0;
}

double fal(double e,float a,float deta)
{  
  double e_ = 0;
  if(e<=deta && e>=-deta) // |e|<=deta
  {
    e_ =  e/pow(deta,1-a);
    return e_;
  }
  else // |e|>deta
  {
    e_ =  pow(fabs(e),a)/sign(e);      
    return e_;
  }
}

void adrcLESO(double U,double Y,double B0,double H,leso_t* Leso){
    Leso->u = U;
    Leso->y = Y;
    Leso->h = H;
    Leso->e = Leso->z1 - Leso->y;
	if(fabs( Leso->e) <= 1E-9)
		 Leso->e = 0.0;
	Leso->z2 -= Leso->h * Leso->beta2 *Leso->e; 
    // Leso->z2 = (Leso->z2 > 8  ? 8 : Leso->z2);
    // Leso->z2 = (Leso->z2 < -8 ? -8 : Leso->z2);
    Leso->z1 += Leso->h * (Leso->z2 - Leso->beta1 *Leso->e + 1 * Leso->u); 
	//test.esoCounter ++;
}

/**
 * @brief fsg
*/
int16_t adrcFsg(float x,float d)
{
  int16_t output=0;
  output=(sign(x+d)-sign(x-d))/2;
  return output;
}

void adrcFhanInputUpdate(FHAN_STRUCT *fhan,double x1,double x2,double r,double h){
  fhan->x1 = x1;
  fhan->x2 = x2;
  fhan->r  = r;
  fhan->h0  = h;
}

/**
 * @brief fhan
*/
float adrcFhan(FHAN_STRUCT *fhan_Input)//安排ADRC过度过程
{
  double d=0,a0=0,y=0,a1=0,a2=0,a=0;
  float output=0;
  d=fhan_Input->r*fhan_Input->h0*fhan_Input->h0;//d=rh^2;
  a0=fhan_Input->h0*fhan_Input->x2;//a0=h*x2
  y=fhan_Input->x1+a0;//y=x1+a0
  a1=sqrt(d*(d+8*fabs(y)));//a1=sqrt(d*(d+8*fabs(y))])
  a2=a0+sign(y)*(a1-d)/2;//a2=a0+sign(y)*(a1-d)/2;
  a=(a0+y)*adrcFsg(y,d)+a2*(1-adrcFsg(y,d));
  fhan_Input->fh=-fhan_Input->r*(a/d)*adrcFsg(a,d)
                  -fhan_Input->r*sign(a)*(1-adrcFsg(a,d));//得到最速微分加速度跟踪量
  output = (float)fhan_Input->fh;
  return output;
}

#define RAPID_FACTOR 1.05f

/**
 * @brief adrcIntFhan
 * 
 */

/**
 * @brief when system init,should update the initPos in z1
 * @param Leso
 * @param initPos
*/
void adrcInit(leso_t* Leso,float initPose){
  Leso->z1 = initPose;
  Leso->z2 = 0;
  Leso->e = 0;
  Leso->u = 0;
  Leso->y = 0;
  Leso->uPos = 0;
  AL.lesoU = 0;
}

/**
 * @brief adrc control law Load control interl loop - pos loop ctrol
 * @param[in] setPose external load loop output 
 * @param[in] realPose real pose 
 * @param[in] setSpeed (setPose) / (external load loop time) ,the velocity corresponding to a single closed-loop cycle
 * @param[in] realSpeed real pose speed
*/
void adrcControlLawLoadInterlPos(float setPose,float realPose,float setSpeed,float realSpeed){
  float vt;
    vt = fabsf(setSpeed * RAPID_FACTOR);
		adrcFhanInputUpdate(&fhan,setPose-realPose,setSpeed-realSpeed,vt,posCtrlPeriod.real/1000.0);//0.05
}

/**
 * @brief adrc control law Load control interl loop - pos loop ctrol
 * @param[in] setPose     pg.st 
 * @param[in] realPose    real pose 
 * @param[in] setSpeedMax pg.vm
 * @param[in] setSpeed    pg.vt ,the velocity corresponding to a single closed-loop cycle
 * @param[in] realSpeed   real pose speed
*/
void adrcControlLawPos(float setPose,float realPose,float setSpeedMax,float setSpeed,float realSpeed){
  float vt;
	vt = fabsf(setSpeed * RAPID_FACTOR);
	if(fabsf(setSpeedMax) <= 0.0005f){
		adrcFhanInputUpdate(&fhan,setPose-realPose,pg.aa,vt,posCtrlPeriod.real/1000.0);//r upper,h lower
	}else if(fabsf(setSpeedMax) <= 0.005f){
		vt = fabsf(setSpeed * 1.8f);
		adrcFhanInputUpdate(&fhan,(setPose-realPose),pg.aa,vt,posCtrlPeriod.real/1000.0);//0.05
	}else{
		adrcFhanInputUpdate(&fhan,setPose-realPose,setSpeed-realSpeed,vt,posCtrlPeriod.real/1000.0);//0.05
	}

	
}

#define MAX_CREEP_SPEED 0.1
#define MIN_CREEP_SPEED 0.000015
/**
 * @brief tail end creep 
 * @param[in] setPose     pg.st 
*/
void adrcControlLawPoswithCreep(const float endPose,const float setPose,const float realPose,
                        const float setSpeedMax,const float setSpeed,const float realSpeed){
  float vt,vm;
	vm = fabsf(setSpeedMax * RAPID_FACTOR);
  // vCreep = vm / 10.0f;
  // if(vCreep > MAX_CREEP_SPEED){
  //   vCreep = MAX_CREEP_SPEED;
  // }else if(vCreep < MIN_CREEP_SPEED){
  //   vCreep = MIN_CREEP_SPEED;
  // }

  // if( fabs(realPose - endPose) <= vm*0.1f && AL.movectrl == POS_MODE){
  //   vt = vCreep;
  //    AL.creepFlag = 1;
  // }else{
  //    AL.creepFlag = 0;
    vt = vm;
 // }
	if(fabsf(setSpeedMax) <= 0.005f){
		adrcFhanInputUpdate(&fhan,setPose-realPose,setSpeed-realSpeed,vt,posCtrlPeriod.real/1000.0);//r upper,h lower
	}else{
		adrcFhanInputUpdate(&fhan,setPose-realPose,setSpeed-realSpeed,vt,posCtrlPeriod.real/1000.0);//0.05
	}
}







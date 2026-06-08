#include "in_out.h"
#include "DI.h"
#include "DO.h"
#include "usart.h"
#include "cmsis_os.h"
#include "Computer_vision.h"
#include "control.h"
#include "gParameter.h"
#include "posGenerator.h"
#include "in_out.h"

void LEDlight_Tog()
{
		HAL_GPIO_TogglePin(GPIOC, P_LEDWK0_Pin);
		HAL_GPIO_TogglePin(GPIOC,P_LEDWK1_Pin);
}

void LEDlight_Off()
{
		HAL_GPIO_WritePin(GPIOC, P_LEDWK0_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,P_LEDWK1_Pin,GPIO_PIN_RESET);
}

uint8_t view_flag=1;
//波形显示数组
float view[4];
void computer_view()
{
      if(view_flag == 0)
      {
//        view[0]= ads_par.value_calcu[2];
//        view[1]= ads_par.value_calcu[1];
//        view[2]= ads_par.value_calcu[0];
        vcan_sendware((uint8_t *)view, sizeof(view));
      }
      else if(view_flag == 1)
      {
        view[0]= pose.orig;
				view[1]= svPWM.pos/1000.0f;
        view[2]= speedPose.filter*10.0f;
				view[3]= pg.st;
        vcan_sendware((uint8_t *)view, sizeof(view));
      }
      else if(view_flag == 2)
      {
//        view[0]= adc_x;
//        view[1]= adc_y;
        vcan_sendware((uint8_t *)view, sizeof(view));
      }

}

void forceTareTestPrint(void)
{
  printf("%d,%d,%f,%f,%f\r\n",force.code,force.codeFiter,force.filter,force.filter,AL.tare.fValue[4]);
}


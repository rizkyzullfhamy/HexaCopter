/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    13-November-2013
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "tm_stm32f4_delay.h"
#include "main.h"
#include "tm_stm32f4_pwm.h"
#include "Global_Variable.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
extern TM_PWM_TIM_t TIM2_Data, TIM9_Data, TIM12_Data;;
extern float dataYaw,dataPitch,dataRoll, dataYaw_Zero, dataPitch_Zero, dataRoll_Zero;
extern float lastError1, hasilError1,lastError2, hasilError2, lastError3, hasilError3, lastError4, hasilError4, lastError5, hasilError5, lastError6, hasilError6, errRoll,errSUMR,errPitch, errSUMP, errSUMY;
extern float lastErrorAltitude, hasilErrorAltitude, setPointAltitude, errorAltitude, pidAl, HasilPidAl;   // Altitude PID
extern float kpAl, kdAl, kiAl;
extern float kpr,kdr,kir, kpp,kdp,kip, kpy,kdy,kiy;
extern float setPointRoll, setPointYaw, setPointPitch, pidkpr,pidkdr, pidkpy,pidkdy, pidkpp,pidkdp, sudut_belok;
extern float pidp,pidr,pidy;
extern float error1,error2,error3;
extern int OutpidP1, OutpidP2, OutpidP3, OutpidP4;
extern float w1,w2,w3,w4;
extern float lastErrorAltitude, hasilErrorAltitude, setPointAltitude, errorAltitude, pidAl, HasilPidAl;   // Altitude PID
extern float data_altitude_zero;
extern int data_altitude;
extern float derivative, derivative1, derivative2;
extern double input5,input6;
extern float Tr, Tp, Tz, Ty;
extern float w1,w2,w3,w4,w5,w6;

//########################## PID HEXACOPTER ###################################//
		extern float Time_Sampling;
		
	// GLOBAL VARAIBEL PID ROLL
		extern float ErrRoll, IntegralErrRoll, DerivatifErrRoll, LastErrorRoll;
		extern float PropotionalRoll, IntegralRoll, DerivatifRoll,PIDROLL;
		
	// GLOBAL VARAIBEL PID PITCH
		extern float ErrPitch, IntegralErrPitch, DerivatifErrPitch, LastErrorPitch;
		extern float PropotionalPitch, IntegralPitch, DerivatifPitch, PIDPITCH;
	// GLOBAL VARAIBEL PID YAW
		extern float ErrYaw, IntegralErrYaw, DerivatifErrYaw, LastErrorYaw;
		extern float PropotionalYaw, IntegralYaw, DerivatifYaw, PIDYAW;
		
	// GLOBAL VARIABEL OUTPID ROLL PITCH YAW //
		extern int OutpidP1, OutpidP2, OutpidP3, OutpidP4,OutpidP5,OutpidP6;
	
//#########################################################################//


/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function decrement timing variable
  *	@with __weak parameter to prevent errors
  * @param  None
  * @retval None
  */
__weak void TimingDelay_Decrement(void) {

}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//PID CONTROL QUADCOPTER Interrupt

void AltitudePID(){
	errorAltitude = data_altitude_zero - data_altitude;
	errorAltitude *= -1;
	hasilErrorAltitude = errorAltitude - lastErrorAltitude;
	lastErrorAltitude = errorAltitude;
	
	pidAl = ((errorAltitude *kpAl) + (hasilErrorAltitude*kdAl)) / 0.001;
	HasilPidAl = w1 + pidAl;    // Menurunkan Hingga Posisi Awal
	
	if(HasilPidAl >= 1700) {HasilPidAl = 1700;}
	if(HasilPidAl <= 1000) {HasilPidAl = 1000;}
}




//########################### CALCULATION PID HEXACOPTER ################################//
void SETPID_PITCH(){
	//PROPOTIONAL
	ErrPitch 					 	 = Tp - dataPitch;			//(Tp = Data Parameter Remote Kondisi noll)
	
	//INTEGRAL
	IntegralErrPitch  	+= ErrPitch * Time_Sampling;
	if (IntegralErrPitch >= 2000) {IntegralErrPitch = 2000;}
	else if (IntegralErrPitch <= -2000) { IntegralErrPitch = -2000;}
	
	//DERIVATIF
	DerivatifErrPitch 	 = (ErrPitch - LastErrorPitch) / Time_Sampling;
	LastErrorPitch 				 = ErrPitch;
	
	PropotionalPitch  = ErrPitch * kpp;									//(FORMULA PROPOTIONAL PITCH)
	IntegralPitch			= IntegralErrPitch * kip;					//(FORMULA INTEGRAL PITCH)
	DerivatifPitch 		= DerivatifErrPitch * kdp;				//(FORMULA DERIVATIF PITCH)
	
	PIDPITCH = PropotionalPitch + IntegralPitch + DerivatifPitch;		//(FORMULA PID PITCH)
}

void SETPID_ROLL(){
	//PROPOTIONAL
	ErrRoll 					 	 = Tp - dataRoll;			//(Tp = Data Parameter Remote Kondisi noll)
	
	//INTEGRAL
	IntegralErrRoll 	+= ErrRoll * Time_Sampling;
	if (IntegralErrRoll >= 2000) {IntegralErrRoll = 2000;}
	else if (IntegralErrRoll <= -2000) { IntegralErrRoll = -2000;}
	
	//DERIVATIF
	DerivatifErrRoll 	 = (ErrRoll - LastErrorRoll) / Time_Sampling;
	LastErrorRoll				 = ErrRoll;
	
	PropotionalRoll   = ErrRoll * kpr;									//(FORMULA PROPOTIONAL ROLL)
	IntegralRoll			= IntegralErrRoll * kir;					//(FORMULA INTEGRAL ROLL)
	DerivatifRoll 		= DerivatifErrRoll * kdr;				//(FORMULA DERIVATIF ROLL)
	
	PIDROLL = PropotionalRoll + IntegralRoll + DerivatifRoll;		//(FORMULA PID ROLL)
}

void SETPID_YAW(){
	//PROPOTIONAL
	ErrYaw 					 	 = Tp - dataYaw;			//(Tp = Data Parameter Remote Kondisi noll)
	
	//INTEGRAL
	IntegralErrYaw 	+= ErrYaw * Time_Sampling;
	if (IntegralErrYaw >= 2000) {IntegralErrYaw = 2000;}
	else if (IntegralErrYaw <= -2000) {IntegralErrYaw = -2000;}
	
	//DERIVATIF
	DerivatifErrYaw 	   = (ErrYaw - LastErrorYaw) / Time_Sampling;
	LastErrorYaw				 = ErrYaw;
	
	PropotionalYaw   = ErrYaw * kpy;									//(FORMULA PROPOTIONAL PITCH)
	IntegralYaw			= IntegralErrYaw * kiy;					//(FORMULA INTEGRAL PITCH)
	DerivatifYaw 		= DerivatifErrYaw * kdy;				//(FORMULA DERIVATIF PITCH)
	
	PIDYAW = PropotionalYaw + IntegralYaw + DerivatifYaw;		//(FORMULA PID PITCH)
}

void Trust_Control_HEXACOPTER(){

	OutpidP1  = w1 - PIDPITCH + PIDYAW;
	
	OutpidP2  = w2 + PIDROLL  -  PIDPITCH - PIDYAW;
	OutpidP3  = w3 - PIDROLL  +  PIDPITCH + PIDYAW;
	
	OutpidP4  = w4 + PIDPITCH - PIDYAW;
	
	OutpidP5  = w5 + PIDROLL  +  PIDPITCH + PIDYAW;
	OutpidP6  = w6 - PIDROLL  -  PIDPITCH - PIDYAW;	

	if (OutpidP1>=2000){OutpidP1=2000;}
	if (OutpidP1<=1000){OutpidP1=1000;}

	if (OutpidP2>=2000){OutpidP2=2000;}
	if (OutpidP2<=1000){OutpidP2=1000;}
	
	if (OutpidP3>=2000){OutpidP3=2000;}
	if (OutpidP3<=1000){OutpidP3=1000;}
	
	if (OutpidP4>=2000){OutpidP4=2000;}
	if (OutpidP4<=1000){OutpidP4=1000;}
	
	if (OutpidP5>=2000){OutpidP5=2000;}
	if (OutpidP5<=1000){OutpidP5=1000;}
	
	if (OutpidP6>=2000){OutpidP6=2000;}
	if (OutpidP6<=1000){OutpidP6=1000;}
	
	if (input6 > 5 ) {		
	TM_PWM_SetChannelMicros(&TIM9_Data, TM_PWM_Channel_1, OutpidP1); 		 //PE5		
  TM_PWM_SetChannelMicros(&TIM12_Data,TM_PWM_Channel_1, OutpidP4); 	   //PB14			
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, OutpidP2);		 //PA5
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_3, OutpidP6);		 //PA2
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_2, OutpidP3); 		 //PA1
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_4, OutpidP5);		 //PA3	
	}
	
}
//##################################################################################//


//============================= PID QUADCOPTER ================================//
/*
void pitchPID(){
	error1 = Tp - dataPitch;
	errSUMP += error1 * 0.01;
	if (errSUMP >= 2000) {errSUMP =2000;}
	else if (errSUMP <= -2000) {errSUMP =-2000;}
	derivative = (error1 - lastError1)/0.01;
	lastError1 =  error1;
	pidp = (error1*kpp) + (kip*errSUMP) + (derivative*kdp);
		
	//OutpidP1 = w1 + pidp + pidy;
	//OutpidP3 = w3 - pidp + pidy;
	
	//if (OutpidP1>=2000){OutpidP1=2000;}
	//if (OutpidP1<=1000){OutpidP1=1000;}
	
	//if (OutpidP3>=2000){OutpidP3=2000;}
	//if (OutpidP3<=1000){OutpidP3=1000;}
	//if (input5 > 8) {
	//	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, OutpidP3);	//PA5		pitch	
	//	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_3, OutpidP1);	//PA2		pitch
	//}
	
}
void rollPID(){
	error2=Tr - dataRoll;
	errSUMR += error2 * 0.01;
	if (errSUMR >= 2000) {errSUMR =2000;}
	else if (errSUMR <= -2000) {errSUMR =-2000;}
	derivative1 = (error2 - lastError2)/0.01;
	lastError2 = error2;
	
	pidr = (error2*kpr) + (kir*errSUMR) + (derivative1*kdr);
	
	//OutpidP4 = w4 + pidr - pidy;
	//OutpidP2 = w2 - pidr - pidy;
	
	//if (OutpidP4>=2000){OutpidP4=2000;}
	//if (OutpidP4<=1000){OutpidP4=1000;}
						
	//if (OutpidP2>=2000){OutpidP2=2000;}
	//if (OutpidP2<=1000){OutpidP2=1000;}
	//if (input5 > 8 ) {
	//TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_2, OutpidP2);	//PA1		roll
	//TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_4, OutpidP4);	//PA3		roll
	//}	
	
}
void yawPID(){
	error3 = Ty - dataYaw;
	errSUMY += error3 * 0.01;
	if (errSUMY >= 2000) {errSUMY = 2000;}
	else if(errSUMY <= -2000) {errSUMY =-2000;}
	derivative2 =  (error3 - lastError3) /0.01;
	lastError3 = error3;
	
	pidy = (error3*kpy) + (kiy*errSUMY) + (derivative2 *kdy);
	
}
void Trust_Control(){
	OutpidP4 =w4+pidr+pidy;	 		// w4 + pidr + pidy;	
	OutpidP2 =w2-pidr+pidy;			// w2 - pidr + pidy;
	
	OutpidP1 = w1 + pidp -pidy; 	  // w1 + pidp - pidy;
	OutpidP3 = w3 - pidp -pidy;		// w3 - pidp - pidy;
	
	if (OutpidP4>=2000){OutpidP4=2000;}
	if (OutpidP4<=1000){OutpidP4=1000;}
						
	if (OutpidP2>=2000){OutpidP2=2000;}
	if (OutpidP2<=1000){OutpidP2=1000;}
	
	if (OutpidP1>=2000){OutpidP1=2000;}
	if (OutpidP1<=1000){OutpidP1=1000;}
	
	if (OutpidP3>=2000){OutpidP3=2000;}
	if (OutpidP3<=1000){OutpidP3=1000;}
	
	if (input5 > 8 ) {
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_2, OutpidP2);	//PA1		roll
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_4, OutpidP4);	//PA3		roll
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, OutpidP3);	//PA5		pitch	
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_3, OutpidP1);	//PA2		pitch
	}
}

*/

int counter1=0;
void SysTick_Handler(void)
{
	counter1++;
	
	if (counter1 >= 10)
		{
			SETPID_YAW();
			SETPID_PITCH();
			SETPID_ROLL();
			Trust_Control_HEXACOPTER();
			counter1 = 0;
		}
		
	TimingDelay_Decrement();
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

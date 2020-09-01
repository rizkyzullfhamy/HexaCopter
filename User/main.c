//############################################ Include Library ##############################################//
	/* Include core modules */
	#include "Global_Variable.h"
	#include "stm32f4xx.h"
	/* Include my libraries here */
	#include "defines.h"
	#include "string.h"
	#include "stdio.h"
	#include "stdlib.h"
	#include "tm_stm32f4_timer_properties.h"
	#include "tm_stm32f4_pwmin.h"
	#include "tm_stm32f4_pwm.h"
	#include "tm_stm32f4_bmp180.h"
	#include "tm_stm32f4_delay.h"
	#include "tm_stm32f4_usart.h"
	#include "tm_stm32f4_usart_dma.h"
	#include "tm_stm32f4_adc.h"
	#include "tm_stm32f4_exti.h"
	#include "arm_math.h"
	
//##################### PID HEXACOPTER ###################################//
		
		float Time_Sampling = 0.01; //millsecond
	// GLOBAL VARAIBEL PID ROLL
		float ErrRoll, IntegralErrRoll, DerivatifErrRoll, LastErrorRoll;
		float PropotionalRoll, IntegralRoll, DerivatifRoll,PIDROLL;
		
	// GLOBAL VARAIBEL PID PITCH
		float ErrPitch, IntegralErrPitch, DerivatifErrPitch, LastErrorPitch;
		float PropotionalPitch, IntegralPitch, DerivatifPitch, PIDPITCH;
	// GLOBAL VARAIBEL PID YAW
		float ErrYaw, IntegralErrYaw, DerivatifErrYaw, LastErrorYaw;
		float PropotionalYaw, IntegralYaw, DerivatifYaw, PIDYAW;
		
	// GLOBAL VARIABEL OUTPID ROLL PITCH YAW //
		int OutpidP1, OutpidP2, OutpidP3, OutpidP4,OutpidP5,OutpidP6;
		
	// SET PID HEXACOPTER //
		//==========PID FIX HEXACOPTER===========//
			float kpr=3.5; //4.5 						
			float kir=1.0;
			float kdr=1.5; //1.5

			float kpp=3.5;
			float kip=1.0;
			float kdp=1.5;

			float kpy=3.5;
			float kiy=1.0; 
			float kdy=1.5;
			
			/*
			float kpr=4.5; 						
			float kir=1.0;
			float kdr=1.5;

			float kpp=4.5;
			float kip=1.0;
			float kdp=1.5;

			float kpy=4.5;
			float kiy=1.0; 
			float kdy=1.5;
			*/
	//==========================================//
	
//#########################################################################//
/*
float kpr=2.5; //10
float kir=0.1;
float kdr=1.0;

float kpp=2.5;
float kip=0.1;
float kdp=1.0;

float kpy=2.5;
float kiy=0.1; 
float kdy=1.0;
*/

float derivative, derivative1, derivative2;
//#########DEKLARASI VARIABEL GLOBAL##########//
	union Floating{
	uint8_t bytes[4];
	float value;	
	};
	union Doublee{
	uint8_t bytes[8];
	double value1;	
	};
//BUFFER
	 char buffer[200];
	 unsigned long timer1=0, timer2=0, timer3=0;
// MODE
	 char serial3[100];
	 int takeoff_flag=0;
	 int fmode;
//VAR-CONTROL PID
	 char kprpid[15],kdrpid[15],kpypid[15],kdypid[15],kpppid[15],kdppid[15];
	 float lastError1, hasilError1,lastError2, hasilError2, lastError3, hasilError3, lastError4, hasilError4, lastError5, hasilError5, lastError6, hasilError6, errRoll,errSUMR,errPitch, errSUMP, errSUMY;
	 float lastErrorAltitude, hasilErrorAltitude, setPointAltitude, errorAltitude, pidAl, HasilPidAl;   // Altitude PID
	 float kpAl=0.01, kdAl=0, kiAl=0;
	// float kpr=1.5,kir=0, kdr=0, kpp=5,kip=0,kdp=0,kpy=1,kiy=0, kdy=0;
	 float setPointRoll, setPointYaw, setPointPitch, pidkpr,pidkdr, pidkpy,pidkdy, pidkpp,pidkdp, sudut_belok;
	 float pidp,pidr,pidy;
	 float error1,error2,error3;
	 int OutpidP1, OutpidP2, OutpidP3, OutpidP4;
//VAR-GPS
	 char gps[200];
	 char serial4[200];
	 int cek2,a2,b2,flag2,nomor_parsing2=0,ke2=0;
	 float data_lat,data_longi;
	 int data_time;
	 char lat[15],lat_char[15],longi[15],longi_char[15],valid[15],time[15];  
	 float latitude,longitude,latitude_zero,longitude_zero,selisih_gps_lat,selisih_gps_long;
	 union Floating currentLat, currentLong;
	 union Doublee currentLat1, currentLong1;
//VAR-AIRSPEED & BateraiMonitor
	 const float offset = 0.6; // The offset voltage is 1.0V when there is no pressure difference.
	 const float sensitivity = 0.6; // The sensitivity is 1.0V per kPa for the sensor.
	 int rawADC, rawADC1;
	 union Floating baterai;	
	 float voltage;
	 float pressure;
	 union Floating windspeed;
//IMU
	 float YPR[3];
	 unsigned char Re_buf[8],counter=0;
	 unsigned char Re_buf[8];
	 unsigned char sign=0;
	 float dataYaw, dataPitch, dataRoll;
	 union Floating dataYaw_1,dataPitch_1,dataRoll_1;
	 float dataYaw_Zero, dataPitch_Zero, dataRoll_Zero;
	 int resetKalIMU=0;
//KINEMATIK 
	float Tr, Tp, Tz, Ty;
	float KT=1, KQ=1, L=0.28;
	float w1,w2,w3,w4,w5,w6;
//TIM IN
	 double input1, input2, input3, input4, input5, input6;
	 double input1_gui, input2_gui, input3_gui, input4_gui, input5_gui, input6_gui;
	 TM_PWMIN_t PWMIN_TIM1;
	 TM_PWMIN_t PWMIN_TIM4;
	 TM_PWMIN_t PWMIN_TIM3;
	 TM_PWMIN_t PWMIN_TIM5;
	 TM_PWMIN_t PWMIN_TIM9;
	 TM_PWMIN_t PWMIN_TIM8;
//TIM OUT
	 TM_PWM_TIM_t TIM2_Data,TIM9_Data,TIM12_Data;
	 NVIC_InitTypeDef NVIC_InitStructure;
// BMP 180
	 TM_BMP180_t BMP180_Data;
	 float data_altitude_zero;
	 union Floating altitude;
	 int data_altitude;
	 int resetKalBMP=0;
//######## DEKLARASI FUNGSI ########### //
double map (double x, double in_min, double in_max, double out_min, double out_max){
	return (x - in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}
//IMU
void resetIMU(){
	TM_USART_Puts(USART3,"Masuk IMU");
	TM_USART_Putc(USART2, 0xA5);
	TM_USART_Putc(USART2, 0x55);
	Delayms(170);
	TM_USART_Putc(USART2,0xA5);
	TM_USART_Putc(USART2,0x54);
  Delayms(170);
	TM_USART_Putc(USART2,0xA5);
	TM_USART_Putc(USART2,0x52);
	TM_USART_Puts(USART3,"keluar IMU");
}
void interruptInit(){
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void USART2_IRQHandler(void){
		if(USART_GetITStatus(USART2, USART_IT_RXNE)) {
			Re_buf[counter] = USART_ReceiveData(USART2);
			if(counter == 0 && Re_buf[0] != 0xAA) return;
			counter++;
			if (counter == 8) {
				counter = 0;
				sign = 1;
			}
		}
		if(sign == 1 &&Re_buf[0] == 0xAA && Re_buf[7] == 0x55) { sign = 0;
			YPR[0] = (Re_buf[1] << 8 | Re_buf[2]) * 0.01f;
			YPR[1] = (Re_buf[3] << 8 | Re_buf[4]) * 0.01f;
			YPR[2] = (Re_buf[5] << 8 | Re_buf[6]) * 0.01f;
			//Setelah nilai IMU 179 lalu melompat ke 475
			//range sudut 0-179 lalu 475-655 
			if (YPR[0] > 179)		{dataYaw = (655-YPR[0]);
																if(YPR[0]>475){
																dataYaw = -dataYaw;
																dataYaw_1.value = dataYaw;
																} //RANGE 0 <-> 180 || -179 <-> -1
													}
			else{									dataYaw = YPR[0]; dataYaw_1.value = dataYaw;}
			
			if (YPR[1] > 179)		{dataPitch = (655-YPR[1]);
															if(YPR[1]>475){
															dataPitch= -dataPitch;
															dataPitch_1.value = dataPitch;
															} //RANGE 0 <-> 180 || -179 <-> -1
													}
			else{									dataPitch = YPR[1]; dataPitch_1.value = dataPitch;}

			if (YPR[2] > 179)		{dataRoll = (655-YPR[2]);
																if(YPR[2]>475){
																dataRoll=-dataRoll;
																dataRoll_1.value = dataRoll;
																} //RANGE 0 <-> 180 || -179 <-> -1
													}
			else{									dataRoll = YPR[2]; dataRoll_1.value = dataRoll;	}
		}
}
void KalibrasiIMU(){
	while(1){
		dataYaw_Zero 		= dataYaw_Zero + dataYaw;
		dataPitch_Zero  = dataPitch_Zero + dataPitch;
		dataRoll_Zero 	= dataRoll_Zero + dataRoll;
		resetKalIMU 		= resetKalIMU + 1;
		sprintf(buffer,"\n\rBISMILLAH KALIBRASI IMU...%d %%", resetKalIMU );
		Delayms(50);
		TM_USART_Puts(USART3, buffer);
		if (resetKalIMU == 100){
			dataYaw_Zero   = dataYaw_Zero   / 100.0;
			dataPitch_Zero = dataPitch_Zero / 100.0;
			dataRoll_Zero  = dataRoll_Zero  / 100.0;
			TM_USART_Puts(USART3,"\n\rALHAMDULILLAH KALIBRASI IMU BERHASIL...");
			Delayms(1000);
			resetKalIMU=0;
			break;
		}
	}
}
//PWM INPUT & OUTPUT (INTERRUPT TIMER)
void TIM1_CC_IRQHandler(void) {
	/* Interrupt request, don't forget! */
	TM_PWMIN_InterruptHandler(&PWMIN_TIM1);
}
void TIM4_IRQHandler(void) {
	/* Interrupt request, don't forget! */
	TM_PWMIN_InterruptHandler(&PWMIN_TIM4);
}
void TIM3_IRQHandler(void) {
	/* Interrupt request, don't forget! */
	TM_PWMIN_InterruptHandler(&PWMIN_TIM3);
}
void TIM5_IRQHandler(void) {
	/* Interrupt request, don't forget! */
	TM_PWMIN_InterruptHandler(&PWMIN_TIM5);
}
void TIM1_BRK_TIM9_IRQHandler(void) {
	/* Interrupt request, don't forget! */
	TM_PWMIN_InterruptHandler(&PWMIN_TIM9);
}
void TIM8_CC_IRQHandler(void) {
	/* Interrupt request, don't forget! */
	TM_PWMIN_InterruptHandler(&PWMIN_TIM8);
}
void InitPWMIN(){
	TM_PWMIN_InitTimer(TIM1, &PWMIN_TIM1, TM_PWMIN_Channel_1, TM_PWMIN_PinsPack_1, 50, TIM1_CC_IRQn);       //PA8
	TM_PWMIN_InitTimer(TIM4, &PWMIN_TIM4, TM_PWMIN_Channel_1, TM_PWMIN_PinsPack_2, 50, TIM4_IRQn);          //PD12
  TM_PWMIN_InitTimer(TIM3, &PWMIN_TIM3, TM_PWMIN_Channel_1, TM_PWMIN_PinsPack_2, 50, TIM3_IRQn);          //PB4
  TM_PWMIN_InitTimer(TIM5, &PWMIN_TIM5, TM_PWMIN_Channel_1, TM_PWMIN_PinsPack_1, 50, TIM5_IRQn);          //PA0
	TM_PWMIN_InitTimer(TIM9, &PWMIN_TIM9, TM_PWMIN_Channel_2, TM_PWMIN_PinsPack_2, 50, TIM1_BRK_TIM9_IRQn); //PE6
	TM_PWMIN_InitTimer(TIM8, &PWMIN_TIM8, TM_PWMIN_Channel_2, TM_PWMIN_PinsPack_1, 50, TIM8_CC_IRQn );			//PC7
}
void InitPWMOUT(){
	//Init TimerPWM TIM2
	TM_PWM_InitTimer(TIM2, &TIM2_Data, 50);
	TM_PWM_InitTimer(TIM9, &TIM9_Data, 50);
	TM_PWM_InitTimer(TIM12, &TIM12_Data,50);
	
	//Brushless
	TM_PWM_InitChannel(&TIM2_Data, TM_PWM_Channel_1, TM_PWM_PinsPack_2);    //PA5
	TM_PWM_InitChannel(&TIM2_Data, TM_PWM_Channel_2, TM_PWM_PinsPack_1);		//PA1
	TM_PWM_InitChannel(&TIM2_Data, TM_PWM_Channel_3, TM_PWM_PinsPack_1);		//PA2
	TM_PWM_InitChannel(&TIM2_Data, TM_PWM_Channel_4, TM_PWM_PinsPack_1);		//PA3
	
	TM_PWM_InitChannel(&TIM9_Data, TM_PWM_Channel_1, TM_PWM_PinsPack_2); 		// PE5
	TM_PWM_InitChannel(&TIM12_Data, TM_PWM_Channel_1, TM_PWM_PinsPack_1); 	// PB14
}
void GetPWM(){
	if(TM_DELAY_Time() - timer2 >= 50){
			TM_PWMIN_Get(&PWMIN_TIM1);
			TM_PWMIN_Get(&PWMIN_TIM4);
			TM_PWMIN_Get(&PWMIN_TIM3);
			TM_PWMIN_Get(&PWMIN_TIM5);
			TM_PWMIN_Get(&PWMIN_TIM9);
			TM_PWMIN_Get(&PWMIN_TIM8);
		
			input1 = PWMIN_TIM1.DutyCycle;
			input2 = PWMIN_TIM4.DutyCycle;
			input3 = PWMIN_TIM3.DutyCycle;
			input4 = PWMIN_TIM5.DutyCycle;
			input5 = PWMIN_TIM9.DutyCycle;
			input6 = PWMIN_TIM8.DutyCycle;
		
		  Tr = map(input1, 5, 10, -10, 10);
		  Tp = map(input2, 5, 10, 10, -10);
		  Tz = map(input3, 5, 10, 1000, 2000);
		  Ty = map(input4, 5, 10, -5, 5);
		
		  if (Tz == 1000 && Tr == 0 && Tp == 0 && Ty == 0)
			{
				w1 = 1000;w2 = 1000;w3 = 1000;w4 = 1000; w5=1000; w6=1000;
			}
			else
			{
				//Hexacopter
				w1 = Tz/(4*KT) + Tp/(KT*L) - Ty/(2*KQ); 
				w2 = Tz/(4*KT) + Tr/(2*0.86602540378*KT*L/2) + Tp/(KT/2) + Ty/(2*KQ); 
				w3 = Tz/(4*KT) - Tr/(2*0.86602540378*KT*L/2) - Tp/(KT/2) - Ty/(2*KQ);  
				w4 = Tz/(4*KT) - Tp/(KT*L) + Ty/(2*KQ);
				w5 = Tz/(4*KT) + Tr/(2*0.86602540378*KT*L/2) - Tp/(KT/2) - Ty/(2*KQ);
				w6 = Tz/(4*KT) - Tr/(2*0.86602540378*KT*L/2) + Tp/(KT/2) + Ty/(2*KQ);
						
				w1 = map(w1, 250, 500, 1000, 2000);
				w2 = map(w2, 250, 500, 1000, 2000);
				w3 = map(w3, 250, 500, 1000, 2000);
				w4 = map(w4, 250, 500, 1000, 2000);
				w5 = map(w5, 250, 500, 1000, 2000);
				w6 = map(w6, 250, 500, 1000, 2000);
				
				if (w1<1000) {w1=1000;}		// ROLL
				if (w1>2000) {w1=2000;}
				
				if (w2<1000) {w2=1000;}		// PITCH
				if (w2>2000) {w2=2000;}
				
				if (w4<1000) {w4=1000;}		// YAW
				if (w4>2000) {w4=2000;}
				
				if (w3<1000) {w3=1000;}		// HOVER
				if (w3>2000) {w3=2000;}
				
				if (w5<1000) {w5=1000;}		
				if (w5>2000) {w5=2000;}
				
				if (w6<1000) {w6=1000;}		
				if (w6>2000) {w6=2000;}
				
				if (Tz<1000) {Tz=1000;}		
				if (Tz>2000) {Tz=2000;}
			}
			
		  TM_PWM_SetChannelMicros(&TIM9_Data, TM_PWM_Channel_1, w1); 		 //PE5		
		  TM_PWM_SetChannelMicros(&TIM12_Data, TM_PWM_Channel_1, w4); 	 //PB14			
			
			TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, w2);		 //PA5
			TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_3, w6);		 //PA2
			TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_2, w3); 		 //PA1
			TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_4, w5);		 //PA3		

			timer2=TM_DELAY_Time();
		}
}
// BMP ALTITUDE 
void InitBmp(){
	if (TM_BMP180_Init(&BMP180_Data) == TM_BMP180_Result_Ok ) {
		//TM_USART_Puts(USART1, "\n\rBMP_OK READY TO USE");
	}
	else {
				TM_USART_Puts(USART3, "\n\rBMP_ERROR");
				while(1);
	}
}
void GetBMP(){
	if(TM_DELAY_Time() -timer3>= 200) {
			TM_BMP180_StartTemperature(&BMP180_Data);
			Delay(BMP180_Data.Delay);
			TM_BMP180_ReadTemperature(&BMP180_Data);
			TM_BMP180_StartPressure(&BMP180_Data, TM_BMP180_Oversampling_UltraHighResolution);
			Delay(BMP180_Data.Delay);
		
			/* Read pressure value */
      TM_BMP180_ReadPressure(&BMP180_Data);
			data_altitude=BMP180_Data.Altitude-data_altitude_zero;
			altitude.value = data_altitude;
			timer3=TM_DELAY_Time();
	}
}
void KalibrasiBMP(){
	while(1){
		TM_BMP180_StartTemperature(&BMP180_Data);
		Delay(BMP180_Data.Delay);
		
		//Read Temperature
		TM_BMP180_ReadTemperature(&BMP180_Data);
		// Start Pressure Ultra High Resulution
		TM_BMP180_StartPressure(&BMP180_Data, TM_BMP180_Oversampling_UltraHighResolution);
		Delay(BMP180_Data.Delay);
		//Read Pressure
		TM_BMP180_ReadPressure(&BMP180_Data);
		
		data_altitude_zero = data_altitude_zero + BMP180_Data.Altitude;
		resetKalBMP =resetKalBMP+1;
		sprintf(buffer,"\n\rBISMILLAH KALIBRASI BMP...%d %%", resetKalBMP );
		TM_USART_Puts(USART3, buffer);
		
	if(resetKalBMP == 100){
		data_altitude_zero = data_altitude_zero /100.0;
		TM_USART_Puts(USART3,"\n\rALHAMDULILLAH KALIBRASI BMP BERHASIL...");
		Delayms(1000);
		resetKalBMP=0;
		break;
	}
}
}


// GPS LAT LONG
void ParsedataGPS_GNGLL(void){
				if (flag2==2)
				{
					for(cek2=a2; cek2<sizeof(gps);cek2++)
					{
							if(gps[cek2] == ',')
								{
										nomor_parsing2++;
										ke2 = 0;
										continue;
								}
							else
								{				
									if (nomor_parsing2 == 1)
									{
										lat[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 2)
									{
									  lat_char[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 3)
									{
									  longi[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 4)
									{
										longi_char[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 5)
									{
										time[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 6)
									{
										valid[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 > 6)
									{
										nomor_parsing2 = 0;
										flag2 = 0;
										break;
									}
									ke2++;	
								}
						}
				}
			data_time	=	atoi(time);
			data_lat = atof(lat);
			data_longi = atof(longi);
}
void ParsedataGPS_GNGGA(void){
				if (flag2==2)
				{
					for(cek2=a2; cek2<sizeof(gps);cek2++)
					{
							if(gps[cek2] == ',')
								{
										nomor_parsing2++;
										ke2 = 0;
										continue;
								}
							else
								{				
									if (nomor_parsing2 == 1)
									{
										time[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 2)
									{
									  lat[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 3)
									{
									  lat_char[ke2] = gps[cek2]; 
									}
									else if (nomor_parsing2 == 4)
									{
										longi[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 5)
									{
										longi_char[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 6)
									{
										valid[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 > 6)
									{
										nomor_parsing2 = 0;
										flag2 = 0;
										break;
									}
									ke2++;	
								}
						}
				}
			data_time	=	atoi(time);
			data_lat = atof(lat);
			data_longi = atof(longi);
}
void ParsedataGPS_GNRMC(void){
				if (flag2==2)
				{
					for(cek2=a2; cek2<sizeof(gps);cek2++)
					{
							if(gps[cek2] == ',')
								{
										nomor_parsing2++;
										ke2 = 0;
										continue;
								}
							else
								{				
									if (nomor_parsing2 == 1)
									{
										time[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 2)
									{
									  valid[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 3)
									{
									  lat[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 4)
									{
										lat_char[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 5)
									{
										longi[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 6)
									{
										longi_char[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 > 6)
									{
										nomor_parsing2 = 0;
										flag2 = 0;
										break;
									}
									ke2++;	
								}
						}
				}
			data_time	=	atoi(time);
			data_lat = atof(lat);
			data_longi = atof(longi);
}
void GPS_Validasi(void){
				for (cek2 = 0; cek2 < sizeof(gps); cek2++){
					if(('$' == gps[cek2]) && ('G' == gps[cek2+1]) && ('N' == gps[cek2+2]) && ('G' == gps[cek2+3]) && ('L' == gps[cek2+4]) && ('L' == gps[cek2+5]) && (flag2 == 0))
					{
						if(flag2 == 0) {
							a2 = cek2+6;
							flag2 = 2;
						}
						ParsedataGPS_GNGLL();
					}
					else if (('$' == gps[cek2]) && ('G' == gps[cek2+1]) && ('N' == gps[cek2+2]) && ('G' == gps[cek2+3]) && ('G' == gps[cek2+4]) && ('A' == gps[cek2+5]) && (flag2 == 0))
					{
						if(flag2 == 0) {
							a2 = cek2+6;
							flag2 = 2;
						}
						ParsedataGPS_GNGGA();
					}
					else if(('$' == gps[cek2]) && ('G' == gps[cek2+1]) && ('N' == gps[cek2+2]) && ('R' == gps[cek2+3]) && ('M' == gps[cek2+4]) && ('C' == gps[cek2+5]) && (flag2 == 0))
					{
							if(flag2 == 0) {
							a2 = cek2+6;
							flag2 = 2;
					}
							ParsedataGPS_GNRMC();
				}
			}
}
float dms_dd(float in_coords, char angin){
	float f=in_coords;
	char arah=angin;
//	float per;
	int firstdig=((int)f)/100;
	float nextdig=f-(float)(firstdig*100);
		
	if('W'==arah) {
		float final=(float)((firstdig+(nextdig/60.0))*-1.0);
		return final;
	}
		
	if('N'==arah) {
		float final=(float)((firstdig+(nextdig/60.0))*1.0);
		return final;
	}
	
	if('E'==arah) {
		float final=(float)((firstdig+(nextdig/60.0))*1.0);
		return final;
	}
	
	if('S'==arah) {
		float final=(float)((firstdig+(nextdig/60.0))*-1.0);
		return final;	
	}
// W-,N+,E+,S-
	}

void GPSAccess(void){
			if (TM_USART_Gets(USART3, serial4, sizeof(serial4))) {		// GPS 
			strcpy(gps, serial4);
			GPS_Validasi();
			currentLat1.value1  	= dms_dd(data_lat,lat_char[0]);
			currentLong1.value1  	= dms_dd(data_longi,longi_char[0]);
			currentLat.value 			= currentLat1.value1;
			currentLong.value			= currentLong1.value1;
		}
}
// BateraiMonitor & Airspeed
void BateraiMonitor(){
	rawADC  = TM_ADC_Read(ADC1, ADC_Channel_14);
	baterai.value = ((float)rawADC/4096.0) *16.5;
}
void Airspeed(){
	rawADC1  = TM_ADC_Read(ADC1, ADC_Channel_14);
	voltage  = (float) rawADC / 4095.0 * 3.0; // Voltage at Arduino pin. Range is 5V, 10 bits.
	pressure = (voltage - offset) / sensitivity;  // differential pressure in kPa
	
	if (pressure <0.0) pressure = 0;
	windspeed.value =  sqrt (2.0 * pressure / 1.2 ) * 1.943844;
}
// MODE
// Send GCS
void SendGCS(int waktu){
				if (TM_DELAY_Time() -timer1 >= waktu){
					
					sprintf(buffer,"\r\n R:%.2f P:%.2f Y:%.2f |=====| TZ:%.2f TR:%.2f TY:%.2f TP:%.2f |====| OUT1 %d OUT2 %d OUT3 %d OUT4 %d OUT5 %d OUT6 %d",
					dataRoll, dataPitch, dataYaw, Tz, Tr, Ty, Tp,OutpidP1,OutpidP2,OutpidP3,OutpidP4,OutpidP5,OutpidP6);
					//sprintf(buffer, "\r\nW1 : %.2f | W2 : %.2f | W3 : %.2f | W4 : %.2f | W5 : %.2f | W6 : %.2f", w1,w2,w3,w4,w5,w6);
					//sprintf(buffer,"\r\n %.2f\t%.2f\t%.2f", dataRoll, dataPitch, dataYaw);
					//sprintf(buffer, "\r\n IN1: %.2f IN2: %.2f IN3: %.2f IN4: %.2f IN5 : %.2f IN6 : %.2f",input1,input2,input3,input4,input5,input6);
					TM_USART_Send(USART3, (uint8_t *)buffer, strlen(buffer));
				timer1 = TM_DELAY_Time();
		}
}


//#####################################################################################################################################################################################################################################################//
// MAIN PROGRAM 
int main(void){
	SystemInit(); 		// Initialize system //
	TM_DELAY_Init(); 	// Initialize delay //
	
	// Initialize USART2, 115200 baud, TX: PD5, RX: PD6    		
	TM_USART_Init(USART2, TM_USART_PinsPack_2, 115200); 		//(GY25/RAZOR)
	
	// Initialize USART3    57600 baud, TX: PC10, RX: PC11 
	TM_USART_Init(USART3, TM_USART_PinsPack_1, 57600);			  //TELEMETRY


	interruptInit();  // Initialize Interrupt
	InitPWMIN();   		// Initialize PWM INPUT
	InitPWMOUT(); 	 	// Initialize PWM OUTPUT

	resetIMU();    		// Initialize IMU 
	Delayms(2000);
	// KALIBRASI IMU
	KalibrasiIMU();
	
	// KALIBRASI BRUSHLESS SET NILAI MOTOR 1000
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, 1000);		 //PA5
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_3, 1000);		 //PA1
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_2, 1000); 		 //PA2
	TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_4, 1000);		 //PA3		
	TM_PWM_SetChannelMicros(&TIM9_Data, TM_PWM_Channel_1, 1000); 		 //PE5
	TM_PWM_SetChannelMicros(&TIM12_Data,TM_PWM_Channel_1, 1000); 		 //PB14		
	
	//########>>>>>>>>START<<<<<<<########//
	TM_USART_Puts(USART3,"\n\rBISMILLAH SYSTEM EFRISA HEXACOPTER... ");
	Delayms(3000);
	
	while(1){
			GetPWM();				    // Aktif PWMIN
 	
		//Kirim Ke Ground Control Station
			SendGCS(100);   
	}
}
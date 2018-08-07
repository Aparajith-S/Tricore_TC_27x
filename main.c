/**
 * \file Cpu0_Main.c
 *  Created on: Feb 12, 2018
 *      Author: Aparajith Sridharan
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "Cpu0_Main.h"
#include "SysSe/Bsp/Bsp.h"
#include "IfxScuWdt.h"
#include "IfxPort.h"
#include "PWM_Measure.h"
#include "IfxPort_Io.h"
#include "IfxGtm_Tim_In.h"
#include "IfxStm_Timer.h"
#include <stdio.h>
#include "GtmTomPwmHl.h"
#include "Stmd.h"
#include "IfxStm.h"
#include "glcd.h"


/******************************************************************************/
/*------------------------Inline Function Prototypes--------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/
#define CMU_FREQ 10000000

#define ISR_PROVIDER_TIMER_1MS IfxSrc_Tos_cpu0
#define INTERRUPT_TIMER_1MS    ISR_ASSIGN(ISR_PRIORITY_TIMER_1MS, ISR_PROVIDER_TIMER_1MS)
#define N_DC_G3 2.7368
#define K_V_G3  -2.1853
#define K_E_G3 -0.0003
#define N_DC_G4 3.4701
#define K_V_G4 -2.9180
#define K_E_G4 -0.0003
#define N_DC_G5 5.5481
#define K_V_G5 -5.0157
#define K_E_G5 -0.0003

/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/******************************************************************************/
static Ifx_P *const Port00 = (Ifx_P*)&MODULE_P00;
static Ifx_P *const Port02 = (Ifx_P*)&MODULE_P02;
static Ifx_P *const Port14 = (Ifx_P*)&MODULE_P14;
static Ifx_P *const Port23 = (Ifx_P*)&MODULE_P23;
static Ifx_P *const Port22 = (Ifx_P*)&MODULE_P22;
static Ifx_P *const Port15 = (Ifx_P*)&MODULE_P15;
IfxStm_Timer myTimer;

static const unsigned short textColorTab[] =
{
	COLOR_BLUE,
	COLOR_GREEN,
	COLOR_CYAN,
	COLOR_RED,
	COLOR_MAGENTA,
	COLOR_OLIVE,
	COLOR_PURPLE,
	COL_RGB565(200, 176, 255),
	COL_RGB565(200,  48, 255),
	COL_RGB565(  0, 128, 144),
	COL_RGB565( 32, 104, 168),
};

IFX_INTERRUPT(STM_Int0Handler, 0, ISR_PRIORITY_STM_INT0);

/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/

static volatile unsigned int event=0;
App_Cpu0 g_AppCpu0; /**< \brief CPU 0 global data */
App_Stm g_Stm;/* for Timer interrupt */

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/

void STM_Int0Handler(void)
{
    IfxStm_clearCompareFlag(g_Stm.stmSfr, g_Stm.stmConfig.comparator);
#ifdef SIMULATION
	IfxStm_increaseCompare(g_Stm.stmSfr, g_Stm.stmConfig.comparator, 1000);
#else
	IfxStm_increaseCompare(g_Stm.stmSfr, g_Stm.stmConfig.comparator, TimeConst_1ms);
#endif
    IfxCpu_enableInterrupts();
    event=1;
}


void IfxStm_init(void)
{

    /* disable interrupts */
    boolean interruptState = IfxCpu_disableInterrupts();

    initTime();
    g_Stm.stmSfr = &MODULE_STM0;
    IfxStm_initCompareConfig(&g_Stm.stmConfig);

    g_Stm.stmConfig.triggerPriority = ISR_PRIORITY_STM_INT0;
    g_Stm.stmConfig.typeOfService   = IfxSrc_Tos_cpu0;
#ifdef SIMULATION
    g_SrcSwInt.stmConfig.ticks      = 1000;
#else
    g_Stm.stmConfig.ticks           = TimeConst_1ms;
#endif
    IfxStm_initCompare(g_Stm.stmSfr, &g_Stm.stmConfig);

    /* enable interrupts again */
    IfxCpu_restoreInterrupts(interruptState);
}


void Delay(unsigned int millis)
{
 do{ event=0;
	  while(!event)
	  ;
	  millis--;
 }while(millis);
}

void boot_up()
{
	/*
	     * !!WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE!!
	     * Enable the watchdog if it is required and also service the watchdog periodically
	     * */
	    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
	    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

	    /* Initialise the application state */
	    g_AppCpu0.info.pllFreq = IfxScuCcu_getPllFrequency();
	    g_AppCpu0.info.cpuFreq = IfxScuCcu_getCpuFrequency(IfxCpu_getCoreIndex());
	    g_AppCpu0.info.sysFreq = IfxScuCcu_getSpbFrequency();
	    g_AppCpu0.info.stmFreq = IfxStm_getFrequency(&MODULE_STM0);

	    /* Enable the global interrupts of this CPU */
	    IfxCpu_enableInterrupts();

}


float32 cruise(uint32 gear,float32 V,float32 ref_V, float32 E_T)
{float32 throttle;
 switch(gear)
 {case 3:
	     throttle = ((ref_V-16.667)*N_DC_G3 + (V-16.667)*K_V_G3+(E_T-16.77)*K_E_G3)+3.95;
	     break;
  case 4:
	     throttle = ((ref_V-22.22223)*N_DC_G4 + (V-22.22223)*K_V_G4+(E_T-35.1735)*K_E_G4)+5.09;
	     break;
  case 5:throttle = ((ref_V-33.3333)*N_DC_G5 + (V-33.3333)*K_V_G5+(E_T-112.74)*K_E_G5)+9.42;
	     break;
 default:throttle=0;
 }
 (throttle>100)?throttle=100:0;
 (throttle<0)?throttle=0:0;
 return throttle;
}

float32 timegap(uint32 gear,float32 V,float32 del_R, float32 del_V)
{ float32 throttle,acc;
  acc = del_R*(0.2546) + 2.5479*del_V;
  switch(gear)
   {case 3:
  	     throttle = ((V-16.667)*0.5508+(acc)*11.8991)+3.95;
  	     break;
    case 4:throttle = ((V-22.2222)*0.5514+(acc)*15.7734)+5.09;
  	     break;
    case 5:throttle = ((V-33.3333)*0.5305 + (acc)*23.1176)+9.42;
  	     break;
   default:throttle=0;
   }
  (throttle>100)?throttle=100:0;
  (throttle<0)?throttle=0:0;
  return throttle;
}

/*****************************************************************************************
 * IO pins Used:
 *  INPUTS:
 *    P00.9  		-	Vehicle velocity PWM signal
 *    P00.11		-   Relative Velocity PWM signal
 *    P00.12		-   Vehicle Engine Torque PWM signal
 *    P14.6			-   relative Distance PWM Signal
 * 	  P22.0			-	Target Detected [0/1]
 * 	  P22.1			-   Gear_lever position bit 2
 * 	  P22.2			-   Gear_lever position bit 1
 * 	  P22.3			-	Gear_lever position bit 0
 *	  P15.2         -   ACC disable flag
 * 	OUTPUTS:
 *	  P20.x         -   port LCD QSPI interface
 *	  P23.0			- 	ACC_Enable
 *	  P00.0			-	Throttle[%] PWM OUT
 *****************************************************************************************/


int core0_main(void)
{  																/* No code shall precede this comment in the main section*/
	boot_up();
																				/*code starts after this line*/
	 /*Declare pointer for the module location.*/

	Ifx_GTM *G = &MODULE_GTM;

    /*Declare variables for the drivers, config.*/

	IfxGtm_Tim_In driver,driver1,driver2,driver3,driver4;
	IfxGtm_Tim_In_Config config,config1,config2,config3,config4;


    /* local Variables*/
    	/*Inputs*/
    	float32 velocity=0,distance=0,rel_velocity=0,Engine_torque=0,RSU_velocity=0,del_R;
        uint32 x_axis,y_axis;
        uint32 gear_lever=0;
        uint32 gear_bit0,gear_bit1,gear_bit2;
        boolean target_detected;
        /*Outputs*/
        float32 Throttle_percent=0;
    	boolean ACC_enable=FALSE,ACC_Disable;
        float32 set_speed=60;
        float32 ref_speed;
    	/*internal variables*/
        boolean coherent,ACC_flag=FALSE,set_switch=FALSE;
        boolean Vcontrol=FALSE,Dcontrol=FALSE ;
        char disp[30];
        float32 R_Desired=0.0,R_Threshold=0.0,R_0 = 5.0,V_leader=0.0;


      												/*GTM TIM TIMER INITIALIZATION AND TOM TIMER INITIALIZATION */

   /* Initialize the input Timers for PWM reading*/
    boolean interruptState = IfxCpu_disableInterrupts();

    IfxGtm_Tim_Config(&config,G,&IfxGtm_TIM0_0_TIN18_P00_9_IN,IfxGtm_Tim_Ch_0);
    IfxGtm_Tim_Config(&config1,G,&IfxGtm_TIM0_1_TIN86_P14_6_IN,IfxGtm_Tim_Ch_1);
    IfxGtm_Tim_Config(&config2,G,&IfxGtm_TIM0_2_TIN20_P00_11_IN,IfxGtm_Tim_Ch_2);
    IfxGtm_Tim_Config(&config3,G,&IfxGtm_TIM0_3_TIN21_P00_12_IN,IfxGtm_Tim_Ch_3);
    IfxGtm_Tim_Config(&config4,G,&IfxGtm_TIM0_4_TIN4_P02_4_IN,IfxGtm_Tim_Ch_4);
    /*Enable the GTM Module*/
    IfxGtm_enable(G);
    IfxGtm_Tim_In_initialize(&driver,&config);
    IfxGtm_Tim_In_initialize(&driver1,&config1);
	IfxGtm_Tim_In_initialize(&driver2,&config2);
	IfxGtm_Tim_In_initialize(&driver3,&config3);
	IfxGtm_Tim_In_initialize(&driver4,&config4);

   /* Enable the clock for the timers*/
    IfxGtm_Cmu_enableClocks(G, IFXGTM_CMU_CLKDIS_ALL);
    IfxGtm_Cmu_setGclkFrequency(G,CMU_FREQ);
    IfxGtm_Cmu_setClkFrequency(G,IfxGtm_Cmu_Clk_0,CMU_FREQ);
   /*Enable interrupts again*/
    IfxCpu_restoreInterrupts(interruptState);

    GtmTomPwmHl_init();
   /*STM 1ms timer for delays*/
    IfxStm_init();
    /*PORT pin initialization*/
    /* configure port pins as PWM in pins */
    Port00->IOCR8.B.PC9=0;
    Port00->IOCR8.B.PC11=0;
    Port00->IOCR12.B.PC12=0;
    Port14->IOCR4.B.PC6=0;
    Port02->IOCR4.B.PC4=0;
    /*  GPIOs */
    Port15->IOCR0.B.PC2=0;
    Port22->IOCR0.B.PC0=0;
    Port22->IOCR0.B.PC1=0;
    Port22->IOCR0.B.PC2=0;
	Port22->IOCR0.B.PC3=0;
	Port23->IOCR0.B.PC0=0b10000;



	/*LCD initialization*/
	GLCD_init();
    GLCD_clear(COLOR_WHITE);
  	GLCD_setBackColor(COLOR_BLUE);
  	GLCD_setTextColor(COLOR_WHITE);
  	GLCD_displayStringLn(LINE0, "ACC ECU DEMO Ver 1.0");
  	GLCD_setBackColor(COLOR_RED);
  	GLCD_displayStringLn(LINE2, "        ACC OFF     ");
   	GLCD_setBackColor(COLOR_WHITE);
   	GLCD_setTextColor(COLOR_BLACK);
   	GLCD_displayStringLn(LINE5, "    -    Set    +   ");

   	/* background endless loop */
  while (TRUE)
   {

    /*Read Sensor data from SCALEXIO*/

	IfxGtm_Tim_In_update(&driver);
    IfxGtm_Tim_In_update(&driver1);
    IfxGtm_Tim_In_update(&driver2);
    IfxGtm_Tim_In_update(&driver3);
    IfxGtm_Tim_In_update(&driver4);
    /*velocity calculation from Duty Cycle*/
    velocity=IfxGtm_Tim_In_getDutyCycle(&driver,&coherent);
	velocity=velocity*240.0*5.0/18.0 ;
	/*distance calculation from Duty Cycle*/
	distance=IfxGtm_Tim_In_getDutyCycle(&driver1,&coherent);
    distance=distance*130;
    /*relative velocity calculation from Duty Cycle*/
    rel_velocity=IfxGtm_Tim_In_getDutyCycle(&driver2,&coherent);
	rel_velocity=(rel_velocity*40)-20;
    /*Engine Torque calculation from Duty Cycle*/
	Engine_torque=IfxGtm_Tim_In_getDutyCycle(&driver3,&coherent);
	Engine_torque=(Engine_torque*420)-70;
	/*RSU velocity*/
	RSU_velocity=IfxGtm_Tim_In_getDutyCycle(&driver4,&coherent);
	RSU_velocity=RSU_velocity*240.0/3.6;
	/*Get Gear data from SCALEXIO*/
	gear_bit0= Port22->IN.B.P3;
	gear_bit1= Port22->IN.B.P2;
	gear_bit2= Port22->IN.B.P1;
	target_detected = Port22->IN.B.P0;
    ACC_Disable = Port15->IN.B.P2;
	/*End of acquisition of data from SCALEXIO*/
	/*reading complete*/

	gear_lever = (gear_bit2 << 2)|(gear_bit1<<1)|gear_bit0;

	R_Desired = 1.5*velocity + R_0;
	/*R Rdot equation*/
	R_Threshold = R_Desired+rel_velocity*((R_Desired-130.0)/10.0);
	/*Leader velocity calculation */
	V_leader = rel_velocity+velocity;
    del_R= distance-R_Desired;

    /*Acquire touch data from user*/
	GLCD_GetTouchXY(&x_axis,&y_axis);
	GLCD_setBackColor(COLOR_WHITE);
	GLCD_setTextColor(COLOR_BLACK);
	GLCD_displayStringLn(LINE7,disp);
	sprintf(disp,"SetSpeed:%3.2f km/h  ",set_speed);

	/* read touch */
	if(y_axis>2600 && y_axis<3200 && gear_lever>=3 && !ACC_Disable)
    { if(FALSE==ACC_flag)
       {
    	  ACC_flag=TRUE;
    	  ACC_enable =!ACC_enable;
        }
	}
	else
		ACC_flag=FALSE;

	if(y_axis>1350 && y_axis<2250 && x_axis>550 && x_axis<1350)
	{ if(FALSE==set_switch&&set_speed>60)
			{set_switch=TRUE;
			 set_speed = set_speed - 5;
			}
	}
	else if(y_axis>1350 && y_axis<2250 && x_axis>2700 && x_axis<3600)
	{ if(FALSE==set_switch&&set_speed<160)
			{set_switch=TRUE;
			 set_speed = set_speed + 5;
			}
	}
	else
	{set_switch=FALSE;
	}

	/*convert from km|h to m|s */

    /*RSU reference*/
	if(RSU_velocity>16.6&&RSU_velocity<=set_speed/3.6)
    ref_speed=RSU_velocity;
	else
	ref_speed=set_speed/3.6;
    /* RSU reference ends*/

	if (ACC_enable&&gear_lever>=3&&!ACC_Disable)
	{	GLCD_setBackColor(COLOR_GREEN);
		GLCD_setTextColor(COLOR_BLACK);
		GLCD_displayStringLn(LINE2, "        ACC ON      ");
		Port23->OMR.B.PCL0=0;
		Port23->OMR.B.PS0=1;

	}
	else
	{	GLCD_setBackColor(COLOR_RED);
		GLCD_setTextColor(COLOR_BLACK);
		GLCD_displayStringLn(LINE2, "        ACC OFF     ");
		Port23->OMR.B.PCL0=1;
		Port23->OMR.B.PS0=0;
	    ACC_enable=FALSE;
	}

	/* Controller code starts here */

	/* controller switching */
	if(!ACC_enable)
	{
		Vcontrol=FALSE;
		Dcontrol=FALSE;
	}
	else
	if(distance<=R_Threshold && target_detected && V_leader<ref_speed)
    {
    	Vcontrol=FALSE;
		Dcontrol=TRUE;
    }
    else if((distance)>(R_Threshold+30.0) || !target_detected || V_leader>=ref_speed)
    {
    	Vcontrol=TRUE;
		Dcontrol=FALSE;
    }

/*compute throttle position using the respective controller*/

 if(Vcontrol)
	{
	 Throttle_percent = cruise(gear_lever,velocity,ref_speed,Engine_torque);
	 GLCD_displayStringLn(LINE8,"   Cruise Control  ");
	}
 else if(Dcontrol)
	{
	 Throttle_percent = timegap(gear_lever,velocity,del_R,rel_velocity);
	 GLCD_displayStringLn(LINE8,"   Spacing Control  ");
	}

 /*send actuator data through PWM to SCALEXIO*/

    GtmTomPwmHl_run(Throttle_percent);

	REGRESSION_RUN_STOP_PASS;
    }
    return 0;
}




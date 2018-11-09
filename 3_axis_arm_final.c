#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"

//*****************************************************************************
//
//!
//! Design for 3-Axis Robot Arm using 4 micro servos
//! Using PWM, 4 servos will control arms location and grabbing motion
//! Potentiometer will control servo angular position
//
//*****************************************************************************

#define PWM_FREQUENCY 50 //50Hz Base Frequency to Control Servo
#define Scale 81 				 //81 to scale average data to 0-50 range

volatile uint32_t ui32Load; //Variable for Load Register
volatile uint8_t ui8Adjust = 75; //Allow to adjust position of servo. Set center postion at 1.5ms

volatile uint8_t servo_choice = 0; //Variable to pick servo

uint32_t ui32ADC0Value[4]; //Variable holds raw sampled data
volatile uint32_t ui32POT; //Variable holds average of sampled data

volatile uint8_t last1 = 75; //Holds last position servo 1
volatile uint8_t last2 = 75; //Holds last position servo 2
volatile uint8_t last3 = 75; //Holds last position servo 3
volatile uint8_t last4 = 75; //Holds last position servo 4
volatile bool oktogo = false; //Determines if adjust variable = last variable

//Initizliaes GPIO Ports for functions
void
PortFunctionInit(void)
{
	//Clock Inititalization
	ROM_SysCtlClockSet( SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //Sets CPU clock to 16Mhz
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); //Enables PWM1
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); //Enables PWM0
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); //Enables ADC0
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //Enables clock for GPIOA
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //Enables clock for GPIOB
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); //Enables Clock for GPIOD 
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //Enables clock for GPIOE
	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64); //Enables PWM Clock
	SysCtlDelay(5);	//Few Cycles for clock to be fully enabled
	
	//GPIOA configuration for servo select control
	ROM_GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_2  | GPIO_PIN_3 | GPIO_PIN_4  | GPIO_PIN_7, GPIO_DIR_MODE_IN); //Configure GPIOA for digital switch function
	ROM_GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4  | GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); //Enable pull-up resistor GPIOB

	//GPIOB configuration for LED control
	ROM_GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_DIR_MODE_OUT); //Configure PB6 for digital out
}

//Initializes PWM for servo control
void
PWMFunctionInit(volatile uint32_t* ui32PointLoad, volatile uint8_t* ui8PointAdjust)
{
	volatile uint32_t ui32PWMClock; //PWM clock
	
	ROM_GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1); //Configure PD0 and PD1 as PWM output pin for module 1 (Servo 1, 2)
	ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5); //Configure PB4 and PB5 as PWM output pin for module 0 (Servo 3, 4)

	ROM_GPIOPinConfigure(GPIO_PD0_M1PWM0); // Set PWM module 1, output pin 0 for PD0
	ROM_GPIOPinConfigure(GPIO_PD1_M1PWM1); // Set PWM module 1, output pin 1 for PD1
	ROM_GPIOPinConfigure(GPIO_PB4_M0PWM2); // Set PWM module 0, output pin 2 for PB4
	ROM_GPIOPinConfigure(GPIO_PB5_M0PWM3); // Set PWM module 0, output pin 3 for PB5
	
	ui32PWMClock = SysCtlClockGet() / 64; //PWM Clock
	*ui32PointLoad = (ui32PWMClock / PWM_FREQUENCY) -1 ; //Divide PWM Clock by 50Hz to determine count to load into Load Register
	
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN); // Conifugre module 1 PWM generator 0 as a down-counter
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, *ui32PointLoad); //Load count value 
	
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN); // Conifugre module 0 PWM generator 1 as a down-counter
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, *ui32PointLoad); //Load count value 
	
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, *ui8PointAdjust * *ui32PointLoad / 1000); //Sets pulse-width for module 1, output pin 0
	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);//PWM module 1, output pin 0 enabled as output
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, *ui8PointAdjust * *ui32PointLoad/ 1000); //Sets pulse-width for module 1, output pin 1
	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);//PWM module 1, output pin 1 enabled as output
	
	ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, *ui8PointAdjust * *ui32PointLoad / 1000); //Sets pulse-width for module 0, output pin 2
	ROM_PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);//PWM module 0, output pin 2 enabled as output
	ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, *ui8PointAdjust * *ui32PointLoad/ 1000); //Sets pulse-width for module 0, output pin 3
	ROM_PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);//PWM module 0, output pin 3 enabled as output
	
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_0); // Enable to have module 1, generator 0 to run
	ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_1); // Enable to have module 0, generator 1 to run
}

void
ADC0Init(void)
{
	ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); //Enable PE0 for ADC function
	
	ADCSequenceDisable(ADC0_BASE, 1); //disable ADC0 before the configuration is complete
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0); // will use ADC0, SS1, processor-trigger, priority 0
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH3); //ADC0 SS1 Step 0, channel 3
	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH3); //ADC0 SS1 Step 1, channel 3
	ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH3); //ADC0 SS1 Step 2, channel 3
	ADCSequenceStepConfigure(ADC0_BASE,1,3,ADC_CTL_CH3|ADC_CTL_IE|ADC_CTL_END); 
	
	ADCSequenceEnable(ADC0_BASE, 1); //enable ADC0
}

void
SwitchInterruptInit(void)
{
	IntPrioritySet(INT_ADC0SS1, 0x01);  	 // configure ADC0 SS1 interrupt priority as 1
	IntEnable(INT_ADC0SS1);    				// enable interrupt 31 in NVIC (ADC0 SS1)
	ADCIntEnableEx(ADC0_BASE, ADC_INT_SS1);      // arm interrupt of ADC0 SS1
	
	IntEnable(INT_GPIOA);
	IntPrioritySet(INT_GPIOA, 0x00); //Priority 0
	GPIO_PORTA_IM_R |= 0x9C;   		// arm interrupt on PA2, PA3, PA4, PA7
	GPIO_PORTA_IS_R &= ~0x9C;     // PA2, PA3, PA4, PA7 are edge-sensitive
  	GPIO_PORTA_IBE_R &= ~0x9C;   	// PA2, PA3, PA4, PA7 not both edges trigger 
  	GPIO_PORTA_IEV_R &= ~0x9C;  	// PA2, PA3, PA4, PA7 falling edge event
	IntMasterEnable();        		// globally enable interrupt
}

void ADC0_Handler(void)
{
	ADCIntClear(ADC0_BASE, 1); //Clears interrupt flag
	ADCProcessorTrigger(ADC0_BASE, 1); //Initiates processor trigger to begin sampling
	ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value); //Samples data from PE0
	ui32POT = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3])/4; //Average of raw data

	ui8Adjust =  ui32POT/Scale + 50; //Sets adjust variable to scale from 50 - 100
}

//Interrupt to control servo selection
void
GPIOPortA_Handler(void)
{
	if (ROM_GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) == 0x00) //Servo 1 select
	{
		GPIO_PORTA_ICR_R |= 0x04; //Clear interrupt flag
		servo_choice = 1;
	}
	else if (ROM_GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3) == 0x00) //Servo 2 select
	{
		GPIO_PORTA_ICR_R |= 0x08; //Clear interrupt flag
		servo_choice = 2;
	}
	else if (ROM_GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4) == 0x00) //Servo 3 select
	{
		GPIO_PORTA_ICR_R |= 0x10; //Clear interrupt flag
		servo_choice = 3;
	}
	else if (ROM_GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7) == 0x00) //Servo 4 select
	{
		GPIO_PORTA_ICR_R |= 0x80; //Clear interrupt flag
		servo_choice = 4;
	}
	oktogo = false; //Adjust variable is not equal to previous value
}

int main(void)
{
	//Initalize Ports, Interrupts, and PWM
	PortFunctionInit(); //Intialize GPIO ports
	PWMFunctionInit(&ui32Load, &ui8Adjust); //Initialize PWM
	SwitchInterruptInit(); //Interupt initialization
	ADC0Init(); //ADC0 initialization
	ADCProcessorTrigger(ADC0_BASE, 1); //Processor trigger to intitiate ADC
	
	while(1)
	{
		if(ui8Adjust < 50) //Check if adjust variable reach lower limit (1 ms)
			ui8Adjust = 50; //Sets to lower limit 
		if(ui8Adjust > 100) //Check if adjust variable reach lower limit (1 ms)
			ui8Adjust = 100; //Sets to lower limit 
					
		if (servo_choice == 1) //Arm X-Axis
		{
			if (last1 ==ui8Adjust) //Check if potentiometer is at previous position
				oktogo = true; 
			if (oktogo == true) //Sets pulse width based on potentiometer position
			{
				ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000); //Load PWM pulse width register with new vlaue
				last1 = ui8Adjust;
			}
		}
		else if (servo_choice == 2) //Arm bottom Y_Axis
		{
			if (last2 == ui8Adjust)
				oktogo =true;
			if (oktogo == true)
			{
				ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui8Adjust * ui32Load / 1000); //Load PWM pulse width register with new vlaue
				last2 = ui8Adjust;
			}
		}
		else if (servo_choice == 3) //Arm top Y_Axis
		{
			if(last3 == ui8Adjust)
				oktogo = true;
			if (oktogo == true)
			{
				ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ui8Adjust * ui32Load / 1000); //Load PWM pulse width register with new vlaue
				last3 = ui8Adjust;
			}
		}
		else if (servo_choice == 4) //Clamp
		{
			if(last4 == ui8Adjust)
				oktogo = true;
			if (oktogo==true)
			{
				ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, ui8Adjust * ui32Load / 1000); //Load PWM pulse width register with new vlaue
				last4 = ui8Adjust;
			}
		}
		ROM_SysCtlDelay(30000); //Delay for a bit (Determines speed of loop) *Change if servo moves to fast or slow)
	}
}

/*

Project 2 Application
Created By: Gene Hu & Hector Soto
Description: The application for Project 2 for ECE 544.

*/

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "nexys4IO.h"
#include <stdlib.h>
#include "platform.h"
#include "microblaze_sleep.h"
#include "xiic.h"

/* BSP includes. */
#include "xtmrctr.h"
#include "xgpio.h"
#include "sleep.h"

//Project Includes
#include "Utility.h"
#include "tsl2561.h"

/*Definitions for NEXYS4IO Peripheral*/
#define N4IO_DEVICE_ID		    XPAR_NEXYS4IO_0_DEVICE_ID
#define N4IO_BASEADDR		    XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define N4IO_HIGHADDR		    XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

#define BTN_CHANNEL		1
#define SW_CHANNEL		2

#define mainQUEUE_LENGTH					( 1 )

/* A block time of 0 simply means, "don't block". */
#define mainDONT_BLOCK						( portTickType ) 0

//---| Project Defines |---
#define MAX_LUX 999
#define MIN_LUX 0
#define MAX_DUTY_CYCLE 1023
#define MIN_DUTY_CYCLE 0
#define DCINCR			5		// duty cycle increments, about 15 steps for each duty cycle
#define MAXBRIGHTNESS	255		// max brightness/duty cycle of rgb2 (based on self test where 99% = 255)
//These are defines for the PID controller task's control message.
#define PID_CTRL_PC_ENABLE		1 //Enable Proportional Control
#define PID_CTRL_IC_ENABLE		2 //Enable Integral Control
#define PID_CTRL_DC_ENABLE		4 //Enable Derivative Control
#define PID_CTRL_SET_UPDATE		8 //Update the setpoint.
#define PID_CTRL_SET_INCR_01	16 //Increment the setpoint by 1.
#define PID_CTRL_SET_INCR_05	32 //Increment the setpoint by 5.
#define PID_CTRL_SET_INCR_10	64 //Increment the setpoint by 10.
#define PID_CTRL_SET_DECREMENT	128 //Decrement instead of increment.
#define PID_CTRL_SET_INCREMENT	256 //Increment instead of decrement.
#define PID_CTRL_KP_UPDATE		512 //Update the Kp value.
#define PID_CTRL_KI_UPDATE		1024 //Update the Ki value.
#define PID_CTRL_KD_UPDATE		2048 //Update the Kd value.

//Button defines.
#define BUTTON_BTNU 8
#define BUTTON_BTND 4

//Create Instances
static XGpio xInputGPIOInstance;

static XIic TSL2561_Sensor;

//Function Declarations
static void prvSetupHardware( void );


//Declare input semaphore
xSemaphoreHandle Semaphore_Input;
xSemaphoreHandle Semaphore_Display;
xSemaphoreHandle Semaphore_PID;


static xQueueHandle Queue_PIT_DT = NULL; //ParseInputTask->DisplayTask Queue
static xQueueHandle Queue_PIT_PIDT = NULL; //ParseInputTask->PIDTask Queue
static xQueueHandle Queue_PIDT_DT = NULL; //PIDTask->DisplayTask Queue

//Global Variables.


//ISR, to handle interrupt of GPIO btns
//Give a Semaphore
static void InputControllHandler(void *pvUnused)
{
	xSemaphoreGiveFromISR(Semaphore_Input,NULL);
	XGpio_InterruptClear( &xInputGPIOInstance, XGPIO_IR_MASK );

}

//A task which takes the Interrupt Semaphore and sends a queue to task 2.
void ParseInputTask(void *p)
{

	uint16_t MessageBitMask = 0;
	volatile uint16_t Switches = 0;	//Value of switches.
	volatile uint8_t Buttons = 0;	//Value of buttons.
	uint8_t LastButtons = 0; //State of buttons in last function call.
	uint8_t PushedButtons = 0; //Buttons that were pressed and released.
	while(1)
	{
		if(xSemaphoreTake(Semaphore_Input, 1))
		{
			usleep(25000); //Wait 25ms to ensure button/switch states are stable.
			MessageBitMask = 0;
			Switches = NX4IO_getSwitches();
			LastButtons = Buttons;
			Buttons = NX4IO_getBtns();
			PushedButtons = (LastButtons ^ Buttons) & (~Buttons); //Buttons that were pressed and released.

			//---| Interpret Switches |---
			MessageBitMask |= Switches & 0x0F; //Get first 4 control signals.
			switch((Switches & 0x30) >> 4) //Get increment value control by bits 5:4.
			{
				case 0: //Increment by 1.
					MessageBitMask |= PID_CTRL_SET_INCR_01;
					break;
				case 1: //Increment by 5.
					MessageBitMask |= PID_CTRL_SET_INCR_05;
					break;
				case 2: //Increment by 10.
				case 3:
					MessageBitMask |= PID_CTRL_SET_INCR_10;
					break;
			}
			switch((Switches & 0x30) >> 6) //Get which constant to change.
			{
				case 1: //Update Kp.
					MessageBitMask |= PID_CTRL_KP_UPDATE;
					break;
				case 2: //Update Ki.
					MessageBitMask |= PID_CTRL_KI_UPDATE;
					break;
				case 3: //Update Kd.
					MessageBitMask |= PID_CTRL_KD_UPDATE;
					break;
			}

			//---| Interpret Buttons |---
			if(PushedButtons & BUTTON_BTNU)
			{
				MessageBitMask |= PID_CTRL_SET_INCREMENT;
			}
			else if(PushedButtons & BUTTON_BTND)
			{
				MessageBitMask |= PID_CTRL_SET_DECREMENT;
			}
			xQueueSend(Queue_PIT_DT, &MessageBitMask, mainDONT_BLOCK); //Send message to Display Task.
			xQueueSend(Queue_PIT_PIDT, &MessageBitMask, mainDONT_BLOCK ); //Send message to PID Task.
			xSemaphoreGive(Semaphore_PID);
		}
	}
}

void DisplayTask(void *p)
{

	uint32_t MessageBitMask;
	uint32_t MessageSetpointLux;
	long ReceivedMessage = pdFALSE;

	while(1)
	{
		//Update 7-segment display with setpoint and lux values.
		ReceivedMessage = xQueueReceive(Queue_PIDT_DT, &MessageSetpointLux, pdMS_TO_TICKS(50)); //Get message from PIDTask.
		Update7Display(MessageSetpointLux & 0x0FFFF, (MessageSetpointLux >> 16) & 0x0FFFF);
		if(xSemaphoreTake(Semaphore_Display, 1))
		{
			MessageBitMask = 0;
			//ReceivedMessage = xQueueReceive(Queue_PIT_DT, &MessageBitMask, portMAX_DELAY); //Get message from Parse Input Task.
			ReceivedMessage = xQueueReceive(Queue_PIT_DT, &MessageBitMask, pdMS_TO_TICKS(25)); //Get message from Parse Input Task.
			if(ReceivedMessage == pdFALSE){continue;} //Skip rest of loop.
			//Write to LED.
			NX4IO_setLEDs(MessageBitMask);
		}
	}
}

uint32_t PIDT_Increment(uint32_t MessageBitMask)
{
	return 0 - (((MessageBitMask & PID_CTRL_SET_DECREMENT) != 0)) *
			(((MessageBitMask & PID_CTRL_SET_INCR_01) != 0) +
			5*((MessageBitMask & PID_CTRL_SET_INCR_05) != 0) +
			10*((MessageBitMask & PID_CTRL_SET_INCR_10) != 0))
			+ (((MessageBitMask & PID_CTRL_SET_INCREMENT) != 0)) *
			(((MessageBitMask & PID_CTRL_SET_INCR_01) != 0) +
			5*((MessageBitMask & PID_CTRL_SET_INCR_05) != 0) +
			10*((MessageBitMask & PID_CTRL_SET_INCR_10) != 0));
}

void PIDTask(void *p)
{
	uint32_t MessageBitMask = 0;
	uint32_t Setpoint = 500; //Start setpoint at 500.
	uint32_t Lux = 0; //Lux read from sensor.
	float LuxF = 0; //Float lux value.
	float Kp = 5;
	float Ki = 5;
	float Kd = 5;
	static pid_t pid;
	float pid_error; //How far from the setpoint we are.
	float drive;
	long ReceivedMessage = pdFALSE;
	static bool isInitialized = false;
	static uint16_t pwm_led;
	uint16_t brightness;
	while(1)
	{
		//GetLux
		LuxF = tsl2561_calculateLux(tsl2561_readChannel(&TSL2561_Sensor, 0),
				tsl2561_readChannel(&TSL2561_Sensor, 1));
		Lux = (uint32_t) LuxF;
		MessageBitMask = Setpoint | (Lux << 16); //Send setpoint and lux data.
		xQueueSend(Queue_PIDT_DT, &MessageBitMask, mainDONT_BLOCK); //Send message to Display task.
		xSemaphoreGive(Semaphore_Display);
		if(xSemaphoreTake(Semaphore_PID, 1))
		{
			ReceivedMessage = xQueueReceive(Queue_PIT_PIDT, &MessageBitMask, pdMS_TO_TICKS(50)); //Get message from Parse Input task.
			if(ReceivedMessage == pdFALSE){continue;} //Skip rest of loop.
			//If we pushed an increment or decrement button.
			if((MessageBitMask & PID_CTRL_SET_DECREMENT) || (MessageBitMask & PID_CTRL_SET_INCREMENT))
			{
				if(MessageBitMask & PID_CTRL_SET_UPDATE)
				{
					Setpoint += PIDT_Increment(MessageBitMask);
				}
				else if(MessageBitMask & PID_CTRL_KP_UPDATE)
				{
					Kp += PIDT_Increment(MessageBitMask);
				}
				else if(MessageBitMask & PID_CTRL_KI_UPDATE)
				{
					Ki += PIDT_Increment(MessageBitMask);
				}
				else if(MessageBitMask & PID_CTRL_KD_UPDATE)
				{
					Kd += PIDT_Increment(MessageBitMask);
				}
			}
		}
		//Update PID stuff.
		if (!isInitialized) {
			pid.integratMax = MAX_LUX;
			pid.integratMin = MIN_LUX;
			pwm_led = MAX_DUTY_CYCLE / 2;
			brightness = 0;
			isInitialized = true;
		}
		pid.propGain = Kp;
		pid.integratGain = Ki;
		pid.derGain = Kd;
		/* calculate error */
		pid_error = Setpoint - LuxF;
		drive = updatePID(&pid, pid_error, LuxF, MessageBitMask & PID_CTRL_PC_ENABLE,
											MessageBitMask & PID_CTRL_IC_ENABLE,
											MessageBitMask & PID_CTRL_DC_ENABLE);
		if(drive)
		{
			xil_printf("Drive: %i\r\n", (int) drive);
		}
		/* Adjust PWM and then drive PWM signal to LED */
		// if error is positive, means luxReading needs to increase -> increase PWM/brightness
		if (pid_error > 0) {
			pwm_led = ((pwm_led + DCINCR) <= MAX_DUTY_CYCLE) ? pwm_led + DCINCR : MAX_DUTY_CYCLE;
		}
		// if error is negative, means luxReading needs to decrease -> decrease PWM/brightness
		else if (pid_error < 0) {
			pwm_led = ((pwm_led - DCINCR) >= MIN_DUTY_CYCLE) ? pwm_led - DCINCR : MIN_DUTY_CYCLE;
		}
		// Convert to brightness and write to LED (brightness will be 0-255)
		brightness = (((float)pwm_led) / ((float)MAX_DUTY_CYCLE)) * MAXBRIGHTNESS;
		xil_printf("pwm_led | brightness: %i | %i\r\n", pwm_led, (int) brightness);
		NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, brightness);
	}
}

int main(void)
{
	// Announcement
	xil_printf("---| ECE 544 Project 1 |---\r\n");

	//Initialize the HW
	prvSetupHardware();

	//Initialize the sensor.
	tsl2561_init(&TSL2561_Sensor);

	//Create Semaphore
	vSemaphoreCreateBinary(Semaphore_Input);
	vSemaphoreCreateBinary(Semaphore_Display);
	vSemaphoreCreateBinary(Semaphore_PID);

	/* Create the queue */
	Queue_PIT_DT = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint16_t));
	Queue_PIT_PIDT = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint16_t));
	Queue_PIDT_DT = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));

	/* Sanity check that the queue was created. */
	configASSERT(Queue_PIT_DT);

	//Create ParseInputTask
	xTaskCreate(ParseInputTask,
				 ( const char * ) "ParseT",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 2,
				 NULL );

	//Create DisplayTask
	xTaskCreate(DisplayTask,
				"DisplayT",
				configMINIMAL_STACK_SIZE,
				NULL,
				3,
				NULL );

	//Create PIDTask
	xTaskCreate(PIDTask,
					"PIDT",
					configMINIMAL_STACK_SIZE,
					NULL,
					1,
					NULL );

	vTaskStartScheduler();

	return -1;
}


static void prvSetupHardware( void )
{
	uint32_t xStatus;

	const unsigned char ucSetToInput = 0xFFU;

	/* Initialize the GPIO for the button inputs. */
		xStatus = XGpio_Initialize( &xInputGPIOInstance, XPAR_AXI_GPIO_1_DEVICE_ID );

		if( xStatus == XST_SUCCESS )
		{
		/* Install the handler defined in this task for the button input.
		*NOTE* The FreeRTOS defined xPortInstallInterruptHandler() API function
		must be used for this purpose. */
		xStatus = xPortInstallInterruptHandler( XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR, InputControllHandler, NULL );

		if( xStatus == pdPASS )
		{
			/* Set switches and buttons to input. */
			XGpio_SetDataDirection( &xInputGPIOInstance, BTN_CHANNEL, ucSetToInput );
			XGpio_SetDataDirection( &xInputGPIOInstance, SW_CHANNEL, ucSetToInput );

			/* Enable the button input interrupts in the interrupt controller.
			*NOTE* The vPortEnableInterrupt() API function must be used for this
			purpose. */

			vPortEnableInterrupt( XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR );

			//Enable GPIO channel interrupts on button and switch channels.
			XGpio_InterruptEnable( &xInputGPIOInstance, XGPIO_IR_CH1_MASK  | XGPIO_IR_CH2_MASK);
			XGpio_InterruptGlobalEnable( &xInputGPIOInstance );
		}

		// initialize the Nexys4 driver
		/*uint32_t status = */NX4IO_initialize(N4IO_BASEADDR);
		//Assume we're good.
		/*
		if (status != XST_SUCCESS){
			return XST_FAILURE;
		}
		*/
		NX4IO_RGBLED_setChnlEn(RGB1, false, false, true);
	}

	configASSERT( ( xStatus == pdPASS ) );
}


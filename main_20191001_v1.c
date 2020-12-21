//*****************************************************************************
//
// Motor Control with Tiva
// Open a terminal with 115,200 8-N-1 to control
//
//*****************************************************************************
//
// Connections
//
// PF2 -> DIR Motor 1
// PB5 -> PWM Motor 1
//
// PF3 -> DIR Motor 2
// PE4 -> PWM Motor 2
//
// PD6 -> PhA0 Encoder 1
// PD7 -> PhB0 Encoder 1
//
// PC5 -> PhA1 Encoder 2
// PC6 -> PhB1 Encoder 2
//
//*****************************************************************************


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_qei.h" // ?????????????????????????????????????
#include "driverlib/sysctl.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/qei.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"


//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
int Step1 = 0;		              	// Input from user
uint32_t Setpoint1 = 2000000000;	// Setpoint for motor command

volatile int32_t Direction1;    	// Motor 1 Direction
volatile int32_t Velocity1;     	// Motor 1 Velocity [counts/period]
volatile uint32_t Position1;    	// Motor 1 Position [counts] 
int32_t error1 = 0;             	// Control error [Counts]
double Kp1 = 0.0020;            	// Kp gain for position control
double u1 = 0;                  	// Output command(%)

volatile int32_t Direction2;        // Motor 2 Direction
volatile int32_t Velocity2;         // Motor 2 Velocity [counts/period]
volatile uint32_t Position2;        // Motor 2 Position [counts]
int32_t error2 = 0;                 // Control error [Counts]
double Kp2 = 0.0020;                // Kp gain for position control
double u2 = 0;                      // Output command(%)

#define UpLimit 40              	// Maximum PWM output value

int32_t planning_counter = 0;

int PWM_output = 0;                 // Decimal value after conversion



//*****************************************************************************
//
// Configure the UART and its pins
//
//*****************************************************************************
void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O
    //
    UARTStdioConfig(0, 115200, SysCtlClockGet());
}


//*****************************************************************************
//
// Timer 0 configuration
//
//*****************************************************************************
void ConfigureTimer0(void)
{
    uint32_t ui32Period;

    //
    // Before calling any peripheral specific driverLib function
    // enable the clock to that peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    //
    // Configure Timer 0 as a 32-bit timer in periodic mode
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    //
    // Period
    //
    ui32Period = (SysCtlClockGet() / 10000);

    //
    // Load the calculated period into the Timer’s Interval Load register
    //
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);

    //
    // Enable the interrupt not only in the timer module, but also in the NVIC
    // (the Nested Vector Interrupt Controller, the Cortex M4’s interrupt controller)
    //
    // -> IntMasterEnable() is the master interrupt enable API for all interrupts
    // -> IntEnable() enables the specific vector associated with Timer0A
    // -> TimerIntEnable() enables a specific event within the timer to generate an interrupt
    //
    // In this case we are enabling an interrupt to be generated on a timeout of Timer 0A
    //
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();

    //
    // Enable the timer - This will start the timer and interrupts
    // will begin triggering on the timeouts
    //
    TimerEnable(TIMER0_BASE, TIMER_A);
}


//*****************************************************************************
//
// SW1 configuration
//
//*****************************************************************************
void ConfigureSW1(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}


//*****************************************************************************
//
// Configure PWM Pin for motor 1
//
//*****************************************************************************
void ConfigureMotor1PWM(void)
{
    //
    // Configure PWM Clock to match system's clock
    //
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    SysCtlDelay(10);	
	
    //
    // Enable the PWM0 peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    //
    // Enable the GPIO port B for the output PWM signal
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlDelay(10);
	
    //
    // Pin PB5 as PWM (M0PWM3)
    //
    GPIOPinConfigure(GPIO_PB5_M0PWM3);

    //
    // Pin PB5 as output PWM - M0PWM3
    //
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);
	
	//
    // Configure the PWM generator
    //
    //PWMGenConfigure(PWM0_BASE, PWM_GEN_1,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    SysCtlDelay(10);
	
	//
    // Set the PWM period
    // Desired PWM frequency: 20KHz -> Period: 1/20.000s = 50us
	// N = (1 / f) * SysClk.  Where N [cycles] is the function parameter, 
	// f is the desired frequency, and SysClk is the system clock frequency.
    // In this case: (1 / 20KHz) * 50MHz = 2500 cycles.
    //
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 2500);
    SysCtlDelay(10);
}


//*****************************************************************************
//
// Configure PWM Pin for motor 2
//
//*****************************************************************************
void ConfigureMotor2PWM(void)
{
    //
    // Configure PWM Clock to match system's clock
    //
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    SysCtlDelay(10);

    //
    // Enable the PWM1 peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

	//
    // Enable the GPIO port E for the output PWM signal
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlDelay(10);

    //
    // Pin PE4 as PWM (M1PWM2)
    //
    GPIOPinConfigure(GPIO_PE4_M1PWM2);

    //
    // Pin PE4 as output PWM - M1PWM2
    //
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);

    //
    // Configure the PWM generator for count down mode with immediate updates to the parameters
    //
    //PWMGenConfigure(PWM1_BASE, PWM_GEN_1,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    SysCtlDelay(10);

	//
    // Set the PWM period
    // Desired PWM frequency: 20KHz -> Period: 1/20.000s = 50us
	// N = (1 / f) * SysClk.  Where N [cycles] is the function parameter,
	// f is the desired frequency, and SysClk is the system clock frequency.
    // In this case: (1 / 20KHz) * 50MHz = 2500 cycles.
    //
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, 2500);
    SysCtlDelay(10);
}


//*****************************************************************************
//
// Configure Direction Pin for Motor 1
//
//*****************************************************************************
void ConfigureMotor1DirectionPin(void)
{
    //
    // Enable the GPIOF port F for the output direction signal
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Pin PF2 Direction for Motor
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
}


//*****************************************************************************
//
// Configure Direction Pin for Motor 2
//
//*****************************************************************************
void ConfigureMotor2DirectionPin(void)
{
    //
    // Enable the GPIOF port F for the output direction signals
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Pin PF3 Direction for Motor
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
}


//*****************************************************************************
//
// Configure QEI Pins for Motor 1
//
//*****************************************************************************
void ConfigureMotor1QEI(void)
{
    //
    // Enable Peripheral port D for the encoder
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	
    //
    // Enable QEI0 Peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
	
	//
    // Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
    // In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
    //
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
	
	//
    // Pin PD6 as PhA0
	// Pin PD7 as PhB0
    //
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	
	//
    // Disable QEI0
    //
    QEIDisable(QEI0_BASE);
	
    //
    // Disable QEI0 velocity capture
    //
    QEIVelocityDisable(QEI0_BASE);
	
    //
    // Disable QEI0 interrupt sources
    //
    QEIIntDisable(QEI0_BASE, (QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX));
	
	//
    // Configure QEI0
    //
    // capture on both A and B
    // do not reset when there is an index pulse
    // do not swap signals PHA0 and PHB0
    // set the maximum position as 4294967200, since max value for uint32_t is 4294967295
    //
    QEIConfigure(QEI0_BASE,(QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 4294967200);
    SysCtlDelay(10);
	
    //
    // Configure the velocity capture for QEI0
    // 40000 is the period at which the velocity will be measured
    //
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_16, 40000);
    SysCtlDelay(10);
	
	//
    // Enable QEI0
    //
    QEIEnable(QEI0_BASE);
    SysCtlDelay(10);

    //
    // Set the current position for QEI0 somewhere in the
    // middle of the uint32_t range [0, 4294967295]
    //
    QEIPositionSet(QEI0_BASE, 2000000000);
    SysCtlDelay(10);
	
	//
    // Enable velocity capture for QEI0
    //
    QEIVelocityEnable(QEI0_BASE);
}


//*****************************************************************************
//
// Configure QEI Pins for Motor 2
//
//*****************************************************************************
void ConfigureMotor2QEI(void)
{
    //
    // Enable Peripheral port C for the encoder
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	
    //
    // Enable QEI1 peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
	
    //
    // Pin PC5 as PhA1
	// Pin PC6 as PhB1
    //
    GPIOPinConfigure(GPIO_PC5_PHA1);
    GPIOPinConfigure(GPIO_PC6_PHB1);
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);
	
    //
    // Disable QEI1
    //
    QEIDisable(QEI1_BASE);
	
    //
    // Disable QEI1 velocity
    //
    QEIVelocityDisable(QEI1_BASE);
	
    //
    // Disable QEI1 interrupt sources
    //
    QEIIntDisable(QEI1_BASE, (QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX));
	
    //
    // Configure QEI1
    //
    // capture on both A and B
    // do not reset when there is an index pulse
    // do not swap signals PHA0 and PHB0
    // set the maximum position as 4294967200, since max value for uint32_t is 4294967295
    //
    QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 4294967200);
    SysCtlDelay(10);
	
    //
    // Configure the velocity capture for QEI1
    // 40000 is the period at which the velocity will be measured
    //
    QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_16, 40000);
    SysCtlDelay(10);
	
    //
    // Enable QEI1
    //
    QEIEnable(QEI1_BASE);
    SysCtlDelay(10);
	
    //
    // Set the current position for QEI1 somewhere in the
    // middle of the uint32_t range [0, 4294967295]
    //
    QEIPositionSet(QEI1_BASE, 2000000000);
    SysCtlDelay(10);
	
    //
    // Enable velocity capture for QEI1
    //
    QEIVelocityEnable(QEI1_BASE);
}


//*****************************************************************************
//
// Drive Motor 1
//
//*****************************************************************************
void DriveMotor1(int8_t u)
{
    //
    // If duty cycle is 0 , disable the PWM generator and output
    //
    if(!u)
    {
        PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, false);
        PWMGenDisable(PWM0_BASE, PWM_GEN_1); // CHECK GENERATOR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }
    //
    // else set direction and PWM output accordingly
    //
    else
    {
        //
        // Set direction
        //
        if (u < 0)
        {
            u = -u;
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
        }
        else
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        }

        //
        // Set motor duty cycle
        //
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1) * (uint32_t)u / 100);
        //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, PWMGen1Period * (uint32_t)u / 100);
        PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
        PWMGenEnable(PWM0_BASE, PWM_GEN_1); // CHECK GENERATOR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }
}


//*****************************************************************************
//
// Drive Motor 2
//
//*****************************************************************************
void DriveMotor2(int8_t u)
{
    //
    // If duty cycle is 0 , disable the PWM generator and output
    //
    if(!u)
	{
	    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, false);
        PWMGenDisable(PWM1_BASE, PWM_GEN_1);
	}
	//
	// else set direction and PWM output accordingly
	//
	else
	{
		//
		// Set direction
		//
		if (u < 0)
		{
			u = -u;
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
		}
		else
		{
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
		}

		//
		// Set motor duty cycle
		//
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_1) * (uint32_t)u / 100);
		//PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, PWMGen1Period * (uint32_t)u / 100);
		PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
		PWMGenEnable(PWM1_BASE, PWM_GEN_1);
	}
}


//*****************************************************************************
//
// Position Control - Motor 1
//
//*****************************************************************************
void Motor1PositionControl(void)
{
    //
    // Read encoder Position, Velocity and Direction
    // Get direction (1 = forward, -1 = backward)
    // Get velocity (counts per period) and multiply by direction so that it is signed
    //
    Position1 = QEIPositionGet(QEI0_BASE);
    Direction1 = QEIDirectionGet(QEI0_BASE);
    Velocity1 = (int32_t)QEIVelocityGet(QEI0_BASE) * Direction1;

    //
    // Control Algorithm
    //
    error1 = (int32_t)(Setpoint1 - Position1);
    //u1 = Kp1 * error1 - Kd1 * Velocity1;
    u1 = - Kp1 * error1;
	
	//
    // Apply saturation limits
    //
    if (u1 > UpLimit)
        u1 = UpLimit;
    else if (u1 < -UpLimit)
        u1 = -UpLimit;

	//
	// Drive Motor 1
	//	
	//DriveMotor1((int8_t)u1);
}


//*****************************************************************************
//
// Position Control - Motor 2
//
//*****************************************************************************
void Motor2PositionControl(void)
{
    //
    // Read encoder Position, Velocity and Direction
    // Get direction (1 = forward, -1 = backward)
    // Get velocity (counts per period) and multiply by direction so that it is signed
    //
    Position2 = QEIPositionGet(QEI1_BASE);
    Direction2 = QEIDirectionGet(QEI1_BASE);
    Velocity2 = (int32_t)QEIVelocityGet(QEI1_BASE) * Direction2;

    //
    // Control Algorithm
    //
    error2 = (int32_t)(Setpoint1 - Position2);
    u2 = - Kp2 * error2;

    //
    // Apply saturation limits
    //
    if (u2 > UpLimit)
        u2 = UpLimit;
    else if (u2 < -UpLimit)
        u2 = -UpLimit;

    //
    // Drive Motor 2
    //
    //DriveMotor2((int8_t)u2);
}


//*****************************************************************************
//
// Timer 0 handler
//
//*****************************************************************************
void Timer0IntHandler(void)
{
    //
    // Clear the timer interrupt
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	//
    // Planning
    //
    planning_counter++;
    if (planning_counter % 20 == 0)
        Setpoint1 += Step1;
	
    //
    // Control Motor 1
    //
    Motor1PositionControl();

    //
    // Control Motor 2
    //
    Motor2PositionControl();

    //
    // Print in terminal
    //
    if (planning_counter % 2000 == 0)
    {
        //UARTprintf("\nM1 | p: %u, e: %d, u: %d", Position1, error1, (int)u1);
        //UARTprintf("%u, %d, %d, %d\n", Position1, error1, (int)u1, Step1);
        //UARTprintf("M2 | p: %u, e: %d, u: %d\n\n", Position2, error2, (int)u2);
        UARTprintf("P1 = %u | P2 = %u | PWM = %d\n", Position1, Position2, PWM_output);

    }
}


//*****************************************************************************
//
// Main
//
//*****************************************************************************
int main(void)
{
	//
	// Variables
	//
	char user_input[8];  		        // Buffer for storing keyboard input
	int user_input_dec = 0;             // Decimal value after conversion
	
    //
    // Run clock at 50MHz
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    //
    // Configure PWM Pins, Direction Pins and QEI for Motor 1
    //
    ConfigureMotor1PWM();
    ConfigureMotor1DirectionPin();
    ConfigureMotor1QEI();

    //
    // Configure PWM Pins, Direction Pins and QEI for Motor 2
    //
    ConfigureMotor2PWM();
    ConfigureMotor2DirectionPin();
    ConfigureMotor2QEI();

    //
    // Configure Switch 1
    //
    ConfigureSW1();

    //
    // Initialize the UART and say hello
    //
    ConfigureUART();
    UARTprintf("\n\nHi!\n\n");

    //
    // Configure Timer 0
    //
    ConfigureTimer0();

    //
    // Main loop
    //
    while(1)
    {
        //
        // Ask for input
        //
//        UARTprintf("Give command ");

        //
        // Read < 8 bytes and hit enter
        //
        UARTgets(user_input,8);

        //
        // Convert input to decimal
        //
        sscanf(user_input,"%d",&user_input_dec);

        //
        // Filter Input
        //
        if ((user_input_dec > 85) || (user_input_dec < -85))
        {
            //
            // INVALID INPUT - Do not update motor command
            //
            UARTprintf("Desired PWM: %d\n", user_input_dec);
            UARTprintf("INVALID INPUT\n\n");
        }
        else
        {
            //
            // Update motor command - Change ang. rate
            //
            //Step1 = user_input_dec;
            PWM_output = user_input_dec;
            DriveMotor1(PWM_output);
            DriveMotor2(PWM_output);
        }
    }
}



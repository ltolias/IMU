// main.c 
// Leonidas Tolias

//Main Program 



#include "main.h"


/*
    Initializes a serial console for general talking/debugging
    \param none
    \return none
*/
void InitConsole(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
   
    GPIOPinConfigure(GPIO_PC4_U1RX);
    GPIOPinConfigure(GPIO_PC5_U1TX);

    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    UARTStdioInitExpClk(1, 115200);
}

/*
    Filter Update Interrupt Handler
    \param none
    \return none
*/
void Timer0IntHandler(void)
{
    //
    // Clear the timer interrupt flag.  16384
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    #ifdef USECOMPLEMENTARY
    //this gets raw accel/gyro update from mpu
    getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    //trueangle[1] =  trueangle[1] - (float)(gx)/30/FILTERUPDATE;
    
    trueangle[0] = ka * (trueangle[0] - (float)(gy)/DIVGX/FILTERUPDATE) + (1-ka) * (float)(ax)/DIVAX;
    trueangle[1] = ka * (trueangle[1] + (float)(gx)/DIVGY/FILTERUPDATE) + (1-ka) * (float)(ay)/DIVAY;
    #endif

    #ifdef USEKALMAN

    K_Predict_State();

    K_Predict_P();

    getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    K_Get_Observation((float)(ax)/DIVAX, (float)(-gy)/DIVGX, (float)(ay)/DIVAY, (float)(gx)/DIVGY);

    K_Update_K();

    K_Update_State();

    K_Report_Attitude(trueangle);

    K_Update_P();
    #endif

}

/*
    PWM Update Interrupt Handler
    \param none
    \return none
*/
void Timer3IntHandler(void)
{
    //
    // Clear the timer interrupt flag.
    //
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

    #ifdef USEPID
    
    //incase filter interrupt changes trueangle halfway through...
    error[0].currangle = trueangle[0];
    error[1].currangle = trueangle[1];


    //lets do pid controll! (x axis is 0, y axis is 1)
    //find error
    error[0].current = (desiredangle[0] - error[0].currangle);
    error[1].current = (desiredangle[1] - error[1].currangle);

    //integrate (multiply gain here so we can change it on the fly without consequences)
    error[0].sum += error[0].current * ki;
    error[1].sum += error[1].current * ki;
    if (error[0].sum > 100) error[0].sum = 100;
    if (error[0].sum < 0) error[0].sum = 0;
    if (error[1].sum > 100) error[1].sum = 100;
    if (error[1].sum < 0) error[1].sum = 0;
    //differentiate (use Proccess Var instead of error to fix derivative kick)
    error[0].derr = error[0].currangle - error[0].lastangle;
    error[1].derr = error[0].currangle - error[1].lastangle;
    //sum
    error[0].delta = kp * error[0].current + error[0].sum - kd * error[0].derr;
    error[1].delta = kp * error[1].current + error[1].sum - kd * error[1].derr;
    if (error[0].delta > 100) error[0].delta = 100;
    if (error[1].delta < 0) error[1].delta = 0;

    //allow for killswitch 
    if (thrust == 0) 
    { 
        error[0].delta = 0; 
        error[1].delta = 0; 
        error[0].sum = 0;
        error[1].sum = 0;
    }


    //deal with the x configuration of the motors
    motorDelta[1] = error[0].delta;
    motorDelta[2] = error[0].delta;

    motorDelta[3] = -error[0].delta;
    motorDelta[4] = -error[0].delta;

    motorDelta[1] += error[1].delta;
    motorDelta[3] += error[1].delta;

    motorDelta[2] += -error[1].delta;
    motorDelta[4] += -error[1].delta;

    //todo: if desired angle is not zero then add cxs accordingly



    //set escs
    motorSet(1, thrust + motorDelta[1]);
    motorSet(2, thrust + motorDelta[2]);
    motorSet(3, thrust + motorDelta[3]);
    motorSet(4, thrust + motorDelta[4]);

    //keep current angle for next time
    error[0].lastangle = error[0].currangle;
    error[1].lastangle = error[1].currangle;

    #endif
    #ifdef USEPROPORTIONAL
    //decide if we are straying from desired angle and calculate how much to change in each axis
    float xdelta = (desiredangle[0] - trueangle[0]);
    float ydelta = (desiredangle[1] - trueangle[1]);


    //allow for killswitch 
    if (thrust == 0) 
    { 
        xdelta = 0; 
        ydelta = 0; 
    }
    //deal with the x configuration of the motors
    motorDelta[1] = xdelta;
    motorDelta[2] = xdelta;

    motorDelta[3] = -xdelta;
    motorDelta[4] = -xdelta;

    motorDelta[1] += ydelta;
    motorDelta[3] += ydelta;

    motorDelta[2] += -ydelta;
    motorDelta[4] += -ydelta;

    //todo: if desired angle is not zero then add cxs accordingly


    //set escs
    motorSet(1, thrust + motorDelta[1]);
    motorSet(2, thrust + motorDelta[2]);
    motorSet(3, thrust + motorDelta[3]);
    motorSet(4, thrust + motorDelta[4]);
    #endif
    

}

/*
    Setup the Interupts for filter and motor updates
    \param none
    \return none
*/
void configureInterupts()
{
    //timer0 will be the filter
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); 
    TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);

    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / FILTERUPDATE);

    IntEnable(INT_TIMER0A); 
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT); 

    //timer3 will be for the motors
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3); 
    TimerConfigure(TIMER3_BASE, TIMER_CFG_32_BIT_PER);

    TimerLoadSet(TIMER3_BASE, TIMER_A, SysCtlClockGet() / MOTORUPDATE);
    
    IntEnable(INT_TIMER3A); 
    TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT); 
    
    //set priorities
    IntPrioritySet(INT_TIMER0A, 0x00);
    IntPrioritySet(INT_TIMER3A, 0x01);

    IntMasterEnable();


    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerEnable(TIMER3_BASE, TIMER_A);
}

/*
    Main Program loop
    \param none
    \return shouldn't
*/
int main(void)
{

    //set clock to 80MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    //say hello 
    InitConsole();
    UARTprintf("\n\nBooting up:");

    //we love the fpu...
    UARTprintf("\nfpu");
    FPUEnable();
    FPULazyStackingEnable();
    UARTprintf("...");
    
    //initialize angles and thrust at zero
    UARTprintf("initialize flight variables...");

    trueangle[0] = 0;
    trueangle[1] = 0;

    desiredangle[0] = 0;
    desiredangle[1] = 0;

    #ifdef USEPID
    error[0].current = 0;
    error[0].lastangle = 0;
    error[0].currangle = 0;
    error[0].sum = 0;
    error[0].derr = 0;
    error[0].delta = 0;

    error[1].current = 0;
    error[1].lastangle = 0;
    error[1].currangle = 0;
    error[1].sum = 0;
    error[1].derr = 0;
    error[1].delta = 0;

    kp = 0.9;
    kd = 0.2 * (float)MOTORUPDATE;
    ki = 0.2 / (float)MOTORUPDATE;
    #endif
    #ifdef USEPROPORTIONAL
    kp = 0.5;
    #endif

    thrust = 0;
    #ifdef USECOMPLEMENTARY
    ka = .99;
    #endif
    #ifdef USEKALMAN
    UARTprintf("\ninitialize kalman filter...");
    K_initialize();
    #endif
    //set everything else up
    UARTprintf("\nI2C");
    initI2C();
    UARTprintf("...IMU");
    initIMU();
    UARTprintf("...PWM");
    initPWM();
    UARTprintf("...Interrupts");
    configureInterupts();
    UARTprintf("...\nboot complete");

    SysCtlDelay(SysCtlClockGet()/20);
 
    //ready to go
    UARTprintf("\n\nQuadcopter Control Board\n");
    UARTprintf("MCU: Stellaris Launchpad LM4F120H5QR\n");
    UARTprintf("IMU Device: MPU6050\tInterface: I2C 400kbps\n");
    UARTprintf("Configuration: X\n");
    UARTprintf("Controls:\n\ta/z to increase and decrease thrust, 1/2 set increment to 1 or 10, 3 to 100\n");
    UARTprintf("\ti/k/j/l for f/b/l/r, u to hover, q is killswitch\n\n");

    unsigned char c;
    c = 0;
    int inc = 1;
    goto run;

    run:
    while(1)
    {
        
       
        SysCtlDelay(SysCtlClockGet() / 80); 
    
        UARTprintf("\r");
        UARTprintf("true angle: (%4i,%4i)\t\tdesired angle: (%4i,%4i)\tthrust:%3i\t", (int)trueangle[0], (int)trueangle[1], (int)desiredangle[0], (int)desiredangle[1], (int)thrust);
       
        c = UARTCharGetNonBlocking(UART1_BASE);
        //switch case doesnt seem to play nice with 'c' type chars...
        if (c == 'a') thrust += inc;
        if (c == 'z') thrust -= inc;
        if (c == '1') inc = 1;
        if (c == '2') inc = 10;
        if (c == 'i') desiredangle[0] -= inc;
        if (c == 'k') desiredangle[0] += inc;
        if (c == 'j') desiredangle[1] -= inc;
        if (c == 'l') desiredangle[1] += inc;
        if (c == '3') thrust = 100;
        if (c == 'r')
        {
            reset();
            SysCtlDelay(SysCtlClockGet()/ 20);
            initIMU();
        }
        if (c == 'u')
        {
            desiredangle[0] = 0;
            desiredangle[1] = 0;
        }
        if (c == 'q') 
        {
            thrust = 0; 
            desiredangle[0] = 0; 
            desiredangle[1] = 0; 
        }
        #ifdef USECOMPLEMENTARY
        if (c == 'f') goto setf; //for on the fly changing of filter and proportionality constants
        #endif
 

       

        if (thrust > 100) thrust = 100;
        if (thrust < 0) thrust = 0;

        
        
       
        
    }


    #ifdef USECOMPLEMENTARY
    setf:
    UARTprintf("\r                                                                          ");
    while(1)
    {
        SysCtlDelay(SysCtlClockGet() / 80);
        UARTprintf("\r");
        UARTprintf("Filter: 0.%3i\t\ttrue angle: (%4i,%4i)", (int) (ka * 1000), (int)trueangle[0], (int)trueangle[1]);
        c = UARTCharGetNonBlocking(UART1_BASE);
        ka = ka * 1000;
        if (c == 'a') ka += 1;
        if (c == 'z') ka -= 1;
        if (c == 'f') goto setp;
        ka = ka / 1000;
    }
    #endif
    #ifdef USEPROPORTIONAL
    setp:
    UARTprintf("\r                                                                          ");
    while(1)
    {
        SysCtlDelay(SysCtlClockGet() / 80);
        UARTprintf("\r");
        UARTprintf("P: 0.%3i   ", (int) (kp * 100));
        c = UARTCharGetNonBlocking(UART1_BASE);
        kp = kp * 100;
        if (c == 'a') kp += 1;
        if (c == 'z') kp -= 1;
        if (c == 'f') goto run;
        kp = kp / 100;
    }
    #endif
                        
       



  
    return(0);
}






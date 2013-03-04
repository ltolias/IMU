// main.c 
// Leonidas Tolias

//Main Program Loop and Interrupt Handlers

/*This program is designed to use a Stellaris Launchpad from Texas Instrumentsand an MPU-6050 from Invensense
to maintain and control the attitude of a quadcopter.

Attitude Estimation is performed by two offline Kalman Filters, each of which tracks angle and angular velocity about one axis.  A description of the derivation of the filter can be found at 
www.leonidastolias.com.

Motor Control assumes that brushless motors are being used, and uses 2 timers (TIMER1 and TIMER2) split into 4 subtimers
to create pulses that vary between 1 and 20 ms which are standard control inputs for brushless Electronic Speed Controllers.
The Pulse repetition rate can be controlled with MOTORUPDATE.

Communications use simple serial control over a bluetooth serial port.  In the future, a NRF24L01 controll interface will be added.

The program exists in the following files:
main.c - System Initialization and main control loop
motors.c - methods for controlling Brushless Electronic Speed Controllers
offline_kalman.c - methods for precomputing Kalman Gain
MPU6050.c - methods for controlling the MPU-6050 from Invensense
I2Clib.c - methods for hardware I2C communications

The code attempts to be as easy to port as possible.  Platform specific code exists only in:
motors.c - timer routines
I2Clib.c - Hardware I2C routines
main.c - Interrupt Handlers and Configuration, Hardware UART routines

*/



#include "main.h"

void transferToUART(void)
{
    UARTIntEnable(UART5_BASE, UART_INT_TX);

    while (UARTSpaceAvail(UART5_BASE) && ( (tx.readIndex < tx.writeIndex)  || (tx.over == 1) ) )
    {
        UARTCharPutNonBlocking(UART5_BASE, tx.buffer[tx.readIndex]);
        tx.readIndex ++;
        if (tx.readIndex == TX_BUFF_SIZE) { tx.readIndex = 0; tx.over = 0;}

    }
}

/*
    Get Everything up and running
    \param none
    \return none
*/
void Initialize_System()
{
    //say hello 
    initConsole();

    bprints("\n\nBooting up:");
    SysCtlDelay(SysCtlClockGet() / 40);
    
    bprints("\nFPU...");

    FPUEnable();
    FPULazyStackingEnable();
    SysCtlDelay(SysCtlClockGet() / 40);

    bprints("I2C");
    initI2C();
    SysCtlDelay(SysCtlClockGet() / 40); 

    bprints("...IMU");
    initIMU();
    SysCtlDelay(SysCtlClockGet() / 40);

    bprints("...PWM");
    initPWM();
    SysCtlDelay(SysCtlClockGet() / 40);  
    
    //initialize angles and thrust at zero
    bprints("...\ninitialize flight variables...");
    initFlightVars();
    SysCtlDelay(SysCtlClockGet() / 40); 

    bprints("\ninitialize filter constants...\n");

    #ifdef USEPID
    kp = 0.01f;
    kd = 0.0f * (float)MOTORUPDATE;
    ki = 0.0f;//927f / (float)MOTORUPDATE;
    kdd = 0;
    kii = 0;
    #endif

    #ifdef USEPROPORTIONAL
    kp = 0.1f;
    #endif

    #ifdef USECOMPLEMENTARY
    ka = 0.994f;
    #endif

    SysCtlDelay(SysCtlClockGet() / 40); 

    //IMU Calibration
    unsigned char c = 0;
    while (c == 0)
    {
        bprints("Press c to calibrate IMU, or s to use preset kalman variances\r");
        c = UARTCharGetNonBlocking(UART5_BASE);
        if (c == 'c')
        {
            bprints("\nCalibrating");
            calibrateIMU();
            i = 100;
            bprints("\nMeans and Variances:\n\tax\tay\tgx\tgy\nmean\t");
            bprintifw((int)(zero.x*i), 4); bprints("\t");
            bprintifw((int)(zero.y*i), 4); bprints("\t");
            bprintifw((int)(zero.dx*i), 4); bprints("\t");
            bprintifw((int)(zero.dy*i), 4); bprints("\t\nvar\t");
            bprintifw((int)(Accel_Variance * i), 4); bprints("\t\t");
            bprintifw((int)(Gyro_Variance * i), 4); bprints("\tx 10^-2");

            zero.x = 0.0f;
            zero.y = 0.0f;
        }
        else if (c == 's')
        {
            calibrateIMU();
            Accel_Variance = 200.0f;
            Gyro_Variance =  200.0f;
            i = 100;
            bprints("\nMeans and Variances:\n\tax\tay\tgx\tgy\nmean\t");
            bprintifw((int)(zero.x*i), 4); bprints("\t");
            bprintifw((int)(zero.y*i), 4); bprints("\t");
            bprintifw((int)(zero.dx*i), 4); bprints("\t");
            bprintifw((int)(zero.dy*i), 4); bprints("\n");
            bprints("Measurement variances set to default (5)");
        }
        else c = 0;
        SysCtlDelay(SysCtlClockGet()/10);
    }
   
    SysCtlDelay(SysCtlClockGet() / 40);

    #ifdef USEKALMAN
    bprints("\nInitialize kalman filter...");
   
    K_Initialize(Gyro_Variance, Accel_Variance, 3.0f);

    bprints("\nPrecalculate gains");
    i = K_Precalculate_Gains();
    bprints("...completed in %i iterations\nKalman Gain is:\n", i);
    i = 100;
    bprints("%6i\t%6i\n%6i\t%6i\t\t x 10^-2", (int)(K[0][0] * i), (int)(K[0][1] * i), (int)(K[0][2] * i), (int)(K[0][3] * i));

    SysCtlDelay(SysCtlClockGet() / 40); 
    #endif
    
    bprints("\nInterrupts");
    configureInterupts();
    SysCtlDelay(SysCtlClockGet() / 40); 
    yaw = YAWNAUGHT; 

    bprints("...\nboot complete");

}


void TelemetryInterrupt_T5(void)
{
    TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);


    //pretend there isnt a fifo
    while (UARTCharsAvail(UART5_BASE))
    {
        c = UARTCharGetNonBlocking(UART5_BASE);
    }

    bprints("\r");
    bprints("m:(");
    bprintifw((int)(state.x), 4); bprints(",");
    bprintifw((int)(state.y), 4); bprints(")\td:(");
    bprintifw((int)(desiredangle[0]), 4); bprints(",");
    bprintifw((int)(desiredangle[1]), 4); bprints(")\tcorr:(");
    bprintifw((int)(error[0].delta), 4); bprints(",");
    bprintifw((int)(error[1].delta), 4); bprints(")\tup:");
    bprintifw((int)(thrust), 3); bprints("\tyaw:");
    bprintifw((int)(yaw * 2), 3); bprints("\tkp:");
    bprintifw((int)(kp*100), 2);   bprints("\tkd:");
    bprintifw((int)(kdd*100), 2);   bprints("\tki:");
    bprintifw((int)(kii*100), 2);   bprints("\t\t");
        
    //switch case doesnt seem to play nice with 'c' type chars...
    if (c == 'a') thrust += inc;
    if (c == 'z') thrust -= inc;
    if (c == '1') inc = 1;
    if (c == '2') inc = 5;
    if (c == '3') inc = 10;
    if (c == 'i') desiredangle[0] -= inc;
    if (c == 'k') desiredangle[0] += inc;
    if (c == 'j') desiredangle[1] -= inc;
    if (c == 'l') desiredangle[1] += inc;
    if (c == '5') thrust = 60;
    if (c == 'u') yaw += (float)(inc) * 0.5f;
    if (c == 'o') yaw -= (float)(inc) * 0.5f;
    if (c == 'c')
    {
        stopInterrupts();
        bprints("\nCalibrating");
        calibrateIMU();
        i = 100;
        bprints("\nMeans and Variances:\n\tax\tay\tgx\tgy\nmean\t");
        bprintifw((int)(zero.x*i), 4); bprints("\t");
        bprintifw((int)(zero.y*i), 4); bprints("\t");
        bprintifw((int)(zero.dx*i), 4); bprints("\t");
        bprintifw((int)(zero.dy*i), 4); bprints("\t\nvar\t");
        bprintifw((int)(Accel_Variance * i), 4); bprints("\t\t");
        bprintifw((int)(Gyro_Variance * i), 4); bprints("\tx 10^-2\n\n");

        #ifdef USEKALMAN
        bprints("\ninitialize kalman filter...");

        //gyro, accel, process
        K_Initialize(Gyro_Variance, Accel_Variance, 3.0f);

        bprints("\nPrecalculate gains");
        i = K_Precalculate_Gains();
        bprints("...completed in %i iterations\nKalman Gain is:\n", i);
        i = 100;
        bprints("%6i\t%6i\n%6i\t%6i\t\t x 10^-2\n", (int)(K[0][0] * i), (int)(K[0][1] * i), (int)(K[0][2] * i), (int)(K[0][3] * i));
        #endif
        configureInterupts();
    }

    if (c == 'q')
    {
        thrust = 0; 
        //desiredangle[0] = XNAUGHT; 
        //desiredangle[1] = YNAUGHT; 
    }
    if (c == 'w') kp += 0.01f;
    if (c == 's') kp -= 0.01f;
    if (c == 'e') kdd += 0.01f;
    if (c == 'd') kdd -= 0.01f;
    if (c == 'r') kii += 0.01f;
    if (c == 'f') kii -= 0.01f;
    if (thrust > 100) thrust = 100;
    if (thrust < 0) thrust = 0;
    c = 0;
    kd = kdd * FILTERUPDATE;
    ki = kii / FILTERUPDATE;
}


/*
    Comms Interrupt Handler
    \param none
    \return none
*/
void UARTInterrupt_U5(void)
{
    unsigned long ulInts = UARTIntStatus(UART5_BASE, true);
    UARTIntClear(UART5_BASE, ulInts);

    //unsigned int a = UARTCharGetNonBlocking(UART5_BASE);
    //if (a != c)

    if (ulInts & UART_INT_TX)
    {
         transferToUART();
         if (!((tx.readIndex < tx.writeIndex)  || (tx.over == 1) ) ) UARTIntDisable(UART5_BASE, UART_INT_TX);
    }

    if (ulInts & (UART_INT_RX | UART_INT_RT))
    {

    }
   


}



/*
    Filter Update Interrupt Handler
    \param none
    \return none
*/
void FilterInterrupt_0A(void)
{
    //
    // Clear the timer interrupt flag.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //this gets raw accel/gyro update from mpu
    getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    //calibrate raw data
    observation.x = (float)(ax)/DIVA - zero.x;
    observation.dx = (float)(-gy)/DIVG - zero.dx;
    observation.y = (float)(ay)/DIVA - zero.y;
    observation.dy = (float)(gx)/DIVG - zero.dy;

    #ifdef USECOMPLEMENTARY
    
    state.x = ka * (state.x + observation.dx/FILTERUPDATE) + (1-ka) * observation.x;
    state.y = ka * (state.y + observation.dy/FILTERUPDATE) + (1-ka) * observation.y;
   
    #endif


    #ifdef USEKALMAN    

    //The following expressions and variables neither look like a Kalman filter nore make much sense.
    //An explanation of how they were derived can be found at www.leonidastolias.com

    //K_theta_dobs and K_dtheta_obstheta will generally be very close to zero (as long as the filter rate is fast), 
    //so those terms (especially K_dtheta_obstheta) could probably be left out.

    state.x =  K_theta_theta * state.x  +  K_theta_dtheta * state.dx  +  K_theta_obs * observation.x  +  K_theta_dobs * observation.dx;
    state.dx =  K_dtheta_dtheta * state.dx  +  K_dtheta_obstheta * (observation.x - state.x)  + K_dtheta_dobs * observation.dx;

    state.y =  K_theta_theta * state.y  +  K_theta_dtheta * state.dy  +  K_theta_obs * observation.y  +  K_theta_dobs * observation.dy ;
    state.dy =  K_dtheta_dtheta * state.dy  +  K_dtheta_obstheta * (observation.y - state.y)  + K_dtheta_dobs * observation.dy ;

    #endif
    //the filter runs faster than motor update, so we can average the values in between each motor update
    //sumx += state.x * UPDATERATIO;

    //sumy += state.y * UPDATERATIO;

    doPID();


}

/*
    PWM Update Interrupt Handler
    \param none
    \return none
*/
void MotorInterrupt_3A(void)
{
    //
    // Clear the timer interrupt flag.
    //
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

    //doPID();
    

}

void doPID()
{
    /*/grab the average and clear it so it can start getting new updates
    avx = sumx;
    avy = sumy;
    sumx = 0;
    sumy = 0;
    */

    #ifdef USEPID
    
    //get current angle (will be usefull later)
    error[0].currangle = 0;//state.x;
    error[1].currangle = 0;//state.y;


    //lets do pid controll! (x axis is 0, y axis is 1)
    //find error
    error[0].current = (desiredangle[0] - error[0].currangle);
    error[1].current = (desiredangle[1] - error[1].currangle);

    //integrate (multiply gain here so we can change it on the fly without consequences)
    error[0].sum += error[0].current * ki;
    error[1].sum += error[1].current * ki;
    //make sure it doesnt go crazy
    if (error[0].sum > IRANGE) error[0].sum = IRANGE;
    if (error[0].sum < -IRANGE) error[0].sum = -IRANGE;
    if (error[1].sum > IRANGE) error[1].sum = IRANGE;
    if (error[1].sum < -IRANGE) error[1].sum = -IRANGE;

    //differentiate (use Proccess Var instead of error to fix derivative kick)
    error[0].derr = error[0].currangle - error[0].lastangle;
    error[1].derr = error[1].currangle - error[1].lastangle;

    //calculate PID correction
    error[0].delta = kp * error[0].current + error[0].sum - kd * error[0].derr;
    error[1].delta = kp * error[1].current + error[1].sum - kd * error[1].derr;
   
    //put some reasonable limits (motor values are in percent)
    if (error[0].delta > 50) error[0].delta = 50;
    if (error[1].delta > 50) error[1].delta = 50;
    if (error[0].delta < -50) error[0].delta = -50;
    if (error[1].delta < -50) error[1].delta = -50;

    //allow for killswitch 
    if (thrust == 0) 
    { 
        error[0].sum = 0;
        error[1].sum = 0;
    }


    //deal with the x configuration of the motors
    motorDelta[1] =  error[0].delta;
    motorDelta[2] = error[0].delta;

    motorDelta[3] = -error[0].delta;
    motorDelta[4] = -error[0].delta;

    motorDelta[1] += error[1].delta;
    motorDelta[3] += error[1].delta;

    motorDelta[2] += -error[1].delta;
    motorDelta[4] += -error[1].delta;


    //keep current angle for next time
    error[0].lastangle = error[0].currangle;
    error[1].lastangle = error[1].currangle;

    #endif
    #ifdef USEPROPORTIONAL


    //decide if we are straying from desired angle and calculate how much to change in each axis
    float xdelta = (desiredangle[0] - avx) * kp;
    float ydelta = (desiredangle[1] - avy) * kp;


    //deal with the x configuration of the motors
    motorDelta[1] = xdelta;
    motorDelta[2] = xdelta;

    motorDelta[3] = -xdelta;
    motorDelta[4] = -xdelta;

    motorDelta[1] += ydelta;
    motorDelta[3] += ydelta;

    motorDelta[2] += -ydelta;
    motorDelta[4] += -ydelta;



    #endif


    //todo: if desired angle is not zero then add cxs accordingly


    //Yaw Controll
   
    motorDelta[1] += -yaw;
    motorDelta[2] += yaw;
    motorDelta[3] += yaw;
    motorDelta[4] += -yaw;


    //allow for killswitch 
    if (thrust == 0) 
    { 
        motorDelta[1] = 0;
        motorDelta[2] = 0;
        motorDelta[3] = 0;
        motorDelta[4] = 0;
    }
    
    //these are scales in case some motors are much more powerful than others
    float m1 = 1.0f;
    float m2 = 1.0f;
    float m3 = 0.95f;
    float m4 = 1.0f;

    //set ESCs
    motorSet(1, (m1*thrust) + motorDelta[1]);
    motorSet(2, (m2*thrust) + motorDelta[2]);
    motorSet(3, (m3*thrust) + motorDelta[3]);
    motorSet(4, (m4*thrust) + motorDelta[4]);
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

    //set everything up
    Initialize_System();
 
    //ready to go
    bprints("\n\nQuadcopter Control Board\n");
    bprints("MCU: Stellaris Launchpad LM4F120H5QR\n");
    bprints("IMU Device: MPU6050\tInterface: I2C 400kbps\n");
    bprints("Configuration: X\n");
    bprints("Controls:\n\ta/z to increase and decrease thrust, 1/2 set increment to 1 or 10, 3 to 60\n");
    bprints("\ti/k/j/l for f/b/l/r, u/o yaw, q is killswitch\n\n");

    SysCtlDelay(SysCtlClockGet()/20);

    //start telemetry

    TimerEnable(TIMER5_BASE, TIMER_A);
    


    while(1)
    {
        //chill
    }



  
    return(0);
}

/*
    Calibrates the IMU
    Experimentally determines zero values for the Accelerometers and Gyroscopes and calculates
    their covariances (for the Kalman Filter)
    \param none
    \return none
*/
void calibrateIMU()
{
    float f = 800.0f;
    for(i = 0; i < f; i ++)
    {
        getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        zero.x += ((float)(ax) /DIVA) / f;
        zero.y += ((float)(ay) /DIVA) / f;
        zero.dx += ((float)(gx)/DIVG) / f;
        zero.dy += ((float)(gy)/DIVG) / f;
        
        //mean of the squares
        Accel_Variance += square((float)(ax)/DIVA) / (2*f) + square((float)(ay)/DIVA) / (2*f);
        Gyro_Variance += square((float)(gx)/DIVG) / (2*f) + square((float)(gy)/DIVG)/ (2*f);

        if (i % ((int)(f/10)) == 0) bprints(" .");

        SysCtlDelay(SysCtlClockGet()/3/800);
    }
    //minus the square of the means
    Accel_Variance -= square((zero.x+zero.y)/2.0f);
    Gyro_Variance -= square((zero.x+zero.y)/2.0f);

    //not sure why negatives pop up sometimes, they really shouldn't, but they have been, so...
    if (Accel_Variance < 0) 
    {
        Accel_Variance = -Accel_Variance;
        bprints("\nfixed accelerometer negative variance");
    }
    if (Gyro_Variance < 0)
    { 
        Gyro_Variance = -Gyro_Variance;
        bprints("\nfixed gyroscope negative variance");
    }
   
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
    /*SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3); 
    TimerConfigure(TIMER3_BASE, TIMER_CFG_32_BIT_PER);

    TimerLoadSet(TIMER3_BASE, TIMER_A, SysCtlClockGet() / MOTORUPDATE);
    
    IntEnable(INT_TIMER3A); 
    TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);


    */

    
    //set priorities
    IntPrioritySet(INT_TIMER0A, 0x00);
    //IntPrioritySet(INT_TIMER3A, 0x01);


    IntMasterEnable();


    TimerEnable(TIMER0_BASE, TIMER_A);
    //TimerEnable(TIMER3_BASE, TIMER_A);
    
}

/*
    Stop the Interrupts
    \param none
    \return none
*/
void stopInterrupts()
{
    TimerDisable(TIMER0_BASE, TIMER_A);
    TimerDisable(TIMER3_BASE, TIMER_A);
}

/*
    Initializes a lot of stuff to zero...
    \param none
    \return none
*/
void initFlightVars(void)
{
    state.x = 0;
    state.y = 0;

    sumx = 0;
    sumy = 0;
    avx = 0;
    avy = 0;

    inc = 1;

    desiredangle[0] = XNAUGHT;
    desiredangle[1] = YNAUGHT;

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
    #endif

    thrust = 0;
    yaw = 0;
    Accel_Variance = 0;
    Gyro_Variance = 0;
    zero.x = 0;
    zero.y = 0;
    zero.dy = 0;
    zero.dx = 0;

}

/*
    Initializes a serial console for general talking/debugging
    \param none
    \return none
*/
void initConsole(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
   
    GPIOPinConfigure(GPIO_PE4_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);

    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);



    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);

    //configure the UART for 115200, n, 8, 1
    UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE | UART_CONFIG_WLEN_8));

    //set it up to interrupt when the fifos almost out
    UARTFIFOEnable(UART5_BASE);
    UARTFIFOLevelSet(UART5_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    UARTIntDisable(UART5_BASE, 0xFFFFFFFF);
    //UARTIntEnable(UART5_BASE, UART_INT_TX);
    IntMasterEnable();
    IntEnable(INT_UART5);

    UARTTxIntModeSet(UART5_BASE, UART_TXINT_MODE_FIFO);
    IntPrioritySet(INT_UART5, 0x02);

    UARTEnable(UART5_BASE);



    //timer 5 is for telemetry
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5); 
    TimerConfigure(TIMER5_BASE, TIMER_CFG_32_BIT_PER);

    TimerLoadSet(TIMER5_BASE, TIMER_A, SysCtlClockGet() / TELEMETRYUPDATE );
    
    IntEnable(INT_TIMER5A); 
    TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    IntPrioritySet(INT_TIMER5A, 0x03);
    
}

/*
    Squares a float
    \param a the number to square
    \return the square of a
*/
float square(float a)
{
    return (a * a);
}








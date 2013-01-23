//motors.c
//Leonidas Tolias

//Methods for controlling Brushless Motor Electronic Speed Controllers on the Stellaris platform

#include "motors.h"


/*
    Sets a given motor to a given percent value
    \param motor is the motor number (1-4)
    \param percent is the percentage to set (0-100)
    \return none
*/
void motorSet(int motor, uint16_t percent)
{   
    //sterilize input
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;

    //prepare prescale and match values
    uint32_t cycles_per_ms = SysCtlClockGet() / 1000;
    //*debug: straight pwm leds- */uint32_t matchl = SysCtlClockGet() / MOTORUPDATE / 100 * percent;
    uint32_t matchl = cycles_per_ms + (cycles_per_ms / 100) * percent;
    uint8_t matchh = matchl >> 16;
    matchl &= 0xFFFF;

    //write to correct pwm module
    switch (motor) {
        case 1: { TimerPrescaleMatchSet(TIMER1_BASE, TIMER_A, matchh); TimerMatchSet(TIMER1_BASE, TIMER_A, matchl); }
        case 2: { TimerPrescaleMatchSet(TIMER1_BASE, TIMER_B, matchh); TimerMatchSet(TIMER1_BASE, TIMER_B, matchl); }
        case 3: { TimerPrescaleMatchSet(TIMER2_BASE, TIMER_A, matchh); TimerMatchSet(TIMER2_BASE, TIMER_A, matchl); }
        case 4: { TimerPrescaleMatchSet(TIMER2_BASE, TIMER_B, matchh); TimerMatchSet(TIMER2_BASE, TIMER_B, matchl); }
    }
    
}

/*
    Initializes PWM timers for ESC control
    \param none
    
    Motor 1: PB4 using TIMER1_A
    Motor 2: PB5 using TIMER1_B
    Motor 3: PB0 using TIMER2_A
    Motor 4: PB1 using TIMER2_B

    \return none
*/
void initPWM()
{
    // Configure output pins
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB4_T1CCP0);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_4);
    GPIOPinConfigure(GPIO_PB5_T1CCP1);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_5);  

    GPIOPinConfigure(GPIO_PB0_T2CCP0);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PB1_T2CCP1);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_1); 

    // Configure the two timers as half width (16bit) pwm (though we get 24 bits with the "prescale")
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);  
    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM);
    TimerControlLevel(TIMER1_BASE, TIMER_BOTH, true);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);  
    TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM);
    TimerControlLevel(TIMER2_BASE, TIMER_BOTH, true);

    //calculate pulse repetition rate
    uint32_t cycles_per_interval = SysCtlClockGet() / MOTORUPDATE /*Hz*/;
    uint32_t periodl = cycles_per_interval;
    uint8_t periodh = periodl >> 16;
    periodl &= 0xFFFF;


    //set pulse repetition rate
    TimerPrescaleSet(TIMER1_BASE, TIMER_A, periodh);
    TimerLoadSet(TIMER1_BASE, TIMER_A, periodl);
    TimerPrescaleSet(TIMER1_BASE, TIMER_B, periodh);
    TimerLoadSet(TIMER1_BASE, TIMER_B, periodl);

    TimerPrescaleSet(TIMER2_BASE, TIMER_A, periodh);
    TimerLoadSet(TIMER2_BASE, TIMER_A, periodl);
    TimerPrescaleSet(TIMER2_BASE, TIMER_B, periodh);
    TimerLoadSet(TIMER2_BASE, TIMER_B, periodl); 

    //set all motors to zero
    motorSet(1,0);
    motorSet(2,0);
    motorSet(3,0);
    motorSet(4,0);

    //enable timers
    TimerEnable(TIMER1_BASE, TIMER_A);
    TimerEnable(TIMER1_BASE, TIMER_B);
    TimerEnable(TIMER2_BASE, TIMER_A);
    TimerEnable(TIMER2_BASE, TIMER_B);
}


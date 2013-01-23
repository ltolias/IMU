//spi.c
//Leonidas Tolias

//SPI Abstraction Library for Stellaris



#include "spi.h"


/*
    Initializes SSI0 Module for SPI duties
    \param none
    \return none
*/
void SPI_Init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 8);
    SSIEnable(SSI0_BASE);


    //setup IRQ pin
    //GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6);
    //GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);

    //GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
    //GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
}


/*
    Send and receive multiple bytes over SPI
    \param send is array of bytes to send
    \param receive is array of bytes to receive
    \param length is length of the arrays (must be equal)
    \return none
*/
void SPI_ReadWrite_Block(uint8_t* data, uint8_t* buffer, uint8_t len)
{
    int i;
    for (i = 0; i < len; i ++)
    {
        buffer[i] = SPI_Write_Byte(data[i]);
    }
}
/*
    Send  multiple bytes over SPI
    \param send is array of bytes to send
    \param length is length of the array
    \return none
*/
void SPI_Write_Block(uint8_t* data, uint8_t len)
{
    int i;
    for (i = 0; i < len; i ++)
    {
        SPI_Write_Byte(data[i]);
    }
}

/*
    Send and Receive 1 byte over SPI
    \param send is byte to send
    \return is the received byte
*/
uint8_t SPI_Write_Byte(uint8_t byte)
{
    SSIDataPut(SSI0_BASE, byte);
    long unsigned int *receive;
    long unsigned int x = 0;
    receive = &x;
    SSIDataGet(SSI0_BASE, receive);
    return (uint8_t)*receive;
}

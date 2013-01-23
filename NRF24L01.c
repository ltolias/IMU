


//! - SSI0 peripheral
//! - GPIO Port A peripheral (for SSI0 pins)
//! - SSI0CLK - PA2
//! - SSI0Fss - PA3
//! - SSI0Rx  - PA4
//! - SSI0Tx  - PA5


#include "NRF24L01.h"

/*
    Initializes SSI0 Module for SPI duties
    \param none
    \return none
*/
void initRadio()
{
    

}
/*
    Send and Receive 1 byte over SPI
    \param send is byte to send
    \return is the received byte
*/
uint8_t SPI_TXRX(uint8_t send)
{

    SSIDataPut(SSI0_BASE, send);
    long unsigned int *receive;
    long unsigned int x = 0;
    receive = &x;
    SSIDataGet(SSI0_BASE, receive);
    return (uint8_t)*receive;
}
/*
    Send and receive multiple bytes over SPI
    \param send is array of bytes to send
    \param receive is array of bytes to receive
    \param length is length of the arrays (must be equal)
    \return none
*/
void SPI_TXRXm(uint8_t* send, uint8_t* receive, int length)
{
    i
}
void SPI_TXm(uint8_t* send, int length)
{
    int i;
    for (i = 0; i < length; i ++)
    {
        SPI_TXRX(send[i]);
    }
}
/*
    Receive multiple bytes over SPI
    \param receive is array of bytes to receive
    \param length is length of the array
    \return none
*/
void SPI_RXm(uint8_t* receive, int length)
{
    int i;
    for (i = 0; i < length; i ++)
    {
        receive[i] = SPI_TXRX(0x00);
    }
}








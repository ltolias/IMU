//spi.h
//Leonidas Tolias

//SPI Abstraction Library for Stellaris


#include "stdint.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/ssi.h"


void SPI_Init();


void SPI_ReadWrite_Block (uint8_t * data, uint8_t * buffer, uint8_t len);


void SPI_Write_Block (uint8_t * data, uint8_t len);


uint8_t SPI_Write_Byte (uint8_t data);





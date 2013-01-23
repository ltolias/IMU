//I2Clib.c
//Leonidas Tolias

//I2C method abstraction library

//This is a port of the excellent I2Cdev library by Jeff Rowberg to the Stellaris platform.  
//The port is best characterized as barely functioning

//Basically, all there was to do was reimplement writeBytes, writeWords, readBytes, and readWords with stellaris type I2C commands
//Status:

//writeBytes works as intended
//writeWords has never been tested, but I know it has problems
//readBytes works...ish, it seems to be doing something it shouldn't be, but this doesnt seem to affect its functionality, luckily ; )
// (basically, everytime it tries to read an extra byte, but since it doesnt try to store this anywhere, its not really a problem, as long as the I2C device doesn't get angry about it (which should be pretty uncommon))
//readWords has never been tested, but I know it has problems

//basically, what is ported is the bare minimum necessary to use the MPU6050 I2C device.
//who needs words anyways... clearly not invensense...


//oh yeah, and in an act of probably poor judgement, I stripped the c++ out of it...
//that and the timeout feature from alot of things

//Mr. Rowberg's excellent comments, liscence, and code are included below my adaptations (read: mutilations)


#include "I2Clib.h"





void initI2C()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);


    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinConfigure(GPIO_PE4_I2C2SCL);
    GPIOPinConfigure(GPIO_PE5_I2C2SDA);


    GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);
    GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);

    //GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
    //GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);

     // Give control to the I2C2 Module
    //GPIODirModeSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_DIR_MODE_HW);
    //GPIODirModeSet(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_DIR_MODE_HW);



    // false: 100kbps true:400kbps
    I2CMasterInitExpClk(I2C2_MASTER_BASE, SysCtlClockGet(), true);

    I2CMasterEnable(I2C2_MASTER_BASE);
}


bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data)
{
    I2CMasterSlaveAddrSet(I2C2_MASTER_BASE, devAddr, false);
    
    I2CMasterDataPut(I2C2_MASTER_BASE, regAddr);
    I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C2_MASTER_BASE));
    if (length == 1)
    {
        I2CMasterDataPut(I2C2_MASTER_BASE, data[0]);
        I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        while(I2CMasterBusy(I2C2_MASTER_BASE));
    }
    else
    {
        int i;
        for (i = 0; i < length - 1; i ++)
        {
            I2CMasterDataPut(I2C2_MASTER_BASE, data[i]);
            I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
            while(I2CMasterBusy(I2C2_MASTER_BASE));
        }    
        I2CMasterDataPut(I2C2_MASTER_BASE, data[length - 1]);
        I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        while(I2CMasterBusy(I2C2_MASTER_BASE));

    }
    
    return false;
}

bool writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data)
{
    I2CMasterSlaveAddrSet(I2C2_MASTER_BASE, devAddr, false);
    
    I2CMasterDataPut(I2C2_MASTER_BASE, regAddr);
    I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C2_MASTER_BASE));

    int i;
    for (i = 0; i < length; i ++)
    {
        I2CMasterDataPut(I2C2_MASTER_BASE, (uint8_t)(data[i] >> 8));
        I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        while(I2CMasterBusy(I2C2_MASTER_BASE));

        I2CMasterDataPut(I2C2_MASTER_BASE, (uint8_t)(data[i]));
        I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        while(I2CMasterBusy(I2C2_MASTER_BASE));
    }    

    I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C2_MASTER_BASE));
    return false;
}


int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
    I2CMasterSlaveAddrSet(I2C2_MASTER_BASE, devAddr, false);
    
    I2CMasterDataPut(I2C2_MASTER_BASE, regAddr);
    I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C2_MASTER_BASE));

    I2CMasterSlaveAddrSet(I2C2_MASTER_BASE, devAddr, true);
    I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C2_MASTER_BASE));
    data[0] = I2CMasterDataGet(I2C2_MASTER_BASE);
    if (length == 1)
    {
        I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        while(I2CMasterBusy(I2C2_MASTER_BASE));
    }
    else
    {
        int i;
        for (i = 1; i < length; i ++)
        {
            I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            while(I2CMasterBusy(I2C2_MASTER_BASE));
            data[i] = I2CMasterDataGet(I2C2_MASTER_BASE);
        }    
        I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        while(I2CMasterBusy(I2C2_MASTER_BASE));

    }
    

    I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(I2C2_MASTER_BASE));
    return length;
}

int8_t readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data)
{
    I2CMasterSlaveAddrSet(I2C2_MASTER_BASE, devAddr, false);
    
    I2CMasterDataPut(I2C2_MASTER_BASE, regAddr);
    I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C2_MASTER_BASE));

    I2CMasterSlaveAddrSet(I2C2_MASTER_BASE, regAddr, true);
    I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C2_MASTER_BASE));
    data[0] = ((uint8_t)I2CMasterDataGet(I2C2_MASTER_BASE)) << 8;

    I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(I2CMasterBusy(I2C2_MASTER_BASE));
    data[0] |= ((uint8_t)I2CMasterDataGet(I2C2_MASTER_BASE));

    int i;
    for (i = 1; i < length; i ++)
    {
        I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while(I2CMasterBusy(I2C2_MASTER_BASE));
        data[i] = ((uint8_t)I2CMasterDataGet(I2C2_MASTER_BASE)) << 8;

        I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while(I2CMasterBusy(I2C2_MASTER_BASE));
        data[i] |= ((uint8_t)I2CMasterDataGet(I2C2_MASTER_BASE));
    }    

    I2CMasterControl(I2C2_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(I2C2_MASTER_BASE));
    return length;

}

//Coppied from I2CDevlib

// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// 6/9/2012 by Jeff Rowberg <jeff@rowberg.net>
//
// Changelog:
//     2012-06-09 - fix major issue with reading > 32 bytes at a time with Arduino Wire
//                - add compiler warnings when using outdated or IDE or limited I2Cdev implementation
//     2011-11-01 - fix write*Bits mask calculation (thanks sasquatch @ Arduino forums)
//     2011-10-03 - added automatic Arduino version detection for ease of use
//     2011-10-02 - added Gene Knight's NBWire TwoWire class implementation with small modifications
//     2011-08-31 - added support for Arduino 1.0 Wire library (methods are different from 0.x)
//     2011-08-03 - added optional timeout parameter to read* methods to easily change from default
//     2011-08-02 - added support for 16-bit registers
//                - fixed incorrect Doxygen comments on some methods
//                - added timeout value for read operations (thanks mem @ Arduino forums)
//     2011-07-30 - changed read/write function structures to return success or byte counts
//                - made all methods static for multi-device memory savings
//     2011-07-28 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/



/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    uint8_t count = readByte(devAddr, regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}

/** Read a single bit from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data) {
    uint16_t b;
    uint8_t count = readWord(devAddr, regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = readByte(devAddr, regAddr, &b)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/** Read multiple bits from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
int8_t readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data) {
    // 1101011001101001 read byte
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    //    010           masked
    //           -> 010 shifted
    uint8_t count;
    uint16_t w;
    if ((count = readWord(devAddr, regAddr, &w)) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        w &= mask;
        w >>= (bitStart - length + 1);
        *data = w;
    }
    return count;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data) {
    return readBytes(devAddr, regAddr, 1, data);
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data) {
    return readWords(devAddr, regAddr, 1, data);
}


/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readByte(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, b);
}

/** write a single bit in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data) {
    uint16_t w;
    readWord(devAddr, regAddr, &w);
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return writeWord(devAddr, regAddr, w);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (readByte(devAddr, regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(devAddr, regAddr, b);
    } else {
        return false;
    }
}

/** Write multiple bits in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data) {
    //              010 value to write
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    // 0001110000000000 mask byte
    // 1010111110010110 original value (sample)
    // 1010001110010110 original & ~mask
    // 1010101110010110 masked | value
    uint16_t w;
    if (readWord(devAddr, regAddr, &w) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        w &= ~(mask); // zero all important bits in existing word
        w |= data; // combine data with existing word
        return writeWord(devAddr, regAddr, w);
    } else {
        return false;
    }
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    return writeBytes(devAddr, regAddr, 1, &data);
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
    return writeWords(devAddr, regAddr, 1, &data);
}




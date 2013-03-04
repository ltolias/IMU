
#include "buffereduart.h"




unsigned int bprint(const char *pcBuf, unsigned long ulLen)
{
     unsigned int uIdx;

  
    //
    // Send the characters
    //
    for(uIdx = 0; uIdx < ulLen; uIdx++)
    {
        //
        // If the character to the UART is \n, then add a \r before it so that
        // \n is translated to \n\r in the output.
        //
        if(pcBuf[uIdx] == '\n')
        {
            tx.buffer[tx.writeIndex] = '\r';
            tx.writeIndex ++;
            if (tx.writeIndex == TX_BUFF_SIZE) { tx.writeIndex = 0; tx.over = 1;}
            if (tx.writeIndex > tx.readIndex) tx.over = 0;
        }

        //
        // Send the character to the UART output.
        //
        tx.buffer[tx.writeIndex] = pcBuf[uIdx];
        tx.writeIndex ++;
        if (tx.writeIndex == TX_BUFF_SIZE) { tx.writeIndex = 0; tx.over = 1;}
        if (tx.writeIndex > tx.readIndex) tx.over = 0;
    }
    transferToUART();
    //
    // Return the number of characters written.
    //
    return(uIdx);
}


void bprints(const char *pcString)
{
    unsigned long ulIdx;
    
    for(ulIdx = 0; (pcString[ulIdx] != '\0'); ulIdx++) { };

        //
        // Write this portion of the string.
        //
    bprint(pcString, ulIdx);
}

int power(int a, int i)
{
    if (i == 0) return 1;

    int temp = 1;
    int r;
    for (r = i; r > 0; r --)
    {
        temp *= a;
    }
    return temp;
}



void bprintifw(int a, int width)
{

    if (a < 0) { bprint("-", 1); width --; a = -a; }
    int temp = a;
    int i = 0;
    //find msd
    for (i = 0; temp > 0; temp /= 10)
    {
        i ++;
    }
    if (i == 0) i ++;

    int r = 0;
    if (i < width)
    {
        for (r = 0; r < (width - i); r ++) bprint(" ", 1);
    }
    r = 0;
    while (i > 0 && r < width)
    {
        bprint(Int_Hex[(a/power(10,i-1)) % 10], 1);
        i --;
        r ++;
 
    } 
}
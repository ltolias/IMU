


#define TX_BUFF_SIZE 1024

typedef struct {
    unsigned char buffer[TX_BUFF_SIZE];
    int writeIndex;
    int readIndex;
    int over;
}UARTBuffer;

UARTBuffer rx;
UARTBuffer tx;



static const char *Int_Hex[10] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9"};


extern void transferToUART(void);

unsigned int bprint(const char *pcBuf, unsigned long ulLen);


void bprints(const char *pcString);

int power(int a, int i);



void bprintifw(int a, int width);
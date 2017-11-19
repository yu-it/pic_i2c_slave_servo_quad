#define	I2C_ADDR	0x13	// I2C 7bit address

#define RXBUFFER_SIZE 3
extern /*volatile*/ unsigned int RXBuffer[RXBUFFER_SIZE]; 
extern /*volatile*/ unsigned int RXBufferIndex;
extern /*volatile*/ int data_accepting_stat;
extern void (*i2c_handler)(unsigned int, unsigned int); //int com, int data


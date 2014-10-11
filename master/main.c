#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>


#define BaudRate 9600
#define MYUBRR ((F_CPU / 16UL / BaudRate ) - 1)


void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSRA & (1<<UDRE)) );
	/* Put data into buffer, sends the data */
	UDR = data;
}

void p(const char *data) {
  for(int i = 0; data[i] != '\0'; i++) {
    USART_Transmit(data[i]);
  }
}

void print(int number) {
	for(int i = 3; i >= 0; i--) {
		USART_Transmit((int)(number / pow(10, i))%10 + '0');
	}
}

#define F_SCL 100000L

void I2C_Init()
{
  TWSR = 0;
  TWBR = ((F_CPU/F_SCL)-16)/2;
}

//#define TW_START 0xA4 // send start condition (TWINT,TWSTA,TWEN)
#define TW_READY (TWCR & 0x80) // ready when TWINT returns to logic 1.
//#define TW_STATUS (TWSR & 0xF8) // returns value of status register

uint8_t I2C_Start()
// generate a TW start condition
{
 TWCR = TW_START; // send start condition
 while (!TW_READY); // wait
 return (TW_STATUS==0x08); // return 1 if found; 0 otherwise
}

#define DS1307 0xD0 // I2C bus address of DS1307 RTC
#define TW_SEND 0x84 // send data (TWINT,TWEN)
uint8_t I2C_SendAddr(uint8_t addr)
// send bus address of slave
{
 TWDR = addr; // load device's bus address
 TWCR = TW_SEND; // and send it
 while (!TW_READY); // wait
 return (TW_STATUS==0x18); // return 1 if found; 0 otherwise
}

uint8_t I2C_Write (uint8_t data) // sends a data uint8_t to slave
{
 TWDR = data; // load data to be sent
 TWCR = TW_SEND; // and send it
 while (!TW_READY); // wait
 return (TW_STATUS!=0x28); // return 1 if found; 0 otherwise
}

#define TW_STOP 0x94 // send stop condition (TWINT,TWSTO,TWEN)
#define I2C_Stop() TWCR = TW_STOP // inline macro for stop condition




uint8_t I2C_Detect(uint8_t addr)
// look for device at specified address; return 1=found, 0=not found
{
	uint8_t   twst;

TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

while(!(TWCR & (1<<TWINT)));

twst = TW_STATUS & 0xF8;
if ( (twst != TW_START) && (twst != TW_REP_START)) return 0;

 TWDR = addr; // load device's bus address
TWCR = (1<<TWINT) | (1<<TWEN);
while(!(TWCR & (1<<TWINT)));
twst = TW_STATUS & 0xF8;
if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 0;
 return 1; // return 1 if found; 0 otherwise
}

#define TW_NACK 0x84 // read data with NACK (last byte)
#define READ 1
uint8_t I2C_ReadNACK () // reads a data byte from slave
{
 TWCR = TW_NACK; // nack = not reading more data
 while (!TW_READY); // wait
 return TWDR;
}


void I2C_WriteRegister(uint8_t deviceRegister, uint8_t data)
{
 I2C_Start();
 I2C_SendAddr(DS1307); // send bus address
 I2C_Write(deviceRegister); // first uint8_t = device register address
 I2C_Write(data); // second uint8_t = data for device register
 I2C_Stop();
}
uint8_t I2C_ReadRegister(uint8_t deviceRegister)
{
 uint8_t data = 0;
 I2C_Start();
 I2C_SendAddr(DS1307); // send device bus address
 I2C_Write(deviceRegister); // set register pointer
 I2C_Start();
 I2C_SendAddr(DS1307+READ); // restart as a read operation
 data = I2C_ReadNACK(); // read the register data
 I2C_Stop(); // stop
 return data;
}

int main() {
  UBRRL = MYUBRR;
	UBRRH = (MYUBRR>>8);

	/* Enable receiver and transmitter   */
	UCSRB = (1<<RXEN)|(1<<TXEN);
	/* Frame format: 8data, No parity, 1stop bit */
	//UCSRC = (0<<USBS)|(1<<UCSZ1)|(1<<UCSZ0);
	UCSRC=(1<<URSEL)|(3<<UCSZ0);



	p("Master ready\r\n");



  I2C_Init();

// I2C_Detect
while(1) {
    p("\r\nScaning....\r\n");
    for(uint8_t i = 5; i < 127; i++) {
			  if(I2C_Detect(i<<1)) {
				TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);//TWI STOP
				while(TWCR & (1<<TWSTO));
          p("Found: ");
          print(i);
          p("\r\n");
        }

				_delay_ms(10);
     }

     //I2C_WriteRegister(0x00, 52);
		_delay_ms(1000);
	}




	return 0;
}

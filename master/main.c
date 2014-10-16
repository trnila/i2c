#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdio.h>


#define BaudRate 9600
#define MYUBRR ((F_CPU / 16UL / BaudRate ) - 1)


void USART_Transmit( unsigned char data ){
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

void print(const char *str, ...) {
	va_list args;

	char buffer[50];

	va_start(args, str);
	vsprintf(buffer, str, args);
	va_end(args);
	p(buffer);
}

#define F_SCL 100000UL // SCL frequency
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)

void I2C_init(void) {
	TWBR = TWBR_val;
}

uint8_t I2C_start(uint8_t address) {
	// reset TWI control register
	TWCR = 0;
	// transmit START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );

	// check if the start condition was successfully transmitted
	if(TW_STATUS != TW_START){ p("TW_START FAILED"); return 0; }

	// load slave address into data register
	TWDR = address;
	// start transmission of address
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );

	// check if the device has acknowledged the READ / WRITE mode
	if(TW_STATUS != TW_MT_SLA_ACK && TW_STATUS != TW_MR_SLA_ACK) {print(" I2C_Start: %X ", TW_STATUS); return 0;}

	return 1;
}

uint8_t I2C_write(uint8_t data){
	// load data into data register
	TWDR = data;
	// start transmission of data
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );

	if(TW_STATUS != TW_MT_DATA_ACK ){ print(" Write failed: %X ", TW_STATUS); }

	return 0;
}

/*uint8_t I2C_read_ack(void){

	// start TWI module and acknowledge data after reception
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	// return received data from TWDR
	return TWDR;
}*/

uint8_t I2C_read_nack(void){
	// start receiving without acknowledging reception
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	// return received data from TWDR
	return TWDR;
}

void I2C_stop(void){
	// transmit STOP condition
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

	//while(TWCR & (1<<TWSTO));
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

	I2C_init();
	_delay_ms(1000);

	p("\r\nScaning....");

	for(int addr = 0; addr < 128; addr++) {
		if(addr % 10 == 0) {
			print("\r\n");
		}

		if(I2C_start(addr << 1)) {
			print("%3d ", addr << 1);
			I2C_write(0);
			I2C_stop();
		} else {
			print("%3s ", "-");
		}
	}

	//
	#define ADDR 208
	#define REG_ADDR 0
	#define REG_DATA 18

	print("\r\n\n\nSetting clock...\r\n");

	// write some seconds, CH - MSB bit must be set to zero, to start clock oscilator
	if(!I2C_start(ADDR + TW_WRITE)) {
		print("I2CStart failed\r\n");
		return 1;
	}
	I2C_write(REG_ADDR); // 0 register contains seconds
	I2C_write(REG_DATA);
	I2C_stop();

	// read clock
	print("Clock set.\r\n");
	while(1) {
			_delay_ms(1000);
			if(!I2C_start(ADDR + TW_WRITE)) {
				print(" I2C_start write failed!!!\r\n");
				I2C_stop();
				continue;
			}
			I2C_write(REG_ADDR);

			if(!I2C_start(ADDR + TW_READ)) {
				print("I2C read failed!\r\n");
				I2C_stop();
				continue;
			}
			print("Received: %d\r\n", I2C_read_nack());
			I2C_stop();
	}

	return 0;
}

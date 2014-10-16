#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdio.h>
#include "avr/pgmspace.h"


#define BaudRate 9600
#define MYUBRR ((F_CPU / 16UL / BaudRate ) - 1)
#define DS1307_ADDR (0x68<<1)
#define I2C_WRITE   0
#define I2C_READ    1
#define F_SCL 100000UL // SCL frequency
#define SCL_CLOCK  10000L
const uint8_t ds1307_daysinmonth [] PROGMEM = { 31,28,31,30,31,30,31,31,30,31,30,31 };

void i2c_init(void)
{
/* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */

TWSR = 0;                         /* no prescaler */
TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */

}/* i2c_init */

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


#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)

void I2C_init(void) {
	TWSR = 0;
	TWBR = TWBR_val;
}

void i2c_start_wait(unsigned char address)
{
	uint8_t   twst;


	while ( 1 )
	{
		// send START condition
		TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

		// wait until transmission completed
		while(!(TWCR & (1<<TWINT)));

		// check value of TWI Status Register. Mask prescaler bits.
		twst = TW_STATUS & 0xF8;
		if ( (twst != TW_START) && (twst != TW_REP_START)) continue;

		// send device address
		TWDR = address;
		TWCR = (1<<TWINT) | (1<<TWEN);

		// wail until transmission completed
		while(!(TWCR & (1<<TWINT)));

		// check value of TWI Status Register. Mask prescaler bits.
		twst = TW_STATUS & 0xF8;
		if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) )
		{
			/* device busy, send stop condition to terminate write operation */
			TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

			// wait until stop condition is executed and bus released
			while(TWCR & (1<<TWSTO));

			continue;
		}
		//if( twst != TW_MT_SLA_ACK) return 1;
		break;
	}

}/* i2c_start_wait */

unsigned char i2c_start(unsigned char address)
{
	uint8_t   twst;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

	return 0;

}/* i2c_start */

unsigned char i2c_write( unsigned char data )
{
	uint8_t   twst;

	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return 1;
	return 0;

}


unsigned char i2c_rep_start(unsigned char address)
{
	return i2c_start( address );

}/* i2c_rep_start */

unsigned char i2c_readNak(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

	return TWDR;

}/* i2c_readNak */

void i2c_stop(void)
{
	/* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

	// wait until stop condition is executed and bus released
	while(TWCR & (1<<TWSTO));

}/* i2c_stop */

unsigned char i2c_readAck(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));

	return TWDR;

}
uint8_t ds1307_dec2bcd(uint8_t val) {
	return val + 6 * (val / 10);
}

/*
* transform bcd value to deciaml
*/
static uint8_t ds1307_bcd2dec(uint8_t val) {
	return val - 6 * (val >> 4);
}

/*
* get number of days since 2000/01/01 (valid for 2001..2099)
*/
static uint16_t ds1307_date2days(uint8_t y, uint8_t m, uint8_t d) {
	uint16_t days = d;
	for (uint8_t i = 1; i < m; ++i)
		days += pgm_read_byte(ds1307_daysinmonth + i - 1);
	if (m > 2 && y % 4 == 0)
		++days;
	return days + 365 * y + (y + 3) / 4 - 1;
}

/*
* get day of a week
*/
uint8_t ds1307_getdayofweek(uint8_t y, uint8_t m, uint8_t d) {
	uint16_t day = ds1307_date2days(y, m, d);
	return (day + 6) % 7;
}

/*
* set date
*/
uint8_t ds1307_setdate(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
	//sanitize data
	if (second < 0 || second > 59 ||
		minute < 0 || minute > 59 ||
		hour < 0 || hour > 23 ||
		day < 1 || day > 31 ||
		month < 1 || month > 12 ||
		year < 0 || year > 99)
		return 8;

	//sanitize day based on month
	if(day > pgm_read_byte(ds1307_daysinmonth + month - 1))
		return 0;

	//get day of week
	uint8_t dayofweek = ds1307_getdayofweek(year, month, day);

	//write date
	i2c_start_wait(DS1307_ADDR | I2C_WRITE);
	i2c_write(0x00);//stop oscillator
	i2c_write(ds1307_dec2bcd(second));
	i2c_write(ds1307_dec2bcd(minute));
	i2c_write(ds1307_dec2bcd(hour));
	i2c_write(ds1307_dec2bcd(dayofweek));
	i2c_write(ds1307_dec2bcd(day));
	i2c_write(ds1307_dec2bcd(month));
	i2c_write(ds1307_dec2bcd(year));
	i2c_write(0x00); //start oscillator
	i2c_stop();

	return 1;
}

/*
* get date
*/
void ds1307_getdate(uint8_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *minute, uint8_t *second) {
	i2c_start_wait(DS1307_ADDR | I2C_WRITE);
	i2c_write(0x00);//stop oscillator
	i2c_stop();

	i2c_rep_start(DS1307_ADDR | I2C_READ);
	*second = ds1307_bcd2dec(i2c_readAck() & 0x7F);
	*minute = ds1307_bcd2dec(i2c_readAck());
	*hour = ds1307_bcd2dec(i2c_readAck());
	i2c_readAck();
	*day = ds1307_bcd2dec(i2c_readAck());
	*month = ds1307_bcd2dec(i2c_readAck());
	*year = ds1307_bcd2dec(i2c_readNak());
	i2c_stop();
}


int main() {
	uint8_t year = 0;
	uint8_t month = 0;
	uint8_t day = 0;
	uint8_t hour = 0;
	uint8_t minute = 0;
	uint8_t second = 0;

	UBRRL = MYUBRR;
	UBRRH = (MYUBRR>>8);

	/* Enable receiver and transmitter   */
	UCSRB = (1<<RXEN)|(1<<TXEN);
	/* Frame format: 8data, No parity, 1stop bit */
	//UCSRC = (0<<USBS)|(1<<UCSZ1)|(1<<UCSZ0);
	UCSRC=(1<<URSEL)|(3<<UCSZ0);



	p("Master ready\r\n");

	i2c_init();
	_delay_ms(1000);

	ds1307_setdate(12, 12, 31, 23, 59, 35);

	while(1){
		ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
		char buf[50];
		sprintf(buf, "%d/%d/%d %d:%d:%d", year, month, day, hour, minute, second);
		p(buf);
		p("\r\n");
		_delay_ms(500);

	}

/*	p("\r\nScaning....");

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
	}*/



	return 0;
}

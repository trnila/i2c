#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

//#define TWI_ADDR 0xAF

unsigned char data;

ISR(TWI_vect){
	unsigned char TWI_status;
	TWI_status=TWSR & 0xF8; // get TWI status

	switch(TWI_status){
		case TW_SR_SLA_ACK:      // 0x60: SLA+W received, ACK returned
			TWCR |= (1<<TWINT);
			break;

		case TW_SR_DATA_ACK:     // 0x80: data received, ACK returned
			data = TWDR;
			TWCR |= (1<<TWINT);
			break;

		case TW_SR_STOP:         // 0xA0: stop or repeated start condition received while selected
			TWCR |= (1<<TWINT);
			break;

		case TW_ST_SLA_ACK:      // 0xA8: SLA+R received, ACK returned
		case TW_ST_DATA_ACK:     // 0xB8: data transmitted, ACK received
			TWDR = data;      // Store data in TWDR register
			TWCR |= (1<<TWINT);    // Clear TWINT Flag
			break;

		case TW_ST_DATA_NACK:    // 0xC0: data transmitted, NACK received
		case TW_ST_LAST_DATA:    // 0xC8: last data byte transmitted, ACK received
		case TW_BUS_ERROR:       // 0x00: illegal start or stop condition
		default:
			TWCR |= (1<<TWINT);    // Clear TWINT Flag
	}
}

int main(void){
	TWAR = 16 << 1; // set TWI address defined in header
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
	sei();
	while(1){
	}
	return 0;
}

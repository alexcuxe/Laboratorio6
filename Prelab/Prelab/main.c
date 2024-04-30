/*
 * Prueba.c
 *
 * Created: 23/04/2024 06:02:12 a. m.
 * Author : Eber Alexander
 */ 

#define F_CPU 16000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void initUART9600(void);
void writeUART(char singleChar);
//void writeText(char* text);

volatile char bufferRX;		//it is volatile 'cuase it could change anytime


int main(void)
{
	DDRD = 0xFC;		//PD2 - PD7 as output
	DDRB = 0x03;		//PB0 y PB1 as output
	
	initUART9600();
	sei();
	
	
	writeUART('H');
	writeUART('i');
	writeUART('\n');
	
	//writeText("hola putos\n");
	
    while (1) 
    {
		
    }
}

void initUART9600(void){
	//settigns for RX and TX
	DDRD &= ~(1 << DDD0);		//Rx as input
	DDRD |= (1 << DDD1);		//TX as output
	
	//Fast mode, U2X0
	UCSR0A = 0;
	UCSR0A |= (1 << U2X0);
	
	//Settigns for register B
	UCSR0B = 0;
	UCSR0B |= (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0); //ISR, enable for RX and TX
	
	// settigns for register C
	UCSR0C = 0;
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);		// character size: 8 bits, no parity, 1 stop bit
	
	//Baudrate
	UBRR0 = 207;		// 9600
}


void writeUART(char singleChar){
	//if buffer is emptym, if it is not, it waits
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = singleChar;
	
}

/*
void writeText(char* text){
	//uint8_t i;
	for (uint8_t i = 0; text[i] != '\0'; i++)
	{
		while(!(UCSR0A & (1 << UDRE0)));
		UDR0 = text[i];
	}
}
*/

//ISR, recieve
ISR(USART_RX_vect){
	bufferRX = UDR0;
	
	//if buffer is emptym, if it is not, it waits
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = bufferRX;
	PORTD = bufferRX;
	PORTB = 0x03 & bufferRX;
}
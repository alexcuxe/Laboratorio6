/*
 * Postlab.c
 *
 * Created: 23/04/2024 06:02:12 a. m.
 * Author : Eber Alexander
 */ 

#define F_CPU 16000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>


void initUART9600(void);
void initADC(void);
void writeText(char* text);
void menu(void);

volatile char bufferRX;		//it is volatile 'cuase it could change anytime
uint8_t option = 0;
uint8_t convertedBuffer = 0;
uint8_t sendChar = 0;

int main(void)
{
	cli();
	DDRD = 0xFC;		//PD2 - PD7 as output
	DDRB = 0x03;		//PB0 y PB1 as output
	DDRC &= ~(1 << PINC0);	//pc0 as input
	
	initUART9600();
	initADC();
	sei();

	menu();
	
    while (1) 
    {
		if (option == 1){
			ADCSRA |= (1 << ADSC);
			PORTD = ADCH;
			PORTB = 0x03 & ADCH;
		}else{
			option = 0;
		}
    }
}


//*****************************************************************************
//                                 Funciones
//*****************************************************************************


void menu(void){
	writeText("\n\n\n   *** MENU ***\n");
	writeText("1. Leer potenciómetro\n");
	writeText("2. Enviar ASCII\n");
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


void writeText(char* text){
	//uint8_t i;
	for (uint8_t i = 0; text[i] != '\0'; i++)
	{
		while(!(UCSR0A & (1 << UDRE0)));
		UDR0 = text[i];
	}
}


//ISR, recieve
ISR(USART_RX_vect){
	bufferRX = UDR0;
	
	//if buffer is emptym, if it is not, it waits
	while(!(UCSR0A & (1 << UDRE0)));
	/*UDR0 = bufferRX;
	PORTD = bufferRX;
	PORTB = 0x03 & bufferRX;*/
	char copyBuffer = bufferRX;
	
	convertedBuffer = atoi(&copyBuffer);
	
	if (sendChar != 1){
		switch (convertedBuffer){
			case 1:
				option = 1;
				UDR0 = bufferRX;
				writeText(" -> Caracter enviado");
				writeText("\n\n");
				menu();
				break;
			
			case 2:
				option = 0;
				sendChar = 1;
				UDR0 = bufferRX;
				writeText(" -> Caracter enviado");
				writeText("\nEnvie un caracter...");
				writeText("\n\n");
				PORTD = 0;
				PORTB = 0;
				break;
			
			default:
			option = 0;
		}
		
	}else {
		sendChar = 0;
		UDR0 = bufferRX;
		PORTD = bufferRX;
		PORTB = 0x03 & bufferRX;
		writeText(" -> Caracter mostrado");
		menu();
	}
}

//*****************************************************************************
//                                     ADC
//*****************************************************************************

void initADC(void){
	ADMUX = 0;
	//Vref = AVcc = 5Vs
	ADMUX |= (1 << REFS0);
	ADMUX &= ~(1 << REFS1);
	
	ADMUX |= (1 << ADLAR);	//left adjust
	
	ADCSRA = 0;
	ADCSRA |= (1 << ADEN);	//turn on ADC
	ADCSRA |= (1 << ADIE);	//interruption
	
	//prescaler 128 > 125kHz
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	
	DIDR0 |= (1 << ADC0D);	//disable PC0 digital input
}

ISR (ADC_vect){
	//PORTD = ADCH;			//show in portd value of adc
	ADCSRA |= (1 << ADIF);	//turn off flag
}

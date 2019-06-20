//exam ultimate shorthand
/*
 * EIE3105_exam.c
 *
 * Created: 12/4/2017 12:51:11 AM
 * Author : CenzSA12
 */ 

#define F_CPU 16000000UL  // 16 MHz for ATmega328p, UL stand for unsigned integer
#define prescaler 256
#define F_CPU_prescaler 16000000UL/prescaler

#include <avr/io.h>
#include <util/delay.h>
#include "avr/interrupt.h"

typedef unsigned char BYTE; //8bit 0-255
typedef unsigned int BYTE2;	//16bit 0-65535
typedef unsigned long BYTE4;//32bit 0-4294967295
typedef enum { FALSE, TRUE } bool; // 0 false, 1 true
//Marcos of register

#define SETBIT(ADDRESS,BIT)		(ADDRESS |= (1<<BIT)) //Set the bit to one left other untouch
#define CLEARBIT(ADDRESS,BIT)	(ADDRESS &= ~(1<<BIT))//Clear the bit to zero left other untouch
#define TOGGLEBIT(ADDRESS,BIT)	(ADDRESS ^= (1<<BIT)) //toggle the bit in address
#define CHECKBIT(ADDRESS,BIT)	(ADDRESS & (1<<BIT))	//check if 0 = false, 1 = true
#define InterruptEn()			sei()
#define InterruptStop()			cli()

//ATMEGA328P
//Marco: Short Hand
//8 bit timer/counter0 with PWM==========
#define TC01Prescaler0 0x01
#define TC01Prescaler8 0x02
#define TC01Prescaler64  0x03
#define TC01Prescaler256 0x04
#define TC01Prescaler1024 0x05
#define TC01CounterFalling 0x06
#define TC01CounterRising 0x07

#define TC0Stop()		(TCCR0B &= ~((1<<CS02)|(1<<CS01)|(1<<CS00)))	//close the clock
#define TC0En(mode)		(TCCR0B |= mode)

#define TC0ModeNormal(_TCNT0) {\
	TCNT0 = _TCNT0;\
	TCCR0A &= ~((1<<WGM01)|(1<<WGM00));\
}
#define TC0ModeCTC(_TCNT0,_OCR0A){\
	TCNT0 = _TCNT0;\
	OCR0A = _OCR0A;\
	TCCR0A |= (1<<WGM01);\
}

#define TC0ClearOverflowFlag()	(SETBIT(TIFR0,TOV0))
#define TC0ClearCompareMatchFlag() (SETBIT(TIFR0,OCF0A))
//interrupt
#define TC0INTModeNormalEn()		SETBIT(TIMSK0,TOIE0)
#define TC0INTModeCTCEn()			SETBIT(TIMSK0,OCIE0A)
#define TC0INTModeCTCStop()			CLEARBIT(TIMSK0,OCIE0A)
#define TC0INTModeNormalStop()		CLEARBIT(TIMSK0,TOIE0)

//Timer1=====================================
#define TC1ModeCTC(_TCNT1H,_TCNT1L,_OCR1AH,_OCR1AL){\
	SETBIT(TCCR1B,WGM12);\
	TCNT1 = (_TCNT1H << 8) + _TCNT1L;\
	OCR1A = (_OCR1AH << 8) + _OCR1AL;\
}
#define TC1ModeNormal(_TCNT1H,_TCNT1L){\
	TCNT1 = (_TCNT1H << 8) + _TCNT1L;\
}

#define TC1En(mode)		(TCCR1B |= mode)
#define TC1Stop()		(TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10)))

#define TC1ClearOverflowFlag()	(SETBIT(TIFR1,TOV1))
#define TC1ClearCompareMatchFlag() (SETBIT(TIFR1,OCF1A))

#define TC1INTModeNormalEn()		SETBIT(TIMSK1,TOIE1)
#define TC1INTModeCTCEn()			SETBIT(TIMSK1,OCIE1A)
#define TC1INTModeCTCStop()			CLEARBIT(TIMSK1,OCIE1A)
#define TC1INTModeNormalStop()		CLEARBIT(TIMSK1,TOIE1)

//Timer2=======================
#define TC2prescaler0 1
#define TC2prescaler8 2
#define TC2prescaler32 3
#define TC2prescaler64 4
#define TC2prescaler128 5
#define TC2prescaler256 6
#define TC2prescaler1024 7

#define TC2Stop()		(TCCR2B &= ~((1<<CS22)|(1<<CS21)|(1<<CS20)))	//close the clock
#define TC2En(prescaler)		(TCCR2B |= prescaler)

#define TC2ModeNormal(_TCNT2) {\
	TCNT2 = _TCNT2;\
	TCCR2A &= ~((1<<WGM21)|(1<<WGM20));\
}
#define TC2ModeCTC(_TCNT2,_OCR2A){\
	TCNT2 = _TCNT2;\
	OCR2A = _OCR2A;\
	TCCR2A |= (1<<WGM21);\
}

#define TC2ClearOverflowFlag()	(SETBIT(TIFR2,TOV2))
#define TC2ClearCompareMatchFlag() (SETBIT(TIFR2,OCF2A))
//interrupt
#define TC2INTModeNormalEn()		SETBIT(TIMSK2,TOIE2)
#define TC2INTModeCTCEn()			SETBIT(TIMSK2,OCIE2A)
#define TC2INTModeCTCStop()			CLEARBIT(TIMSK2,OCIE2A)
#define TC2INTModeNormalStop()		CLEARBIT(TIMSK2,TOIE2)

//external interrupt
#define ExtIntLevelLow	0
#define ExtIntEdgeFalling 2
#define ExtIntEdgerising 3
#define ExtIntLogicChange 1
#define ExtIntx(INTx,Mode) {\
	SETBIT(PORTD,(INTx+2)); \
	SETBIT(EIMSK,INTx);\
	EICRA = (Mode << (INTx*2));\
}

#define setBaudRate(_BAUD,_double){\
	UCSR0A |= (_double << U2X0);\
	UBRR0 = (F_CPU/(16UL* _BAUD /( _double +1)))-1;\
}

#define BufferisEmpty CHECKBIT(UCSR0A,UDRE0)
#define RxisComplete CHECKBIT(UCSR0A,RXC0)
#define TxisComplete CHECKBIT(UCSR0B,TXC0)

//8bits
//1 stop bit
//no parity check
#define USARTEn(){\
	UCSR0B |= (1<<TXEN0)|(1<<RXEN0);\
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);\
}

#define USARTEnINT() (UCSR0B |= (1<<RXCIE0))


void usart_send(char);
char usart_receive(void);

void usart_send_string(char* _str);
void usart_receive_string(char* _str);

void usart_send(char ch){
	while (!BufferisEmpty);
	UDR0 = ch;
}

char usart_receive(void){
	while(!RxisComplete);	//wait until the receive complete
	return UDR0;
};

void usart_send_string(char* _str){
	for (BYTE i = 0; i < strlen(_str);i++)
	{
		usart_send(_str[i]);
	}
}

void usart_receive_string(char* _str){
	char ch[255] = {0};
	strcpy(_str,ch);
	//usart_send_string(_str);
	char temp = 0;
	BYTE i = 0;

	for (i = 0; temp != '\n' ; i++)
	{
		temp = usart_receive();
		ch[i] = temp;
	}
	//usart_send_string(ch);
	strcpy(_str,ch);
}


//====================================
typedef struct
{
	BYTE2 duty_Cycle;
	BYTE2 out_Freq;
}PWM;

#define FPWMEn(_timeEn){\
	TCCR0A |= (1<<COM0A1)|(1<<COM0B1);\
	TCCR0A |= (1<<WGM00)|(1<<WGM01);\
	TCCR0B |= (1<<WGM02);\
	DDRD |= (1 << DDD5);\
	TC0En(_timeEn);\
}



void FPWMSetup(PWM pwm){
	OCR0A = F_CPU_prescaler/pwm.out_Freq - 1;
	OCR0B = ((OCR0A+1)*pwm.duty_Cycle)/100 -1;
}

void PWMinputCapSetup(){
	CLEARBIT(PORTB,PORTB0);
	SETBIT(PORTB,PORTB0);
	TC1ModeNormal(0,0);
}

BYTE2 PWMinputCapturePW(){
	SETBIT(TCCR1B, ICES1);
	TC1En(TC01Prescaler256);

	SETBIT(TIFR1,ICF1);
	while (!CHECKBIT(TIFR1,ICF1));
	BYTE2 t1 = ICR1;
	
	SETBIT(TIFR1,ICF1);
	CLEARBIT(TCCR1B, ICES1);
	while (!CHECKBIT(TIFR1,ICF1));
	BYTE2 _return = ICR1 - t1;
	
	SETBIT(TIFR1,ICF1);
	while (!CHECKBIT(TIFR1,ICF1));
	return _return;
	
}

BYTE2 PWMinputCapturePeriod(){
	SETBIT(TCCR1B, ICES1);
	TC1En(TC01Prescaler256);
	
	SETBIT(TIFR1,ICF1);
	while (!CHECKBIT(TIFR1,ICF1));
	BYTE2 t1 = ICR1;
	
	SETBIT(TIFR1,ICF1);
	while (!CHECKBIT(TIFR1,ICF1));
	BYTE2 _return = ICR1 - t1;
	
	SETBIT(TIFR1,ICF1);
	while (!CHECKBIT(TIFR1,ICF1));
	return _return;
}
//=========Timer1=============
#define FPWMTC1En(_timeEn){\
	TCCR1A |= (1<<COM1A1)|(1<<COM1B1);\
	TCCR1A |= (1<<WGM10)|(1<<WGM11);\
	TCCR1B |= (1<<WGM12);\
	DDRB |= (1 << DDB2);\
	TC1En(_timeEn);\
}



void FPWMTC1Setup(PWM pwm){
	OCR1A = F_CPU_prescaler/pwm.out_Freq - 1;
	OCR1B = ((OCR1A+1)*pwm.duty_Cycle)/100 -1;
}

//====================================
//ADCCC
#define ADC0 0
#define ADC1 1
#define ADC2 2
#define ADC3 3
#define ADC4 4
#define ADC5 5
#define ADC6 6
#define ADC7 7


#define ADCinit(){\
	SETBIT(ADMUX,REFS0);\
	SETBIT(ADCSRA,ADEN);\
	ADCSRA |= (1<<ADPS0)|(1<<ADPS1);\
}

#define ADCintEn(){\
	ADCSRA |= (1<<ADIE);\
	sei();\
}



#define ADCStartConv(ADCx){\
	ADMUX &= ~((1<<MUX0)|(1<<MUX1)|(1<<MUX2)|(1<<MUX3));\
	ADMUX |= ADCx;\
	SETBIT(ADCSRA ,ADSC);\
}


#define ADCLAdj(){\
	SETBIT(ADMUX,ADLAR);\
}

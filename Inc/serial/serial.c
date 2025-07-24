//Example ATmega2560 Project
//File: serial.h
//Author: Robert Howie
//Created: 2 March 2015
//V1.0 Basic serial setup for printing to the serial terminal

//Derived from: http://www.github.com/abcminiuser/avr-tutorials/blob/master/USART/Output/USART.pdf?raw=true by Dean Camera
//See http://www.fourwalledcubicle.com/AVRArticles.php for more

#include "serial.h"

// void serial_init(void)
// {
// 	UCSR0B = (1<<RXEN0)|(1<<TXEN0); //turn on the transmission and reception circuitry
// 	UCSR0C = (1<<UCSZ00)|(1<<UCSZ01); //use 8- bit character sizes
// 	UBRR0 = BAUD_PRESCALE; //load baud rate prescaler into register
// } //end serial2_init

void serial_print_string(char * string_pointer) //function to print a string via serial
{
	while(*string_pointer) //while not null character (end of string)
	{
		while((UCSR0A&(1<<UDRE0))==0){} //wait until data register is ready
		UDR0 = *string_pointer; //send what's at the string pointer to serial data register
		string_pointer++; //increment string pointer to go to next letter in string
	}
} //end serial0_print_string

void serial_write_byte(uint8_t data_byte)
{
	while((UCSR0A&(1<<UDRE0))==0){} //wait until data register is ready
	UDR0 = data_byte;	
}
/*
 * Serial.h
 *
 * Created: 2/03/2015 5:41:43 PM
 *  Author: Robert
 */ 


#ifndef SERIAL_H_
#define SERIAL_H_


//function prototypes
// void serial_init(void);
void serial_print_string(char * string_pointer);
void serial_write_byte(uint8_t data_byte);

#endif /* SERIAL_H_ */

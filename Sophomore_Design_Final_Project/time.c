#include <stdint.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h>
#include <stdlib.h>
#include "lcd_driver.h"
#include "port_macros.h"
#include "lcd_driver.h"


int main(){


	//initialization of button variables
	initialize_LCD_driver();


	unsigned int last_left_button_state = (PINB & (1<<1));
	unsigned int left_button_pressed=0;
	unsigned int last_middle_button_state = (PINB & (1<<4));
	unsigned int middle_button_pressed=0;
	unsigned int last_right_button_state = (PINB & (1<<5));
	unsigned int right_button_pressed=0;

	//initialization of direction variables
	//char direction1[3] = " ";
	//char speed1[3] = " ";
	int time1 = 500;


	//Configure ports
	//configure motors
	DDRD &= ~(1<<5);

	DDRD &= ~(1<<6);

	DDRD &= ~(1<<3);

	DDRB &= ~(1<<3);

	//Configure left push-button
	DDRB &= ~(1<<1); //configure pin as input
	PORTB |= (1<<1); //enable pull-up resistor

	//Configure middle push-button
	DDRB &= ~(1<<4); //configure pin as input
	PORTB |= (1<<4); //enable pull-up resistor

	//Configure right push-button
	DDRB &= ~(1<<5); //configure pin as input
	PORTB |= (1<<5); //enable pull-up resistor

	while(1){

		//Pulser for left button
		//Left button = down button
		if( (PINB & (1<<1)) != last_left_button_state ){
			if( (PINB & (1<<1)) == 0 ){
				left_button_pressed=1;
			}
			last_left_button_state = (PINB & (1<<1));
		}
		else{
			left_button_pressed=0;
		}

		//Pulser for middle button
		//Middle button = up button
		if( (PINB & (1<<4)) != last_middle_button_state ){
			if( (PINB & (1<<4)) == 0 ){
				middle_button_pressed=1;
			}
			last_middle_button_state = (PINB & (1<<4));
		}
		else{
			middle_button_pressed=0;
		}

		//Pulser for right button
		//Right button = select
		if( (PINB & (1<<5)) != last_right_button_state ){
			if( (PINB & (1<<5)) == 0 ){
				right_button_pressed=1;
			}
			last_right_button_state = (PINB & (1<<5));
		}
		else{
			right_button_pressed=0;
		}

		//Choosing time1
		time1 = time_set(time1, middle_button_pressed,left_button_pressed);

		}
	return 0;
}


int time_set(int time, int increment, int decrement){
//Choosing time
char timestring[100];
//if(right_button_pressed != 1){
	if( decrement == 1 ){
		//decrement time by 0.1 when left button pressed
		if( time >= 100 ){
			time = time - 100;
		}
	}
	else if( increment == 1 ){
		//increment time by 0.1 when middle button pressed
		time = time + 100;
	}

	LCD_execute_command(TURN_ON_DISPLAY);
	LCD_execute_command(CLEAR_DISPLAY);
	itoa(time,timestring,10);
	LCD_print_String(timestring);

	_delay_ms(50);
	return time;

}

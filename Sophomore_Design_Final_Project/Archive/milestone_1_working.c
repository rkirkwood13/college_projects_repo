#include <stdint.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h>
#include <stdlib.h>
#include "lcd_driver.h"
#include "port_macros.h"
#include "lcd_driver.h"

#define SLOW 15
#define MEDIUM 40
#define FAST 100

#define CW 1
#define CCW 2
#define F 3
#define R 4

#define LEFT (PINB & (1<<1))
#define MIDDLE (PINB & (1<<4))
#define RIGHT (PINB & (1<<5))

#define PWM_TOP 100
#define TIME_STEP 100

#define DELAY 100 //microseconds

int main(){

	//initialization of LCD display
	initialize_LCD_driver();

	//unsigned int last_left_button_state = (PINB & (1<<1));
	unsigned int last_left_button_state = LEFT;
	unsigned int last_middle_button_state = MIDDLE;
	unsigned int last_right_button_state = RIGHT;
	unsigned int left_button_pressed = 0, middle_button_pressed = 0, right_button_pressed = 0;

	unsigned int right_button_counter = 0;
	unsigned int time_counter = 0;
	unsigned int command_counter = 0;

	//initialization of direction, speed and time variables
	int direction1 = CW, direction2 = CW, direction3 = CW, direction4 = CW;
	int speed1 = SLOW, speed2 = SLOW, speed3 = SLOW, speed4 = SLOW;
	int time1 = 500, time2 = 500, time3 = 500, time4 = 500;

	unsigned int pwm_counter_m1 = 0;
	unsigned int pwm_counter_m2 = 0;

	//Configure ports and motors
	initialize_motors();
	initialize_buttons();

	while(1){

		//Pulsers for left, middle and right buttons
		//left = down, middle = up, right = select
		left_button_pressed = button_debouncer(LEFT, left_button_pressed, last_left_button_state);
		last_left_button_state = button_debouncer_state(LEFT, last_left_button_state);

		middle_button_pressed = button_debouncer(MIDDLE, middle_button_pressed, last_middle_button_state);
		last_middle_button_state = button_debouncer_state(MIDDLE, last_middle_button_state);

		right_button_pressed = button_debouncer(RIGHT, right_button_pressed, last_right_button_state);
		last_right_button_state = button_debouncer_state(RIGHT, last_right_button_state);

		//counter for right button to select commands
		if(right_button_pressed == 1){
			right_button_counter++;
		}

		//command select for time, speed, and direction
		switch(right_button_counter) {
			case 0:
				time1 = time_set(time1, middle_button_pressed, left_button_pressed);
				break;
			case 1:
				speed1 = speed_set(speed1, middle_button_pressed, left_button_pressed);
				break;
			case 2:
				direction1 = direction_set(direction1, middle_button_pressed, left_button_pressed);
				break;
			case 3:
				time2 = time_set(time2, middle_button_pressed, left_button_pressed);
				break;
			case 4:
				speed2 = speed_set(speed2, middle_button_pressed, left_button_pressed);
				break;
			case 5:
				direction2 = direction_set(direction2, middle_button_pressed, left_button_pressed);
				break;
			case 6:
				time3 = time_set(time3, middle_button_pressed, left_button_pressed);
				break;
			case 7:
				speed3 = speed_set(speed3, middle_button_pressed, left_button_pressed);
				break;
			case 8:
				direction3 = direction_set(direction3, middle_button_pressed, left_button_pressed);
				break;
			case 9:
				time4 = time_set(time4, middle_button_pressed, left_button_pressed);
				break;
			case 10:
				speed4 = speed_set(speed4, middle_button_pressed, left_button_pressed);
				break;
			case 11:
				direction4 = direction_set(direction4, middle_button_pressed, left_button_pressed);
				break;
			case 12: //case to run the commands
				switch(command_counter){ //counts what command we are on, increases when the time of each command has passed
				case 0: //delay before first command is executed
						pwm_counter_m1 = m1_forwards(pwm_counter_m1, 0); //set both motors to speed 0
						pwm_counter_m2 = m2_forwards(pwm_counter_m2, 0);
						time_counter = time_counter + 1; //time_counter is increase after each loop, checking the time elapsed
						if(time_counter >= ( 1000 * (DELAY/10) )){
							command_counter = command_counter + 1;
							time_counter= 0;
							pwm_counter_m1 = 0;
							pwm_counter_m2 = 0;
						}
				case 1: //first command run
					pwm_counter_m1 = m1_direction(pwm_counter_m1, direction1, speed1);
					pwm_counter_m2 = m2_direction(pwm_counter_m2, direction1, speed1);
					/*
					switch(direction1){
					case 1://clockwise
							pwm_counter_m1 = m1_backwards(pwm_counter_m1, speed1);
							pwm_counter_m2 = m2_forwards(pwm_counter_m2, speed1);
							break;
					case 2: //counterclockwise
							pwm_counter_m1 = m1_forwards(pwm_counter_m1, speed1);
							pwm_counter_m2 = m2_backwards(pwm_counter_m2, speed1);
							break;
					case 3: //Forward
							pwm_counter_m1 = m1_forwards(pwm_counter_m1, speed1);
							pwm_counter_m2 = m2_forwards(pwm_counter_m2, speed1);
							break;
					case 4: //Reverse
							pwm_counter_m1 = m1_backwards(pwm_counter_m1, speed1);
							pwm_counter_m2 = m2_backwards(pwm_counter_m2, speed1);
							break;
					}
					*/
					time_counter = time_counter + 1; //time_counter is increase after each loop, checking the time elapsed
					if(time_counter >= ( time1 * (DELAY/10) )){
						command_counter = command_counter + 1;
						time_counter= 0;
						pwm_counter_m1 = 0;
						pwm_counter_m2 = 0;
					}
					break;
				case 2: //second command run
				switch(direction2){
					case 1://clockwise
							pwm_counter_m1 = m1_backwards(pwm_counter_m1, speed2);
							pwm_counter_m2 = m2_forwards(pwm_counter_m2, speed2);
							break;
					case 2: //counterclockwise
							pwm_counter_m1 = m1_forwards(pwm_counter_m1, speed2);
							pwm_counter_m2 = m2_backwards(pwm_counter_m2, speed2);
							break;
					case 3: //Forward
							pwm_counter_m1 = m1_forwards(pwm_counter_m1, speed2);
							pwm_counter_m2 = m2_forwards(pwm_counter_m2, speed2);
							break;
					case 4: //Reverse
							pwm_counter_m1 = m1_backwards(pwm_counter_m1, speed2);
							pwm_counter_m2 = m2_backwards(pwm_counter_m2, speed2);
							break;
					}
					time_counter= time_counter + 1;
					if(time_counter >= ( time2 * (DELAY/10) )){
						command_counter = command_counter + 1;
						time_counter= 0;
						pwm_counter_m1 = 0;
						pwm_counter_m2 = 0;
					}
					break;
					case 3: //third command run
						switch(direction3){
						case 1://clockwise
								pwm_counter_m1 = m1_backwards(pwm_counter_m1, speed3);
								pwm_counter_m2 = m2_forwards(pwm_counter_m2, speed3);
								break;
						case 2: //counterclockwise
								pwm_counter_m1 = m1_forwards(pwm_counter_m1, speed3);
								pwm_counter_m2 = m2_backwards(pwm_counter_m2, speed3);
								break;
						case 3: //Forward
								pwm_counter_m1 = m1_forwards(pwm_counter_m1, speed3);
								pwm_counter_m2 = m2_forwards(pwm_counter_m2, speed3);
								break;
						case 4: //Reverse
								pwm_counter_m1 = m1_backwards(pwm_counter_m1, speed3);
								pwm_counter_m2 = m2_backwards(pwm_counter_m2, speed3);
								break;
						}
						time_counter= time_counter + 1;
						if(time_counter >= ( time3 * (DELAY/10) )){
							command_counter = command_counter + 1;
							time_counter= 0;
							pwm_counter_m1 = 0;
							pwm_counter_m2 = 0;
						}
						break;
					case 4: //fourth command run
						switch(direction4){
							case 1://clockwise
									pwm_counter_m1 = m1_backwards(pwm_counter_m1, speed4);
									pwm_counter_m2 = m2_forwards(pwm_counter_m2, speed4);
									break;
							case 2: //counterclockwise
									pwm_counter_m1 = m1_forwards(pwm_counter_m1, speed4);
									pwm_counter_m2 = m2_backwards(pwm_counter_m2, speed4);
									break;
							case 3: //Forward
									pwm_counter_m1 = m1_forwards(pwm_counter_m1, speed4);
									pwm_counter_m2 = m2_forwards(pwm_counter_m2, speed4);
									break;
							case 4: //Reverse
									pwm_counter_m1 = m1_backwards(pwm_counter_m1, speed4);
									pwm_counter_m2 = m2_backwards(pwm_counter_m2, speed4);
									break;
							}
						time_counter= time_counter + 1;
						if(time_counter >= ( time4 * (DELAY/10) )){
								command_counter = command_counter + 1;
								time_counter = 0;
								pwm_counter_m1 = 0;
								pwm_counter_m2 = 0;
							}
						break;
					case 5: //stop after fourth command is executed
							pwm_counter_m1 = m1_forwards(pwm_counter_m1, 0); //set both motors to speed 0
							pwm_counter_m2 = m2_forwards(pwm_counter_m2, 0);
						break;
					default:
						break;
						}
			default:
				break;
		}
	_delay_us(DELAY);
	}
	return 0;
}

//Function to select time
int time_set(int time, int increment, int decrement){
	char timestring[100];
	if( decrement == 1 ){ //decrement time by 0.1 when left button pressed
		if( time >= TIME_STEP ){
			time = time - TIME_STEP;
		}
	}
	else if( increment == 1 ){ //increment time by 0.1 when middle button pressed
		time = time + TIME_STEP;
	}

	LCD_execute_command(TURN_ON_DISPLAY);
	LCD_execute_command(CLEAR_DISPLAY);
	itoa(time,timestring,10); //Cast time to a string and display on LCD screen
	LCD_print_String(timestring);

	_delay_ms(50);
	return time;
}

//Function to select speed
int speed_set(int speed, int increment, int decrement){
	if( decrement == 1 ){ //decrease speed when left button pressed
		if( speed == MEDIUM ){
			speed = SLOW;
		}
		else if( speed == FAST ){
			speed = MEDIUM;
		}
	}
	else if( increment == 1 ){ //increase speed when right button pressed
		if( speed == MEDIUM ){
			speed = FAST;
		}
		else if( speed == SLOW ){
			speed = MEDIUM;
		}
	}

	LCD_execute_command(TURN_ON_DISPLAY);
	LCD_execute_command(CLEAR_DISPLAY);

	if(speed == SLOW){ //print slow, medium, or fast on the LCD when alternating between different speed choices
		LCD_print_String("slow");
	}
	else if(speed == MEDIUM){
		LCD_print_String("medium");
	}
	else if(speed == FAST){
		LCD_print_String("fast");
	}

	_delay_ms(50);
	return speed;
}

//Function to select direction
int direction_set(int direction, int increment, int decrement){
	if( decrement == CW ){ //decrease direction number when left button pressed
		if( direction >= CCW){
			direction--;
		}
	}
	else if( increment == CW ){ //increase direction number when right button pressed
		if( direction <= F){
			direction++;
		}
	}

	LCD_execute_command(TURN_ON_DISPLAY);
	LCD_execute_command(CLEAR_DISPLAY);

	if(direction == CW){ //print CW, CCW, F, or R on the LCD when alternating between different direction choices
		LCD_print_String("CW");
	}
	else if( direction == CCW){
		LCD_print_String("CCW");
	}
	else if( direction == F){
		LCD_print_String("F");
	}
	else if(direction == R){
		LCD_print_String("R");
	}
	_delay_ms(50);
	return direction;
}

int m1_direction(int pwm_counter_m1, int direction, int speed){
  switch(direction){
    case 1://clockwise
        pwm_counter_m1 = m1_backwards(pwm_counter_m1, speed);
        break;
    case 2: //counterclockwise
        pwm_counter_m1 = m1_forwards(pwm_counter_m1, speed);
        break;
    case 3: //Forward
        pwm_counter_m1 = m1_forwards(pwm_counter_m1, speed);
        break;
    case 4: //Reverse
        pwm_counter_m1 = m1_backwards(pwm_counter_m1, speed);
        break;
    default:
        break;
    }
    return pwm_counter_m1;
}

int m2_direction(int pwm_counter_m2, int direction, int speed){
  switch(direction){
    case 1://clockwise
        pwm_counter_m2 = m2_forwards(pwm_counter_m2, speed);
        break;
    case 2: //counterclockwise
        pwm_counter_m2 = m2_backwards(pwm_counter_m2, speed);
        break;
    case 3: //Forward
        pwm_counter_m2 = m2_forwards(pwm_counter_m2, speed);
        break;
    case 4: //Reverse
        pwm_counter_m2 = m2_backwards(pwm_counter_m2, speed);
        break;
    default:
        break;
    }
    return pwm_counter_m2;
}

//MOTOR 1 FORWARDS FUNCTION
int m1_forwards(int pwm_counter, int duty_cycle){
	pwm_counter = pwm_counter + 1;
	if( pwm_counter >= PWM_TOP ){
		pwm_counter = 0;
	}
	//motor 1 forwards
	if( pwm_counter < duty_cycle ){
		PORTD |= (1<<5); //Turn on PD5 for M1
		PORTD &= ~(1<<6); //Turn off PD6 for M1
	}
	else{
		PORTD &= ~(1<<5); //Turn off PD5 for M1
		PORTD &= ~(1<<6);
	}
	return pwm_counter;
}

//MOTOR 2 FORWARDS FUNCTION
int m2_forwards(int pwm_counter, int duty_cycle){
	pwm_counter = pwm_counter + 1;
	if( pwm_counter >= PWM_TOP ){
		pwm_counter = 0;
	}
	//motor 2 forwards
	if( pwm_counter < duty_cycle ){
		PORTD |= (1<<3); //Turn on PD3 for M2
		PORTB &= ~(1<<3);
	}
	else{
		PORTD &= ~(1<<3); //Turn off PD3 for M2
		PORTB &= ~(1<<3);
	}
	return pwm_counter;
}

//MOTOR 1 BACKWARDS FUNCTION
int m1_backwards(int pwm_counter, int duty_cycle){
	pwm_counter = pwm_counter + 1;
	if( pwm_counter >= PWM_TOP ){
		pwm_counter = 0;
	}
	//motor 1 backwards
  if( pwm_counter < duty_cycle ){
    PORTD |= (1<<6); //Turn on PD6 for M1
    PORTD &= ~(1<<5);
  }
  else{
    PORTD &= ~(1<<6); //Turn off PD6 for M1
    PORTD &= ~(1<<5);
  }
	return pwm_counter;
}

//MOTOR 2 BACKWARDS FUNCTION
int m2_backwards(int pwm_counter, int duty_cycle){
	pwm_counter = pwm_counter + 1;
	if( pwm_counter >= PWM_TOP ){
		pwm_counter = 0;
	}
	//motor 2 backwards
  if( pwm_counter < duty_cycle ){
    PORTB |= (1<<3); //Turn on PB3 for M2
    PORTD &= ~(1<<3);
  }
  else{
    PORTB &= ~(1<<3); //Turn off PB3 for M2
    PORTD &= ~(1<<3);
  }
	return pwm_counter;
}

//Function to initialize motors
void initialize_motors(){
	//configure motors
	DDRD &= ~(1<<5);

	DDRD &= ~(1<<6);

	DDRD &= ~(1<<3);

	DDRB &= ~(1<<3);
}

//Function to initialize buttons
void initialize_buttons(){
	//Configure left push-button
	DDRB &= ~(1<<1); //configure pin as input
	PORTB |= (1<<1); //enable pull-up resistor

	//Configure middle push-button
	DDRB &= ~(1<<4); //configure pin as input
	PORTB |= (1<<4); //enable pull-up resistor

	//Configure right push-button
	DDRB &= ~(1<<5); //configure pin as input
	PORTB |= (1<<5); //enable pull-up resistor
}

//Pulsers for buttons, returns whether button is pressed
int button_debouncer(unsigned int button, unsigned int button_pressed, unsigned int button_state){
	if( button != button_state ){
		if( button == 0 ){
			button_pressed=1;
		}
	}
	else{
		button_pressed=0;
	}
	return button_pressed;
}

//Pulsers for buttons, returns button state
int button_debouncer_state(unsigned int button, unsigned int button_state){
	if( button != button_state ){
		button_state = button;
	}
	return button_state;
}

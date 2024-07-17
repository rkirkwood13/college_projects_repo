#include <stdint.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h>
#include <stdlib.h>
#include "lcd_driver.h"
#include "port_macros.h"
#include "lcd_driver.h"

#define SLOWER 15
#define SLOW 25
#define MEDIUM 40
#define FAST 80

#define CW 1
#define CCW 2
#define F 3
#define R 4

#define PWM_TOP 100
#define TIME_STEP 100

#define DELAY 100 //microseconds

#define LMS (PINC & (1<<0)) //leftmost sensor
#define LS (PINC & (1<<1)) //left sensor
#define MS (PINC & (1<<2)) //middle sensor
#define RS (PINC & (1<<3)) //right sensor
#define RMS (PINC & (1<<4)) //rightmost sensor

#define LEFT (PINB & (1<<1)) //left button
#define MIDDLE (PINB & (1<<4)) //middle button
#define RIGHT (PINB & (1<<5)) //right button

#define TURN_1 260

int main(){

	unsigned int sensor0, sensor1, sensor2, sensor3, sensor4;

	unsigned int last_left_button_state = LEFT;
	unsigned int last_middle_button_state = MIDDLE;
	unsigned int last_right_button_state = RIGHT;
	unsigned int left_button_pressed = 0, middle_button_pressed = 0, right_button_pressed = 0;

	unsigned int speed = SLOW; //speed set at beginning to determine how fast the robot goes through the course
	unsigned int bunker_max = 0;
	unsigned int case_counter = 0;
	unsigned int sensor_counter = 0;
	unsigned int right_button_counter = 0;
	unsigned int bunker_counter = 0;
	unsigned int time_counter = 0;

	unsigned int pwm_counter_m1 = 0;
	unsigned int pwm_counter_m2 = 0;

  //initialization of LCD display, sensors and motors
	initialize_LCD_driver();
  initialize_sensors();
  initialize_motors();
	initialize_buttons();

	while(1){

		//Pulsers for 5 sensors
		sensor0 = (LMS >> 0);
		sensor1 = (LS >> 1);
		sensor2 = (MS >> 2);
		sensor3 = (RS >> 3);
		sensor4 = (RMS >> 4);

		//Pulsers for left, middle and right buttons
		//left = down, middle = up, right = select
		left_button_pressed = button_debouncer(LEFT, left_button_pressed, last_left_button_state);
		last_left_button_state = button_debouncer_state(LEFT, last_left_button_state);

		middle_button_pressed = button_debouncer(MIDDLE, middle_button_pressed, last_middle_button_state);
		last_middle_button_state = button_debouncer_state(MIDDLE, last_middle_button_state);

		right_button_pressed = button_debouncer(RIGHT, right_button_pressed, last_right_button_state);
		last_right_button_state = button_debouncer_state(RIGHT, last_right_button_state);


		//main code goes here
		switch(case_counter){
			case 0:
				pwm_counter_m1 = m1_direction(pwm_counter_m1, F, SLOWER);
				pwm_counter_m2 = m2_direction(pwm_counter_m2, F, SLOWER);
				time_counter++;
				if (time_counter >= (300 * (DELAY/10))){
					stop();
					case_counter++;
					pwm_counter_m1 = 0;
					pwm_counter_m2 = 0;
				}
			case 1:
				stop();
				display_int(time_counter);
		}



	_delay_us(DELAY);
	}

}

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

int bunker_set(int bunkers, int increment, int decrement){
  char numstring[100];
  if( decrement == 1 ){ //decrement time by 0.1 when left button pressed
    if( bunkers >= 2 ){
    	bunkers = bunkers - 1;
    }
  }
  else if( increment == 1 ){ //increment time by 0.1 when middle button pressed
  	bunkers = bunkers + 1;
  }

  LCD_execute_command(TURN_ON_DISPLAY);
  LCD_execute_command(CLEAR_DISPLAY);
  itoa(bunkers,numstring,10); //Cast time to a string and display on LCD screen
  LCD_print_String(numstring);

  _delay_ms(50);
  return bunkers;
}

int find_sensors(unsigned int sensor0, unsigned int sensor1, unsigned int sensor2, unsigned int sensor3, unsigned int sensor4){
  int sensor_counter = 0;
  if( sensor0 && !sensor1 && !sensor2 && !sensor3 && !sensor4 ){ //only 0 is on
    sensor_counter = 1;
  }
  else if(  !sensor0 && !sensor1 && !sensor2 && !sensor3 && sensor4 ){ // only 4 is on
    sensor_counter = 2;
  }
	else if(  !sensor0 && sensor1 && sensor2 && sensor3 && !sensor4 ){ //only 1, 2 and 3 are on
    sensor_counter = 3;
  }
	else if(  !sensor0 && !sensor1 && sensor2 && sensor3 && !sensor4 ){ //only 2 and 3 are on
    sensor_counter = 3;
  }
	else if(  !sensor0 && sensor1 && sensor2 && !sensor3 && !sensor4 ){ //only 1 and 2 are on
    sensor_counter = 3;
  }
	else if(  sensor0 && sensor1 && sensor2 && !sensor3 && !sensor4 ){ //only 0, 1 and 2 are on
    sensor_counter = 3;
  }
	else if(  !sensor0 && !sensor1 && sensor2 && sensor3 && sensor4 ){ //only 2,3 and 4 are on
    sensor_counter = 3;
  }
	else if( sensor0 && sensor4 ){ //all 5 on
    sensor_counter = 4;
  }
	else if( !sensor0 && !sensor1 && !sensor2 && !sensor3 && !sensor4 ){ //all 5 off
		sensor_counter = 5;
	}
	else if (!sensor4){
		sensor_counter = 6;
	}
	else{
		sensor_counter = 7;
	}
  return sensor_counter;
}

int m1_direction(unsigned int pwm_counter_m1, int direction, int speed){
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

int m2_direction(unsigned int pwm_counter_m2, int direction, int speed){
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

//These functions allow for a direction command to be implemented for a specified time. The time_counter input should be 0.
void direction_time(int direction, int speed, int time){
	unsigned int pwm_counter_m2 = 0;
	unsigned int pwm_counter_m1 = 0;
	for(int i = 0; i <= (time * (DELAY/10)); i = i + 1){
	  switch(direction){
	    case 1://clockwise
					pwm_counter_m1 = m1_forwards(pwm_counter_m1, speed);
					pwm_counter_m2 = m2_backwards(pwm_counter_m2, speed);
					break;
	    case 2: //counterclockwise
					pwm_counter_m1 = m1_backwards(pwm_counter_m1, speed);
					pwm_counter_m2 = m2_forwards(pwm_counter_m2, speed);
	        break;
	    case 3: //Forward
	        pwm_counter_m1 = m1_forwards(pwm_counter_m1, speed);
					pwm_counter_m2 = m2_forwards(pwm_counter_m2, speed);
	        break;
	    case 4: //Reverse
	        pwm_counter_m1 = m1_backwards(pwm_counter_m1, speed);
					pwm_counter_m2 = m2_backwards(pwm_counter_m2, speed);
	        break;
	    default:
	        break;
		}
		_delay_us(100);
	}
}

void turn_right(time){
	direction_time(CW, SLOWER, time); //Turns CW for 10 ms
}

void turn_left(int time){
	direction_time(CCW, SLOWER, time); //Turns CCW for 10 ms
}

//Function to initialize sensors
void initialize_sensors(){
	//Configure leftmost sensor
	DDRC &= ~(1<<0); //configure pin as input
	PORTC |= (1<<0); //enable pull-up resistor

  //Configure left middle sensor
	DDRC &= ~(1<<1); //configure pin as input
	PORTC |= (1<<1); //enable pull-up resistor

  //Configure middle sensor
	DDRC &= ~(1<<2); //configure pin as input
	PORTC |= (1<<2); //enable pull-up resistor

  //Configure right middle sensor
	DDRC &= ~(1<<3); //configure pin as input
	PORTC |= (1<<3); //enable pull-up resistor

  //Configure rightmost sensor
	DDRC &= ~(1<<4); //configure pin as input
	PORTC |= (1<<4); //enable pull-up resistor
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

//MOTOR 1 FORWARDS FUNCTION
int m1_forwards(unsigned int pwm_counter, int duty_cycle){
	pwm_counter = pwm_counter + 1;
	if( pwm_counter >= PWM_TOP ){
		pwm_counter = 0;
	}
	//motor 1 forwards
	if( pwm_counter < duty_cycle ){
		PORTD |= (1<<6); //Turn on PD5 for M1
		PORTD &= ~(1<<5); //Turn off PD6 for M1
	}
	else{
		PORTD |= (1<<6); //Turn on both ports to brake
    PORTD |= (1<<5);
	}
	return pwm_counter;
}

//MOTOR 2 FORWARDS FUNCTION
int m2_forwards(unsigned int pwm_counter, int duty_cycle){
	pwm_counter = pwm_counter + 1;
	if( pwm_counter >= PWM_TOP ){
		pwm_counter = 0;
	}
	//motor 2 forwards
	if( pwm_counter < duty_cycle ){
		PORTB |= (1<<3); //Turn on PD3 for M2
		PORTD &= ~(1<<3);
	}
	else{
		PORTD |= (1<<3); //Turn on both ports to brake
    PORTB |= (1<<3);
	}
	return pwm_counter;
}

//MOTOR 1 BACKWARDS FUNCTION
int m1_backwards(unsigned int pwm_counter, int duty_cycle){
	pwm_counter = pwm_counter + 1;
	if( pwm_counter >= PWM_TOP ){
		pwm_counter = 0;
	}
	//motor 1 backwards
  if( pwm_counter < duty_cycle ){
    PORTD |= (1<<5); //Turn on PD6 for M1
    PORTD &= ~(1<<6);
  }
  else{
		PORTD |= (1<<6); //Turn on both ports to brake
    PORTD |= (1<<5);
  }
	return pwm_counter;
}

//MOTOR 2 BACKWARDS FUNCTION
int m2_backwards(unsigned int pwm_counter, int duty_cycle){
	pwm_counter = pwm_counter + 1;
	if( pwm_counter >= PWM_TOP ){
		pwm_counter = 0;
	}
	//motor 2 backwards
  if( pwm_counter < duty_cycle ){
    PORTD |= (1<<3); //Turn on PB3 for M2
    PORTB &= ~(1<<3);
  }
  else{
		PORTB |= (1<<3); //Turn on both ports to brake
    PORTD |= (1<<3);
  }
	return pwm_counter;
}

void stop(){
	PORTD |= (1<<3); //Turn on PB3 for M2
	PORTB |= (1<<3);
	PORTD |= (1<<5); //Turn on PD6 for M1
	PORTD |= (1<<6);
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

void display_int(int num){
	char numstring[100];
	LCD_execute_command(TURN_ON_DISPLAY);
	LCD_execute_command(CLEAR_DISPLAY);
	itoa(num,numstring,10);
	LCD_print_String(numstring);
}

void bunker_claimed(bunker_counter, bunker_max){
	char buffer[50];
	LCD_execute_command(TURN_ON_DISPLAY);
	LCD_execute_command(CLEAR_DISPLAY);
	LCD_print_String("Bunker");
	LCD_move_cursor_to_col_row(0,1);
	LCD_print_String("claimed!");
	_delay_ms(500);
	LCD_execute_command(TURN_ON_DISPLAY);
	LCD_execute_command(CLEAR_DISPLAY);
	LCD_print_String("Bunkers:");
	sprintf(buffer, "%d/%d", bunker_counter, bunker_max);
	LCD_move_cursor_to_col_row(0,1);
	LCD_print_String(buffer);
}

All_bunkers_claimed(){
	stop();
	LCD_execute_command(TURN_ON_DISPLAY);
	LCD_execute_command(CLEAR_DISPLAY);
	LCD_print_String("All");
	LCD_move_cursor_to_col_row(0,1);
	LCD_print_String("bunkers");
	_delay_ms(1000);
	LCD_execute_command(TURN_ON_DISPLAY);
	LCD_execute_command(CLEAR_DISPLAY);
	LCD_print_String("claimed!");
	_delay_ms(900);
}

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

int main(){

	unsigned int last_sensor0_state = LMS;
	unsigned int last_sensor1_state = LS;
	unsigned int last_sensor2_state = MS;
	unsigned int last_sensor3_state = RS;
	unsigned int last_sensor4_state = RMS;
	unsigned int sensor0, sensor1, sensor2, sensor3, sensor4;

	unsigned int last_left_button_state = LEFT;
	unsigned int last_middle_button_state = MIDDLE;
	unsigned int last_right_button_state = RIGHT;
	unsigned int left_button_pressed = 0, middle_button_pressed = 0, right_button_pressed = 0;

	unsigned int forward_speed = SLOW; //speed set at beginning to determine how fast the robot goes through the course
	unsigned int bunker_max = 0;
	unsigned int case_counter = 0;
	unsigned int time_counter = 0;
	unsigned int sensor_counter = 0;
	unsigned int right_button_counter = 0;

	unsigned int pwm_counter_m1 = 0;
	unsigned int pwm_counter_m2 = 0;

  //initialization of LCD display, sensors and motors
	initialize_LCD_driver();
  initialize_sensors();
  initialize_motors();
	initialize_buttons();

	while(1){

		//Pulsers for 5 sensors
		sensor0 = sensor_pulser(LMS, sensor0, last_sensor0_state);
		last_sensor0_state = sensor_pulser_state(LMS, last_sensor0_state);

		sensor1 = sensor_pulser(LS, sensor1, last_sensor1_state);
		last_sensor1_state = sensor_pulser_state(LS, last_sensor1_state);

		sensor2 = sensor_pulser(MS, sensor2, last_sensor2_state);
		last_sensor2_state = sensor_pulser_state(MS, last_sensor2_state);

		sensor3 = sensor_pulser(RS, sensor3, last_sensor3_state);
		last_sensor3_state = sensor_pulser_state(RS, last_sensor3_state);

		sensor4 = sensor_pulser(RMS, sensor4, last_sensor4_state);
		last_sensor4_state = sensor_pulser_state(RMS, last_sensor4_state);

		//Pulsers for left, middle and right buttons
		//left = down, middle = up, right = select
		left_button_pressed = button_debouncer(LEFT, left_button_pressed, last_left_button_state);
		last_left_button_state = button_debouncer_state(LEFT, last_left_button_state);

		middle_button_pressed = button_debouncer(MIDDLE, middle_button_pressed, last_middle_button_state);
		last_middle_button_state = button_debouncer_state(MIDDLE, last_middle_button_state);

		right_button_pressed = button_debouncer(RIGHT, right_button_pressed, last_right_button_state);
		last_right_button_state = button_debouncer_state(RIGHT, last_right_button_state);


		//main code goes here

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
    if( bunkers >= 1 ){
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
  int sensor_count = 0;
  if( (sensor0 == 1) & (sensor1&sensor2&sensor3&sensor4 == 0) ){
    sensor_count = 0;
  }
  else if( (sensor4 == 1) & (sensor0&sensor1&sensor2&sensor3 == 0) ){
    sensor_count = 1;
  }
  else if( (sensor0&sensor1 == 1) & (sensor2&sensor3&sensor4 == 0) ){
    sensor_count = 2;
  }
  else if( (sensor3&sensor4 == 1) & (sensor0&sensor1&sensor2 == 0) ){
    sensor_count = 3;
  }
  else if( (sensor0&sensor1 == 0) & (sensor2&sensor3&sensor4 == 1) ){
    sensor_count = 4;
  }
  else if( (sensor3&sensor4 == 0) & (sensor0&sensor1&sensor2 == 1) ){
    sensor_count = 5;
  }
  else if( (sensor1&sensor2&sensor3 == 1) & (sensor0&sensor4 == 0) ){
    sensor_count = 6;
  }
  else if(sensor0&sensor1&sensor2&sensor3&sensor4 == 1){
    sensor_count = 7;
  }
  return sensor_count;
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
	        pwm_counter_m1 = m1_backwards(pwm_counter_m1, speed);
					pwm_counter_m2 = m2_forwards(pwm_counter_m2, speed);
	        break;
	    case 2: //counterclockwise
	        pwm_counter_m1 = m1_forwards(pwm_counter_m1, speed);
					pwm_counter_m2 = m2_backwards(pwm_counter_m2, speed);
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

void turn_right(int time){
	direction_time(CW, SLOW, time);
}

void turn_left(int time){
	direction_time(CCW, SLOW, time);
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

//Pulsers for sensors, returns whether sensor is activated
int sensor_pulser(unsigned int sensor, unsigned int sensor_on, unsigned int last_sensor_state){
	if( sensor != last_sensor_state ){
		if( sensor == 0 ){
			sensor_on = 1;
		}
	}
	else{
		sensor_on = 0;
	}
	return sensor_on;
}

//Pulsers for sensors, returns sensor state
int sensor_pulser_state(unsigned int sensor, unsigned int last_sensor_state){
	if( sensor != last_sensor_state ){
		last_sensor_state = sensor;
	}
	return last_sensor_state;
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

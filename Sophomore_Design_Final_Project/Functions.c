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


//MOTOR 1 FORWARDS FUNCTION
int m1_forwards(unsigned int pwm_counter, int duty_cycle){
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
int m2_forwards(unsigned int pwm_counter, int duty_cycle){
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
int m1_backwards(unsigned int pwm_counter, int duty_cycle){
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
int m2_backwards(unsigned int pwm_counter, int duty_cycle){
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

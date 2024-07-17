void motor_speed(speed){
  unsigned int pwm_counter = 0;
  unsigned int duty_cycle;

  if( strcmp(speed,"slow") ){
    duty_cycle = 20;
  }
  else if( strcmp(speed,"medium") ){
    duty_cycle = 50;
  }
  else if( strcmp(speed,"fast") ){
    duty_cycle = 100;
  }

  pwm_counter = pwm_counter + 1;
  if( pwm_counter >= 100 ){
    pwm_counter = 0;
  }
  //Do PWM on motor 1
  if( pwm_counter < duty_cycle ){
    PORTD |= (1<<5); //Turn on PD5 for M1
  }
  else{
    PORTD &= ~(1<<5); //Turn off PD5 for M1
  }

  //Do PWM on motor 2
  if( pwm_counter < duty_cycle ){
    PORTD |= (1<<3); //Turn on PD5 for M2
  }
  else{
    PORTD &= ~(1<<3); //Turn off PD5 for M2
  }

  //small delay to slow down main loop
  _delay_us(10);
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

void motor_speed(int speed){
  unsigned int pwm_counter = 0;
  unsigned int duty_cycle;

	duty_cycle = speed;

  pwm_counter = pwm_counter + 1;
  if( pwm_counter >= 100 ){
    pwm_counter = 0;
  }
  //Do PWM on motor 1
  if( pwm_counter < duty_cycle ){
 		//Turn on PD5 for M1
		PORTD &= ~(1<<5);
  }
  else{
		//Turn off PD5 for M1
		PORTD |= (1<<5);
  }

  //Do PWM on motor 2
  if( pwm_counter < duty_cycle ){
    //Turn on PD3 for M2
		PORTD &= ~(1<<3);
  }
  else{
    //Turn off PD3 for M2
		PORTD |= (1<<3);
  }

  //small delay to slow down main loop
  _delay_us(10);
}

void motor_direction(int direction){
	if(direction == 1){ //PB3 on, PD6 off for cw direction
		PORTD &= ~(1<<6);
		PORTB |= (1<<3);
	}
	else if (direction == 2){ //PB3 off, PD6 on for ccw direction
		PORTB &= ~(1<<3);
		PORTD |= (1<<6);
	}
	else if (direction == 3){ //PB3 and PD6 on for forward direction
		PORTB |= (1<<3);
		PORTD |= (1<<6);
	}
	else if (direction == 4){ //PB3 and PD6 off for reverse direction
		PORTB &= ~(1<<3);
		PORTD &= ~(1<<6);
	}
	_delay_us(10);
}

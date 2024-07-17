//MOTOR 1 FORWARDS FUNCTION
int motor_forwards_M1(int pwm_counter, int duty_cycle){
	pwm_counter = pwm_counter + 1;
	if( pwm_counter >= 100 ){
		pwm_counter = 0;
	}
//motor 1 forwards
	if( pwm_counter < duty_cycle ){
		PORTD |= (1<<5); //Turn on PD5 for M1
		PORTD &= ~(1<<6);
	}
	else{
		PORTD &= ~(1<<5); //Turn off PD5 for M1
		PORTD &= ~(1<<6);
	}
	return pwm_counter;
}

//MOTOR 2 FORWARDS FUNCTION
int motor_forwards_M2(int pwm_counter, int duty_cycle){
	pwm_counter = pwm_counter + 1;
	if( pwm_counter >= 100 ){
		pwm_counter = 0;
	}
	//Motor 2 forwards
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
int motor_backwards_M1(int pwm_counter, int duty_cycle){
	pwm_counter = pwm_counter + 1;
	if( pwm_counter >= 100 ){
		pwm_counter = 0;
	}
	//Motor 1 backwards
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
int motor_backwards_M2(int pwm_counter, int duty_cycle){
	pwm_counter = pwm_counter + 1;
	if( pwm_counter >= 100 ){
		pwm_counter = 0;
	}
	//Motor 2 backwards
  if( pwm_counter < duty_cycle ){
    PORTB |= (1<<3); //Turn on PB5 for M2
    PORTD &= ~(1<<3);
  }
  else{
    PORTB &= ~(1<<3); //Turn off PB5 for M2
    PORTD &= ~(1<<3);
  }
	return pwm_counter;
}

int command(int pwm_counter, int speed, int direction, int time){

	switch(direction){
	case 1://clockwise
			pwm_counter1 = motor_backwards_M1(pwm_counter1, speed);
			_delay_us(10);
			pwm_counter2 = motor_forwards_M2(pwm_counter2, speed);
			break;
	case 2: //counterclockwise
			pwm_counter1 = motor_forwards_M1(pwm_counter1, speed);
			_delay_us(10);
			pwm_counter2 = motor_backwards_M2(pwm_counter2, speed);
			break;
	case 3: //Forward
			pwm_counter1 = motor_forwards_M1(pwm_counter1, speed);
			_delay_us(10);
			pwm_counter2 = motor_forwards_M2(pwm_counter2, speed);
			break;
	case 4: //Reverse
			pwm_counter1 = motor_backwards_M1(pwm_counter1, speed);
			_delay_us(10);
			pwm_counter2 = motor_backwards_M2(pwm_counter2, speed);
			break;
}
}

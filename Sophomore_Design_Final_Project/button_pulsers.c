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

//Pulsers for buttons
int button_debouncer(unsigned int button, unsigned int button_state, unsigned int button_pressed){
	if( button != button_state ){
		if( button == 0 ){
			button_pressed=1;
		}
		button_state = button;
	}
	else{
		button_pressed=0;
	}
	return button_pressed;
}

int speed_set(int speed, int increment, int decrement);
int bunker_set(int bunkers, int increment, int decrement);

int m1_direction(unsigned int pwm_counter_m1, int direction, int speed);

int m2_direction(unsigned int pwm_counter_m2, int direction, int speed);

//These functions allow for a direction command to be implemented for a specified time. The time_counter input should be 0.
void m1_direction_time(int direction, int speed, int time);
void m2_direction_time(int direction, int speed, int time);

//Function to initialize sensors
void initialize_sensors();

//Function to initialize motors
void initialize_motors();

//Function to initialize buttons
void initialize_buttons();


//MOTOR 1 FORWARDS FUNCTION
int m1_forwards(unsigned int pwm_counter, int duty_cycle);

//MOTOR 2 FORWARDS FUNCTION
int m2_forwards(unsigned int pwm_counter, int duty_cycle);
//MOTOR 1 BACKWARDS FUNCTION
int m1_backwards(unsigned int pwm_counter, int duty_cycle);

//MOTOR 2 BACKWARDS FUNCTION
int m2_backwards(unsigned int pwm_counter, int duty_cycle);
//Pulsers for buttons, returns whether button is pressed
int button_debouncer(unsigned int button, unsigned int button_pressed, unsigned int button_state);

//Pulsers for buttons, returns button state
int button_debouncer_state(unsigned int button, unsigned int button_state);

//Function to display and integer on LCD screen
void display_int(int num){
	char numstring[100];
	LCD_execute_command(TURN_ON_DISPLAY);
	LCD_execute_command(CLEAR_DISPLAY);
	itoa(num,numstring,10);
	LCD_print_String(numstring);
}


void bunker_claimed();

void all_bunkers_claimed();

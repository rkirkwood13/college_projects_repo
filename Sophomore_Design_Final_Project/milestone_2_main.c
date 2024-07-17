//Code for inside while loop, case statements
unsigned int forward_speed = 0; //speed set at beginning to determine how fast the robot goes through the course
unsigned int bunker_max = 0;
unsigned int case_counter = 0;
unsigned int time_counter = 0;
unsigned int sensor_counter = 0;
unsigned int bunker_counter = 0;

if(right_button_pressed == 1){
  right_button_counter++;
}

sensor_counter = find_sensors(sensor0, sensor1, sensor2, sensor3, sensor4);

switch(right_button_counter){
  case 0:
    speed = speed_set(speed, middle_button_pressed, left_button_pressed);
    break;
  case 1:
    bunker_max = bunker_set(bunker_max, middle_button_pressed, left_button_pressed);
    break;
  case 2:
    case_counter = 1;
    break;
  default:
    break;
}

switch(case_counter){
  case 1:
    direction_time(F, 0, 1500); //delay robot for 1.5 seconds to set in warzone
    case_counter++;
    break;
  default:
    break;
}

switch(sensor_counter){
  case 1:
    turn_left();
    break;
  case 2:
    turn_right();
    break;
  case 3: //bunker may have been found, move forward
    direction_time(F, SLOW, 100);
    if( !sensor0 && sensor1 && sensor2 && sensor3 && !sensor4 ){ //if only middle 3 are still on
      bunker_counter++;
      bunker_claimed();
      direction_time(R, SLOW, 200);
      direction_time(CW, SLOW, 300);
    }
    if( bunker_counter >= bunker_max){ //If all bunkers have been claimed, stop
      pwm_counter_m1 = m1_direction(pwm_counter_m1, F, 0);
      pwm_counter_m2 = m2_direction(pwm_counter_m2, F, 0);
    }
    break;
  case 4:
    turn_left();
    break;
  case 5:
    turn_left();
    break;
  case 6:
    turn_left();
    break;
  case 7:
    turn_right();
    break;
  case 8:
    turn_right();
    break;
  case 9:
    turn_right();
    break;
  case 10: //border
    direction_time(R, SLOW, 200);
    direction_time(CW, SLOW, 300);
    break;
  case 11: //no sensors on, keep moving forward
    pwm_counter_m1 = m1_direction(pwm_counter_m1, F, speed);
    pwm_counter_m2 = m2_direction(pwm_counter_m2, F, speed);
    if(sensor0||sensor1||sensor2||sensor3||sensor4){
      case_counter++;
    }
    break;
  default:
    break;
  }

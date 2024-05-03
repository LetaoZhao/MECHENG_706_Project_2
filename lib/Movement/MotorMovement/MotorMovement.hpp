
void disable_motors();
void enable_motors();
void stop();
void forward();
void reverse();
void ccw();
void cw();
void strafe_left(int power);
void strafe_right(int power);
void strafe_left_until(float target_distance_IRs, int F_B, float delay_time);
void strafe_right_until(float target_distance_IRs, int F_B, float delay_time);
void ccw_low();
void cw_low();
#include <project2/pid.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846

PID::PID(){

    /* TO DO
     *
     * find appropriate values for PID contorl, and initialize them.
     *
    */

    error = 0;
    error_sum = 0;
    error_diff = 0;
    Kp = 1.5;
    Ki = 0;
    Kd = 5; 
}

void PID::reset() {
    error = 0;
    error_sum = 0;
    error_diff = 0;
}

float PID::get_control(point car_pose, point goal_pose){
// copied are last algoritem and pasted it hear
    float ctrl;
	float ex = goal_pose.x - car_pose.x;
	float ey = goal_pose.y - car_pose.y;

	float angle;
	float angle2 = atan2(ey, ex);

	error_diff = (angle2 - car_pose.th) - error;

	error = angle2 - car_pose.th; 
	error_sum = error_sum + (error * 0.1);


	if (error > M_PI) {

	    error = error - (2 * M_PI);
	}
	

	ctrl = Kp*error + Ki * error_sum + Kd / 0.1 * error_diff;
    
}

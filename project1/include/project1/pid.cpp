#include <project1/pid.h>
#include <cmath>
#include "stdio.h"
PID::PID(){

    /* TO DO
     *
     * find appropriate value of the coefficients for PID control, and initialize them.
     *
    */
	error = 0;
	error_sum = 0;
	error_diff = 0;
	// this is just to try some values and show how it is suposed to look like
	Ki = 0;
	Kp = 0.9;
	Kd = 0;

}

float PID::get_control(point car_pose, point goal_pose){

   	float ctrl;
	float ex = goal_pose.x - car_pose.x;
	float ey = goal_pose.y - car_pose.y;

	float angle;
	float angle2 = atan2(ey, ex);

	error_diff = (error - (angle2 - car_pose.th))/0.1;

	error = angle2 - car_pose.th; 
	error_sum = (error_sum + error)*0.1;


	if (error > M_PI) {

	    error = error - (2 * M_PI);
	}
	
printf("error %f angle %f\n\r", error, angle2);

	ctrl = Kp*error + Ki*error_sum + Kd*error_diff;
printf("control: %f\n\r", ctrl);
    /* TO DO
     *
     * implement pid algorithm
     *
    */

    return ctrl;
}

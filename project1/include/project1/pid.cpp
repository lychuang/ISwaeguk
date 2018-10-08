#include <project1/pid.h>
#include <cmath>

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
	Kp = 1;
	Kd = 0;

}

float PID::get_control(point car_pose, point goal_pose){

   	float ctrl;
	float ex = goal_pose.x - car_pose.x;
	float ey = goal_pose.y - car_pose.y;
	error_diff = (error - (car_pose.th - tan(ey/ex)))/0.1;
	error = car_pose.th - tan(ey/ex);
	error_sum = error_sum + error*0.1;
	
	
	ctrl = Kp*error + Ki*error_sum + Kd*error_diff;
    /* TO DO
     *
     * implement pid algorithm
     *
    */

    return ctrl;
}

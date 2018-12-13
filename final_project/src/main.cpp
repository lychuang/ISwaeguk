//state definition
#define INIT 0
#define PATH_PLANNING 1
#define RUNNING_M1 2
#define M2_PLAN 3
#define RUNNING_M2 4
#define FINISH -1

#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <project2/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <project2/pid.h>
#include <math.h>
#include <pwd.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

//map spec
cv::Mat map;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;
int look_ahead_idx;

traj current_point;
PID pid_ctrl;

//parameters we should adjust : K, margin, MaxStep
int margin = 5;
int K = 1500;
double MaxStep = 7;
int waypoint_margin = 16;

//way points
std::vector<point> waypoints;

//path
std::vector<traj> path_RRT;

//robot
point robot_pose;
ackermann_msgs::AckermannDriveStamped cmd;

//FSM state
int state;

//function definition
void setcmdvel(double v, double w);
void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs);
void set_waypoints();
void generate_path_RRT(int start, int finish);
double dist(point p1, point p2);

int main(int argc, char** argv){
    ros::init(argc, argv, "slam_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Publisher cmd_vel_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_0",1);
    
    ros::Subscriber gazebo_pose_sub = n.subscribe("/amcl_pose", 100, callback_state);

    printf("Initialize topics\n");
    
    // FSM
    state = INIT;
    bool running = true;
    ros::Rate control_rate(60);

    while(running){
        switch (state) {
        case INIT: {
            // Load Map
            char* user = getpwuid(getuid())->pw_name;
            cv::Mat map_org = cv::imread((std::string("/home/") + std::string(user) +
                              std::string("/catkin_ws/src/ISwaeguk/final_project/src/final.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);

            cv::transpose(map_org,map);
            cv::flip(map,map,1);

            map_y_range = map.cols;
            map_x_range = map.rows;
            map_origin_x = map_x_range/2.0 - 0.5;
            map_origin_y = map_y_range/2.0 - 0.5;
            world_x_min = -4.7;
            world_x_max = 4.7;
            world_y_min = -10.2;
            world_y_max = 10.2;
            res = 0.05;
            printf("Load map\n");

             if(! map.data )                              // Check for invalid input
            {
                printf("Could not open or find the image\n");
                return -1;
            }
            state = PATH_PLANNING;
        } break;

        case PATH_PLANNING:
            
            // Set Way Points
            set_waypoints();
            printf("Set way points\n");

            // RRT
            generate_path_RRT(0, 4);
            printf("Generate RRT\n");

            ros::spinOnce();
            ros::Rate(0.33).sleep();

            printf("Initialize ROBOT\n");
            current_point = path_RRT[0];
            look_ahead_idx = 0;
	    state = RUNNING_M1;

        case RUNNING_M1: {
            //TODO 3
            //get current point
            point curr_point = {current_point.x, current_point.y, current_point.th};
            
            //retrieve the next steering angle
            setcmdvel(1.1, pid_ctrl.get_control(robot_pose, curr_point));
            //publish it to robot
            cmd_vel_pub.publish(cmd);
            
            //dist between robot and point is < threshold
            //move on to next point
            if(sqrt(pow((robot_pose.x - current_point.x), 2)
                    + pow((robot_pose.y - current_point.y), 2)) < 0.4) {
                
                look_ahead_idx++; //increment index
                current_point = path_RRT[look_ahead_idx]; //increment current point
                printf("current goal %d\r\n", look_ahead_idx);
            }
            
            if (look_ahead_idx >= path_RRT.size()) { //no more points in path - reached GOAL!
		setcmdvel(0,0);
		cmd_vel_pub.publish(cmd);
		ros::spinOnce();
                printf("Mission 1 done. Starts on planing Mission 2 now. \n\r");
                state = M2_PLAN; //update FSM to finished
		
            }
            
            //ensure commands get processed immediately
            ros::spinOnce();
            
            //maintain desired frequency of looping
            control_rate.sleep();
        } break;
	case M2_PLAN:{
	path_RRT.clear();
	look_ahead_idx = 0;
	generate_path_RRT(4, (waypoints.size() - 1));
	current_point = path_RRT[0];
	state = RUNNING_M2;	
	}
	case RUNNING_M2: {
 	//get current point
            point curr_point = {current_point.x, current_point.y, current_point.th};
            
            //retrieve the next steering angle
            setcmdvel(1.1, pid_ctrl.get_control(robot_pose, curr_point));
            //publish it to robot
            cmd_vel_pub.publish(cmd);
            
            //dist between robot and point is < threshold
            //move on to next point
            if(sqrt(pow((robot_pose.x - current_point.x), 2)
                    + pow((robot_pose.y - current_point.y), 2)) < 0.4) {
                
                look_ahead_idx++; //increment index
                current_point = path_RRT[look_ahead_idx]; //increment current point
                printf("current goal %d\r\n", look_ahead_idx);
            }
            
            if (look_ahead_idx >= path_RRT.size()) { //no more points in path - reached GOAL!
                
                state = FINISH; //update FSM to finished
            }
            
            //ensure commands get processed immediately
            ros::spinOnce();
            
            //maintain desired frequency of looping
            control_rate.sleep();
		
	} break;
        case FINISH: {
            setcmdvel(0,0);
            cmd_vel_pub.publish(cmd);
            running = false;
            ros::spinOnce();
            control_rate.sleep();
        } break;
        default: {
        } break;
        }
    }
    return 0;
}

void setcmdvel(double vel, double deg){
    cmd.drive.speed = vel;
    cmd.drive.steering_angle = deg;
}

void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs){
    robot_pose.x = msgs->pose.pose.position.x;
    robot_pose.y = msgs->pose.pose.position.y;
    robot_pose.th = tf::getYaw(msgs->pose.pose.orientation);
    //printf("x,y : %f,%f \n",robot_pose.x,robot_pose.y);
}

void set_waypoints()
{
    point waypoint_candid[7];

    // Starting point. (Fixed)
    waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 8.5;


    //TODO 2
    // Set your own waypoints.
    // The car should turn around the outer track once, and come back to the starting point.
    waypoint_candid[1].x = 4.5;
    waypoint_candid[1].y = 5;
    waypoint_candid[2].x = 2.5;
    waypoint_candid[2].y = -8.2;
    waypoint_candid[3].x = -4;
    waypoint_candid[3].y = -5;
    waypoint_candid[4].x = -2.2;
    waypoint_candid[4].y = 8.2;


    // Waypoints for arbitrary goal points.
    // TA will change this part before scoring.
    // This is an example.
    waypoint_candid[5].x = 1.5;
    waypoint_candid[5].y = 1.5;
    waypoint_candid[6].x = -2;
    waypoint_candid[6].y = -9.0;

    int order[] = {0,1,2,3,4,5,6};
    int order_size = 7;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}

void generate_path_RRT(int start, int finish)
{   
    //TODO 1
    std::vector<traj> one_path;

    bool valid_path = true;
    int path_count = 0;
    //create a path between each waypoint
    for (int i = start; i < finish; i++) {
	    
        valid_path = false;
	path_count = 0;
        ///printf("DAFUQ");
        //point curr_point = waypoints[i];

        printf("i: %d, pathcount: %d\n\r", i, path_count);
        point curr_point = waypoints[i];
        printf("1");
        //fflush(stdout);
        if (i > start) {
            curr_point = {path_RRT[path_RRT.size()-1].x, path_RRT[path_RRT.size()-1].y, path_RRT[path_RRT.size()-1].th};
        }
        //DEBUG// printf("%d\n\r", i);
        
        //instance of rrtTree class for each iteration
        //create the rrtTree for the next goal
        //
        point me = waypoints[i+1]; 
	
        rrtTree tree (curr_point, waypoints[i+1], map, map_origin_x, map_origin_y, res, margin);
        //generate the search tree
	while(!valid_path) {
		path_count++;
		valid_path = true;
		int notok = tree.generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);
		//check if anything was generated
		if (notok) {
		    printf("generate RRT failed\n\r");
		    valid_path = false;
		    
		    break;
		}
		
		//generate the path, store it
		one_path = tree.backtracking_traj(MaxStep);
		
		//DEBUG//
		printf("onepathsize = %d\n\r", static_cast<int>(one_path.size()));
		if (static_cast<int>(one_path.size()) == 0) {
		    printf("generate path failed. Makes a new path. \n\r");
		    valid_path = false;
		}
		if(valid_path) {
			point last_point = {one_path[0].x, one_path[0].y, one_path[0].th};

			if (dist(last_point, waypoints[i+1]) > 2) {
			    valid_path = false;
			    printf("generate path failed. Makes a new path. \n\r");
			}
		}
		
		//add the path to the overall path
		    if(valid_path){
			    for (int j = static_cast<int>(one_path.size()) - 1; j >= 0; j--) {

				path_RRT.push_back(one_path[j]);
			    }
		    
		    path_count = 0;
	
		} else {
			one_path.clear();
		}

		if (path_count == 3) {
			break;
	 
			one_path.clear();
	    	}
	}
	if (valid_path == false) {
			break;
	}
        
       

    	tree.visualizeTree(path_RRT);
	    sleep(5);
    }	
	

    if(!valid_path) {
        path_RRT.clear();
        generate_path_RRT(start, finish);
    }
}

double dist(point p1, point p2) {
	return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}

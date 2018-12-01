//state definition
#define INIT 0
#define PATH_PLANNING 1
#define RUNNING 2
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


traj current_point;
PID pid_ctrl;
int look_ahead_idx;
//parameters we should adjust : K, margin, MaxStep
int margin = 3;
int K = 3000;
double MaxStep = 10;
int waypoint_margin = 25;

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
void generate_path_RRT();
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
                              std::string("/catkin_ws/src/project4/src/slam_map.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);

            cv::transpose(map_org,map);
            cv::flip(map,map,1);

            map_y_range = map.cols;
            map_x_range = map.rows;
            map_origin_x = map_x_range/2.0 - 0.5;
            map_origin_y = map_y_range/2.0 - 0.5;
            world_x_min = -4.5;
            world_x_max = 4.5;
            world_y_min = -13.5;
            world_y_max = 13.5;
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
            generate_path_RRT();
            printf("Generate RRT\n");

            ros::spinOnce();
            ros::Rate(0.33).sleep();

            printf("Initialize ROBOT\n");
            current_point = path_RRT[0];
            state = RUNNING;

        case RUNNING: {
                //TODO 1
            //get current point
            point curr_point = {current_point.x, current_point.y, current_point.th};
            
            //retrieve the next steering angle
            setcmdvel(1, pid_ctrl.get_control(robot_pose, curr_point));
            //publish it to robot
            cmd_vel_pub.publish(cmd);
            
            
            //dist between robot and point is < threshold
            //move on to next point
            if(sqrt(pow((robot_pose.x - current_point.x), 2)
                    + pow((robot_pose.y - current_point.y), 2)) < 0.5) {
                
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
    std::srand(std::time(NULL));
    point waypoint_candid[5];
    waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 12.0;

    cv::Mat map_margin = map.clone();
    int jSize = map.cols; // the number of columns
    int iSize = map.rows; // the number of rows

    for (int i = 0; i < iSize; i++) {
        for (int j = 0; j < jSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - waypoint_margin; k <= i + waypoint_margin; k++) {
                    for (int l = j - waypoint_margin; l <= j + waypoint_margin; l++) {
                        if (k >= 0 && l >= 0 && k < iSize && l < jSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    //TODO 2
 /** 
      Car goes 1 -> 4 -> 3 -> 2 -> 1
      Map Origin -> o
        j 
    
  jsize -----------
        |  1 |  4 |
        |    |    |
        -----------
        |  2 |  3 |
        |    |    |
        o----------  i
                 isize
    **/

    //TODO 2
    //4th quadrant
    while(true)
    {
        int rand_i = ((1+iSize)/2) + std::rand() % (iSize/2);
        int rand_j = ((1+jSize)/2) + std::rand() % (jSize/2);
        if(map_margin.at<uchar>(rand_i,rand_j)>125)
        {
          waypoint_candid[1].x = res*(rand_i-map_origin_x);
          waypoint_candid[1].y = res*(rand_j-map_origin_y);
          break;
        }
    }

    //3th quadrant
    while(true)
    {
        int rand_i = ((1+iSize)/2) + std::rand() % (iSize/2);
        int rand_j = std::rand() % (jSize/2);
        if(map_margin.at<uchar>(rand_i,rand_j)>125)
        {
          waypoint_candid[2].x = res*(rand_i-map_origin_x);
          waypoint_candid[2].y = res*(rand_j-map_origin_y);
          break;
        }
    }

    //2th quadrant
    while(true)
    {
        int rand_i = std::rand() % (iSize/2);
        int rand_j = std::rand() % (jSize/2);
        if(map_margin.at<uchar>(rand_i,rand_j)>125)
        {
          waypoint_candid[3].x = res*(rand_i-map_origin_x);
          waypoint_candid[3].y = res*(rand_j-map_origin_y);
          break;
        }
    }

    //final point
    waypoint_candid[4].x = -3.5;
    waypoint_candid[4].y = 12.0;

    int order[] = {0,1,2,3,4};
    int order_size = 5;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}

void generate_path_RRT()
{
    //TODO 1
    std::vector<traj> one_path;
    //DEBUG//
    printf("size waypoints: %d\r\n", static_cast<int>(waypoints.size()));
    bool valid_path = true;
	int path_count = 0;
    //create a path between each waypoint
    for (int i = 0; i + 1 < static_cast<int>(waypoints.size()); i++) {
	valid_path = true;
	path_count++;
        point curr_point = waypoints[i];
        if (i > 0) {
            curr_point = {path_RRT[path_RRT.size()-1].x, path_RRT[path_RRT.size()-1].y, path_RRT[path_RRT.size()-1].th};
        }
        //DEBUG// printf("%d\n\r", i);
        
        //instance of rrtTree class for each iteration
        //create the rrtTree for the next goal
        rrtTree tree (curr_point, waypoints[i+1], map, map_origin_x, map_origin_y, res, margin);
        
        //generate the search tree
        int notok = tree.generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);
        
        //check if anything was generated
        if (notok) {
            printf("generate RRT failed\n\r");
            valid_path = false;
            
        }
        
        //generate the path, store it
        one_path = tree.backtracking_traj(MaxStep);
        
        //DEBUG//
        printf("onepathsize = %d\n\r", static_cast<int>(one_path.size()));
        if (static_cast<int>(one_path.size()) == 0) {
            printf("generate path failed. Makes a new path. \n\r");
            valid_path = false;
            
        }
        point last_point = {one_path[0].x, one_path[0].y, one_path[0].th};
        if (dist(last_point, waypoints[i+1]) > 2.5) {
            valid_path = false;
            printf("generate path failed. Makes a new path. \n\r");
          
        }
        
        //add the path to the overall path
	if(valid_path){
		for (int j = static_cast<int>(one_path.size()) - 1; j >= 0; j--) {
		    
		    path_RRT.push_back(one_path[j]);
		}
	path_count = 0;
	} else {
	one_path.clear();
	i--;
	}
	if (path_count == 3) {
	break;
	one_path.clear();
	}
        
        //ensure the next path is aware of car's current heading direction
        waypoints[i+1].th = path_RRT[path_RRT.size()-1].th;
	tree.visualizeTree(path_RRT);
	sleep(5);
    }
    if(!valid_path) {
        path_RRT.clear();
        generate_path_RRT();
    }

}

double dist(point p1, point p2) {
	return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}

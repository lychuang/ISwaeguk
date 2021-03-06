#include "rrtTree.h"
#include <unistd.h>
#include <ros/ros.h>
#define PI 3.14159265358979323846

double max_alpha = 0.2;
double L = 0.325;

rrtTree::rrtTree() {
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::rrtTree(point x_init, point x_goal) {
    this->x_init = x_init;
    this->x_goal = x_goal;

    std::srand(std::time(NULL));
    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
    root->alpha = 0;
    root->d = 0;
    root->has_chiled = false;
    root->valid_node = true;
    root->length = 0;
}

rrtTree::~rrtTree(){
    for (int i = 1; i <= count; i++) {
       // delete ptrTable[i];
    }
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin) {
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map.clone();
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;
    std::srand(std::time(NULL));

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
    root->has_chiled = false;
    root->valid_node = true;
    root->length = 0;

}

cv::Mat rrtTree::addMargin(cv::Mat map, int margin) {
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - margin; k <= i + margin; k++) {
                    for (int l = j - margin; l <= j + margin; l++) {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}

void rrtTree::visualizeTree(){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);
    
    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
	    for(int j = 0; j < 10; j++) {
	        double alpha = this->ptrTable[i]->alpha;
	        double d = this->ptrTable[i]->d;
	        double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
	        double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
	        double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	    }
    }
    cv::namedWindow("Mapping");
    cv::imshow("Mapping", imgResult);
    cv::waitKey(1);
}

void rrtTree::visualizeTree(std::vector<traj> path){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res*(path[0].y/res + map_origin_y)), (int)(Res*(path[0].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(path[path.size()-1].y/res + map_origin_y)), (int)(Res*(path[path.size()-1].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
	    for(int j = 0; j < 10; j++) {
	        double alpha = this->ptrTable[i]->alpha;
	        double d = this->ptrTable[i]->d;
	        double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
	        double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
	        double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	    }
    }

    thickness = 3;
    for(int i = 1; i < path.size(); i++) {
	    for(int j = 0; j < 10; j++) {
	        double alpha = path[i].alpha;
	        double d = path[i].d;
	        double p1_th = path[i-1].th + d*j/10*tan(alpha)/L; // R = L/tan(alpha)
            double p2_th = path[i-1].th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = path[i-1].x + L/tan(alpha)*(sin(p1_th) - sin(path[i-1].th));
	        double p1_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p1_th));
            double p2_x = path[i-1].x + L/tan(alpha)*(sin(p2_th) - sin(path[i-1].th));
	        double p2_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	    }
    }
    cv::namedWindow("Mapping");
    cv::imshow("Mapping", imgResult);
    cv::waitKey(1);
}





void rrtTree::addVertex(point x_new, point x_rand, int idx_near, double alpha, double d) {
	point xNew = x_new;
	point xRand = x_rand;
 	node* new_vertex = new node;
	ptrTable[count] = new_vertex;
   	new_vertex -> idx = count;
    	new_vertex -> idx_parent = idx_near;
    	new_vertex -> location = xNew;
    	new_vertex -> rand = xRand; 
	new_vertex -> alpha = alpha;
	new_vertex -> d = d;
        new_vertex -> has_chiled = false;
	new_vertex -> valid_node = true;
	new_vertex -> length = (ptrTable[idx_near]->length) + d;
	ptrTable[idx_near] -> has_chiled = true;
	

	count++;	
}





int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {

    point x_rand;
    point x_new;
    int x_near;
    int node;
    double* out = new double[5];

    

    
    out[0] = x_max + 1; //failure case

    int noCollision;
	srand(time(NULL));
    for (int i = 0; i <= K; i++) {
        x_rand = randomState(x_max, x_min, y_max, y_min);
        x_near = nearestNeighbor(x_rand, MaxStep);
	if (x_near < 0 || x_near >= count){
		continue;
	}
        
        noCollision = newState(out, ptrTable[x_near] -> location, x_rand, MaxStep);	
        
    	if (noCollision) {

            x_new.x = out[0];
            x_new.y = out[1];
            x_new.th = out[2];

            //printf("%d x near\n\r", x_near);
            addVertex(x_new, x_rand, x_near, out[3], out[4]);
        }
    }

    if (out[0] == x_max + 1) { //we unsuccessfully generated vertices

        return 1;
    }

    return 0;
}



point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min) {
    point p; // this might not be random but fix that later.

    p.x = (x_max - x_min) * ((double)rand() / (double)RAND_MAX) + x_min;
    p.y = (y_max - y_min) * ((double)rand() / (double)RAND_MAX) + y_min;
    return p;
}




int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {
    double max_th = MaxStep * tan(max_alpha)/L; // if MaxStep is the greatest d can be it should be something like this. and if max_th is the biggest diffrens in th possible.
    double length = 10000;
    int index = 0;

    //printf("point x %f y %f\n\r", x_rand.x, x_rand.y);


    //printf("count %d\n\r", count);
    for (int i = 0; i < count; i++) {

	//the angle to x_rand from car's current direction
	    double theta_rand = atan2(ptrTable[i]->location.y - x_rand.y
					                , ptrTable[i]->location.x - x_rand.x);
	    double distance = dist(x_rand, ptrTable[i] -> location);
	
	    if ((distance < length) && ((ptrTable[i]->location.th - theta_rand) < max_th) && (ptrTable[i]->valid_node) && ((ptrTable[i]->location.th - theta_rand) > -max_th)) {

	        length = distance;
	        index = i;
            //printf("here\n\r");
	    }
    }


    return index;
}

int rrtTree::nearestNeighbor(point x_rand) {

    double length = 10000;
    int index = 0;

    for (int i = 0; i < count; i++) {

	double distance = dist(x_rand, ptrTable[i]->location);

	if ((distance < length) and (ptrTable[i]->valid_node)) {
	    length = distance; 
	    index = i;
	}
    }
    return index;
}



//private function for calculating distance between 2 points
double rrtTree::dist(point p1, point p2) {
	return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}




//return 1 if there is no collision and new state is valid
//return 0 if there is a collision - invalid new state
int rrtTree::newState(double *out, point x_near, point x_rand, double MaxStep) {

    //store the shortest distance & the corresponding point
    double shortest_dist = 10000;
    point x_best;
    double alph_best;
    double d_best;
	x_best.x = 10000;

//generate many potential points & store the closest one to x_rand
    for (int i = 0; i < 21; i++) {

	//we test 20 angles between max_alpha and -max_alpha
    	double alph = max_alpha - (i * max_alpha / 10);

	//for each angle test 10 lengths less than or equal to MaxStep
        for (int k = 0; k < 10; k++) {
	    
	        double d = MaxStep - (k * MaxStep / 10);
	    
	        double R = L / tan(alph);
 
	        double xc = x_near.x - R * sin(x_near.th);
	        double yc = x_near.y + R * cos(x_near.th);

	        double beta = d / R;


            point x_new;
            x_new.x = xc + R*sin(x_near.th + beta);
            x_new.y = yc - R*cos(x_near.th + beta);
            x_new.th = x_near.th + beta;
   /// printf("x_new.x = %f\r\n", x_new.x);


	    
	    //if there is no collision we can then compute dist to x_rand
	        if (!isCollision(x_near, x_new, d, alph)) {
		
		        double distance = dist(x_new, x_rand);
		        if (distance == 0) {
		    
		            shortest_dist = distance;
		            x_best = x_new;

		            alph_best = alph;
		            d_best = d;

                } else if (shortest_dist > distance) {
			
                    alph_best = alph;
    			    d_best = d;
	
    	    	    shortest_dist = distance;
	    	        x_best = x_new;
		            alph_best = alph;
		            d_best = d;
	            } 
   	        }
	    
	    } 

    } 

    if (x_best.x == 10000) {

	    return 0;
    }
	
    //before returning -> assign the out array values
    out[0] = x_best.x;
    out[1] = x_best.y;
    out[2] = x_best.th;
    out[3] = alph_best;
    out[4] = d_best;
 
    return 1;//I might change this later since the spec is dumb

//OLD CODE
/*

    double yc = (pow((tan(x_near.th)*x_near.y), 2) + pow(x_near.y, 2) - pow((x_rand.x - x_near.x - x_near.y*tan(x_near.th)), 2) + pow(x_rand.y, 2))/(2*tan(x_near.th)*(x_rand.x-x_near.x - x_near.y*tan(x_near.th)) - 2*x_rand.y + 2*x_near.y*tan(x_near.th)-2*x_near.y);
    double xc = x_near.x+(x_near.y - yc)*tan(x_near.th);
    double R = sqrt(pow(xc - x_rand.x, 2) + pow(yc - x_rand.y, 2));
    double alpha = atan(L/R);
    double d = MaxStep; //am not sure if we can calculate d or not
    double B = d/R;
    point x_new;
    x_new.x = xc + R*sin(B + x_near.th);
    x_new.y = yc - R*cos(B + x_near.th);
    x_new.th = x_near.th + B;
    printf("x_new.x = %f\r\n", x_new.x);
    out[0] = x_new.x;
    out[1] = x_new.y;
    out[2] = x_new.th;
    out[3] = alpha;
    out[4] = d;

    //if there is a collision return 0
    if (isCollision(x_near, x_new, d, alpha)) {
	return 0;
    }

    //valid new state
    return 1;

*/
}


//returns true if collision
//false if noti
//params: points x1->x2 origin->goal
//        d = 

bool rrtTree::isCollision(point x_near, point x_new, double d, double a) {
// am not 100% sure that these ecuations are right?
    double th = x_near.th;
    double R = L/tan(a);
    double B = d/R;
    point x = x_near;
    //double yc = (pow((tan(x_near.th)*x_near.y), 2) + pow(x_near.y, 2) - pow((x_rand.x - x_near.x - x_near.y*tan(x_near.th)), 2) + pow(x_rand.y, 2))/(2*tan(x_near.th)*(x_rand.x-x_near.x - x_near.y*tan(x_near.th)) - 2*x_rand.y + 2*x_near.y*tan(x_near.th)-2*x_near.y);
    //double xc = x_near.x+(x_near.y - yc)*tan(x_near.th);

    double xc = x.x - R * sin(x.th);
    double yc = x.y + R * cos(x.th);


    double xp;
    double yp;
    double thp;
	
    //////////////////////////////////////

    while(dist(x, x_new) > 0.01){
        th = th + B*0.01;
	x.x = xc + R*sin(th);
	x.y = yc - R*cos(th);

	if (this->map.at<uchar>(x.x/res + map_origin_x, x.y/res + map_origin_y) == 0) {
	    return true;
	
	}
    }
	
    return false;
}

std::vector<traj> rrtTree::backtracking_traj(int MaxStep){
    int tracked_node;
    traj path_info;
	bool has_grand_chiled = false;
	int limit_keeper = 0;
	while(!has_grand_chiled) {
		tracked_node = nearestNeighbor(x_goal, MaxStep);
		if(ptrTable[tracked_node]->has_chiled)
		for(int i = 0; i < this->count; i++){
			if((ptrTable[i]->idx_parent == tracked_node) && (ptrTable[i]->has_chiled)) {
			has_grand_chiled = true;
			break;
			}
		}
		if(!has_grand_chiled){
			ptrTable[tracked_node]->valid_node = false;
			limit_keeper++;
		}
		if(limit_keeper > 50) {
		break;
		}
		
	}
    std::vector<traj> path;
    while (tracked_node > 0) {
        //traj path_info = new traj;
        path_info.x = ptrTable[tracked_node]->location.x;
        path_info.y = ptrTable[tracked_node]->location.y;
        path_info.th = ptrTable[tracked_node]->location.th;
        path_info.alpha = ptrTable[tracked_node]->alpha;
        path_info.d = ptrTable[tracked_node]->d;
        tracked_node = ptrTable[tracked_node]->idx_parent;
        path.push_back(path_info);
    }
    return path;
}

int rrtTree::closestandshortest(point p) {
	int index;
	index = nearestNeighbor(p);
	double distance = dist(p, ptrTable[index]->location);
	distance = distance + 1;
	//find the one in a arbitrary radius of the goal point that has the shortes way to it.
	for(int i = 0; i < count; i++){
		if((dist((ptrTable[i]->location), p) < distance) and ((ptrTable[i]->length) < ptrTable[index]->length) and (ptrTable[i]->valid_node)) {
			index = i;
		}
	}
	return index;
}

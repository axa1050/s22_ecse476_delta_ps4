// lidar alarm for mobot 02/16/22

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <cmath> //math library for trigonometry operations


const double MIN_SAFE_DISTANCE = 2; // set alarm if anything is within 0.5m of the front of robot
const double BOX_HALF_WIDTH = 0.2; // rectangle box half width
const double BOX_EDGE = 0.2; // rectangle box outer edge distance to center of the robot
const double PI = 3.14159265359; // pi for trigonometry

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
bool laser_alarm_=false;
sensor_msgs::LaserScan lsr; // global variable to store laser information
int ping_index_min_; // min index for the laser set
int ping_index_mid_; // max index for the laser set

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    angle_min_ = laser_scan.angle_min;
    angle_max_ = laser_scan.angle_max;
    angle_increment_ = laser_scan.angle_increment;
    angle_min_ = -2.09439516068;
    angle_max_ = 2.09439516068;
    angle_increment_ = 0.00628947513178;
    double angle_a =atan(BOX_HALF_WIDTH/MIN_SAFE_DISTANCE);
    double angle_b =atan(BOX_HALF_WIDTH/0.4);
    // determine the index boundaries
	int ping_initial_index = (int)((-angle_b - angle_min_)/angle_increment_);
	int ping_final_index = (int)((angle_b - angle_min_)/angle_increment_);
	int ping_rect_index = (int)((angle_b - angle_a)/angle_increment_);
    //int ping_index_max_ = (int) ((angle_max_ - angle_min_)/angle_increment_); 
    //int ping_index_mid_ = (int) (((PI/2) - angle_min_)/angle_increment_);
    //int ping_index_min_ = (int) (ping_index_mid_ - (ping_index_max_-ping_index_mid_));
    /*ROS_INFO("angle a %f",angle_a);
    ROS_INFO("angle b %f",angle_b);
    ROS_INFO("ping_initial_index %d",ping_initial_index);
    ROS_INFO("ping_final_index %d",ping_final_index);
    ROS_INFO("ping_rect_index %d",ping_rect_index);*/
    
    // save range measurements
    lsr.ranges = laser_scan.ranges;

    // set the angles
    double angles[ping_final_index] = {0};
    int indi = 0;
    for (indi = ping_initial_index; indi<=ping_final_index; indi++) {
    	//double temp_angle = angle_min_ + (indi)*angle_increment_;
    	//ROS_INFO("doubleangle %f",temp_angle);
        angles[indi] = angle_min_ + (indi)*angle_increment_;
    }
    	
	laser_alarm_=false;
	/*for (indi=ping_initial_index;indi<ping_initial_index+ping_rect_index;indi++){
		if(lsr.ranges[indi]<(BOX_HALF_WIDTH/abs(sin(angles[indi])))) {
			ROS_INFO("laser range %f",lsr.ranges[indi]);
			ROS_INFO("value from sin %f",MIN_SAFE_DISTANCE/abs(sin(angles[indi])));
			ROS_WARN("Danger first sin");
			laser_alarm_=true;
		}
	}*/
		
	for (indi=ping_initial_index;indi<ping_final_index;indi++){
		//ROS_INFO("laser range %f",lsr.ranges[indi]);
		//ROS_INFO("value from cos %f",MIN_SAFE_DISTANCE/abs(cos(angles[indi])));
		if(lsr.ranges[indi]<(MIN_SAFE_DISTANCE/abs(cos(angles[indi])))) {
			ROS_INFO("laser range %f",lsr.ranges[indi]);
			ROS_INFO("value from cos %f",MIN_SAFE_DISTANCE/abs(cos(angles[indi])));
			ROS_WARN("Danger  cos");
			laser_alarm_=true;
		}
	}
	
/*	for (indi=ping_final_index-ping_final_index;indi<ping_final_index;indi++){
		ROS_INFO("laser range %f",lsr.ranges[indi]);
		ROS_INFO("angle %f",angles[indi]);
		ROS_INFO("value %f",BOX_HALF_WIDTH/abs(sin(angles[indi])));
		if(lsr.ranges[indi]<(BOX_HALF_WIDTH/abs(sin(angles[indi])))) {
			//ROS_WARN("second  sin");
			laser_alarm_=true;
		}
	}*/
	
	  if (laser_alarm_ == false) { // Summary on terminal output for user
        ROS_INFO("front side is good.");
    }
    else if (laser_alarm_ == true) {
        ROS_INFO("Obstacle in front!");
    }
	
		

    // check conditions for southwest edge of the robot
   /* laser_alarm_ = false; // set alarm to false before all the checks
    double region1_cutoff_angle_ = atan(BOX_HALF_WIDTH/BOX_OUTER_DIST); // find the corner of the rectangle
    int ping_region1_cutoff_ = (int) (((region1_cutoff_angle_+PI/2) - angle_min_)/angle_increment_); // find its index
    for (indi = ping_index_max_; indi>ping_region1_cutoff_;indi--) {
        if (lsr.ranges[indi]<(BOX_HALF_WIDTH/abs(cos(angles[indi])))) { // all lasers in this part share box width
            ROS_WARN("DANGER ON LEFT! 7 O'clock!");
            laser_alarm_=true;
        }
    }

    // check conditions for west edge of the robot
    int ping_region2_cutoff_ = (int) (((-region1_cutoff_angle_+PI/2) - angle_min_)/angle_increment_); //find the other corner of the rectangle
    for (indi = ping_region1_cutoff_ ; indi>ping_region2_cutoff_;indi--) { 
        if (lsr.ranges[indi]<(BOX_OUTER_DIST/sin(angles[indi]))) { // all lasers in this part share box distance
            ROS_WARN("DANGER ON LEFT! 9 O'clock!");
            laser_alarm_=true;
        }
    }

    // check conditions for northwest edge of the robot
    for (indi = ping_region2_cutoff_; indi>ping_index_max_;indi--) {
        if (lsr.ranges[indi]<(BOX_HALF_WIDTH/abs(cos(angles[indi])))) { // all lasers in this part share box width
            ROS_WARN("DANGER ON LEFT! 11 O'clock!");
            laser_alarm_=true;
        }
    }

    if (laser_alarm_ == false) { // Summary on terminal output for user
        ROS_INFO("Left side is good.");
    }
    else if (laser_alarm_ == true) {
        ROS_INFO("Obstacle on left!");
    }*/
    
    std_msgs::Bool lidar_alarm_msg;
    lidar_alarm_msg.data = laser_alarm_;
    lidar_alarm_publisher_.publish(lidar_alarm_msg);
    std_msgs::Float32 lidar_dist_msg;
    lidar_dist_msg.data = ping_dist_in_front_;
    lidar_dist_publisher_.publish(lidar_dist_msg);   
}

int main(int argc, char **argv) {
    // c
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}




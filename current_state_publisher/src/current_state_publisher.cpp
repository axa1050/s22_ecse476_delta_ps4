//
// Created by Yuqi Hu on 2/16/22.
//

#include <ros/ros.h> //ALWAYS need to include this
#include <nav_msgs/Odometry.h>

ros::Subscriber odom_subscriber;
ros::Publisher current_state_publisher;


void odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    current_state_publisher.publish(odom_rcvd);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "current_state_publisher"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    odom_subscriber = nh.subscribe("/odom",1,odomCallback);
    current_state_publisher = nh.advertise<nav_msgs::Odometry>("/current_state", 1);
    ros::spin();
    return 0;

}

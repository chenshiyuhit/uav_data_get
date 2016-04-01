#include <stdio.h>

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

float uav_x;
float uav_y;
float uav_z;


void localposeCallBack(const geometry_msgs::PoseStampedConstPtr &msg)
{
    uav_x = msg->pose.position.x;
    uav_y = msg->pose.position.y;
    uav_z = msg->pose.position.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_data");

    ros::NodeHandle l_nh_;
    ros::Subscriber localpose_sub_;

    localpose_sub_= l_nh_.subscribe("/firefly/ground_truth/pose",1000,localposeCallBack);

	FILE *position_data_file = fopen("/home/csy/catkin_ws/src/uav_data_get/data/position_data.txt","w");

    while(ros::ok())
    {
		ros::spinOnce();
		fprintf(position_data_file,"%f	%f	%f\n",uav_x,uav_y,uav_z);
		ROS_INFO("%f    %f    %f\n",uav_x,uav_y,uav_z);
    }
    
	fclose(position_data_file);
    return 0;
}
#include <stdio.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

float uav_x;
float uav_y;
float uav_z;

FILE *position_data_file = fopen("/home/csy/catkin_ws/src/uav_data_get/data/position_data.txt","w");

void localposeCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    uav_x = msg->pose.pose.position.x;
    uav_y = msg->pose.pose.position.y;
    uav_z = msg->pose.pose.position.z;

    fprintf(position_data_file,"%f	%f	%f\n",uav_x,uav_y,uav_z);
	ROS_INFO("%f    %f    %f\n",uav_x,uav_y,uav_z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_data");
    ros::Time::init();
    ros::Rate loop_rate(10);

    ros::NodeHandle l_nh_;
    ros::Subscriber localpose_sub_;

    // localpose_sub_= l_nh_.subscribe("/hummingbird/ground_truth/odometry",1000,localposeCallBack);
    localpose_sub_= l_nh_.subscribe("/hummingbird/odometry_sensor1/odometry",1000,localposeCallBack);

    ros::spin();
    
	fclose(position_data_file);
    return 0;
}

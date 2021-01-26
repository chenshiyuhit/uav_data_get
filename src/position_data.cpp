#include <stdio.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Eigen>

float px;
float py;
float pz;
float euler_x;
float euler_y;
float euler_z;
float vx;
float vy;
float vz;
float euler_vx;
float euler_vy;
float euler_vz;

float qx;
float qy;
float qz;
float qw;

ros::Time begin;
ros::Duration t;


FILE *position_data_file = fopen("/home/csy/catkin_ws/src/uav_data_get/data/position_data.txt","w");


Eigen::Vector3d quaternionToEulerAnglesZYX(const Eigen::Quaterniond& q)
{
    Eigen::Vector3d euler_angles;
    euler_angles(0) = atan2(
        2.0 * q.w() * q.x() + 2.0 * q.y() * q.z(),
        q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
    euler_angles(1) = -asin(2.0 * q.x() * q.z() - 2.0 * q.w() * q.y());
    euler_angles(2) = atan2(
        2.0 * q.w() * q.z() + 2.0 * q.x() * q.y(),
        q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    return euler_angles;
}


void localposeCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    ros::Time::init();
    t = ros::Time::now() - begin;
    px = msg->pose.pose.position.x;
    py = msg->pose.pose.position.y;
    pz = msg->pose.pose.position.z;
    qx = msg->pose.pose.orientation.x;
    qy = msg->pose.pose.orientation.y;
    qz = msg->pose.pose.orientation.z;
    qw = msg->pose.pose.orientation.w;
    vx = msg->twist.twist.linear.x;
    vy = msg->twist.twist.linear.y;
    vz = msg->twist.twist.linear.z;
    euler_vx = msg->twist.twist.angular.x;
    euler_vy = msg->twist.twist.angular.y;
    euler_vz = msg->twist.twist.angular.z;

    Eigen::Quaterniond q(qw,qx,qy,qz);
    // Eigen::Vector3d euler_zyx = q.toRotationMatrix().eulerAngles(2, 1, 0);
    Eigen::Vector3d euler_zyx = quaternionToEulerAnglesZYX(q);
    euler_x = euler_zyx.x();
    euler_y = euler_zyx.y();
    euler_z = euler_zyx.z();

    fprintf(position_data_file,"%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f\n",t.toSec(),px,py,pz,euler_x,euler_y,euler_z,vx,vy,vz,euler_vx,euler_vy,euler_vz);
	ROS_INFO("%f    %f  %f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f\n",t.toSec(),px,py,pz,euler_x,euler_y,euler_z,vx,vy,vz,euler_vx,euler_vy,euler_vz);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_data");
    ros::Time::init();
    ros::Rate loop_rate(10);
    begin = ros::Time::now();

    ros::NodeHandle l_nh_;
    ros::Subscriber localpose_sub_;

    // localpose_sub_= l_nh_.subscribe("/hummingbird/ground_truth/odometry",1000,localposeCallBack);
    localpose_sub_= l_nh_.subscribe("/hummingbird/odometry_sensor1/odometry",1000,localposeCallBack);

    ros::spin();
    
	fclose(position_data_file);
    return 0;
}

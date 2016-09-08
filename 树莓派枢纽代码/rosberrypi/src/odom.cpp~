#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

class Odometry
{
public:
/******************************************/
ros::Publisher odom_pub; // There is no memory leak in class ROSCPP_DECL Publisher.
tf::TransformBroadcaster odom_broadcaster;
ros::Time current_time, last_time;
double last_left_ticks, last_right_ticks;
double encoder_cpr;
double radii;
double x,y,th;
/******************************************/

Odometry(){}

void InitOdometry(ros::NodeHandle &n)
{
	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
	//
	current_time = ros::Time::now();
	last_time = current_time;
	//
	last_left_ticks = 0;
	last_right_ticks = 0;
	//
	encoder_cpr = 75;
	radii = 0.025;
	x = 0;
	y = 0;
}

void UpdateOdometry(double yaw,int32_t left_ticks,int32_t right_ticks)
{
	ros::spinOnce();               // check for incoming messages
	current_time = ros::Time::now();

	//compute odometry in a typical way given the velocities of the robot
	double dt = (current_time - last_time).toSec();
	double dd = (left_ticks - last_left_ticks + last_right_ticks - last_right_ticks) / 2;
	dd = (dd / 75.0) * 0.165;
	double dx = dd * cos(yaw);
	double dy = dd * sin(yaw);
	double dth = yaw - th;
	x += dx;
	y += dy;
	th = yaw;
	//
	//ROS_INFO("X:%f Y:%f Th:%f\n",x,y,th);
	//
	last_left_ticks = left_ticks;
	last_right_ticks = right_ticks;
	//
	double vx = dx / dt;
	double vy = dy / dt;
	double vth = dth / dt;

	//since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//send the transform
	odom_broadcaster.sendTransform(odom_trans);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";

	//set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = vth;

	//publish the message
	odom_pub.publish(odom);

	last_time = current_time;
}
};

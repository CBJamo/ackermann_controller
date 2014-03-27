#include "basic_twist_integrator/basic_twist_integrator.hpp"

#include <ros/ros.h>

namespace basic_twist_integrator
{
basic_twist_integrator::basic_twist_integrator( const ros::NodeHandle &_nh, const ros::NodeHandle &_nh_priv ) :
	nh( _nh ),
	nh_priv( _nh_priv ),
	odom_callback( boost::bind(&basic_twist_integrator::odom_cb, this) )
{
}

basic_twist_integrator::~basic_twist_integrator( )
{
}

bool basic_twist_integrator::start( )
{
	if( !( odom_pub = nh.advertise<nav_msgs::Odometry>( "odom", 1, odom_callback, odom_callback, ros::VoidConstPtr( ), false ) ) )
		return false;

	odom_cb( );

	return true;
}

void basic_twist_integrator::stop( )
{
	if( twist_stamped_sub )
		twist_stamped_sub.shutdown( );

	if( odom_pub )
		odom_pub.shutdown( );
}

bool basic_twist_integrator::stat( )
{
	return odom_pub;
}

void basic_twist_integrator::odom_cb( )
{
	if( odom_pub.getNumSubscribers( ) > 0 )
	{
		if( !twist_stamped_sub && !( twist_stamped_sub = nh.subscribe( "twist", 1, &basic_twist_integrator::twist_stamped_cb, this ) ) )
			ROS_ERROR( "Failed to start joint state subscription" );
	}
	else if( twist_stamped_sub )
		twist_stamped_sub.shutdown( );
}

void basic_twist_integrator::twist_stamped_cb( const geometry_msgs::TwistStampedPtr &msg )
{
	static ros::Time last_time = msg->header.stamp;
	double dt;	

	nav_msgs::Odometry odom;
	odom.header = msg->header;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_footprint";

	//compute odometry in a typical way given the velocities of the two wheels
	dt = (odom.header.stamp - last_time).toSec();
	
	odom.twist.twist.linear.x = msg->twist.linear.x * cos(th);
	odom.twist.twist.linear.y = msg->twist.linear.x * sin(th);
	odom.twist.twist.angular.z = msg->twist.angular.z;

	x += odom.twist.twist.linear.x * dt;
	y += odom.twist.twist.linear.y * dt;
	th += odom.twist.twist.angular.z * dt;

	//set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;

	//since all odometry is 6DOF we'll need a quaternion created from yaw
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);

	//set covariances
	odom.pose.covariance[0] = .1;
	odom.pose.covariance[7] = .1;
	odom.pose.covariance[14] = 1000000000;
	odom.pose.covariance[21] = 1000000000;
	odom.pose.covariance[28] = 1000000000;
	odom.pose.covariance[35] = .1;

	odom.twist.covariance[0] = .1;
	odom.twist.covariance[7] = .1;
	odom.twist.covariance[14] = 1000000000;
	odom.twist.covariance[21] = 1000000000;
	odom.twist.covariance[28] = 1000000000;
	odom.twist.covariance[35] = .1;

	//publish the message
	odom_pub.publish(odom);

	last_time = odom.header.stamp;
}
}

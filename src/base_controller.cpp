#include "base_controller/base_controller.hpp"

#include <ros/ros.h>

namespace base_controller
{
base_controller::base_controller( const ros::NodeHandle &_nh, const ros::NodeHandle &_nh_priv ) :
	nh( _nh ),
	nh_priv( _nh_priv ),
	joint_traj_callback( boost::bind(&base_controller::joint_traj_cb, this) )
{
	nh_priv.param( "wheel_base", wheel_base, 0.2635 );
	nh_priv.param( "wheel_diam", wheel_diam, 0.0750 );
	nh_priv.param<std::string>( "left_wheel_joint", left_joint_name, "left_wheel_joint" );
        nh_priv.param<std::string>( "right_wheel_joint", right_joint_name, "right_wheel_joint" );

	joint_traj_template.header.frame_id = frame_id;
	joint_traj_template.joint_names.resize( 2 );
	joint_traj_template.points.resize( 1 );
	joint_traj_template.points[0].velocities.resize( 2 );
	joint_traj_template.joint_names[0] = left_joint_name;
	joint_traj_template.joint_names[1] = right_joint_name;
}

base_controller::~base_controller( )
{
}

bool base_controller::start( )
{
	if( !( joint_traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>( "joint_trajectory", 1, joint_traj_callback, joint_traj_callback, ros::VoidConstPtr( ), false ) ) )
		return false;

	joint_traj_cb( );

	return true;
}

void base_controller::stop( )
{
	if( twist_sub )
		twist_sub.shutdown( );

	if( joint_traj_pub )
		joint_traj_pub.shutdown( );
}

bool base_controller::stat( )
{
	return joint_traj_pub;
}

void base_controller::joint_traj_cb( )
{
	if( joint_traj_pub.getNumSubscribers( ) > 0 )
	{
		if( !twist_sub && !( twist_sub = nh.subscribe( "cmd_vel", 1, &base_controller::twist_cb, this ) ) )
			ROS_ERROR( "Failed to start twist subscription" );
	}
	else if( twist_sub )
		twist_sub.shutdown( );
}

void base_controller::twist_cb( const geometry_msgs::TwistPtr &msg )
{
	trajectory_msgs::JointTrajectoryPtr new_msg( new trajectory_msgs::JointTrajectory( joint_traj_template ) );

	new_msg->header.stamp = ros::Time::now( );

	new_msg->points[0].velocities[0] = ( 2 * msg->linear.x - msg->angular.z * wheel_base ) / wheel_diam;
	new_msg->points[0].velocities[1] = ( 2 * msg->linear.x + msg->angular.z * wheel_base ) / wheel_diam;

	joint_traj_pub.publish( new_msg );
}
}

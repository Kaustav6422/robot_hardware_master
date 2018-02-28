#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <algorithm>

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/LinearMath/Transform.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <string>
#include "collvoid_msgs/PoseTwistWithCovariance.h"
#include "collvoid_local_planner/Vector2.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/Joy.h>

template<typename T>
void getParam(const ros::NodeHandle nh, const std::string &name, T *place) 
{
    bool found = nh.getParam(name, *place);
    ROS_ASSERT_MSG (found, "Could not find parameter %s", name.c_str());
    ROS_DEBUG_STREAM_NAMED ("init", std::string("Initialized ") << name << " to " << *place);
}

template<class T>
T getParamDef(const ros::NodeHandle nh, const std::string &name, const T &default_val) 
{
    T val;
    nh.param(name, val, default_val);
    ROS_DEBUG_STREAM_NAMED ("init", std::string("Initialized ") << name << " to " << val <<
                                                   "(default was " << default_val << ")");
    return val;
}

using namespace std ;

using namespace collvoid ;

class position_share
{
	public :

		position_share() ;
		void spin() ;

    private:
    	ros::NodeHandle nh ;
    	ros::Publisher position_share_pub ;
    	ros::Subscriber odom_sub ;
    	ros::Publisher debug_pub ;
    	void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) ;
    	void update() ;
    	void init_class() ;


    	std::string my_id_;
    	std::string base_frame_, global_frame_;
    	geometry_msgs::Twist twist_;
    	ros::Time last_seen_, last_time_me_published_;
    	double publish_me_period_;
    	void publishMePoseTwist();
    	bool createMeMsg(collvoid_msgs::PoseTwistWithCovariance &me_msg, std::string target_frame);
    	bool use_polygon_footprint_, holo_robot_, controlled_;
        Vector2 holo_velocity_;
        double uninflated_robot_radius_, radius_, cur_loc_unc_radius_;
        std::vector<Vector2> minkowski_footprint_;
        geometry_msgs::PolygonStamped footprint_msg_;
        bool getGlobalPose(tf::Stamped <tf::Pose> &global_pose, std::string target_frame, const ros::Time stamp);
        bool createMeMsg(collvoid_msgs::PoseTwistWithCovariance &me_msg, std::string target_frame);

  
    	double rate ;
        ros::Time t_next ;
        ros::Duration t_delta ;
        double elapsed ;
        ros::Time then;
};

position_share::position_share()
{
	init_class() ;
}

void position_share::init_class()
{
	ros::NodeHandle private_nh("collvoid");

	//set my id
    my_id_ = nh.getNamespace();
    if (strcmp(my_id_.c_str(), "/") == 0)  // NO !
    {
        char hostname[1024];
        hostname[1023] = '\0';
        gethostname(hostname, 1023);
        my_id_ = std::string(hostname);
    }
    // remove funky "/" to get uniform name in python and here
    my_id_.erase(std::remove(my_id_.begin(), my_id_.end(), '/'), my_id_.end());

    // agent params
    my_id_ = getParamDef<std::string>(private_nh, "name", my_id_);

    private_nh.param<std::string>("base_frame", base_frame_, nh.getNamespace() + "/base_link");
    private_nh.param<std::string>("global_frame", global_frame_, "map");

	t_delta = ros::Duration(1.0 / rate)  ;
    t_next = ros::Time::now() + t_delta  ; 
    then = ros::Time::now()              ;
}

void position_share::update()
{
	ros::Time now = ros::Time::now();

	if ( now > t_next) 
	{
		elapsed = now.toSec() - then.toSec(); 






		then = now;

        ros::spinOnce();
	}
	else 
	{
		ROS_INFO_STREAM("LOOP MISSED");
	} 
}

void position_share::odom_callback(const nav_msgs::Odometry::ConstPtr &msg) 
{
    //we assume that the odometry is published in the frame of the base

    boost::mutex::scoped_lock(me_lock_);
    twist_.linear.x = msg->twist.twist.linear.x;
    twist_.linear.y = msg->twist.twist.linear.y;
    twist_.angular.z = msg->twist.twist.angular.z;

    last_seen_ = msg->header.stamp;


    if ((ros::Time::now() - last_time_me_published_).toSec() > publish_me_period_) 
    {
        last_time_me_published_ = ros::Time::now();

        publishMePoseTwist(); // publish to position_share 

    }
}

// DONE
void position_share::publishMePoseTwist() 
{
    collvoid_msgs::PoseTwistWithCovariance me_msg;
    if (createMeMsg(me_msg, global_frame_)) 
    {
        position_share_pub_.publish(me_msg);
    }
}

// DONE 
bool position_share::createMeMsg(collvoid_msgs::PoseTwistWithCovariance &me_msg, std::string target_frame) // target_frame =  global_frame = map
{
    me_msg.header.stamp = ros::Time::now();
    me_msg.header.frame_id = target_frame;
    tf::Stamped <tf::Pose> global_pose;
    if (getGlobalPose(global_pose, target_frame, me_msg.header.stamp)) // passing empty global_pose, target_frame = map
    {
        geometry_msgs::PoseStamped pose_msg;
        tf::poseStampedTFToMsg(global_pose, pose_msg);
        me_msg.pose.pose = pose_msg.pose;
    }
    else 
    {
        return false;
    }
    me_msg.twist.twist = twist_;

    me_msg.controlled = controlled_;
    me_msg.holonomic_velocity.x = holo_velocity_.x();
    me_msg.holonomic_velocity.y = holo_velocity_.y();

    me_msg.holo_robot = holo_robot_;
    me_msg.radius = (float)(uninflated_robot_radius_ + cur_loc_unc_radius_);
    me_msg.robot_id = my_id_;
    me_msg.controlled = controlled_;

    me_msg.footprint = createFootprintMsgFromVector2(minkowski_footprint_);

    return true;
}

// DONE
bool position_share::getGlobalPose(tf::Stamped <tf::Pose> &global_pose, std::string target_frame, const ros::Time stamp) // target_frame =  global_frame = map
{
    //let's get the pose of the robot in the frame of the plan
    global_pose.setIdentity();
    global_pose.frame_id_ = base_frame_;
    global_pose.stamp_ = stamp;
    //global_pose.setRotation(tf::createIdentityQuaternion());
    try 
    {
        tf_->waitForTransform(target_frame, base_frame_, global_pose.stamp_, ros::Duration(0.2));
        tf_->transformPose(target_frame, global_pose, global_pose);
    }
    catch (tf::TransformException ex) {
        ROS_WARN_THROTTLE(2,"%s", ex.what());
        ROS_WARN_THROTTLE(2, "point odom transform failed");
        return false;
    };
    return true;
}

void position_share::spin()
{
    ros::Rate loop_rate(rate);
    while (ros::ok())
	{
	   update();
	   loop_rate.sleep();
	}
}  

int main(int argc, char **argv)
{ 

  ros::init(argc, argv, "position_share");

  position_share position_share1 ;

  position_share1.spin() ;

  return 0 ;
}
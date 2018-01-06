#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>

class Odometry_calc
{

public:
	Odometry_calc();
	void spin();
private:
	ros::NodeHandle n;

        double arduino_wL ;
        double arduino_wR ;
        double arduino_theta ;
        ros::Time arduino_timestamp ;
        double arduino_vd ;
        double arduino_wd ;
        double Radius ;
        double Length ;
        double odom_vd ;
        double odom_wd ;

        ros::Subscriber arduino_rpm_sub ;
	ros::Publisher odom_pub  ;
	tf::TransformBroadcaster odom_broadcaster;
        tf::TransformBroadcaster map_broadcaster ;

	double rate;
	ros::Duration t_delta;
	ros::Time t_next;
	ros::Time then;
        ros::Time current_time, last_time;

	double dx;
	double dr;
        double dxy;
        double dtheta ;
	double x_final,y_final, theta_final;

        double vd ;
        double wd ;

        void arduino_rpm_callback(const geometry_msgs::Vector3Stamped& rpm);
	void init_variables();
	void update();
};

Odometry_calc::Odometry_calc()
{
	init_variables();

	ROS_INFO("Started odometry computing node");

        // Subscribe
        arduino_rpm_sub = n.subscribe("arduino_vel",50,&Odometry_calc::arduino_rpm_callback, this); // change (change it in arduino)

        // Publish odom
  	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);  //change

       
}

void Odometry_calc::init_variables()
{
        Radius     = 0.045 ;
        Length     = 0.53  ;
        arduino_wL = 0 ;
        arduino_wR = 0 ;
        arduino_theta = 0 ;
        arduino_vd = 0 ;
        arduino_wd = 0 ;

	rate = 20;

	t_delta = ros::Duration(1.0 / rate);
	t_next = ros::Time::now() + t_delta;
	then = ros::Time::now();

	x_final = 0;
        y_final=0;
        theta_final=0;
	
	current_time = ros::Time::now();
  	last_time = ros::Time::now();
}

//Spin function
void Odometry_calc::spin()
{
     ros::Rate loop_rate(rate);

     while (ros::ok())
	{
		update();
		loop_rate.sleep();
	}
}

//Update function
void Odometry_calc::update()
{

	ros::Time now = ros::Time::now();
	
        // ros::Time elapsed;

	double elapsed;


        double dtheta ;
        double dxy ;
        double dx ;
        double dy ;  

	if ( now > t_next) 
        {

		elapsed = now.toSec() - then.toSec(); 
		
                arduino_vd = Radius*(arduino_wL + arduino_wR)/2 ;
                arduino_wd = Radius*(arduino_wL - arduino_wR)/Length ;
               
                dxy    = arduino_vd * elapsed ; //arduino_dt ;
                dtheta = arduino_wd * elapsed ; //arduino_dt ;
                dx =  cos(dtheta) * dxy ;
                dy = -sin(dtheta) * dxy ;
                x_final = x_final + (cos(theta_final)*dx - sin(theta_final)*dy) ;
                y_final = y_final + (sin(theta_final)*dx + cos(theta_final)*dy) ;
                theta_final = theta_final + dtheta ;

                if (theta_final >= 2*3.14)
                   theta_final = theta_final - 2*3.14 ;
                if (theta_final <= -2*3.14)
                   theta_final = theta_final + 2*3.14 ;

                //x_final = x_final + cos(theta_final)*vd*elapsed ;
                //y_final = y_final + sin(theta_final)*vd*elapsed ;
                //theta_final = theta_final + wd*elapsed ;

                //first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = now;
		odom_trans.header.frame_id = "robot_0/odom";                                //change(this affects the tf tree)
		odom_trans.child_frame_id = "robot_0/base_link";                            //change(this affects the tf tree)
                odom_trans.transform.translation.x = x_final;
		odom_trans.transform.translation.y = y_final;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta_final);
                odom_broadcaster.sendTransform(odom_trans);

                /*
                geometry_msgs::TransformStamped map_trans;
		map_trans.header.stamp = now;
		map_trans.header.frame_id = "map";                            //change(this affects the tf tree)
		map_trans.child_frame_id = "robot_0/odom";                            //change(this affects the tf tree)
                map_trans.transform.translation.x = 0.0;
		map_trans.transform.translation.y = 0.0;
		map_trans.transform.translation.z = 0.0;
		map_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);
                map_broadcaster.sendTransform(map_trans);  
                */

                // Publish odometry data over ROS 
                nav_msgs::Odometry odom;
		odom.header.stamp = now;
		odom.header.frame_id = "robot_0/odom";                                     //change
                odom.child_frame_id = "robot_0/base_link";                                 //change
                odom.pose.pose.position.x = x_final;
		odom.pose.pose.position.y = y_final;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_final);
                /*if (arduino_wL == 0 && arduino_wR == 0)
                {
                   odom.pose.covariance[0] = 1e-9;
                   odom.pose.covariance[7] = 1e-3;
                   odom.pose.covariance[8] = 1e-9;
                   odom.pose.covariance[14] = 1e6;
                   odom.pose.covariance[21] = 1e6;
                   odom.pose.covariance[28] = 1e6;
                   odom.pose.covariance[35] = 1e-9;
                   odom.twist.covariance[0] = 1e-9;
                   odom.twist.covariance[7] = 1e-3;
                   odom.twist.covariance[8] = 1e-9;
                   odom.twist.covariance[14] = 1e6;
                   odom.twist.covariance[21] = 1e6;
                   odom.twist.covariance[28] = 1e6;
                   odom.twist.covariance[35] = 1e-9;
                }
                else
                {
                   odom.pose.covariance[0] = 1e-3;
                   odom.pose.covariance[7] = 1e-3;
                   odom.pose.covariance[8] = 0.0;
                   odom.pose.covariance[14] = 1e6;
                   odom.pose.covariance[21] = 1e6;
                   odom.pose.covariance[28] = 1e6;
                   odom.pose.covariance[35] = 1e3;
                   odom.twist.covariance[0] = 1e-3;
                   odom.twist.covariance[7] = 1e-3;
                   odom.twist.covariance[8] = 0.0;
                   odom.twist.covariance[14] = 1e6;
                   odom.twist.covariance[21] = 1e6;
                   odom.twist.covariance[28] = 1e6;
                   odom.twist.covariance[35] = 1e3;
                }*/

                //odom_vd = (arduino_dt == 0) ? 0 : dxy/arduino_dt ;
                //odom_wd = (arduino_dt == 0) ? 0 : dtheta/arduino_dt ; 
                odom.twist.twist.linear.x = arduino_vd;
		odom.twist.twist.linear.y = 0;
		odom.twist.twist.angular.z = arduino_wd;

                
                odom_pub.publish(odom);


                then = now;
                ros::spinOnce();
          }
}
            
void Odometry_calc::arduino_rpm_callback(const geometry_msgs::Vector3Stamped& rpm)
{
   arduino_wL = rpm.vector.x ;
   arduino_wR = rpm.vector.y ;
   arduino_theta = rpm.vector.z ;
   arduino_timestamp = rpm.header.stamp ;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv,"diff_tf");
	Odometry_calc obj;
	obj.spin();
	return 0;
}

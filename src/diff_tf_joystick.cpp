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
        double IMU_theta ;
        double IMU_theta_prev ;
        double arduino_IMUomega ;
        double encoder_theta ;
        ros::Time arduino_timestamp ;
        double arduino_vd ;
        double arduino_wd ;
        double Radius ;
        double Length ;
        double odom_vd ;
        double odom_wd ;

        ros::Subscriber arduino_rpm_sub ;
        ros::Subscriber joy_sub ;
	ros::Publisher odom_pub;
        ros::Publisher cmd_vel_pub ;
	tf::TransformBroadcaster odom_broadcaster;

        ros::Publisher debug_pub ;
        geometry_msgs::Twist debug_msg ;

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
        double joystick_vd ;
        double joystick_wd ;
        

        void arduino_rpm_callback(const geometry_msgs::Twist& rpm);
        void joystick_callback(const sensor_msgs::Joy::ConstPtr& joy);
	void init_variables();
	void update();
};

Odometry_calc::Odometry_calc()
{
	init_variables();

	ROS_INFO("Started odometry computing node");

        // Subscribe
        arduino_rpm_sub = n.subscribe("/arduino_vel",50,&Odometry_calc::arduino_rpm_callback, this);

        // Subcribe to joystick topic
        joy_sub = n.subscribe("joy",50,&Odometry_calc::joystick_callback,this) ;
        //joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
 
        // Publish
  	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50); 

        // Publish cmd_vel
        cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 50) ; 

        // Publish debug message
        debug_pub = n.advertise<geometry_msgs::Twist>("debug",50);
}

void Odometry_calc::init_variables()
{
        Radius     = 0.045 ;
        Length     = 0.55  ;
        arduino_wL = 0 ;
        arduino_wR = 0 ;
        IMU_theta = 0 ;
        IMU_theta_prev = 0 ;
        encoder_theta = 0 ;
        arduino_vd = 0 ;
        arduino_wd = 0 ;

	rate = 50;

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

        double arduino_wd_ENC ;
        double arduino_wd_IMU ; 

	if ( now > t_next) 
        {

		elapsed = now.toSec() - then.toSec(); 
		
                arduino_vd = Radius*(arduino_wL + arduino_wR)/2 ;
                //arduino_wd = Radius*(arduino_wL - arduino_wR)/Length ;
                //arduino_wd_ENC = Radius*(arduino_wL - arduino_wR)/Length ;
                arduino_wd_IMU = arduino_IMUomega ;
                //IMU_theta = IMU_theta + arduino_wd_IMU*elapsed ; 
                //arduino_wd_IMU = (arduino_theta - arduino_theta_prev)/elapsed ;
                arduino_wd = arduino_wd_IMU ; //(arduino_wd_ENC + arduino_wd_IMU)/2 ; (USING ONLY IMU data for wd)
                //arduino_theta_prev = arduino_theta ;
               
                dxy    = arduino_vd * elapsed ; //arduino_dt ;
                dtheta = arduino_wd * elapsed ; //arduino_dt ;
                dx =  cos(dtheta) * dxy ;
                dy = -sin(dtheta) * dxy ;
                theta_final = theta_final + dtheta ;
                x_final = x_final + (cos(theta_final)*dx - sin(theta_final)*dy) ;
                y_final = y_final + (sin(theta_final)*dx + cos(theta_final)*dy) ;
                

                if (theta_final >= 3.14159)
                   theta_final = theta_final - 2*3.14159 ;
                if (theta_final <= -3.14159)
                   theta_final = theta_final + 2*3.14159 ;

                //x_final = x_final + cos(theta_final)*vd*elapsed ;
                //y_final = y_final + sin(theta_final)*vd*elapsed ;
                //theta_final = theta_final + wd*elapsed ;

                //first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = now;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
                odom_trans.transform.translation.x = x_final;
		odom_trans.transform.translation.y = y_final;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta_final);
                odom_broadcaster.sendTransform(odom_trans); 

                // Publish odometry data over ROS 
                nav_msgs::Odometry odom;
		odom.header.stamp = now;
		odom.header.frame_id = "odom";
                odom.child_frame_id = "base_link";
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

                geometry_msgs::Twist cmd_data ;
                cmd_data.linear.x = -joystick_vd ; // "-ve" to make control signal positive
                cmd_data.linear.y = 0;
                cmd_data.linear.z = 0 ;
                cmd_data.angular.x = 0   ;      
                cmd_data.angular.y = 0   ;     
                cmd_data.angular.z = -joystick_wd   ;     
                cmd_vel_pub.publish(cmd_data); 

                // Debug message
                debug_msg.linear.x = (theta_final)*(180/3.14159) ;
                debug_msg.linear.y = 0 ;
                debug_msg.linear.z = 0 ;
                debug_msg.angular.x = arduino_wd_ENC ;
                debug_msg.angular.y = arduino_wd_IMU ;
                debug_msg.angular.z = arduino_wd ;
                debug_pub.publish(debug_msg);

                then = now;
                ros::spinOnce();

          }
}
            
void Odometry_calc::arduino_rpm_callback(const geometry_msgs::Twist& rpm)
{
   arduino_wL = rpm.linear.x ;
   arduino_wR = rpm.linear.y ;
   arduino_IMUomega = rpm.angular.z ;
   //arduino_timestamp = rpm.header.stamp ;
}

void Odometry_calc::joystick_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
   joystick_vd = joy->axes[1] ;
   joystick_wd = joy->axes[2] ;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv,"diff_tf");
	Odometry_calc obj;
	obj.spin();
	return 0;
}

/*
 * base_teleop_joy.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: yusaku
 */

/*
 * base_teleop_joy.cpp
 *
 *  Created on: Aug 27, 2019
 *      Author: shun
 */


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <math.h>

#define pi 3.14159265

class BaseTeleop
{
public:
    BaseTeleop();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;

    double max_lin;
    double max_lin_turbo;
    double max_ang;  
    double max_ang_turbo;
    
    static int ButtonRightThumb;
    static int ButtonLeftThumb; 
    static int AxisLeftThumbX;
    static int AxisRightThumbY;
    static int AxisLeftThumbY;
    static int AxisRightThumbX;


};


int BaseTeleop::AxisLeftThumbX = 0;
int BaseTeleop::AxisLeftThumbY = 1;
int BaseTeleop::AxisRightThumbX = 2;
int BaseTeleop::AxisRightThumbY = 3;
int BaseTeleop::ButtonLeftThumb = 6;
int BaseTeleop::ButtonRightThumb =7;


BaseTeleop::BaseTeleop()
{

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &BaseTeleop::joyCallback, this);

    nh_.getParam("AxisLeftThumbX", AxisLeftThumbX);
    nh_.getParam("AxisRightThumbY", AxisRightThumbY);
    nh_.getParam("AxisLeftThumbY", AxisLeftThumbY);
    nh_.getParam("AxisRightThumbX", AxisRightThumbX);
    nh_.getParam("ButtonLeftThumb", ButtonLeftThumb);
    nh_.getParam("ButtonRightThumb", ButtonRightThumb);

    auto _nh = ros::NodeHandle("~");

    _nh.param("max_lin", this->max_lin, 1.0);
    _nh.param("max_lin_turbo", this->max_lin_turbo, this->max_lin);
    _nh.param("max_ang", this->max_ang, M_PI);
    _nh.param("max_ang_turbo", this->max_ang_turbo, this->max_ang);

    ROS_INFO("max_lin: %lf", this->max_lin);
    ROS_INFO("max_lin_turbo: %lf", this->max_lin_turbo);
    ROS_INFO("max_ang: %lf", this->max_ang);
    ROS_INFO("max_ang_turbo: %lf", this->max_ang_turbo);
}

void BaseTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;

    // double vel_x = joy->axes[AxisLeftThumbY];
    // double vel_y = joy->axes[AxisLeftThumbX];
    // double vel_z = joy->axes[AxisRightThumbX];
    
    double vel_x;
    double vel_y; 
    if ((joy->axes[AxisRightThumbY] || joy->axes[AxisRightThumbX]) && (joy->axes[AxisLeftThumbY] || joy->axes[AxisLeftThumbX]))
    {
     vel_x = 0;
     vel_y = 0;
    }
    else if (joy->axes[AxisRightThumbY] || joy->axes[AxisRightThumbX])
    {
    vel_x = joy->axes[AxisRightThumbY];
    vel_y = -(joy->axes[AxisRightThumbX]);
    }
    else if (joy->axes[AxisLeftThumbY] || joy->axes[AxisLeftThumbX])
    {
    vel_x = (joy->axes[AxisLeftThumbY])*sin(7.0*pi/6.0)-(joy->axes[AxisLeftThumbX])*cos(7.0*pi/6.0);
    vel_y = (joy->axes[AxisLeftThumbY])*sin(2.0*pi/3.0)-(joy->axes[AxisLeftThumbX])*cos(2.0*pi/3.0);
    }

    double vel_z;
    if (joy->buttons[ButtonLeftThumb] && joy->buttons[ButtonRightThumb]){
    vel_z = 0;}
    else if (joy->buttons[ButtonLeftThumb]){
    vel_z = joy->buttons[ButtonLeftThumb];}
    else if (joy->buttons[ButtonRightThumb]){
    vel_z = -(joy->buttons[ButtonRightThumb]);}

    double vel_norm = hypot(vel_x, vel_y);
    if (vel_norm > 1.0)
    {
        vel_x /= vel_norm;
        vel_y /= vel_norm;
    }


        vel_x *= this->max_lin;
        vel_y *= this->max_lin;
        vel_z *= this->max_ang;


    twist.linear.x = vel_x;
    twist.linear.y = vel_y;
    twist.angular.z = vel_z;

    vel_pub_.publish(twist);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_teleop_joy");

    BaseTeleop baseTeleop;

    ros::spin();
}


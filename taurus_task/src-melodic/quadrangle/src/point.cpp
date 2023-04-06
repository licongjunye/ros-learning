#include<ros/ros.h>
#include"turtlesim/Pose.h"
#include"geometry_msgs/Twist.h"
#include<cmath>
#include<iostream>

using namespace std;

void PID_init();
float PID_realize(float speed);

struct _pid{
float SetSpeed;//定义设定值
float ActualSpeed;//定义实际值
float err;//定义偏差值
float err_last;//定义上一个偏差值
float Kp,Ki,Kd;//定义比例、积分、微分系数
float voltage;//定义电压值(控制执行器的变量)
float integral;//定义积分值
}pid;


float originX,originY; //乌龟初始位置
float x, y, theta,lv,av;  //当前地图位置
bool isOrigin = true;  //记录乌龟第一次位置


void doPose(const turtlesim::Pose::ConstPtr& p)
{
  ROS_INFO("turtlesim pose is: x=%0.2f,y=%0.2f,theta=%0.2f,lv=%0.2f,av=%0.2f",
          p->x,p->y,p->theta,p->linear_velocity,p->angular_velocity);
  cout<<p->x<<p->y<<p->theta<<p->linear_velocity<<p->angular_velocity;
  x = p->x;
  y = p->y;
  theta = p->theta;
  lv = p->linear_velocity;
  av = p->angular_velocity;

  if(isOrigin)
  {
    originX = p->x;
    originY = p->y;
    isOrigin = false;
  }

}

int main(int argc,char *argv[])
{
  PID_init();
  float pose[4][2] = {{5.544445, 8.544445},{6.544445,7.544445},{5.544445, 6.544445},{6.544445, 5.544445}};
  setlocale(LC_ALL,"");
  ros::init(argc,argv,"sub_pose");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<turtlesim::Pose>("turtle1/pose",10,doPose);
  ros::Publisher cmdVelPub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 100);
  ros::Rate rate(1);
  geometry_msgs::Twist speed;

  while(ros::ok())
  {
    float tspeed=PID_realize(pose[0][1] - y);

    speed.linear.x = 0;
    // speed.linear.y = (pose[0][1] - y) * 0.25;
    speed.linear.y = tspeed;
    speed.linear.z = 0;
    speed.angular.x = 0;
    speed.angular.y = 0;
    speed.angular.z = 0;
    ROS_INFO("speedX is %0.2f,speedY is %0.2f",speed.linear.x,speed.linear.y);
    cmdVelPub.publish(speed);
    ros::spinOnce();
    rate.sleep();
    
  }

  return 0;

}


void PID_init()
{
  cout<<"PID_init begin \n";
  pid.SetSpeed=0.0;
  pid.ActualSpeed=0.0;
  pid.err=0.0;
  pid.err_last=0.0;
  pid.voltage=0.0;
  pid.integral=0.0;
  pid.Kp=1;
  pid.Ki=0.0005;
  pid.Kd=0;
  cout<<"PID_init end \n";
}


float PID_realize(float speed)
{
  pid.SetSpeed=speed;
  pid.err=pid.SetSpeed-pid.ActualSpeed;
  pid.integral+=pid.err;
  pid.voltage=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
  pid.err_last=pid.err;
  pid.ActualSpeed=pid.voltage*1.0;
  return pid.ActualSpeed;
}
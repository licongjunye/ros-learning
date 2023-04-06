#include<ros/ros.h>
#include"turtlesim/Pose.h"
#include"geometry_msgs/Twist.h"
#include<cmath>

float originX,originY;
float x, y, theta,lv,av;
bool isOrigin = true;

void doPose(const turtlesim::Pose::ConstPtr& p)
{
  ROS_INFO("turtlesim pose is: x=%0.2f,y=%0.2f,theta=%0.2f,lv=%0.2f,av=%0.2f",
          p->x,p->y,p->theta,p->linear_velocity,p->angular_velocity);

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
  int width = 3;
  int height = 3;

  setlocale(LC_ALL,"");
  ros::init(argc,argv,"sub_pose");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<turtlesim::Pose>("turtle1/pose",100,doPose);
  ros::Publisher cmdVelPub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 100);
  ros::Rate rate(2);
  geometry_msgs::Twist speed;

  int count = 1;
  while(ros::ok())
  {
    if(count == 1)
    {
      speed.linear.x =(originX + width - x)*0.5;
      speed.linear.y = 0;
      speed.linear.z = 0;
      speed.angular.x = 0;
      speed.angular.y = 0;
      speed.angular.z = 0;
      ROS_INFO("111vel is:%.2d",speed.linear.x);
      if(x == originX + width){
        speed.angular.z = M_PI;
        ROS_INFO("11111 is over");
        count+=1;
      }
      
    }


    if(count == 2 && speed.angular.z == 0)
    {
      speed.linear.x =(originY + height - y)*0.5;
      speed.linear.y = 0;
      speed.linear.z = 0;
      speed.angular.x = 0;
      speed.angular.y = 0;
      speed.angular.z = 0;
      ROS_INFO("222vel is:%.2d",speed.linear.x);
      if(y == originY + height){
        speed.angular.z = M_PI;
        ROS_INFO("2222 is over");
        count+=1;
      }
    }

    if(count == 3 && speed.angular.z == 0)
    {
      speed.linear.x =(x - originX)*0.5;
      speed.linear.y = 0;
      speed.linear.z = 0;
      speed.angular.x = 0;
      speed.angular.y = 0;
      speed.angular.z = 0;
      ROS_INFO("333vel is:%.2d",speed.linear.x);
      if(x == originX){
        speed.angular.z = M_PI;
        ROS_INFO("3333 is over");
        count+=1;
      }
    }

    if(count == 4 && speed.angular.z == 0)
    {
      speed.linear.x =(y - originY)*0.5;
      speed.linear.y = 0;
      speed.linear.z = 0;
      speed.angular.x = 0;
      speed.angular.y = 0;
      speed.angular.z = 0;
      // ROS_INFO("444vel is:%.2d",speed.linear.x);
      if(x == originX && y == originY){
        // speed.angular.z = M_PI;
        ROS_INFO("444 is over");
        count+=1;
      }
    }
    

    cmdVelPub.publish(speed);
    speed.angular.z = 0;
    ros::spinOnce();
    rate.sleep();
  }


  return 0;

}
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Joy.h"

class PS3Controller
{
private:
  ros::NodeHandle nh;
  ros::Publisher pub_angle;
  ros::Publisher pub_power; 
  ros::Subscriber sub;
  std_msgs::Float32 angleS;
  std_msgs::Float32 angleP;
  std_msgs::Float32 powerAft; //power of aft thruster
  std_msgs::Float32 powerP; //power of port thruster
  std_msgs::Float32 powerS; //Power of stbd thruster
  std_msgs::Float32 powerPY; //power of port for yaw
  std_msgs::Float32 powerSY; //power of stbd for yaw

public:
  PS3Controller() : nh()
  {
    pub_angle = nh.advertise<std_msgs::Float32>("angle", 1);
    pub_power = nh.advertise<std_msgs::Float32>("power", 1);
    sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &PS3Controller::callback, this);
  }
  void callback(const sensor_msgs::Joy::ConstPtr& joy)
  {
    int thrust = joy->axes[1] * -127.0; //thruster power
    powerP.data = thrust; //power of port thruster
    powerS.data = thrust; //power of stbd thruster
    powerAft.data = joy->axes[3] * -127.0; //Power for the aft thruster
    angleS.data = 90 + joy->axes[2] * -90; //Not completely sure if the
    angleP.data = 90 + joy->axes[2] * -90;  //angles are correct...

    powerPY.data = joy->axes[8] * -127.0; //"power" of port for yaw
    powerSY.data = joy->axes[9] * -127.0; //"power" of stbd for yaw

    if (powerPY.data > 0.0)
    {
     powerP.data -= powerPY.data; //makes port thruster dec as LT inc
    }
    if (powerSY.data > 0.0)
    {
     powerS.data -= powerSY.data; //makes stbd thruster dec as RT inc
    }

    pub.publish(angle);
  }
  void loop()
  {
    ros::Rate rate(10);
    while(ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ds3_node");
  PS3Controller ps3;
  ps3.loop();
}

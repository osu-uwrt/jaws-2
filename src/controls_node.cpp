#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Joy.h"

class PS3Controller
{
private:
  ros::NodeHandle nh;
  ros::Publisher pub_angle_stbd; //publisher for stbd angle
  ros::Publisher pub_angle_port; //publisher for port angle
  ros::Publisher pub_power_aftt; //publisher for aft power
  ros::Publisher pub_power_stbd; //publisher for stbd power
  ros::Publisher pub_power_port; //publisher for port power
  ros::Subscriber sub;           //subscriber for joy
  std_msgs::Float32 angle_stbd;  //angle of stbd servo
  std_msgs::Float32 angle_port;  //angle of port servo
  std_msgs::Float32 power_aftt;  //power of aft thruster
  std_msgs::Float32 power_port;  //power of port thruster
  std_msgs::Float32 power_stbd;  //Power of stbd thruster

public:
  PS3Controller() : nh()
  {
    pub_angle_stbd = nh.advertise<std_msgs::Float32>("angle_stbd", 1);
    pub_angle_port = nh.advertise<std_msgs::Float32>("angle_port", 1);
    pub_power_aftt = nh.advertise<std_msgs::Float32>("power_aftt", 1);
    pub_power_stbd = nh.advertise<std_msgs::Float32>("power_stbd", 1);
    pub_power_port = nh.advertise<std_msgs::Float32>("power_port", 1);
    sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &PS3Controller::callback, this);
  }
  void callback(const sensor_msgs::Joy::ConstPtr& joy)
  {
    float thrust = joy->axes[1] * -127.0; //thruster power
    power_port.data = thrust; //power of port thruster
    power_stbd.data = thrust; //power of stbd thruster
    power_aftt.data = joy->axes[3] * -127.0; //Power for the aft thruster
    angle_stbd.data = 90 + joy->axes[2] * -90; //Not completely sure if the
    angle_port.data = 90 + joy->axes[2] * -90;  //angles are correct...

    float powerPY = joy->axes[8] * -127.0; //"power" of port for yaw
    float powerSY = joy->axes[9] * -127.0; //"power" of stbd for yaw

    if (powerPY > 0.0)
    {
     power_port -= powerPY; //makes port thruster dec as LT inc
    }
    if (powerSY > 0.0)
    {
     power_aft -= powerSY; //makes stbd thruster dec as RT inc
    }

    pub_angle_stbd.publish(angle_stbd);
    pub_angle_port.publish(angle_port);
    pub_power_aftt.publish(power_aftt);
    pub_power_stbd.publish(power_stbd);
    pub_power_port.publish(power_port);
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

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Vector3.h"

class PS3Controller
{
private:
  ros::NodeHandle nh;
  ros::Publisher pub_angle_stbd; //publisher for stbd angle
  ros::Publisher pub_angle_port; //publisher for port angle
  ros::Publisher pub_power_vect; //publish the power vector
  ros::Subscriber sub;           //subscriber for joy
  std_msgs::Float32 angle_stbd;  //angle of stbd servo
  std_msgs::Float32 angle_port;  //angle of port servo
  geometry_msgs::Vector3 power_vect;  //creates vector of powers
public:
  PS3Controller() : nh()
  {
    pub_angle_stbd = nh.advertise<std_msgs::Float32>("angle_stbd", 1);
    pub_angle_port = nh.advertise<std_msgs::Float32>("angle_port", 1);
    pub_power_vect = nh.advertise<geometry_msgs::Vector3>("power_vect", 1);
    sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &PS3Controller::callback, this);
  }
  void callback(const sensor_msgs::Joy::ConstPtr& joy)
  {
    float thrust = joy->axes[1] * -127.0; //thruster power
    float power_port = thrust; //power of port thruster
    float power_stbd = thrust; //power of stbd thruster
    float power_port_yaw = joy->axes[8] * -127.0; //"power" of port for yaw
    float power_stbd_yaw = joy->axes[9] * -127.0; //"power" of stbd for yaw

    angle_stbd.data = 90 + joy->axes[2] * 90; //angle of stbd thruster
    angle_port.data = 90 + joy->axes[2] * 90; //angle of port thruster
    power_vect.x = joy->axes[3] * -127.0; //power for the aft thruster in x
    power_vect.y = power_port - power_port_yaw; //power for port thruster y
    power_vect.z = power_stbd - power_stbd_yaw; //power for stbd thruster z

    pub_angle_stbd.publish(angle_stbd);
    pub_angle_port.publish(angle_port);
    pub_power_vect.publish(power_vect);
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

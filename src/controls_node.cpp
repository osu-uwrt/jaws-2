#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Vector3.h"
#include "jaws_msgs/Thrusters.h"

class PS3Controller
{
private:
  ros::NodeHandle nh;
  ros::Publisher pub_thruster; //publisher for thruster message
  ros::Subscriber sub;           //subscriber for joy
  jaws_msgs::Thruster thruster; //creates thruster message
public:
  PS3Controller() : nh()
  {
    pub_thruster = nh.advertise<jaws_msgs::Thruster>("thruster", 1);
    sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &PS3Controller::callback, this);
  }
  void callback(const sensor_msgs::Joy::ConstPtr& joy)
  {
    float thrust = joy->axes[1] * -127.0; //thruster power
    float power_port = thrust; //power of port thruster
    float power_stbd = thrust; //power of stbd thruster
    float power_port_yaw = joy->axes[8] * -127.0; //"power" of port for yaw
    float power_stbd_yaw = joy->axes[9] * -127.0; //"power" of stbd for yaw

    thruster.stbd_angle = 90 + joy->axes[2] * 90; //angle of stbd thruster
    thruster.port_angle = 90 + joy->axes[2] * 90; //angle of port thruster
    thruster.aft_power = joy->axes[3] * -127.0; //power for the aft thruster
    thruster.port_power = power_port - power_port_yaw; //power for port thrust
    thruster.stbd_power = power_stbd - power_stbd_yaw; //power for stbd thrust

    pub_thruster.publish(thruster);
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

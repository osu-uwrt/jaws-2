#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "jaws_msgs/Thrusters.h"
//#include "string"

class Controls
{
  private:
    ros::NodeHandle nh;
    ros::NodeHandle parameters;
    ros::Publisher pub;
    ros::Subscriber sub;
    jaws_msgs::Thrusters thrusters;
    int stbd_thrust_mult;
    int port_thrust_mult;
    int refresh_rate;
  public:
    Controls() : nh()
    {
      sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &Controls::callback, this);
      pub = nh.advertise<jaws_msgs::Thrusters>("thrusters", 1);
    }
    void callback(const sensor_msgs::Joy::ConstPtr& joy)
    {
      nh.getParam("/controls_node/port_thrust_multiplier",port_thrust_mult);
      nh.getParam("/controls_node/stbd_thrust_multiplier",stbd_thrust_mult);	
      float raw_thrust = joy->axes[1] * 127.0;
      float port_power = raw_thrust*port_thrust_mult;      
      float stbd_power = raw_thrust*stbd_thrust_mult;
      float stbd_yaw = joy->axes[13] * -127.0;
      float port_yaw = joy->axes[12] * -127.0;

      thrusters.stbd_angle = (int)(90 + joy->axes[2] * 90);
      thrusters.port_angle = (int)(90 + joy->axes[2] * 90);
      thrusters.aft_power = (int)(joy->axes[3] * 127.0);
      thrusters.stbd_power = (int)(stbd_power - stbd_yaw);
      thrusters.port_power = (int)(port_power - port_yaw);

      pub.publish(thrusters);
    }
    void loop()
    {
      nh.getParam("/controls_node/controls/refresh_rate",refresh_rate);
      ros::Rate rate(refresh_rate);
      while(ros::ok())
      {
        ros::spinOnce();
        rate.sleep();
      }
      nh.getParam("controls_refresh_rate",refresh_rate);
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controls_node");

  Controls controls;
 
  controls.loop();
}

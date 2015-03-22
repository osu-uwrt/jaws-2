#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "jaws_msgs/Thrusters.h"
//#include "string"

const float MAX_THRUST = 500.0;

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
      nh.param("stbd_thrust_multiplier",stbd_thrust_mult,1);
      nh.param("port_thrust_multiplier",port_thrust_mult,1);
      nh.param("controls_refresh_rate",refresh_rate,10);

    }
    void callback(const sensor_msgs::Joy::ConstPtr& joy)
    {
      float raw_thrust = joy->axes[1] * MAX_THRUST;
//      float stbd_power = raw_thrust*nh.getParam("stbd_thrust_multiplier",stbd_thrust_mult);
//      float port_power = raw_thrust*nh.getParam("port_thrust_multiplier",port_thrust_mult);
      float stbd_power = (raw_thrust * raw_thrust * raw_thrust) / (MAX_THRUST * MAX_THRUST);
      float port_power = (raw_thrust * raw_thrust * raw_thrust) / (MAX_THRUST * MAX_THRUST);
      float stbd_yaw = 1 + joy->axes[13];
      float port_yaw = 1 + joy->axes[12];

      float aft_thrust = joy->axes[3] * MAX_THRUST;
      float aft_power = (aft_thrust * aft_thrust * aft_thrust) / (MAX_THRUST * MAX_THRUST); 

      thrusters.stbd_angle = (int)(90 + joy->axes[2] * 90);
      thrusters.port_angle = (int)(90 + joy->axes[2] * 90);
      thrusters.aft_power = (int)(1500 + aft_power);
      thrusters.stbd_power = (int)(1500 + stbd_power * stbd_yaw);
      thrusters.port_power = (int)(1500 + port_power * port_yaw);

      pub.publish(thrusters);
    }
    void loop()
    {
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

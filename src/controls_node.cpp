#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "jaws_msgs/Thrusters.h"

const float MAX_THRUST = 500.0;

class Controls
{
  private:
    ros::NodeHandle nh;
    ros::NodeHandle parameters;
    ros::Publisher pub;
    ros::Subscriber sub;
    jaws_msgs::Thrusters thrusters;
    int refresh_rate;
    double aft_thrust_mult;
    double stbd_thrust_mult;
    double port_thrust_mult;

  public:
    Controls() : nh()
    {
      nh.param<int>("/controls_node/controls_refresh_rate", refresh_rate, 10);
      ROS_INFO("Refresh rate: %i", refresh_rate);

      nh.param<double>("/controls_node/aft_thrust_multiplier", aft_thrust_mult, 1.0);
      ROS_INFO("Aft multiplier: %f", aft_thrust_mult);
      nh.param<double>("/controls_node/port_thrust_multiplier", port_thrust_mult, 1.0);
      ROS_INFO("Port multiplier: %f", port_thrust_mult);
      nh.param<double>("/controls_node/stbd_thrust_multiplier", stbd_thrust_mult, 1.0);
      ROS_INFO("Starboard multiplier: %f", stbd_thrust_mult);

      sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &Controls::callback, this);
      pub = nh.advertise<jaws_msgs::Thrusters>("thrusters", 1);
    }

    void callback(const sensor_msgs::Joy::ConstPtr& joy)
    {
      float raw_thrust = joy->axes[1] * MAX_THRUST;
      float stbd_power = stbd_thrust_mult * (raw_thrust * raw_thrust * raw_thrust) / (MAX_THRUST * MAX_THRUST);
      float port_power = port_thrust_mult * (raw_thrust * raw_thrust * raw_thrust) / (MAX_THRUST * MAX_THRUST);
      float stbd_yaw = 1 + joy->axes[13];
      float port_yaw = 1 + joy->axes[12];

      float aft_thrust = joy->axes[3] * MAX_THRUST;
      float aft_power = (aft_thrust * aft_thrust * aft_thrust) / (MAX_THRUST * MAX_THRUST);

      thrusters.stbd_angle = (int)(90 + joy->axes[2] * 90);
      thrusters.port_angle = (int)(90 + joy->axes[2] * 90);
      thrusters.aft_power = (int)(1500 + aft_power);
      thrusters.stbd_power = (int)(1500 + stbd_power * stbd_yaw);
      thrusters.port_power = (int)(1500 + port_power * port_yaw);

      if(joy->buttons[4]){
	stbd_thrust_mult+=0.001;
      }
      else if(joy->buttons[5]){
 	stbd_thrust_mult-=0.001;
      }
      if(joy->buttons[6]){
	port_thrust_mult+=0.001;
      }
      else if(joy->buttons[7]){
	port_thrust_mult-=0.001;
      }

      ROS_INFO("Current PTM: %4f - Current STM: %4f",port_thrust_mult,stbd_thrust_mult);
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
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controls_node");
  Controls controls;
  controls.loop();
}

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "jaws_msgs/Thrusters.h"

#define THRUSTER_CALIBRATION

class Controls
{
  private:
    ros::NodeHandle nh;
    ros::NodeHandle parameters;
    ros::Publisher pub;
    ros::Subscriber sub;
    jaws_msgs::Thrusters thrusters;
    int refresh_rate;
    double aft_cal;
    double port_cal;
    double stbd_cal;
    float neut_power;
    float neut_angle;
    float max_power;
    float max_angle;

    float curved_power(float raw, float max)
    {
      return (raw * raw * raw) / (max * max);
    }

  public:
    Controls() : nh()
    {
      max_power = 500.0;
      max_angle = 90.0;
      neut_power = 1500.0;
      neut_angle = 90.0;

      nh.param<int>("/controls_node/controls_refresh_rate", refresh_rate, 10);
      ROS_INFO("Refresh rate: %i", refresh_rate);

      nh.param<double>("/controls_node/aft_thrust_multiplier", aft_cal, 1.0);
      ROS_INFO("Aft cal. factor: %f", aft_cal);
      nh.param<double>("/controls_node/port_thrust_multiplier", port_cal, 1.0);
      ROS_INFO("Port cal. factor: %f", port_cal);
      nh.param<double>("/controls_node/stbd_thrust_multiplier", stbd_cal, 1.0);
      ROS_INFO("Stbd cal. factor: %f", stbd_cal);

      sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &Controls::callback, this);
      pub = nh.advertise<jaws_msgs::Thrusters>("thrusters", 1);
    }

    void callback(const sensor_msgs::Joy::ConstPtr& joy)
    {
      float port_angle = joy->axes[2] * max_angle;
      float stbd_angle = joy->axes[2] * max_angle;
      float aft_power = joy->axes[3] * max_power * (float)aft_cal;
      float port_power = joy->axes[1] * max_power * (float)port_cal;
      float stbd_power = joy->axes[1] * max_power * (float)stbd_cal;

      float port_yaw = 1.0 + joy->axes[12];
      float stbd_yaw = 1.0 + joy->axes[13];

      aft_power = curved_power(aft_power, max_power);
      port_power = curved_power(port_power, max_power) * port_yaw;
      stbd_power = curved_power(stbd_power, max_power) * stbd_yaw;

      thrusters.port_angle = (int)(neut_angle + port_angle);
      thrusters.stbd_angle = (int)(neut_angle + stbd_angle);
      thrusters.aft_power = (int)(neut_power + aft_power);
      thrusters.port_power = (int)(neut_power + port_power);
      thrusters.stbd_power = (int)(neut_power + stbd_power);

#ifdef THRUSTER_CALIBRATION
      if(joy->buttons[6])
      {
 	aft_cal -= 0.001;
      }
      if(joy->buttons[8])
      {
	port_cal -= 0.001;
      }
      else if(joy->buttons[7])
      {
	stbd_cal -= 0.001;
      }
      ROS_INFO("Aft cal. factor: %4f", aft_cal);
      ROS_INFO("Port cal. factor: %4f", port_cal);
      ROS_INFO("Stbd cal. factor: %4f", stbd_cal);
#endif

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

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "jaws_msgs/Thrusters.h"

#define CALIBRATION

enum Mode {Airplane, Helicopter, Dummy};

Mode& operator++(Mode &m)
{
  m = static_cast<Mode>(static_cast<int>(m) + 1);
  if(m == Dummy)
    m = Airplane;
  return m;
}

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
    float left_neut_angle;
    float right_neut_angle;
    int max_power;
    float max_angle;
    Mode mode;
    int mode_btn;

    float curved_power(float raw, float max)
    {
      return (raw * raw * raw) / (max * max);
    }

  public:
    Controls() : nh()
    {
      mode = Airplane;
      mode_btn = 0;

      max_power = 60;
      max_angle = 90.0;
      neut_power = 1500.0;
      left_neut_angle = 180.0;
      right_neut_angle = 180.0;

      nh.param<int>("/controls_node/controls_refresh_rate", refresh_rate, 20);
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
      if(!mode_btn && joy->buttons[0])
      {
        ++mode;
      }
      mode_btn = joy->buttons[0];

      float port_angle=0;
      float stbd_angle=0;
      float stbd_power=0;
      float port_power=0;
      float aft_power=0;

      if(mode == Airplane)
      {
        ROS_INFO("Airplane");

        max_angle = 90.0;
        left_neut_angle = 180.0;
        right_neut_angle = 180.0;

        float stbd_yaw=0;
        float port_yaw=0;

        if(joy->axes[14] < 0 || joy->axes[15] < 0){
          if(joy->axes[14] < 0){
            stbd_angle = max_angle;
            port_angle = -1 * max_angle;
          }
          if(joy->axes[15] < 0){
            stbd_angle = -1 * max_angle;
            port_angle = max_angle;
          }
          port_power = joy->axes[1] * max_power;
          stbd_power = joy->axes[1] * max_power;
          aft_power = -1 * joy->axes[3] * max_power * (float)aft_cal;
          port_yaw=1;
          stbd_yaw=1;
        }
        else
        {
          port_angle = joy->axes[2] * max_angle;
          stbd_angle = joy->axes[2] * max_angle;
          aft_power = -1 * joy->axes[3] * max_power * (float)aft_cal;
          port_power = -1 * joy->axes[1] * max_power * (float)port_cal;
          stbd_power = -1 * joy->axes[1] * max_power * (float)stbd_cal;
          stbd_yaw=1;
          port_yaw=1;
          if(joy->axes[12] < 0)
          {
            port_yaw = 1.0 + joy->axes[12];
            stbd_yaw = 1;
          }
          if(joy->axes[13] < 0){
            stbd_yaw = 1.0 + joy->axes[13];
            port_yaw = 1;
          }
        }
#ifdef CALIBRATION
      if(joy->buttons[6] < 0)
      {
 	left_neut_angle -= 1;
      }
      if(joy->buttons[4] < 0)
      {
	left_neut_angle += 1;
      }
      if(joy->buttons[7] < 0)
      {
	right_neut_angle -=1;
      }
      if(joy-> buttons[5] < 0){
        right_neut_angle += 1;
      }
      if(joy->buttons[14] > 0){
        max_power -=1;
      }
      if(joy->buttons[12] > 0){
        max_power +=1;
      }
      ROS_INFO("Power is %3i",max_power);
#endif
        aft_power = curved_power(aft_power, max_power);
        port_power = curved_power(port_power, max_power) * port_yaw;
        stbd_power = curved_power(stbd_power, max_power) * stbd_yaw;
      }
      else
      {
        ROS_INFO("Helicopter");

        left_neut_angle = 90.0;
        right_neut_angle = 270.0;
        max_angle = 30.0;

        float raw_angle = (joy->axes[12] - joy->axes[13]) * max_angle;
 //       port_angle = joy->axes[12] * max_angle;
 //       stbd_angle = joy->axes[13] * max_angle * -1;
        port_angle = raw_angle;
        stbd_angle = raw_angle;
        float roll = joy->axes[2] / 5.0;

        aft_power = -1 * joy->axes[3] * max_power;
        port_power = -1 * joy->axes[1] * max_power;
        stbd_power = -1 * joy->axes[1] * max_power;

        aft_power = curved_power(aft_power, max_power);
        port_power = curved_power(port_power, max_power);
        stbd_power = curved_power(stbd_power, max_power);

        port_power *= (roll + 1);
        stbd_power *= (-1 * roll + 1);
      }

      aft_power = aft_power < 0 ? aft_power : aft_power * 2;

      thrusters.port_angle = (int)(left_neut_angle + port_angle);
      thrusters.stbd_angle = (int)(right_neut_angle + stbd_angle);
      thrusters.aft_power = (int)(neut_power + aft_power);
      thrusters.port_power = (int)(neut_power + port_power);
      thrusters.stbd_power = (int)(neut_power + stbd_power);

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

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Vector3.h"
#include "jaws_msgs/Thrusters.h"
//#include "safestart.cpp"
#include "airplane.cpp"
#include "helicopter.cpp"
#include "stabilization.cpp"

enum Mode {SafeStart, Airplane, Helicopter, Dummy};

Mode& operator++(Mode &m)
{
  m = static_cast<Mode>(static_cast<int>(m) + 1);
  if(m == Dummy)
    m = SafeStart;
  return m;
}

class Controls
{
  public:
    Controls();
    void loop();
  private:
    jaws_msgs::Thrusters thrusters;
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber joy;
    ros::Subscriber imu;
    Stabilization stab;
    int this_select;
    int last_select;
    int this_start;
    int last_start;
    Mode mode;
    int refresh_rate;
    void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
    void imu_callback(const geometry_msgs::Vector3::ConstPtr& imu);
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controls_node");
  Controls controls;
  controls.loop();
}

Controls::Controls() : nh()
{
  nh.param<int>("/controls_node/controls_refresh_rate", refresh_rate, 30);
  ROS_INFO("Refresh rate: %i", refresh_rate);

  mode = SafeStart;
  last_select = 0;
  this_select = 0;
  last_start = 0;
  this_start = 0;

  joy = nh.subscribe<sensor_msgs::Joy>("joy", 1, &Controls::joy_callback, this);
  imu = nh.subscribe<geometry_msgs::Vector3>("euler_angles", 1, &Controls::imu_callback, this);
  pub = nh.advertise<jaws_msgs::Thrusters>("thrusters", 1);
}

void Controls::loop()
{
  ros::Rate rate(refresh_rate);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}

void Controls::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  this_select = joy->buttons[0];
  if(last_select && !this_select)
  {
    ++mode;
  }

  switch(mode)
  {
    case Airplane:
    break;
    case Helicopter:
    break;
    default: // SafeStart
    break;
  }

  this_start = joy->buttons[3];
  if(last_start && !this_start)
  {
    stab.on = !stab.on;
  }

  if(stab.on)
  {
    stab.stablificate(&thrusters);
  }

  last_select = this_select;
  last_start = this_start;
  pub.publish(thrusters);
}

void Controls::imu_callback(const geometry_msgs::Vector3::ConstPtr& imu)
{
  stab.set_angles(imu);
}

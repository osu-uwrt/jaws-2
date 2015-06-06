#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "jaws_msgs/Thrusters.h"
#include "airplane.cpp"
//#include "helicopter.cpp"


enum Mode {SafeStart, Airplane, Helicopter, Dummy};

Mode& Controls::Mode::operator++(Mode &m)
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
    ros::Subscriber sub;
    int this_button;
    int last_button;
    Mode mode;
    void callback(const sensor_msgs::Joy::ConstPtr& joy);
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controls_node");
  Controls controls;
  controls.loop();
}

Controls::Controls() : nh()
{
  nh.param<int>("/controls_node/controls_refresh_rate", refresh_rate, 20);
  ROS_INFO("Refresh rate: %i", refresh_rate);

  mode = SafeStart;
  last_button = 0;
  this_button = 0;

  sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &Controls::callback, this);
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

void Controls::callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  this_button = joy->buttons[0];
  if(!last_button && this_button)
    ++mode;

  switch(mode)
  {
    case Airplane:
    break;
    case Helicopter:
    break;
    default: // SafeStart
    break;
  }

  last_button = this_button;
  pub.publish(thrusters);
}

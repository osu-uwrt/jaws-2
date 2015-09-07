#include "ros/ros.h"
#include "boost/asio.hpp"
#include "jaws2_msgs/ThrustStamped.h"
#include "jaws2_msgs/PwmStamped.h"

class ThrustCal
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber ts;
    ros::Publisher pwm;
    jaws2_msgs::PwmStamped duration;
    double max_force;
    double max_pwm;
    double aft_fwd;
    double aft_rev;
    double stbd_fwd;
    double stbd_rev;
    double port_fwd;
    double port_rev;
    double calibrate(double x, double f, double r);

  public:
    ThrustCal();
    void callback(const jaws2_msgs::ThrustStamped::ConstPtr& force);
    void loop();
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thrust_cal");
  ThrustCal thrust_cal;
  thrust_cal.loop();
}

ThrustCal::ThrustCal() : nh()
{
  ts = nh.subscribe<jaws2_msgs::ThrustStamped>("thrust", 1, &ThrustCal::callback, this);
  pwm = nh.advertise<jaws2_msgs::PwmStamped>("pwm", 1);

  nh.param<double>("thruster/max/force", max_force, 25.0);
  nh.param<double>("thruster/max/pwm", max_pwm, 100.0);

  nh.param<double>("thruster/cal/aft/fwd", aft_fwd, 1.0);
  nh.param<double>("thruster/cal/aft/rev", aft_rev, 1.0);
  nh.param<double>("thruster/cal/stbd/fwd", stbd_fwd, 1.0);
  nh.param<double>("thruster/cal/stbd/rev", stbd_rev, 1.0);
  nh.param<double>("thruster/cal/port/fwd", port_fwd, 1.0);
  nh.param<double>("thruster/cal/port/rev", port_rev, 1.0);
}

void ThrustCal::callback(const jaws2_msgs::ThrustStamped::ConstPtr& force)
{
  duration.header.stamp = force->header.stamp;

  double a = force->thrust.aft;
  double s = force->thrust.stbd;
  double p = force->thrust.port;

  a = calibrate(a, aft_fwd, aft_rev);
  s = calibrate(s, stbd_fwd, stbd_rev);
  p = calibrate(p, port_fwd, port_rev);

  a *= max_pwm / max_force;
  s *= max_pwm / max_force;
  p *= max_pwm / max_force;

  duration.pwm.aft = int(a) + 1000;
  duration.pwm.stbd = int(s) + 1000;
  duration.pwm.port = int(p) + 1000;

  pwm.publish(duration);
}

void ThrustCal::loop()
{
  ros::spin();
}

double ThrustCal::calibrate(double x, double f, double r)
{
  if(x < 0.0)
  {
    x *= r;
  }
  else
  {
    x *= f;
  }
  return x;
}

#include "ros/ros.h"
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

  duration.pwm.aft = calibrate(force->thrust.aft, aft_fwd, aft_rev);
  duration.pwm.stbd = calibrate(force->thrust.stbd, stbd_fwd, stbd_rev);
  duration.pwm.port = calibrate(force->thrust.port, port_fwd, port_rev);

  pwm.publish(duration);
}

void ThrustCal::loop()
{
  ros::spin();
}

int ThrustCal::calibrate(double raw_force, double fwd_cal, double rev_cal)
{
  if(raw < 0.0)
  {
    raw_force /= rev_cal;
  }
  else
  {
    raw_force /= fwd_cal;
  }
  return 1500 + int(raw_force * max_pwm / max_force);
}

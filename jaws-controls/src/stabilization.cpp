#include "math.h"

class Stabilization
{
  public:
    Stabilization();
    void stablificate(jaws_msgs::Thrusters* thrusters);
    void set_angles(const geometry_msgs::Vector3::ConstPtr euler_angles);
    bool on;
  private:
    float pitch;
    float yaw;
    float roll;
    float desired_roll;
    float desired_pitch;
    float pitch_const;
    float roll_const;
};

Stabilization::Stabilization()
{
  on = false;
  desired_roll=0.0;
  desired_pitch=0.0;
  pitch_const = 1;
  roll_const = 1;
}

void Stabilization::stablificate(jaws_msgs::Thrusters* thrusters)
{
  float stbd_x = cos((thrusters->stbd_angle-180)*3.14159/180.0)*thrusters->stbd_power;
  float stbd_z = sin((thrusters->stbd_angle-180)*3.14159/180.0)*thrusters->stbd_power;
  float port_x = cos((thrusters->port_angle-180)*-3.14159/180.0)*thrusters->stbd_power;
  float port_z = sin((thrusters->port_angle-180)*-3.14159/180.0)*thrusters->stbd_power;
  float roll_error = roll-desired_roll;
  stbd_z = stbd_z+roll_error*roll_const;
  port_z = port_z-roll_error*roll_const;
  stbd_x = stbd_x+0.00001;
  port_x = port_x=0.00001;
  float pitch_error = pitch-desired_pitch;
  float aft = thrusters->aft_power;
  aft=aft+pitch_error*pitch_const;
  thrusters->aft_power = aft;
  thrusters->stbd_angle = 3.14159/180.0*tan(stbd_z/stbd_x)+180;
  thrusters->port_angle = -1.0*(3.14159/180.0*tan(port_z/port_x)+180);
  thrusters->stbd_power = sqrt(stbd_x*stbd_x+stbd_z*stbd_z)*stbd_x/abs(stbd_x);
  thrusters->port_power = sqrt(port_x*port_x+port_z*port_z)*port_x/abs(port_x);
}

void Stabilization::set_angles(const geometry_msgs::Vector3::ConstPtr euler_angles)
{
  //angles are in radians, deal with it
  pitch = euler_angles->y;
  yaw = euler_angles->x;
  roll = euler_angles->z;
}

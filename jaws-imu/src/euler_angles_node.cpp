#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include <jaws_imu/FilterOutput.h>
#include <math.h>

#define PI 3.14159

class ConvertToEuler {
private:
  ros::NodeHandle n;
  ros::Publisher pubEuler;
  ros::Subscriber sub;
  geometry_msgs::Vector3 eulerAngles;
public:
  ConvertToEuler() : n()
  {
     pubEuler = n.advertise<geometry_msgs::Vector3>("eulerAngles", 1);
     sub = n.subscribe<jaws_imu::FilterOutput>("imu_3dm_gx4/filter", 1, &ConvertToEuler::publishData, this);
  }
  void publishData(const jaws_imu::FilterOutput::ConstPtr& angles)
  {
    float yaw;
    float pitch;
    float roll;
    float x, y, z, w;

    x=angles->orientation.x;
    y=angles->orientation.y;
    z=angles->orientation.z;
    w=angles->orientation.w;

    yaw= atan((2*(x*y+z*w))/(x*x-y*y-z*z+w*w));
    pitch = asin(2*(y*w-x*z));
    roll = atan((2*(x*w+y*z))/(x*x+y*y-z*z-w*w));

    eulerAngles.x = yaw;
    eulerAngles.y = pitch;
    eulerAngles.z = roll;

    pubEuler.publish(eulerAngles);
  }
  void loop() {
    while(ros::ok()) {
      ros::spin();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "euler_angles_node");
  ConvertToEuler euler;
  euler.loop();

  return 0;

}

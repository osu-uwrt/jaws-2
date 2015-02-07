#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "boost/asio.hpp"
#include <string>

class SerialPort
{
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  std_msgs::Float32 angle;
  boost::asio::io_service i_o;
  boost::asio::serial_port s_p;
  using std::string;	
public:
  SerialPort() : nh(), i_o(), s_p(i_o, "/dev/ttyUSB0")
  {
  }
  void callback(const std_msgs::Float32::ConstPtr& angle)
  {
    for(int i=0;i<999;i++){

    unsigned char d = (char)i->data;
    string out = d;
    s_p.write_some(boost::asio::buffer(&d, 1));
    }
  }
  void loop()
  {
    ros::spin();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_node");
  SerialPort sp;
  sp.loop();
}

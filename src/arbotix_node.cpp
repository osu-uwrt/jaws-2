#include "ros/ros.h"
#include "boost/asio.hpp"
#include "jaws_msgs/Thrusters.h"
#include <iostream>


class Arbotix
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    boost::asio::io_service i_o;
    boost::asio::serial_port s_p;
    std::string port_name;
    int br;
  public:
    Arbotix() : nh(), i_o(), s_p(i_o)
    {
      std::cout << "Calling constructor..." << std::endl;

      nh.getParam("/arbotix_node/port_name",port_name);
      nh.getParam("/arbotix_node/baud_rate",br);
//      s_p.open(port_name);
      s_p.open("/dev/ttyUSB0");
//      s_p.set_option(boost::asio::serial_port_base::baud_rate(br));
      s_p.set_option(boost::asio::serial_port_base::baud_rate(9600));

      std::cout << "Serial port open..." << std::endl;


      sub = nh.subscribe<jaws_msgs::Thrusters>("thrusters", 1, &Arbotix::callback, this);
    }
    void callback(const jaws_msgs::Thrusters::ConstPtr& thrusters)
    {
      const int SIZE = 11;
      unsigned char packet[SIZE];

      packet[0] = '-';

      packet[1] = (thrusters->port_angle >> 8) & 0xFF;
      packet[2] = thrusters->port_angle & 0xFF;

      packet[3] = (thrusters->stbd_angle >> 8) & 0xFF;
      packet[4] = thrusters->stbd_angle & 0xFF;

      packet[5] = (thrusters->aft_power >> 8) & 0xFF;
      packet[6] = thrusters->aft_power & 0xFF;

      packet[7] = (thrusters->port_power >> 8) & 0xFF;
      packet[8] = thrusters->port_power & 0xFF;

      packet[9] = (thrusters->stbd_power >> 8) & 0xFF;
      packet[10] = thrusters->stbd_power & 0xFF;

      s_p.write_some(boost::asio::buffer(&packet, SIZE));

      std::cout << "." << std::endl;
    }
    void loop()
    {
      ros::spin();
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arbotix_node");
  Arbotix arbotix;
  arbotix.loop();
}

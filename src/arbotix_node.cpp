#include "ros/ros.h"
#include "boost/asio.hpp"
#include "jaws_msgs/Thrusters.h"


class Arbotix
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    boost::asio::io_service i_o;
    boost::asio::serial_port s_p;
    std::string port_name;
    int baud_rate;

  public:
    Arbotix() : nh(), i_o(), s_p(i_o)
    {

      nh.param<std::string>("/arbotix_node/port_name", port_name, "/dev/ttyUSB0");
      nh.param<int>("/arbotix_node/baud_rate", baud_rate, 9600);

      s_p.open(port_name);
      ROS_INFO("Serial port: %s ", port_name.c_str());
      s_p.set_option(boost::asio::serial_port_base::baud_rate(9600));
      ROS_INFO("Baud rate: %i", baud_rate);

      sub = nh.subscribe<jaws_msgs::Thrusters>("thrusters", 1, &Arbotix::callback, this);
    }

    void callback(const jaws_msgs::Thrusters::ConstPtr& thrusters)
    {
      const int SIZE = 11;
      unsigned char packet[SIZE];

      packet[0] = '-';

      packet[1] = (thrusters->port_angle >> 8);
      packet[2] = thrusters->port_angle;

      packet[3] = (thrusters->stbd_angle >> 8);
      packet[4] = thrusters->stbd_angle;

      packet[5] = (thrusters->aft_power >> 8);
      packet[6] = thrusters->aft_power;

      packet[7] = (thrusters->port_power >> 8);
      packet[8] = thrusters->port_power;

      packet[9] = (thrusters->stbd_power >> 8);
      packet[10] = thrusters->stbd_power;

      s_p.write_some(boost::asio::buffer(&packet, SIZE));
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

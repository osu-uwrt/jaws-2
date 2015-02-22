#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "boost/asio.hpp"

class SerialPort
{
private:
ros::NodeHandle nh;
ros::Subscriber sub;
std_msgs::Float32 angle;
boost::asio::io_service i_o;
boost::asio::serial_port s_p;
public:
SerialPort() : nh(), i_o(), s_p(i_o, "/dev/ttyUSB0")

{

sub = nh.subscribe<std_msgs::Float32>("angle", 1, &SerialPort::callback, this);
}
void callback(const std_msgs::Float32::ConstPtr& angle)
{
int port_angle=angle->data;
int stbd_angle;
int port_thruster;
int stbd_thruster;
int back_thruster;
unsigned char d[12];
d[0]= '-';
d[1]=port_angle & 0xFF;
d[2]=(port_angle >> 8) & 0xFF;

d[3]=stbd_angle & 0xFF;
d[4]=(stbd_angle >> 8) & 0xFF;

d[5]=port_thruster & 0xFF;
d[6]=(port_thruster >> 8) & 0xFF;

d[7]=stbd_thruster & 0xFF;
d[8]=(stbd_thruster >> 8) & 0xFF;

d[9]=back_thruster & 0xFF;
d[10]=(back_thruster >> 8) & 0xFF;



 s_p.write_some(boost::asio::buffer(&d, 12));

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

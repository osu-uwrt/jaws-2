#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "boost/asio.hpp"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Int8.h"
class SerialPort
{
private:
ros::NodeHandle nh;
ros::Subscriber subStbd;
ros::Subscriber subPort;
ros::Subscriber subPower;
std_msgs::Float32 angle;
boost::asio::io_service i_o;
boost::asio::serial_port s_p;
float angle_port;
float angle_board;
float xyz[3];
public:
SerialPort() : nh(), i_o(), s_p(i_o, "/dev/ttyUSB0")

{

subStbd = nh.subscribe<std_msgs::Float32>("angle_stbd", 1, &SerialPort::callback, this);
subPort = nh.subscribe<std_msgs::Float32>("angle_port",1, &SerialPort::callback2, this);
subPower = nh.subscribe<geometry_msgs::Vector3>("power_vect",1,&SerialPort::callback3,this);
}
void callback(const std_msgs::Float32::ConstPtr& subStbd)
{
 angle_board = subStbd->data;
writeSerial();
}

void callback2(const std_msgs::Float32::ConstPtr& subPort)
{
 angle_port = subPort->data;
writeSerial();
}

void callback3(const geometry_msgs::Vector3::ConstPtr& subPower)
{
 xyz[0]=subPower->x;
 xyz[1]=subPower->y;
 xyz[2]=subPower->z;
writeSerial();

}
void writeSerial()
{
int port_angle =floor(angle_port+.5);
int board_angle = angle_board;
int back_thruster = xyz[0];
int port_thruster = xyz[1];
int stbd_thruster = xyz[2];
unsigned char d[11];
d[0]= '-';
d[1]=port_angle & 0xFF;
d[2]=(port_angle >> 8) & 0xFF;

d[3]=board_angle & 0xFF;
d[4]=(board_angle >> 8) & 0xFF;

d[5]=port_thruster & 0xFF;
d[6]=(port_thruster >> 8) & 0xFF;

d[7]=stbd_thruster & 0xFF;
d[8]=(stbd_thruster >> 8) & 0xFF;

d[9]=back_thruster & 0xFF;
d[10]=(back_thruster >> 8) & 0xFF;

ROS_INFO("%d",port_thruster);

s_p.write_some(boost::asio::buffer(&d, 11));
}

void loop()
{
 ros::spin();
}
};
int main(int argc, char **argv)
{
ros::init(argc, argv, "servo_node_runner");
SerialPort sp;
sp.loop();
}

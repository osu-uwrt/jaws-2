#include "ros/ros.h"
#include "boost/asio.hpp"
#include "jaws_msgs/Thrusters.h"
#include "std_msgs/String.h"

#define FEEDBACK
#define RESET
#define DIAGNOSTIC_PUBLISH

class Arbotix
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    boost::asio::io_service i_o;
    boost::asio::serial_port s_p;
    std::string port_name;
    int br;
    char c;
    int timeout;
    std::string feedback;
  public:
    Arbotix() : nh(), i_o(), s_p(i_o)
    {
      nh.getParam("/arbotix_node/port_name",port_name);
      nh.getParam("/arbotix_node/baud_rate",br);
      nh.param("/arbotix_node/timeout",timeout,2000);
      s_p.open(port_name);
      s_p.set_option(boost::asio::serial_port_base::baud_rate(br));
     
      pub=nh.advertise<std_msgs::String>("arbotix_diagnostic",2);
      sub = nh.subscribe<jaws_msgs::Thrusters>("thrusters", 1, &Arbotix::callback, this);
    }
    void restartPort()
    {
	nh.getParam("/arbotix_node/port_name",port_name);
	nh.getParam("/arbotix_node/baud_rate",br);
	s_p.close();
        s_p.open(port_name);
	s_p.set_option(boost::asio::serial_port_base::baud_rate(br));
    }
    void callback(const jaws_msgs::Thrusters::ConstPtr& thrusters)
    {
      const int SIZE = 11;
      unsigned char packet[SIZE];
	
      unsigned int start=clock();
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
      c='a';
      nh.getParam("/arbotix_node/timeout",timeout);
      #ifdef FEEDBACK
      while(c!='\n'){
         c=s_p.read_some(boost::asio::buffer(&c,1));
         if(c!='\n'){
 	   feedback+=c;
	 }
	else{
	   break;
	   #ifdef DIAGNOTIS_PUBLISH
	    pub.publish(feedback);
	   #endif
	}
        #ifdef RESET
	if((clock()-start/CLOCKS_PER_SEC)>timeout){
	   restartPort();
	   feedback="Port reset, resuming";
	   break;
	}
        #endif
      }
     ROS_INFO("%s",feedback.c_str());
     #endif
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

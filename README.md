# microcontroller

#Have confirmed, it does indeed work
Arduino to Motors works, use all three cables (Power, Ground, PWM Signal) 
Need to run controller_test_node and controller_test_serial as well as ps3 controller setup stuff

I don't have the microcontroller on hand nor servos so there is no way to test this. However, it should work. One problem is that the arbotix does not support more than two traditional PWM outputs. Conor mentioned this is how we would do the thrusters so that's something we need to discuss on sunday.

## Reference Links
* [ArbotiX](http://wiki.ros.org/arbotix) - ROS metapackage.
* [Boost.Asio](http://www.boost.org/doc/libs/1_54_0/doc/html/boost_asio.html) - C++ Serial.
* [ArbotiX Intro](http://learn.trossenrobotics.com/arbotix) - Trossen Robotics
    * [Hardware](http://learn.trossenrobotics.com/arbotix/arbotix-getting-started/38-arbotix-m-hardware-overview#&panel1-1) - ArbotiX-M Overview
    * [Powering](http://learn.trossenrobotics.com/arbotix/arbotix-advanced-topics/40-powering-the-arbotix-m) - Hobby Servo BUS
* [arbotix FAQ](http://www.vanadiumlabs.com/arbotix.html) - Vanadium Labs

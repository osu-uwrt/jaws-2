# Jaws Arbotix

*Jaws 2's ArbotiX-M firmware and ROS bridge.*

## TODO
* Failsafes
    * ArbotiX serial timeout.
    * NUC watchdog reset
* Controls
    * Servo angle feedback.
    * Servo.writeMicroseconds()
* Diagnostics/Debugging
    * LED blinks error codes.
    * Last TX as Thrusters.msg.
* NUC -> ArbotiX programming procedure.

## Reference Links
* [Boost.Asio](http://www.boost.org/doc/libs/1_54_0/doc/html/boost_asio.html) - C++ Serial.
* [ArbotiX Intro](http://learn.trossenrobotics.com/arbotix) - Trossen Robotics
    * [Hardware](http://learn.trossenrobotics.com/arbotix/arbotix-getting-started/38-arbotix-m-hardware-overview#&panel1-1) - ArbotiX-M Overview
    * [Powering](http://learn.trossenrobotics.com/arbotix/arbotix-advanced-topics/40-powering-the-arbotix-m) - Hobby Servo BUS
* [Arduino Command Line Programming](https://github.com/arduino/Arduino/blob/ide-1.5.x/build/shared/manpage.adoc)

# controls
Tanner's project. I am changing this.

This link is a tutorial on how to pair a ps3 controller using ps3joy.

*** In order to power and control the thruster, all three cables must be connected.***


### Connecting a PS3 Controller

https://help.ubuntu.com/community/Sixaxis

Connect using a USB cable, then type:

    sudo sixpair
    
Disconnect the USB cable, then type:

    sixad -s
    
Press the PS button, wait for rumbling.


## Reference Links

### Linux
* [Sixaxis](https://help.ubuntu.com/community/Sixaxis) - Follow "Quick Setup Guide".
* [jstest](http://manpages.ubuntu.com/manpages/trusty/man1/jstest.1.html) - Joystick test, get it?

### ROS
* [joystick_drivers](http://wiki.ros.org/joystick_drivers?distro=indigo) - Meta package for joysticks.
* [joy](http://wiki.ros.org/joy?distro=indigo) - The package we've been learning with.
* [ps3joy](http://wiki.ros.org/ps3joy?distro=indigo) - Button/axis numbers. Do they match joy?
* [sensor_msgs](http://wiki.ros.org/sensor_msgs) - Joy message documentation.

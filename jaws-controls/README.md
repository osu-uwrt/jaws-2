# Jaws Controls

## Connecting a PS3 Controller

### Initial Pairing 

Connect the controller using a USB cable, then type:

    sudo sixpair
    
### Starting Paired

Without the controller connected to a USB cable, type:

    sixad -s
    
Press the PS button, wait for rumbling. 

## Troubleshooting

### Sixpair Fails

For:

    Current Bluetooth master: 00:22:b0:d0:5a:09
    Unable to retrieve local bd_addr from `hcitool dev`.

Try:

    sudo hciconfig hci0

If DOWN:

    sudo hciconfig hci0 up

Else:

    sudo hciconfig hci0 reset

### Sixad Fails

If controller LEDs blink quickly, reset the controller. (Pen-tip button on underside of controller.)

## Reference Links

### Linux
* [Sixaxis](https://help.ubuntu.com/community/Sixaxis) - Follow "Quick Setup Guide".
* [jstest](http://manpages.ubuntu.com/manpages/trusty/man1/jstest.1.html) - Joystick test, get it?

### ROS
* [joystick_drivers](http://wiki.ros.org/joystick_drivers?distro=indigo) - Meta package for joysticks.
* [joy](http://wiki.ros.org/joy?distro=indigo) - The package we've been learning with.
* [ps3joy](http://wiki.ros.org/ps3joy?distro=indigo) - Button/axis numbers. Do they match joy?
* [sensor_msgs](http://wiki.ros.org/sensor_msgs) - Joy message documentation.

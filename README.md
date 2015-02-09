# Jaws Messages

*Contains custom generated messages for Jaws 2.*

Built on top of [`std_msgs`](http://wiki.ros.org/std_msgs) and [`common_msgs`](http://wiki.ros.org/common_msgs).

Add `jaws_msgs` as dependency just as you would any other set of messages.

# Message API/Definitions

*Read them as you would the one for [Vector3](http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html).*

## jaws_msgs/Thrusters.msg Message

### File: `jaws_msgs/msg/Thrusters.msg`

### Raw Message Definition

    int16 stbd_angle
    int16 port_angle
    int16 aft_power
    int16 stbd_power
    int16 port_power

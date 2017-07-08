# common_interfaces

This package defines classes that are used for inter-subsystem communication in Agent/Orocos - based approach.

For more details refer to [wiki page](https://github.com/dseredyn/common_interfaces/wiki).

The package contains unit tests for:
* ROS msg concate from Orocos ports,
* ROS msg split into Orocos ports.

To build unit tests run:<return>
catkin build --catkin-make-args tests

To execute the tests run:<return>
catkin build --catkin-make-args tests -- common_interfaces



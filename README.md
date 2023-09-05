# ROS integration for Franka Emika research robots

[![CI](https://github.com/frankaemika/franka_ros/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros/actions/workflows/ci.yml)


See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

# Code developed by me

I added one controller to this repo. The controller responsible for collision avoidance to a person using the VFI approach. The last version of the controller is in this repo and not in the  "ROS_HMP_Collision_Avoidance " repo.

The controller is the code joint_velocity_side2side_controller.cpp and is in the folder franka_example_controllers/src . The name is not self descriptive because initially this controller had another function. The structure of this controller is similar to the other example controllers.

In addition to the joint_velocity_side2side_controller.cpp, it is presented in the franka_example_controllers/src folder the JacobianHMP.cpp file. This document has functions that are necessary to the developed controller (most of the functions are related to the Jacobians calculation)


## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs

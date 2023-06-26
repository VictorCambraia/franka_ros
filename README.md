# ROS integration for Franka Emika research robots

[![CI](https://github.com/frankaemika/franka_ros/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros/actions/workflows/ci.yml)


See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

# TODO

- Get the exception of the y.... in the get_pose_refactor.py (DONE, I THINK)
- Change the d_safe so it includes the std deviation as well. I am not considering it to the final calculation right now...
- See if the model that I am using right now is more or less trustworthy... Maybe not

## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs

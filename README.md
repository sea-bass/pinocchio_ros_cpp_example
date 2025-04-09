## Pinocchio ROS 2 C++ example

This is a basic example of building a ROS 2 package that uses the [Pinocchio](https://github.com/stack-of-tasks/pinocchio) library.

To run this example:

```shell
export ROS_DISTRO=humble  # or jazzy/rolling
docker compose build
docker compose run base
```

Once you're in the container, you can compile this package:

```shell
colcon build
source install/setup.bash
```

Finally, you can run the example:

```shell
ros2 run pinocchio_ros_example pinocchio_example
```

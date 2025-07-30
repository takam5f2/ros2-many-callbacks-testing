# Many Callbacks testing for ROS 2 

As the famouse economist, Milton Friedman's saying "There's no such thing as a free lunch" implies, the runtime provided by ROS 2 is not without cost.

Developers must carefully consider the resource consumption of the ROS 2 runtime, particularly in large applications that utilize a high number of executors, nodes, callbacks, and messages. Excessive use of these elements can be detrimental to application performance.

However, it is difficult to estimate the resources consumed by the ROS 2 runtime. A quantitative baseline is required for such an estimation. This repository provides test programs to establish this baseline.


```
git clone git@github.com:takam5f2/ros2-many-callbacks-testing.git
cd ros2-many-callbacks-testing
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to executors_loader
source install/setup.bash
ros2 launch executors_loader many_executors_loader.launch.py
# ros2 launch executors_loader many_executors_loader.launch.py executor_config_file:=./executors_loader/config/large_executor_config.yaml 
```

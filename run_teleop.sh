# . simulator_ws/devel/setup.sh
# roslaunch tb_gazebo teleop.launch

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PWD/nav_ws/src
roslaunch my_nav teleop.launch

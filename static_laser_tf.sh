export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PWD/nav_ws/src
# rosrun tf static_transform_publisher 0.0 0.0 0.2 0.0 0.0 0.0 1.0 base_link laser 100
roslaunch my_nav laser_static_publisher.launch

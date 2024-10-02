This project consists on using VIsual servoing to take and to classify the objects in the environment.We use a manipulator xArm6 to take and move the objects and a camera ZED Mini to locate and classify the objects The main software ("xarm_zed") has a node that it controls the robot through the package "xarm_ros2" to activate the tool or move the end-effector and the package "zed-ros2-wrapper" to receive the data of the camera.

REQUIREMENTS:

•	ROS2 (This project has been run in ROS2 Foxy Ubuntu 20.04 on a Jetson AGX Xavier: https://docs.ros.org/en/foxy/index.html)

•	Package “xarm_ros2: https://docs.ufactory.cc/xarm_ros2/readme_en ( This page of UFactory has a lot of files that it can help you with any doubt: https://www.ufactory.cc/download/)

•	Package “zed-ros2-wrapper” : https://www.stereolabs.com/docs/ros2 (If you want to use the example of ROS2 of the ZED camera like rviz2, you will need to download
“zed-ros2-examples”: https://github.com/stereolabs/zed-ros2-examples?tab=readme-ov-file#build-the-package).

•	ZED-SDK (The version of the SDK depends on the version of the Jetpack of the Nvidia Jetson or the version of the CUDA): https://www.stereolabs.com/en-es/developers/release.

•	(Optional)  UFactory Studio let you to control or to program the robot easily with a graphical interface.


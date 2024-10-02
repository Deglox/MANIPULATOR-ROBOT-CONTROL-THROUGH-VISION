In this project it makes a visual control (Visual Servoing) of a Ufactory xArm 6 manipulator from the visual feedback of the ZED Mini camera
by Stereolabs. The "xarm_zed" package has a node that it controls the robot through the package "xarm_ros2" to activate the tool or move the manipulator and the information that arrive from the the package "zed-ros2-wrapper".

REQUIREMENTS:

•	ROS2 (This project has been run in ROS2 Foxy Ubuntu 20.04 on a Jetson AGX Xavier: https://docs.ros.org/en/foxy/index.html)

•	Package “xarm_ros2: https://docs.ufactory.cc/xarm_ros2/readme_en ( In this page of UFactory you could download some user manuals and many other things that it could help you: https://www.ufactory.cc/download/)

•	Package “zed-ros2-wrapper” : https://www.stereolabs.com/docs/ros2 (If you want to use the example of ROS2 of the ZED camera like rviz, you will need to download
“zed-ros2-examples”: https://github.com/stereolabs/zed-ros2-examples?tab=readme-ov-file#build-the-package).

•	ZED-SDK (The version of the SDK depends on the version of the Jetpack of the Nvidia Jetson or the version of the CUDA): https://www.stereolabs.com/en-es/developers/release.

•	(Optional) If you use a UFactory's robot, UFactory Studio will help you, for example, to test the correct operation of the tool and to move the robot without running commnads in the terminal.


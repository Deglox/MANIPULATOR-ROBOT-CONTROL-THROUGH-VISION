En este proyecto se realiza un control visual (Visual Servoing) de un manipulador xArm 6 de Ufactory a partir de la realimentación visual de la cámara ZED Mini
de Stereolabs. El paquete "xarm_zed" se encarga de controlar el robot a partir de toda la información que se recibe desde el sistema de visión. Los requisitos 
del proyecto son:
REQUIREMENTS:

•	ROS2 (This project has been run in ROS2 Foxy Ubuntu 20.04 on a Jetson AGX Xavier)

•	Package “xarm_ros2: https://docs.ufactory.cc/xarm_ros2/readme_en

•	Package “zed-ros2-wrapper” : https://www.stereolabs.com/docs/ros2 (If you want to use the example of ROS2 of the ZED camera like rviz, you will need to download
“zed-ros2-examples”: https://github.com/stereolabs/zed-ros2-examples?tab=readme-ov-file#build-the-package).

•	ZED-SDK (The version of the SDK depends on the version of the Jetpack of the Nvidia Jetson or the version of the CUDA)


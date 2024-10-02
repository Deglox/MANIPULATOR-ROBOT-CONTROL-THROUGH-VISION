// INCLUDE THE NECESSARY .HPP FILES
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <zed_interfaces/msg/objects_stamped.hpp>
#include <zed_interfaces/msg/keypoint3_d.hpp>
#include <xarm_msgs/msg/robot_msg.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <xarm_msgs/srv/move_cartesian.hpp>
#include <xarm_msgs/srv/vacuum_gripper_ctrl.hpp>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>

// STARTS WITH THE DEFINITION OF THE CLASS "Xz"
class Xz : public rclcpp::Node
{
public:
    Xz(); // CLASS CONSTRUCTOR

    ~Xz(); // CLASS DESTROYER

    // FUNCTIONS TO USE IN THE .CPP FILES
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void doCallback(const zed_interfaces::msg::ObjectsStamped::SharedPtr msg);
    void rob_stateCallback(const xarm_msgs::msg::RobotMsg::SharedPtr msg);
    void move_robot(float x, float y, float z);
    void test_robot_movement();
    void keyloop();
    void activate_vacuum(int turn_on);
    void aux_doCallback(const zed_interfaces::msg::ObjectsStamped::SharedPtr msg);

// DEFINITION OF THE NECESSARY VARIABLE{
#define RAD2DEG 57.295779513

    // CAMERA POSITION IN MAP FRAME
    double txp;
    double typ;
    double tzp;

    // ROLL, PITCH AND YAW FROM THE ROTATION MATRIX
    double rollp, pitchp, yawp;

    // POSITIONs and HEIGHTS OF THE DETECTED OBJECTS (MAX NUMBER OF DETECTED OBJECTS AT THE SAME TIME IS 10)
    std::vector<float> x_obd = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<float> y_obd = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<float> z_obd = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<float> height_obd = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // VARIABLES USED TO READ A LETTER FROM THE KEYBOARD
    int kfd = 0;
    struct termios cooked, raw;

    int index; // INDICATES THE INDEX OF THE VECTOR POSITION OF THE DETECTED OBJECTS TO WHICH YOU WANTED TO MOVE

    // INDICATES THE LAST POSITION OF THE MANIPULATOR
    float x_last = 0.3;
    float y_last = 0;
    float z_last = 0.45;

    // VARIABLES PARA LA CLASIFICACION DE OBJETOS
    int size;                                              // INDICATES THE NUMBER OF OBJECT OF THE SAME TYPE THAT YOU WANT TO CLASSIFY
    std::vector<int> ind = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // THIS VARIABLE SAVE THE POSITION OF THE OBJECTS THAT HAVE THE TYPE THAT YOU INDICATE IN THE BEGINING
    int cld;                                               // INDICATES IF THE CAMERA HGAS DETECTED ANY OBJECT OF THE TYPE THAT YOU INDICATE IN THE BEGINING

    //}

private:
    // SUBSCRIPTIONS:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr PoseSub;    // SUBSCRIPTIONS TO THE TOPIC THAT PUBLISH THE POSE OF THE CAMERA
    rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr obdsub; // SUBSCRIPTIONS TO THE TOPIC THAT PUBLISH THE DATA OF THE OBJECTS DETECTED
    rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr robot_state;       // SUBSCRIPTIONS TO THE TOPIC THAT PUBLISH THE STATE OF THE ROBOT
};
#include <xarm_zed/tfg.hpp>

using namespace std;
using namespace std::placeholders;
char c;                      // THIS IS THE LETTER THAT IS READ FROM THE KEYBOARD
int det;                     // THIS IS THE VARIABLE TO SAY IF OBJECT DETECTION IS NEEDED OR NOT
int state;                   // INDICATES THE STATUS OF THE MANIPULATOR
int last_state;              // INDICATES THE PREVIOUS STATE OF THE MAMIPULATOR
int mov;                     // INDICATES IF THE MANIPULATOR IS MOVING OR NOT
int paso;                    // INDICATES WHICH POSITION THE MANIPULATOR HAS TO GO DURING THE MOVEMENT TEST
int cl;                      // INDICATES IF WE WANT TO CLASSIFY OR NOT
std::string sublabel = " ";  // INDICATES THE OBJECT THAT YOU WANT TO CLASSIFY
int watch_state, watch_pose; // VARIABLES TO INDICATE IF YOU WANT TO SEE THE POSE AND STATE OF THE MANIPULATOR
double height_tool_camera;
// INDICATES THE INICIAL POSITION OF THE CAMERA THAT WE WANT TO USE
//{
double posx_ini;
double posy_ini;
double posz_ini;
//}

Xz::Xz() : Node("xzed") // THIS IS THE CLASS CONSTRUCTOR
{
    // INITIALIZE THE QUALITY OF SERVICE OF THE ZED CAMERA
    rclcpp::QoS qos(10);
    qos.keep_last(10);
    qos.best_effort();
    qos.durability_volatile();

    // CREATES CAMERA POSE SUBSCRIBER
    PoseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/zed/zed_node/pose", qos,
        std::bind(&Xz::poseCallback, this, _1));

    // CREATES OBJECT DETECTION SUBSCRIBER
    obdsub = this->create_subscription<zed_interfaces::msg::ObjectsStamped>(
        "/zed/zed_node/obj_det/objects", qos,
        std::bind(&Xz::doCallback, this, _1));

    // CREATES ROBOT STATE SUBSCRIBER
    robot_state = this->create_subscription<xarm_msgs::msg::RobotMsg>(
        "/xarm/robot_states", 10,
        std::bind(&Xz::rob_stateCallback, this, _1));

    // INITIALIZES THE VARIABLE
    mov = 0;
    paso = 1;
    last_state = 1;
    state = 2;
    size = 0;
    cld = 0;
    posy_ini = 0.37;
    posx_ini = 0;
    posz_ini = 0.453;
}

Xz::~Xz() // THIS IS THE CLASS DESTROYER
{
}

void Xz::activate_vacuum(int turn_on)
{
    // CREATE THE CLIENT OF THE SERVICE THAT ACTIVATE THE VACUUM
    std::shared_ptr<rclcpp::Node> node_vacuum = rclcpp::Node::make_shared("robot_vacuum");
    rclcpp::Client<xarm_msgs::srv::VacuumGripperCtrl>::SharedPtr client_vacuum = node_vacuum->create_client<xarm_msgs::srv::VacuumGripperCtrl>("/xarm/set_vacuum_gripper");

    // CREATE THE REQUEST OF THE CLIENT
    auto request_vacuum = std::make_shared<xarm_msgs::srv::VacuumGripperCtrl::Request>();

    if (turn_on == 1)
    {

        request_vacuum->on = true;
    }
    else
    {
        request_vacuum->on = false;
    }

    // CREATE THE RESPONSE OF THE SRVICE TO KNOW IF THE MESSAGE ARRIVE CVORRECTLY TO THE MANIPULATOR
    auto result_vacuum = client_vacuum->async_send_request(request_vacuum);
    RCLCPP_INFO(get_logger(), "SENDING THE REQUEST\n");

    // WAIT FOR THE RESULT
    if (rclcpp::spin_until_future_complete(node_vacuum, result_vacuum) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "VACUUM ACTIVATED OR DESACTIVATED CORRECTLY\n");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "FAILED TO CALL THE SERVICE 'set_vacuum_gripper'\n");
    }
}

// FUNTION TO KNOW WHAT THE STATE OF THE MANIPULATOR IS
void Xz::rob_stateCallback(const xarm_msgs::msg::RobotMsg::SharedPtr msg)
{
    last_state = state; // SAVE THE LAST STATE OF THE ROBOT
    state = msg->state; // UPDATE THE STATE OF THE ROBOT
    if (watch_state == 1)
    {
        RCLCPP_INFO(get_logger(), "THE STATE AND LAST STATE OF THE ROBOT IS:  '%d' ,  '%d'\n", state, last_state);
    }
}

// FUNCTION TO READ A LETTER FROM THE KEYBOARD
void Xz::keyloop()
{

    // GET THE CONSOLE IN RAW MODE
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);

    // SETTING A NEW LINE, THEN END OF FILE
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    RCLCPP_INFO(get_logger(), "READING FROM KEYBOARD--------");

    // GET THE NEXT EVENT FROM THE KEYBOARD
    if (read(kfd, &c, 1) < 0)
    {
        perror("read():");
    }
    RCLCPP_INFO(get_logger(), "YOU HAVE WRITTEN:  '%c'\n ", c);

    // GET THE CONSOLE IN NORMAL MODE
    tcsetattr(kfd, TCSANOW, &cooked);
}

// FUNCTION TO CHECK THE CORRECT OPERATION OF THE ROBOT MOVEMENT
void Xz::test_robot_movement()
{
    if (mov == 0) // CHECK IF THE MANIPULATOR IS MOVING OR NOT
    {
        // IF THE ROBOT IS STOPPED WE CAN SEND IT THE NEXT MOVEMENT

        mov = 1; // INDICATES THAT THE ROBOT IS MOVING

        // CREATE THE CLIENT OF THE SERVICE THAT MOVE THE MANIPULATOR
        std::shared_ptr<rclcpp::Node> node_robot = rclcpp::Node::make_shared("robot_movement_client");
        rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr client = node_robot->create_client<xarm_msgs::srv::MoveCartesian>("/xarm/set_position");

        // INITIALIZE THE POSE THAT WE WILL SEND TO THE ROBOT
        std::vector<float> pose = {0, 0, 0, 0, 0, 0};

        // CREATE THE REQUEST OF THE CLIENT
        auto request = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();

        // DEPEND OF THE STEP THAT THE ROBOT IS, WE CHOOSE A POSE
        switch (paso)
        {
        case (1):
            pose[0] = (0.2) * 1000;
            pose[1] = 0;
            pose[2] = (0.2) * 1000;
            pose[3] = 3.14;
            pose[4] = 0;
            pose[5] = 0;
            break;

        case (2):
            pose[0] = (0.4) * 1000;
            pose[1] = 0;
            pose[2] = (0.2) * 1000;
            pose[3] = 3.14;
            pose[4] = 0;
            pose[5] = 0;
            break;

        case (3):
            pose[0] = (0.4) * 1000;
            pose[1] = 0;
            pose[2] = (0.4) * 1000;
            pose[3] = 3.14;
            pose[4] = 0;
            pose[5] = 0;
            break;
        }

        // FILL THE REQUEST OF THE CLIENT WITH THE NECESSARY VARIABLES
        request->pose = pose;
        request->speed = 50;
        request->acc = 500;
        request->mvtime = 0;

        // CREATE THE RESPONSE OF THE SRVICE TO KNOW IF THE MESSAGE ARRIVE CVORRECTLY TO THE MANIPULATOR
        auto result = client->async_send_request(request);
        RCLCPP_INFO(get_logger(), "SENDING THE REQUEST\n");

        // WAIT FOR THE RESULT
        if (rclcpp::spin_until_future_complete(node_robot, result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ROBOT'S POSE RECEIVED CORRECTLY\n");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "FAILED TO CALL THE SERVICE 'set_position'\n");
        }
    }
    else // THE MANIPULATOR IS STILL MOVING
    {
        RCLCPP_INFO(get_logger(), "ROBOT IS MOVING");
    }
}

// FUNCTION TO MOVE THE MANIPULATOR TO THE INDICATEED POSITION
void Xz::move_robot(float x, float y, float z)
{

    mov = 1; // INDICATE THAT THE ROBOT IS MOVING

    // CREATE THE CLIENT OF THE SERVICE THAT MOVE THE ROBOT
    std::shared_ptr<rclcpp::Node> node_robot = rclcpp::Node::make_shared("robot_movement_client");
    rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr client = node_robot->create_client<xarm_msgs::srv::MoveCartesian>("/xarm/set_position");

    // INITIALIZE THE POSE OF THE ROBOT THAT WE WILL SEND
    std::vector<float> pose = {0, 0, 0, 0, 0, 0};

    // CREATE THE REQUEST OF THE CLIENT
    auto request = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();

    // FILL THE POSE WITH THE GIVED POSITION AND THE ORIENTATION OF THE EN EFFECTOR IS SET DOWNWARDS
    pose[0] = x * 1000;
    pose[1] = y * 1000;
    pose[2] = z * 1000;
    pose[3] = 3.14;
    pose[4] = 0;
    pose[5] = 0;

    // FILL THE REQUEST WITH THE NECESSARY VARIABLES
    request->pose = pose;
    request->speed = 50;
    request->acc = 500;
    request->mvtime = 0;

    // CREATE THE RESPONSE OF THE SERVICE TO KNOW IF THE MESSAGE ARRIVE CVORRECTLY TO THE MANIPULATOR
    auto result = client->async_send_request(request);
    RCLCPP_INFO(get_logger(), "SENDING THE REQUEST\n");

    // WAIT FOR THE RESULT.
    if (rclcpp::spin_until_future_complete(node_robot, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ROBOT'S POSE RECEIVED CORRECTLY\n");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "FAILED TO CALL SERVICE 'set_position'\n");
    }
    if (x == x_last and y == y_last and z == z_last) // IF THE ROBOT MOVE TO THE SAME POSITION IT WAS IN, WE INDICATE THAT THE ROBOT IS STOPPED
    {
        mov = 0;
    }
    x_last = x;
    y_last = y;
    z_last = z;
}

// FUNCTION TO GET THE POSE OF THE CAMERA
void Xz::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // CAMERA POSITION IN MAP FRAME
    txp = msg->pose.position.x;
    typ = msg->pose.position.y;
    tzp = msg->pose.position.z;

    // ORIENTATION QUATERNION
    tf2::Quaternion qp(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

    // 3x3 ROTATION MATRIX FROM QUATERNION
    tf2::Matrix3x3 mp(qp);

    // ROLL, PITCH AND YAW FROM ROTATION MATRIX
    mp.getRPY(rollp, pitchp, yawp);

    // OUTPUT THE MEASURE OF THE POSE OF THE CAMERA RELATIVE TO THE BASE OF THE ROBOT
    if (watch_pose == 1)
    {
        RCLCPP_INFO(this->get_logger(), "RECEIVED POSE OF THE CAMERA RELATED TO THE BASE  IN'%s' FRAME : X: '%.2f',  Y: '%.2f',  Z: '%.2f',  - R: '%.2f',  P: '%.2f',  Y: '%.2f',  - TIMESTAMP: %u.%u SEC \n",
                    msg->header.frame_id.c_str(),
                    typ, txp, tzp,
                    pitchp * RAD2DEG, -rollp * RAD2DEG, yawp * RAD2DEG,
                    msg->header.stamp.sec, msg->header.stamp.nanosec);

        RCLCPP_INFO(this->get_logger(), "AND THE POSITION OF THE END EFFECTOR RELATED TO THE BASE  IN'%s' FRAMNE: X: '%.2f',  Y: '%.2f',  Z: '%.2f',  - TIMESTAMP: %u.%u SEC \n",
                    msg->header.frame_id.c_str(),
                    typ + 0.07, txp, tzp + 0.003,
                    msg->header.stamp.sec, msg->header.stamp.nanosec);
    }
}

// FUNCTION TO GET THE POSITION OF THE DETECTED OBJECTS
void Xz::doCallback(const zed_interfaces::msg::ObjectsStamped::SharedPtr msg)
{

    if (mov == 0 and det == 1) // CHECK IF THE ROBOT IS STOPPED AND IF WE WANT TO USE THE OBJECT DETECTION
    {

        for (int i = 0; i < msg->objects.size(); i++)
        {
            if (sublabel == msg->objects[i].sublabel)
            {

                size = size + 1;   // NUMBER OF OBJECT THAT BELONGS TO THE SUBLABEL
                ind[size - 1] = i; // SAVING THE INDEX OF THE OBJECT THAT BELONGS TO THE SUBLABEL
            }
        }

        if (size == 0)
        {
            cld = 0; // THE CAMERA HAS NOT DETECTED ANY OBJECTS
        }
        else
        {
            cld = 1; // THE CAMERA HAS DETECTED A OBJECT THAT BELONGS TO THE SUBLABEL
        }

        if (cld == 1)
        {
            RCLCPP_INFO(get_logger(), "\n THE ZED CAMERA HAS DETECTED %d %s. ", size, sublabel.c_str());

            for (int i = 0; i < size; i++)
            {
                // GET THE POSITION OF THE DETECTED OBJECTS RELATIVE TO THE CAMERA
                x_obd[i] = msg->objects[ind[i]].position[0];
                y_obd[i] = msg->objects[ind[i]].position[1];
                z_obd[i] = msg->objects[ind[i]].position[2];
                height_obd[i] = msg->objects[ind[i]].dimensions_3d[2];

                RCLCPP_INFO(get_logger(), "Received %s [%d] position in '%s' frame : X: %.2f Y: %.2f Z: %.2f HEIGHT: %.2f - Timestamp: %u.%u sec ",
                            sublabel.c_str(),
                            i,
                            msg->header.frame_id.c_str(),
                            y_obd[i] + posy_ini, -x_obd[i] - posx_ini, z_obd[i] + posz_ini, height_obd[i],
                            msg->header.stamp.sec, msg->header.stamp.nanosec);

                if (abs(-x_obd[i] - posx_ini) < 0.4 and y_obd[i] + posy_ini < 0.75 and y_obd[i] + posy_ini > 0.15 and height_obd[i] < 0.35 and height_obd[i] > 0.04) // CHECK IF THE DETECTED OBJECT IS OUR OF THE WORK AREA
                {

                    RCLCPP_INFO(get_logger(), "MOVING THE ROBOT TO THE OBJECT'S POSE");
                    move_robot(y_obd[i] + posy_ini, -x_obd[i] - posx_ini, 0.45);
                    activate_vacuum(1);
                    move_robot(y_obd[i] + posy_ini, -x_obd[i] - posx_ini, height_obd[i] + (height_tool_camera));

                    move_robot(y_obd[i] + posy_ini, -x_obd[i] - posx_ini, 0.45);
                    move_robot(0.3, 0.35, 0.45);
                    move_robot(0.3, 0.35, height_obd[i] + (height_tool_camera) + 0.1);
                    activate_vacuum(0);
                    move_robot(0.3, 0.35, 0.45);
                    move_robot(0.3, 0, 0.45);
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "THE OBJECT IS NOT IN THE WORK AREA OR THERE IS A ERROR IN THE MEASURE");
                }
            }
        }
        else
        {
            RCLCPP_INFO(get_logger(), "THE CAMERA HAS FINISHED TO TAKE ALL THE %s, ¿DO YOU WANT TO TAKE A DIFFERNT OBJECTS? (BANANA-'B'/APPLE-'A'/ORANGE-'O'/CELLPHONE-'C'/LAPTOP-'L' /NOTHING-'ANY OTHER KEY')\n ", sublabel.c_str());

            keyloop();

            switch (c)
            {
            case ('B'):
                RCLCPP_INFO(get_logger(), "YOU CHOOSE BANANA\n");
                sublabel = "Banana";
                break;

            case ('A'):
                RCLCPP_INFO(get_logger(), "YOU CHOOSE APPLE\n");
                sublabel = "Apple";
                break;

            case ('O'):
                RCLCPP_INFO(get_logger(), "YOU CHOOSE ORANGE\n");
                sublabel = "Orange";
                break;

            case ('C'):
                RCLCPP_INFO(get_logger(), "YOU CHOOSE CELLPHONE\n");
                sublabel = "CellPhone";
                break;

            case ('L'):
                RCLCPP_INFO(get_logger(), "YOU CHOOSE LAPTOP\n");
                sublabel = "Laptop";
                break;

            default:
                RCLCPP_INFO(get_logger(), "YOU HAVE NOT CHOOSEN NOTHING \n");
                sublabel = "Nothing";

                break;
            }
        }
        size = 0;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "NO DETETCTION UNTILL ROBOT FINISHS ITS MOVEMENT");
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // CREATE OBJECTS FROM OUR CLASS
    auto node = std::make_shared<Xz>();

    // RUNNING AT 2HZ
    rclcpp::Rate loop_rate(2.0);

    /* RCLCPP_INFO(node->get_logger(), "DO YOU WANT TO SEE MORE INFORMATION ABOUT THE CAMERA AND THE ROBOT DURING THE USING (Y/N)\n");
     node->keyloop();
     if (c == 'Y')
     {
         watch_pose = 1;
         watch_state = 1;
     }
     else
     {
         watch_pose = 0;
         watch_state = 0;
     }*/
    watch_pose = 0;
    watch_state = 0;

    // MOVE THE MANIPULATOR TO THE INITIAL POSE
    RCLCPP_INFO(node->get_logger(), "MOVING THE ROBOT TO THE INITIAL POSE\n");
    node->move_robot(0.3, 0, 0.45);

    RCLCPP_INFO(node->get_logger(), "¿WHAT TOOL ARE YOU GOING TO USE IN THE ROBOT? (WRITE u (UFACTORY) OR ANY OTHER KEY (PROJECT)\n");
    node->keyloop();
    if (c == 'U')
    {
        height_tool_camera = 0.115 + 0.013;
    }
    else
    {
        height_tool_camera = 0.1 + 0.013;
    }

    RCLCPP_INFO(node->get_logger(), "¿DO YOU WANT TO DO A MOVEMENT TEST OR TO DETECT THE OBJECTS? (WRITE 1 OR 2)\n");
    node->keyloop();

    while (rclcpp::ok() and sublabel != "Nothing")
    {
        switch (c)
        {
        case ('1'):
            node->test_robot_movement();
            det = 0;

            break;

        case ('2'):
            det = 1;

            RCLCPP_INFO(node->get_logger(), "WHAT KIND OF OBJECTS DO YOU WANT TO PICK? (BANANA-'B' /APPLE-'A' /ORANGE-'O' /CELLPHONE-'C' /LAPTOP-'L' /NOTHING-'ANY OTHER KEY')\n");
            node->keyloop();

            switch (c)
            {
            case ('B'):
                RCLCPP_INFO(node->get_logger(), "YOU CHOOSE BANANA\n");
                sublabel = "Banana";
                break;

            case ('A'):
                RCLCPP_INFO(node->get_logger(), "YOU HAVE CHOOSEN APPLE\n");
                sublabel = "Apple";
                break;

            case ('O'):
                RCLCPP_INFO(node->get_logger(), "YOU HAVE CHOOSEN ORANGE\n");
                sublabel = "Orange";
                break;

            case ('C'):
                RCLCPP_INFO(node->get_logger(), "YOU HAVE CHOOSEN CELLPHONE\n");
                sublabel = "CellPhone";
                break;

            case ('L'):
                RCLCPP_INFO(node->get_logger(), "YOU HAVE CHOOSEN LAPTOP\n");
                sublabel = "Laptop";
                break;
            default:
                RCLCPP_INFO(node->get_logger(), "YOU HAVE NOT CHOOSEN NOTHING \n");
                sublabel = "Nothing";

                break;
            }

            break;


        }
        if (sublabel != "Nothing")
        {
            rclcpp::spin_some(node); // ATTEND SUBSCRIPTIONS
        }

        if (state == 2 and last_state == 1) // CHECK IF THE ROBOT IS STOPPED
        {
            if (det == 0) // IF THE ROBOT IS DOING THE MOVEMENT TEST WE INCREASE THE STEP THAT THE ROBOT IS
            {
                if (paso < 3)
                {
                    paso = paso + 1;
                }
                else
                {
                    paso = 1;
                }
            }

            mov = 0;
        }

        loop_rate.sleep(); // SLEPP TILL THE NEXT STEP TIME
    }
    rclcpp::shutdown();
    return 0;
}
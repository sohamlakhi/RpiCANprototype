//C++ library includes
#include <memory> //for smart pointers
#include <chrono> //for time 
#include <math.h> //for isnan, isinf etc.
#include <string>
#include <cstdlib> //for abs value function
//#include <vector> /// CHECK: might cause errors due to double header in linker
#include <functional>

//ROS related headers
#include "rclcpp/rclcpp.hpp"
//message header
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


//headers to code written by you


//other macros
#define NODE_NAME "ackermann_sub_CAN" 
#define _USE_MATH_DEFINES
//#define K_p 1
//#define K_d 0.01
//#define K_i 0.05

//using namespaces 
//used for bind (uncomment below)
using std::placeholders::_1;
//using namespace std::chrono_literals;


class CAN_drive_sub : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    CAN_drive_sub() : Node(NODE_NAME) {
        
        /// TODO: create ROS subscribers and publishers
        //initialise subscriber sharedptr obj
        
        subscription_drive = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 1000, std::bind(&CAN_drive_sub::drive_callback, this, _1));
        //initialise publisher sharedptr obj
        //publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

        //initialise wall timer sharedptr object
        //timer_ = this->create_wall_timer(500ms, std::bind(&CAN_drive_pub::timer_callback, this));

        RCLCPP_INFO (this->get_logger(), "%s node has been launched", NODE_NAME);

    }

private:

    //global static (to be shared by all objects) and dynamic variables (each instance gets its own copy -> managed on the stack)
    std::string drive_topic = "/drive_msg_CAN";

    //declare publisher sharedpointer obj
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_drive;

    //declare timer sharedptr obj

    //helper functions --> can turn them into a package library (in ros package template)

    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr drive_submsgObj){   
        double speed = drive_submsgObj->drive.speed;
        double steering_angle = drive_submsgObj->drive.steering_angle;

        RCLCPP_INFO (this->get_logger(), "speed %f .... steering_angle %f", speed, steering_angle);
    }

};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<CAN_drive_sub>(); // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
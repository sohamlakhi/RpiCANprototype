//C++ library includes
#include <memory> //for smart pointers
#include <chrono> //for time 
#include <math.h> //for isnan, isinf etc.
#include <string>
#include <cstdlib> //for abs value function
//#include <vector> /// CHECK: might cause errors due to double header in linker
#include <functional>
#include <queue> //for queues

//ROS related headers
#include "rclcpp/rclcpp.hpp"
//message header
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


//headers to code written by you


//other macros
#define NODE_NAME "ackermann_sub_CAN" 
#define _USE_MATH_DEFINES


//using namespaces 
//used for bind (uncomment below)
using std::placeholders::_1;
using namespace std;
//using namespace std::chrono_literals;


class CAN_drive_sub : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    CAN_drive_sub() : Node(NODE_NAME) {
        
        /// TODO: create ROS subscribers and publishers
        //initialise subscriber sharedptr obj
        
        subscription_drive = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 1000, bind(&CAN_drive_sub::drive_callback, this, _1));
        //initialise publisher sharedptr obj
        //publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

        //initialise wall timer sharedptr object
        //timer_ = this->create_wall_timer(500ms, std::bind(&CAN_drive_pub::timer_callback, this));

        RCLCPP_INFO (this->get_logger(), "%s node has been launched", NODE_NAME);

    }

private:

    //global static (to be shared by all objects) and dynamic variables (each instance gets its own copy -> managed on the stack)
    string drive_topic = "/drive_msg_CAN";

    //declare publisher sharedpointer obj
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_drive;

    //declare queues
    queue<double> speedQ_incoming;
    queue<double> steeringQ_incoming;

    //declare timer sharedptr obj

    //helper functions --> can turn them into a package library (in ros package template)

    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr drive_submsgObj){   
        //double speed = drive_submsgObj->drive.speed;
        //double steering_angle = drive_submsgObj->drive.steering_angle;

        speedQ_incoming.push(drive_submsgObj->drive.speed+1);
        steeringQ_incoming.push(drive_submsgObj->drive.steering_angle+1);
    
        RCLCPP_INFO (this->get_logger(), "speed %f .... steering_angle %f", speedQ_incoming.front(),steeringQ_incoming.front()); 
        speedQ_incoming.pop();
        steeringQ_incoming.pop();
    }

};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = make_shared<CAN_drive_sub>(); // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}

/*
    write functions to open socket, pack data and transmit
    may need to use state machine and delimiting byte. (may be able to distinguish between data using 11 bit identifier)
    (concurrent) queue data 

    make a class that has all CAN and queue functions. Then make an instance for each stream of data and use corresponding object functions
*/
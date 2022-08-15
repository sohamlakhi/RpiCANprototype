//C++ library includes
#include <memory> //for smart pointers
#include <chrono> //for time 
#include <math.h> //for isnan, isinf etc.
#include <string>
#include <cstdlib> //for abs value function
//#include <vector> /// CHECK: might cause errors due to double header in linker
#include <functional>
//#include <queue> //for queues
#include <thread>
//#include <cstdio.h>//for string representation

//debugging
#include <assert.h>

//CAN include header
//#include "linux/can/raw.h" //for packing and passing can frame 

//ROS related headers
#include "rclcpp/rclcpp.hpp"
//message header
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


//headers to code written by you
#include "threadsafe_queue/threadsafe_queue.hpp"
#include "ros2can/ros2can.hpp"

//other macros
#define NODE_NAME "ackermann_sub_CAN" 
#define _USE_MATH_DEFINES
#define DELIM 255
#define EMPTY 256
#define SPEED_CAN_ID 0x20
#define STEERING_CAN_ID 0x10
#define DELIM_CAN_ID 0x50

//using namespaces 
//used for bind (uncomment below)
using std::placeholders::_1;
using namespace std;
using namespace std::chrono_literals;






class CAN_drive_sub : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    CAN_drive_sub() : Node(NODE_NAME) {
        
        //Initialise callback groups
        drive_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        //setup subscriptionoptions -> need to do this to pass the callbackgroup to the subscription object
        rclcpp::SubscriptionOptions option_drive;
        option_drive.callback_group = drive_cb_group_;
        

        //initialise subscriber sharedptr obj 
        //TODO: add 
        subscription_drive = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 1000, bind(&CAN_drive_sub::drive_callback, this, _1), option_drive);

        //initialise publisher sharedptr obj
        //publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

        //initialise wall timer sharedptr object
        timer_ = this->create_wall_timer(500ms, bind(&CAN_drive_sub::timer_callback, this), timer_cb_group_);//time between every dequeue

        RCLCPP_INFO (this->get_logger(), "%s node has been launched", NODE_NAME);

    }

private:

    //global static (to be shared by all objects) and dynamic variables (each instance gets its own copy -> managed on the stack)
    string drive_topic = "/drive_msg_CAN";

    //declare publisher sharedpointer obj
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_drive;

    //declare timer sharedptr obj
    rclcpp::TimerBase::SharedPtr timer_;

    //declare callback groups
    rclcpp::CallbackGroup::SharedPtr drive_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    //declare queues
    threadsafe_queue<double> speedQ_incoming;
    threadsafe_queue<double> steeringQ_incoming;
    
    //TODO: comment out after debugging
    string string_thread_id() {
        auto hashed = hash<thread::id>()(this_thread::get_id());
        return to_string(hashed);
    }
    
    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr drive_submsgObj){   
        //double speed = drive_submsgObj->drive.speed;
        //double steering_angle = drive_submsgObj->drive.steering_angle;

        speedQ_incoming.push(drive_submsgObj->drive.speed);
        steeringQ_incoming.push(drive_submsgObj->drive.steering_angle);

        //thread id
        RCLCPP_INFO(this->get_logger(),"THREAD %s (drive)" + string_thread_id());
    
        //RCLCPP_INFO (this->get_logger(), "speed %f .... steering_angle %f", speedQ_incoming.front(),steeringQ_incoming.front()); 
        //comment the lines below when adding CAN stuff
        // speedQ_incoming.pop(); 
        // steeringQ_incoming.pop();
    }

    //write function to convert data to uint array & pass to can_frame write function
    //use assert to ensure sizeof(data_frame.data) <= 8 
    //might have to remove trailing '/0'
    void doubleToCAN (ros2can& canObj, double data, uint32_t can_id) {
        sprintf((char *)canObj.data_frame.data, "%f", data); //TODO: might have to change method of bit manipulation (might have to use reinterpret_cast with unsigned char * --> which is just unsigned int8)
        canObj.data_frame.can_dlc = sizeof(data); //TODO: check for size
        canObj.data_frame.can_id = can_id;
    } 

    //TODO: debug by printing 
    void timer_callback() {   
        //add CAN stuff here
        //dequeue here and use state machine to send over CAN

        //write state machine to interleave the data over CAN

        //struct can_frame data_frame; //TODO: global variable OR turn this to static private OR allocate on the heap and explicitly destroy to improve runtime 
        ros2can canObj;

        int state = 0;
        while (!speedQ_incoming.empty() || !steeringQ_incoming.empty()) {

            if (state == 0) {
                //TODO: replace if-else with ternary operator
                //check and send speed
                // double speed;
                // if (!speedQ_incoming.empty()) {
                //     speed = speedQ_incoming.dequeue();
                //     //pass to CAN function as parameter

                //     RCLCPP_INFO (this->get_logger(), "speed (DQ) %f ", speed); 
                //     RCLCPP_INFO(this->get_logger(),"THREAD %s" + string_thread_id());
                // }
                // else {
                //     //send empty CAN frame
                //     RCLCPP_INFO (this->get_logger(), "speed (DQ) %s ", "empty"); 
                //     RCLCPP_INFO(this->get_logger(),"THREAD %s" + string_thread_id());
                // }

                //CAN API implementation
                speedQ_incoming.empty() ? doubleToCAN(canObj, EMPTY, SPEED_CAN_ID) : doubleToCAN(canObj, speedQ_incoming.dequeue(), SPEED_CAN_ID);
                canObj.send();
                state = 1;
            }

            if (state == 1) {
                //check and send steering

                //TODO: replace if-else with ternary operator
                //check and send speed
                // double steering;
                // if (!steeringQ_incoming.empty()) {
                //     steering = steeringQ_incoming.dequeue();
                //     //pass to CAN function as parameter

                //     RCLCPP_INFO (this->get_logger(), "steering (DQ) %f ", steering); 
                //     RCLCPP_INFO(this->get_logger(),"THREAD %s" + string_thread_id());
                // }
                // else {
                //     //send empty CAN frame
                //     RCLCPP_INFO (this->get_logger(), "steering (DQ) %s ", "empty"); 
                //     RCLCPP_INFO(this->get_logger(),"THREAD %s" + string_thread_id());
                // }

                steeringQ_incoming.empty() ? doubleToCAN(canObj, EMPTY, STEERING_CAN_ID) : doubleToCAN(canObj, steeringQ_incoming.dequeue(), STEERING_CAN_ID);
                canObj.send();
                state = 2;
                RCLCPP_INFO(this->get_logger(),"steering angle %f", atof((char *)canObj.data_frame.data));
            }

            if (state == 2) {
                //send full 8byte message as delimiter
                //pass it to CAN send function

                // RCLCPP_INFO (this->get_logger(), "steering (DQ) %s ", "delimiter"); 
                // RCLCPP_INFO(this->get_logger(),"THREAD %s" + string_thread_id());

                doubleToCAN(canObj, DELIM, DELIM_CAN_ID);
                canObj.send();

                state = 0;
            }

            //TODO: get rid of empty frame and change CAN_id instead and compare that to differentiate between speed and steering values
            //can remove last frame id
        }
        
    }

};

//TODO: multithread and spin here
int main(int argc, char ** argv) {
    // rclcpp::init(argc, argv);
    // auto node_ptr = make_shared<CAN_drive_sub>(); // initialise node pointer
    // rclcpp::spin(node_ptr);
    // rclcpp::shutdown();
    // return 0;

    rclcpp::init(argc, argv);
    auto node_ptr = make_shared<CAN_drive_sub>(); // initialise node pointer
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_ptr);
    executor.spin();

    rclcpp::shutdown();
    return 0;
    
}

/*
    write functions to open socket, pack data and transmit
    may need to use state machine and delimiting byte. (may be able to distinguish between data using 11 bit identifier)
    (concurrent) queue data 

    make a class that has all CAN and queue functions. Then make an instance for each stream of data and use corresponding object functions
    add functions 

    TODO:
        set up wireless network

    (if queue not empty) every "timeperiod" dequeue a window of elements and send them through CAN. (use RCLCPP timer or C++ timer -> read about rclcpp timer being used for real time stuff)


    NOTE: queues need to be concurrent safe (either do it in python or C++ or any other language since ROS2 is lang agnostic)
    you can make a thread safe implementation by going down to the thread level using boost
    could use a boost queue itself instead

    TODO: NOTE: values coded using ASCII char (need to decode accordingly --> use atof)
*/
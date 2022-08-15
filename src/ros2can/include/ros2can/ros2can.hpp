#pragma once

//Cpp header files
#include <linux/can/raw.h>

class ros2can {
    public:
        ros2can() = default; //constructor

        void send(); //

        void receive();

        struct can_frame data_frame;
    private:
        //change scope to within function for encapsulation and appropriate destruction
        // struct sockaddr_can addr;
        // struct can_frame send_frame;
        // struct can_frame rec_frame;
        // struct ifreq ifr;

        // int natsock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        
};

/*TODO: design ros2can interface --> what kind of data do you want to send?
                                 --> build frame accordingly
                                 --> look at Martin endler's header only distribution 
                                 --> look at ros help way of doing things

        pass data object by reference
        pack it in CAN struct in here
*/
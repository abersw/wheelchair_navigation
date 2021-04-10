/*
 * wheelchair_go_to.cpp
 * wheelchair_navigation
 * version: 0.0.1 Majestic Maidenhair
 * Status: Pre-Alpha
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h" //main ROS library
#include "std_msgs/String.h" //ROS msg type string
#include <sstream>
using namespace std;

const int DEBUG_main = 1;

void userInstructionCallback(const std_msgs::String userInstructionMsg) {
    cout << userInstructionMsg.data << endl;
}

/**
 * Main function that contains ROS info, subscriber callback trigger and while loop to get room name
 *
 * @param argc - number of arguments
 * @param argv - content of arguments
 * @return 0 - end of program
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "wheelchair_go_to");
    ros::NodeHandle nodeHandler;
    ros::Subscriber userInstruction_sub = nodeHandler.subscribe("/wheelchair_robot/user/instruction", 10, userInstructionCallback);
    ros::Rate rate(1.0);

    while (ros::ok()) {
        //do stuff
        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spinOnce();
        //rate.sleep();
    }

    return 0;
}
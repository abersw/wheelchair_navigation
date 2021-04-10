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

const bool DEBUG_testFindUserInstruction = 0;
const bool DEBUG_userInstructionCallback = 1;
const bool DEBUG_main = 0;

/**
 * Test function for finding strings within a long string 
 *
 * @param parameter 'userInstructionRaw' is a string of the user's intention from the userInstructionCallback function
 *        param belongs to std::string
 */
void testFindUserInstruction(std::string userInstructionRaw) {
    const string test_userInstruction = "take me to the oven";
    std::size_t foundIt = test_userInstruction.find(userInstructionRaw);
    if (foundIt!=std::string::npos) { //if string is not found
        std::cout << "found at: " << foundIt << '\n'; //point to start of string
    }
}

/**
 * Main callback function triggered by received user instruction topic 
 *
 * @param parameter 'userInstructionMsg' is a string of the user's intention from wheelchair_interface
 *        message belongs to std_msgs::String
 */
void userInstructionCallback(const std_msgs::String userInstructionMsg) {
    std::string userInstructionRaw = userInstructionMsg.data;
    if (DEBUG_userInstructionCallback) {
        cout << userInstructionRaw << endl;
    }
    if (DEBUG_testFindUserInstruction) {
        testFindUserInstruction(userInstructionRaw);
    }
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
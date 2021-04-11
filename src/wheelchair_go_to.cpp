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
#include "wheelchair_msgs/roomToObjects.h" //assign rooms to objects
#include "wheelchair_msgs/roomLocations.h" //translations of rooms
#include "wheelchair_msgs/objectLocations.h" //translations of all objects
#include "std_msgs/String.h" //ROS msg type string
#include <sstream>
using namespace std;

const bool DEBUG_testFindUserInstruction = 0;
const bool DEBUG_userInstructionCallback = 1;
const bool DEBUG_objectLocationsCallback = 0;
const bool DEBUG_roomLocationsCallback = 0;
const bool DEBUG_roomObjectCallback = 0;
const bool DEBUG_main = 0;

struct Objects { //struct for publishing topic
    int id; //get object id from ros msg
    string object_name; //get object name/class
    float object_confidence; //get object confidence

    float point_x; //get transform point x
    float point_y; //get transform point y
    float point_z; //get transform point z

    float quat_x; //get transform rotation quaternion x
    float quat_y; //get transform rotation quaternion y
    float quat_z; //get transform rotation quaternion z
    float quat_w; //get transform rotation quaternion w
};
struct Objects objectsFileStruct[100000]; //array for storing object data
int totalObjectsFileStruct = 0; //total objects inside struct

struct Rooms {
    int room_id;
    string room_name;

    float point_x; //get transform point x
    float point_y; //get transform point y
    float point_z; //get transform point z

    float quat_x; //get transform rotation quaternion x
    float quat_y; //get transform rotation quaternion y
    float quat_z; //get transform rotation quaternion z
    float quat_w; //get transform rotation quaternion w
};
struct Rooms roomsFileStruct[1000];
int totalRoomsFileStruct = 0;

struct RoomObjectLinks {
    int object_id;
    string object_name;

    int room_id;
    string room_name;
};
struct RoomObjectLinks roomObjectLinkStruct[100000]; //array for storing all object and room data
int totalRoomObjectLinkStruct = 0;

struct RoomObjectLinks roomObjectDecisionStruct[10000]; //array for storing possible objects to navigate to
int totalRoomObjectDecisionStruct = 0;

struct Decisions {
    int id;
    string name;

    float point_x;
    float point_y;
    float point_z;

    float quat_x;
    float quat_y;
    float quat_z;
    float quat_w;
};

struct Decisions navigateToDecision;

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

    //search for struct object/room name in user string, then add them to a struct
}

/**
 * Callback function triggered by list of all objects 
 *
 * @param parameter 'objLoc' is a objectLocations msg of the objects and associated transforms from wheelchair_dacop
 *        message belongs to wheelchair_msgs::objectLocations
 */
void objectLocationsCallback(const wheelchair_msgs::objectLocations objLoc) {
    totalObjectsFileStruct = objLoc.totalObjects;
    for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) {
        objectsFileStruct[isObject].id = objLoc.id[isObject];
        objectsFileStruct[isObject].object_name = objLoc.object_name[isObject];
        objectsFileStruct[isObject].object_confidence = objLoc.object_confidence[isObject];

        objectsFileStruct[isObject].point_x = objLoc.point_x[isObject];
        objectsFileStruct[isObject].point_y = objLoc.point_y[isObject];
        objectsFileStruct[isObject].point_z = objLoc.point_z[isObject];

        objectsFileStruct[isObject].quat_x = objLoc.quat_x[isObject];
        objectsFileStruct[isObject].quat_y = objLoc.quat_y[isObject];
        objectsFileStruct[isObject].quat_z = objLoc.quat_z[isObject];
        objectsFileStruct[isObject].quat_w = objLoc.quat_w[isObject];

        if (DEBUG_objectLocationsCallback) {
            cout << 
            objectsFileStruct[isObject].id << ", " << 
            objectsFileStruct[isObject].object_name << ", " << 
            objectsFileStruct[isObject].object_confidence << ", " << 
            
            objectsFileStruct[isObject].point_x << ", " << 
            objectsFileStruct[isObject].point_y << ", " << 
            objectsFileStruct[isObject].point_z << ", " << 

            objectsFileStruct[isObject].quat_x << ", " << 
            objectsFileStruct[isObject].quat_y << ", " << 
            objectsFileStruct[isObject].quat_z << ", " << 
            objectsFileStruct[isObject].quat_w << endl;
        }
    }
    
}

/**
 * Callback function triggered by list of all rooms 
 *
 * @param parameter 'roomLoc' is a roomLocations msg of the rooms and associated transforms from wheelchair_dacop
 *        message belongs to wheelchair_msgs::roomLocations
 */
void roomLocationsCallback(const wheelchair_msgs::roomLocations roomLoc) {
    totalRoomsFileStruct = roomLoc.totalRooms;
    for (int isRoom = 0; isRoom < totalRoomsFileStruct; isRoom++) {
        roomsFileStruct[isRoom].room_id = roomLoc.id[isRoom];
        roomsFileStruct[isRoom].room_name = roomLoc.room_name[isRoom];

        roomsFileStruct[isRoom].point_x = roomLoc.point_x[isRoom];
        roomsFileStruct[isRoom].point_y = roomLoc.point_y[isRoom];
        roomsFileStruct[isRoom].point_z = roomLoc.point_z[isRoom];

        roomsFileStruct[isRoom].quat_x = roomLoc.quat_x[isRoom];
        roomsFileStruct[isRoom].quat_y = roomLoc.quat_y[isRoom];
        roomsFileStruct[isRoom].quat_z = roomLoc.quat_z[isRoom];
        roomsFileStruct[isRoom].quat_w = roomLoc.quat_w[isRoom];

        if (DEBUG_roomLocationsCallback) {
            cout << 
            roomsFileStruct[isRoom].room_id << ", " << 
            roomsFileStruct[isRoom].room_name << ", " << 
            
            roomsFileStruct[isRoom].point_x << ", " << 
            roomsFileStruct[isRoom].point_y << ", " << 
            roomsFileStruct[isRoom].point_z << ", " << 

            roomsFileStruct[isRoom].quat_x << ", " << 
            roomsFileStruct[isRoom].quat_y << ", " << 
            roomsFileStruct[isRoom].quat_z << ", " << 
            roomsFileStruct[isRoom].quat_w << endl;
        }
    }
}

/**
 * Callback function triggered by assigned room to objects list 
 *
 * @param parameter 'room2obj' is a roomToObjects msg of the objects which have rooms assigned from wheelchair_dacop
 *        message belongs to wheelchair_msgs::roomToObjects
 */
void roomObjectCallback(const wheelchair_msgs::roomToObjects room2obj) {
    totalRoomObjectLinkStruct = room2obj.totalObjects;
    for (int isLink = 0; isLink < totalRoomObjectLinkStruct; isLink++) {
        roomObjectLinkStruct[isLink].object_id = room2obj.object_id[isLink];
        roomObjectLinkStruct[isLink].object_name = room2obj.object_name[isLink];

        roomObjectLinkStruct[isLink].room_id = room2obj.room_id[isLink];
        roomObjectLinkStruct[isLink].room_name = room2obj.room_name[isLink];

        if (DEBUG_roomObjectCallback) {
            cout << 
            roomObjectLinkStruct[isLink].object_id << ", " << 
            roomObjectLinkStruct[isLink].object_name << ", " << 

            roomObjectLinkStruct[isLink].room_id << ", " << 
            roomObjectLinkStruct[isLink].room_name << endl;
        }
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
    ros::Subscriber object_locations_sub = nodeHandler.subscribe("/wheelchair_robot/dacop/publish_object_locations/objects", 10, objectLocationsCallback);
    ros::Subscriber room_locations_sub = nodeHandler.subscribe("/wheelchair_robot/dacop/assign_room_to_object/rooms", 10, roomLocationsCallback);
    ros::Subscriber assign_room_object_sub = nodeHandler.subscribe("/wheelchair_robot/dacop/assign_room_to_object/objects", 10, roomObjectCallback);
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
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
#include "wheelchair_msgs/objectContext.h" //object context info
#include "move_base_msgs/MoveBaseActionGoal.h" //move_base msg for sending map goals
#include "std_msgs/String.h" //ROS msg type string
#include <algorithm>
#include <sstream>
using namespace std;

const bool DEBUG_testFindUserInstruction = 0;
const bool DEBUG_navigateToObjectWithRoom = 1;
const bool DEBUG_startDecidingGoal = 1;
const bool DEBUG_findObjectAndRoom = 1;
const bool DEBUG_userInstructionCallback = 1;
const bool DEBUG_objectLocationsCallback = 0;
const bool DEBUG_roomLocationsCallback = 0;
const bool DEBUG_roomObjectCallback = 0;
const bool DEBUG_objectContextCallback = 1;
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

struct Decisions {
    int id;
    string name;
    float confidence;

    float point_x;
    float point_y;
    float point_z;

    float quat_x;
    float quat_y;
    float quat_z;
    float quat_w;
};

struct Decisions navigateToDecision;
struct Decisions objectDecisionStruct[10000]; //array for storing possible objects to navigate to
struct Decisions roomDecisionStruct[10]; //array for storing possible rooms to navigate to
int totalObjectDecisionStruct = 0;
int totalRoomDecisionStruct = 0;

struct Context {
    int object_id; //object id
    string object_name; //object name
    float object_confidence; //object confidence from dnn
    int object_detected; //times object has been detected

    float object_weighting; //object weighting result
    float object_uniqueness; //object uniqueness result
    float object_score; //calculation of object weighting and uniqueness
    int object_instances; //number of objects in env
};

struct Context objectContext[100000]; //struct for storing object context info
int totalObjectContextStruct = 0; //total objects in struct

ros::Publisher *ptr_espeak_pub; //publisher for verbal feedback from wheelchair
ros::Publisher *ptr_movebaseGoal_pub; //publisher for sending move_base goals

//function for printing space sizes
void printSeparator(int spaceSize) {
	if (spaceSize == 0) {
		printf("--------------------------------------------\n");
	}
	else {
		printf("\n");
		printf("--------------------------------------------\n");
		printf("\n");
	}
}

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
 * Function to send navigateToDecision struct to move_base goal topic for executing to path planner.
 */
void sendToMovebase() {
    cout << "navigating to " << navigateToDecision.id << ", " << navigateToDecision.name << endl;
    move_base_msgs::MoveBaseActionGoal chosenGoal;

    chosenGoal.header;
    chosenGoal.header.seq = 0;
    chosenGoal.header.stamp;
    chosenGoal.header.stamp.sec = 0;
    chosenGoal.header.stamp.nsec = 0;
    chosenGoal.header.frame_id = "";

    chosenGoal.goal_id;
    chosenGoal.goal_id.stamp.sec = 0;
    chosenGoal.goal_id.stamp.nsec = 0;
    chosenGoal.goal_id.id;

    chosenGoal.goal;
    chosenGoal.goal.target_pose;
    chosenGoal.goal.target_pose.header;
    chosenGoal.goal.target_pose.header.seq = 0;
    chosenGoal.goal.target_pose.header.stamp;
    chosenGoal.goal.target_pose.header.stamp.sec = 0;
    chosenGoal.goal.target_pose.header.stamp.nsec = 0;
    chosenGoal.goal.target_pose.header.frame_id = "map";

    chosenGoal.goal.target_pose.pose;
    chosenGoal.goal.target_pose.pose.position;
    chosenGoal.goal.target_pose.pose.position.x = navigateToDecision.point_x;
    chosenGoal.goal.target_pose.pose.position.y = navigateToDecision.point_y;
    chosenGoal.goal.target_pose.pose.position.z = navigateToDecision.point_z;
    chosenGoal.goal.target_pose.pose.orientation.x = navigateToDecision.quat_x;
    chosenGoal.goal.target_pose.pose.orientation.y = navigateToDecision.quat_y;
    chosenGoal.goal.target_pose.pose.orientation.z = navigateToDecision.quat_z;
    chosenGoal.goal.target_pose.pose.orientation.w = navigateToDecision.quat_w;

    ptr_movebaseGoal_pub->publish(chosenGoal);
    cout << "published goal\n";
}

/**
 * Function for navigating to object with room information 
 *
 * @return position of object in decision struct (not the objectID!) <- come back to this once function is finished
 */
int navigateToObjectWithRoom() {
    int madeDecision = 0;
    int decisionRoomID = roomDecisionStruct[0].id;
    std::string decisionRoomName = roomDecisionStruct[0].name;

    struct Context preProcessedContext[1000];
    int totalPreProcessedContext = 0;
    struct Context processedContext[1000];
    int totalProcessedContext = 0;

    int objectCounter = 0;
    for (int isObject = 0; isObject < totalObjectDecisionStruct; isObject) {
        //run through decision array
        int currentObjectID = objectDecisionStruct[isObject].id;
        for (int isContext = 0; isContext < totalObjectContextStruct; isContext++) {
            //run through entire context array
            int currentContextID = objectContext[isContext].object_id;
            //find object ID match
            if (currentObjectID == currentContextID) {
                //if the IDs match
                preProcessedContext[objectCounter].object_id = objectContext[isContext].object_id;
                preProcessedContext[objectCounter].object_name = objectContext[isContext].object_name;
                preProcessedContext[objectCounter].object_confidence = objectContext[isContext].object_confidence;
                preProcessedContext[objectCounter].object_detected = objectContext[isContext].object_detected;

                preProcessedContext[objectCounter].object_weighting = objectContext[isContext].object_weighting;
                preProcessedContext[objectCounter].object_uniqueness = objectContext[isContext].object_uniqueness;
                preProcessedContext[objectCounter].object_score = objectContext[isContext].object_score;
                preProcessedContext[objectCounter].object_instances = objectContext[isContext].object_instances;
                objectCounter++;
            }
        }
    }

    printSeparator(0);
    cout << "unsorted decision struct" << endl;
    for (int isDecision = 0; isDecision < totalObjectDecisionStruct; isDecision++) {
        if (DEBUG_navigateToObjectWithRoom) {
            cout <<
            objectDecisionStruct[isDecision].id << ", " <<
            objectDecisionStruct[isDecision].name << ", " <<
            objectDecisionStruct[isDecision].point_x << ", " <<
            objectDecisionStruct[isDecision].point_y << ", " <<
            objectDecisionStruct[isDecision].point_z << ", " <<

            objectDecisionStruct[isDecision].quat_x << ", " <<
            objectDecisionStruct[isDecision].quat_y << ", " <<
            objectDecisionStruct[isDecision].quat_z << ", " <<
            objectDecisionStruct[isDecision].quat_w << endl;
        }
    }

    printSeparator(0);
    cout << "sorted decision struct" << endl;
    //std::sort(std::begin(arr), std::end(arr), sortCompare);
    //run through object decision struct and work out the most influential object to navigate to
    /*for (int isObject = 0; isObject < totalObjectDecisionStruct; isObject++) { //run through all object with links to goal room
        //do stuff
    }*/
    return madeDecision;
}

/**
 * Function for starting decision making process to navigate to an object 
 *
 * @param parameter 'navigateToState' is an int of the navigation mode from the finObjectAndRoom function
 *        param belongs to int
 */
void startDecidingGoal(int navigateToState) {
    switch (navigateToState) {
        case 1: //navigate to object - no room info available
            if (DEBUG_startDecidingGoal) {
                cout << "navigate to object, no room info available" << endl;
                cout << "total objects decided " << totalObjectDecisionStruct << endl;
            }
            //start adding object decision code here
            break;
        case 2: //navigate to object with room information
            if (DEBUG_startDecidingGoal) {
                cout << "navigate to object with room information" << endl;
                cout << "total rooms decided " << totalRoomDecisionStruct << endl;
                cout << "total objects and room decided " << totalObjectDecisionStruct << endl;
            }
            //start adding room and object decision code here
            {
            int madeDecision = navigateToObjectWithRoom(); //make decision and return true if successful
            //assign decision to navigate to decision struct
            }
            break;
        case 3: //navigate to a room - no object info available
            if (DEBUG_startDecidingGoal) {
                cout << "navigate to a room, no object info available" << endl;
                cout << "total rooms decided " << totalRoomDecisionStruct << endl;
            }
            for (int isRoom = 0; isRoom < totalRoomDecisionStruct; isRoom++) {
                navigateToDecision.id = roomDecisionStruct[isRoom].id;
                navigateToDecision.name = roomDecisionStruct[isRoom].name;

                navigateToDecision.point_x = roomDecisionStruct[isRoom].point_x;
                navigateToDecision.point_y = roomDecisionStruct[isRoom].point_y;
                navigateToDecision.point_z = roomDecisionStruct[isRoom].point_z;

                navigateToDecision.quat_x = roomDecisionStruct[isRoom].quat_x;
                navigateToDecision.quat_y = roomDecisionStruct[isRoom].quat_y;
                navigateToDecision.quat_z = roomDecisionStruct[isRoom].quat_z;
                navigateToDecision.quat_w = roomDecisionStruct[isRoom].quat_w;
            }
            break;
    }
    //sendToMovebase();
}

/**
 * Function for finding all object and room matches from user instruction, and assigning a navigation mode 
 *
 * @param parameter 'userInstructionRaw' is a string of the user's intention from the userInstructionCallback function
 *        param belongs to std::string
 */
void findObjectAndRoom(std::string userInstructionRaw) {
    //check to see if room name exists in user instruction
    int navigateToState = 0;
    int roomFoundCount = 0; //array position variable when a room has been matched
    for (int isRoom = 0; isRoom < totalRoomsFileStruct; isRoom++) { //iterate through rooms struct
        std::string getRoomName = roomsFileStruct[isRoom].room_name; //get room name from struct
        std::size_t foundRoomMatch = userInstructionRaw.find(getRoomName); //search for corresponding room name
        if (foundRoomMatch != std::string::npos) { //if match is found
            roomDecisionStruct[roomFoundCount].id = roomsFileStruct[isRoom].room_id;
            roomDecisionStruct[roomFoundCount].name = roomsFileStruct[isRoom].room_name;
            //roomDecisionStruct[roomFoundCount].confidence = 0; //only applicable to objects

            roomDecisionStruct[roomFoundCount].point_x = roomsFileStruct[isRoom].point_x;
            roomDecisionStruct[roomFoundCount].point_y = roomsFileStruct[isRoom].point_y;
            roomDecisionStruct[roomFoundCount].point_z = roomsFileStruct[isRoom].point_z;

            roomDecisionStruct[roomFoundCount].quat_x = roomsFileStruct[isRoom].quat_x;
            roomDecisionStruct[roomFoundCount].quat_y = roomsFileStruct[isRoom].quat_y;
            roomDecisionStruct[roomFoundCount].quat_z = roomsFileStruct[isRoom].quat_z;
            roomDecisionStruct[roomFoundCount].quat_w = roomsFileStruct[isRoom].quat_w;
            if (DEBUG_findObjectAndRoom) {
                cout << "added " << roomDecisionStruct[roomFoundCount].id << ":" << roomDecisionStruct[roomFoundCount].name <<
                " to room decision struct to pos " << roomFoundCount << endl;
            }
            roomFoundCount++; //add 1 to found matches
        }
    }
    totalRoomDecisionStruct = roomFoundCount; //set matches found to total matches

    if (totalRoomDecisionStruct == 0) { //if there is no room detected in user instruction
        //if there are no rooms in user instruction, add all objects in user instruction, dacop publish topic
        if (DEBUG_findObjectAndRoom) {
            cout << "no rooms detected in user instruction, add all matching objects" << endl;
        }
        int objectFoundCount = 0; //array position variable when an object has been matched
        for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) { //iterate through entire objects struct
            std::string getObjectName = objectsFileStruct[isObject].object_name; //get object name from struct
            std::size_t foundObjectMatch = userInstructionRaw.find(getObjectName); //search for corresponding object name
            if (foundObjectMatch != std::string::npos) { //if match is found
                objectDecisionStruct[objectFoundCount].id = objectsFileStruct[isObject].id;
                objectDecisionStruct[objectFoundCount].name = objectsFileStruct[isObject].object_name;
                objectDecisionStruct[objectFoundCount].confidence = objectsFileStruct[isObject].object_confidence;

                objectDecisionStruct[objectFoundCount].point_x = objectsFileStruct[isObject].point_x;
                objectDecisionStruct[objectFoundCount].point_y = objectsFileStruct[isObject].point_y;
                objectDecisionStruct[objectFoundCount].point_z = objectsFileStruct[isObject].point_z;

                objectDecisionStruct[objectFoundCount].quat_x = objectsFileStruct[isObject].quat_x;
                objectDecisionStruct[objectFoundCount].quat_y = objectsFileStruct[isObject].quat_y;
                objectDecisionStruct[objectFoundCount].quat_z = objectsFileStruct[isObject].quat_z;
                objectDecisionStruct[objectFoundCount].quat_w = objectsFileStruct[isObject].quat_w;
                if (DEBUG_findObjectAndRoom) {
                    cout << "added " << objectDecisionStruct[objectFoundCount].id << ":" << objectDecisionStruct[objectFoundCount].name <<
                    " to object decision struct to pos " << objectFoundCount << endl;
                }
                objectFoundCount++; //add 1 to found matches
            }
        }
        totalObjectDecisionStruct = objectFoundCount; //set total objects found without a room
        navigateToState = 1; //navigate to object - no room info available
    }

    if (totalRoomDecisionStruct == 1) { //if a room has been identified, reduce list of objects
        int foundObjectInUserInstruction = 0; //leave at 0 if object is not found in user instruction
        int objectFoundCount = 0; //array position variable when an object has been matched
        //if room has been listed, only list objects with associated room - linkage topic
        for (int isRoomDecision = 0; isRoomDecision < totalRoomDecisionStruct; isRoomDecision++) { //go through entire array of rooms decided by user instruction
            //add code, although there should only be one room name applicable - look back at interface...
            int getRoomIdDecision = roomDecisionStruct[isRoomDecision].id;
            std::string getRoomNameDecision = roomDecisionStruct[isRoomDecision].name;
            for (int isLink = 0; isLink < totalRoomObjectLinkStruct; isLink++) { //go through entire room object link struct
                std::string getRoomName = roomObjectLinkStruct[isLink].room_name; //get current room name
                if (getRoomNameDecision == getRoomName) { //if room name decision is equal to room name in link list 
                    std::string getObjectName = roomObjectLinkStruct[isLink].object_name; //get current object name
                    std::size_t foundObjectMatch = userInstructionRaw.find(getObjectName); //search for an object inside user instruction
                    if (foundObjectMatch != std::string::npos) { //if object was found in user instruction
                        //add object and associated room to decision struct
                        objectDecisionStruct[objectFoundCount].id = roomObjectLinkStruct[isLink].object_id;
                        objectDecisionStruct[objectFoundCount].name = roomObjectLinkStruct[isLink].object_name;
                        for (int isObject = 0; isObject < totalObjectsFileStruct; isObject++) { //search for object vector and quaternion
                            if (objectDecisionStruct[objectFoundCount].id == objectsFileStruct[isObject].id) { //if object IDs match
                                objectDecisionStruct[objectFoundCount].point_x = objectsFileStruct[isObject].point_x; //set object vector x to struct
                                objectDecisionStruct[objectFoundCount].point_y = objectsFileStruct[isObject].point_y; //set object vector y to struct
                                objectDecisionStruct[objectFoundCount].point_z = objectsFileStruct[isObject].point_z; //set object vector z to struct

                                objectDecisionStruct[objectFoundCount].quat_x = objectsFileStruct[isObject].quat_x; //set object quaternion x to struct
                                objectDecisionStruct[objectFoundCount].quat_y = objectsFileStruct[isObject].quat_y; //set object quaternion y to struct
                                objectDecisionStruct[objectFoundCount].quat_z = objectsFileStruct[isObject].quat_z; //set object quaternion z to struct
                                objectDecisionStruct[objectFoundCount].quat_w = objectsFileStruct[isObject].quat_w; //set object quaternion w to struct

                                if (DEBUG_findObjectAndRoom) {
                                    cout << "room is " << getRoomIdDecision << ":" << getRoomNameDecision << endl;
                                    cout << "added " << objectDecisionStruct[objectFoundCount].id << ":" << objectDecisionStruct[objectFoundCount].name <<
                                    " to object decision struct to pos " << objectFoundCount << endl;
                                }
                            }
                            else {
                                //don't do anything if IDs don't match
                            }
                        }
                        objectFoundCount++; //add 1 to array pos, for next object match
                        foundObjectInUserInstruction = 1;
                    }
                    else { //if only room available in user instruction
                        //do nothing if room matches, but object doesn't match
                        //go to next element in room object link struct
                    }
                }
            }
        }
        totalObjectDecisionStruct = objectFoundCount;
        if (foundObjectInUserInstruction) {
            navigateToState = 2; //navigate to object with room information
        } 
        else {
            navigateToState = 3; //navigate to a room - no object info available
        }
    }
    if (totalRoomDecisionStruct > 1) { //there shouldn't be more than one room in user instruction, this should be filtered from the interface
        //what should happen if there is more than one element in room decision array?
    }
    //start decision making code here (or in a separate function)
    //create a variable to turn context assistance on/off
    startDecidingGoal(navigateToState); //pass navigate state to start decision making process.
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
    findObjectAndRoom(userInstructionRaw);
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
 * Callback function triggered by objects context list 
 *
 * @param parameter 'objContext' is a roomToObjects msg of the objects which have rooms assigned from wheelchair_dacop
 *        message belongs to wheelchair_msgs::objectContext
 */
void objectContextCallback(const wheelchair_msgs::objectContext objContext) {
    totalObjectContextStruct = objContext.totalObjects;
    for (int isContext = 0; isContext < totalObjectContextStruct; isContext++) {
        if (DEBUG_objectContextCallback) {
            cout <<
            objectContext[isContext].object_id << ", " <<
            objectContext[isContext].object_name << ", " <<
            objectContext[isContext].object_confidence << ", " <<
            objectContext[isContext].object_detected << ", " <<

            objectContext[isContext].object_weighting << ", " <<
            objectContext[isContext].object_uniqueness << ", " <<
            objectContext[isContext].object_score << ", " <<
            objectContext[isContext].object_instances << endl;
        }
        objectContext[isContext].object_id = objContext.object_id[isContext];
        objectContext[isContext].object_name = objContext.object_name[isContext];
        objectContext[isContext].object_confidence = objContext.object_confidence[isContext];
        objectContext[isContext].object_detected = objContext.object_detected[isContext];

        objectContext[isContext].object_weighting = objContext.object_weighting[isContext];
        objectContext[isContext].object_uniqueness = objContext.object_uniqueness[isContext];
        objectContext[isContext].object_score = objContext.object_score[isContext];
        objectContext[isContext].object_instances = objContext.object_instances[isContext];
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
    ros::Subscriber object_context_sub = nodeHandler.subscribe("/wheelchair_robot/context/objects", 10, objectContextCallback);

    ros::Publisher wheelchairGoal_pub = nodeHandler.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);
    ptr_movebaseGoal_pub = &wheelchairGoal_pub;
    ros::Publisher espeak_pub = nodeHandler.advertise<std_msgs::String>("/espeak_node/speak_line", 1000);
    ptr_espeak_pub = &espeak_pub;

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
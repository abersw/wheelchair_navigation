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
const bool DEBUG_navigateToObjectWithoutRoom = 1;
const bool DEBUG_startDecidingGoal = 1;
const bool DEBUG_findObjectAndRoom = 0;
const bool DEBUG_userInstructionCallback = 1;
const bool DEBUG_objectLocationsCallback = 0;
const bool DEBUG_roomLocationsCallback = 0;
const bool DEBUG_roomObjectCallback = 0;
const bool DEBUG_objectContextCallback = 0;
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

struct Decisions navigateToDecision[1000]; //struct array for storing decisions for move base goal
int navigateToDecisionTimes = 0; //current position to store decision in struct array
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

struct Context contextDecisionStruct[10000]; //context for objects in decision struct
int totalContextDecisionStruct = 0;

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

void sortDecisionStruct() {
    //sort elements in decision array
    //rather than create a brand new struct - use the swapping function, like the one below
    for (int isContext = 0; isContext < totalContextDecisionStruct; isContext++) {
        int currentContextID = contextDecisionStruct[isContext].object_id;
        for (int isObject = 0; isObject < totalObjectDecisionStruct; isObject++) {
            int currentObjectID = objectDecisionStruct[isObject].id;
            if (currentContextID == currentObjectID) {
                if (isContext == isObject) {
                    //object already matches the position of context struct array
                }
                else {
                    //swap object positions:
                    struct Decisions tempSortStruct;
                    //take object decision struct using isContext position and save to temp struct
                    tempSortStruct.id = objectDecisionStruct[isContext].id;
                    tempSortStruct.name = objectDecisionStruct[isContext].name;
                    //assign found object - isContext from object decision struct to object decision struct using isContext
                    objectDecisionStruct[isContext].id = objectDecisionStruct[isObject].id;
                    objectDecisionStruct[isContext].name = objectDecisionStruct[isObject].name;
                    //assign temp struct to object decision struct in position isContext
                    objectDecisionStruct[isObject].id = tempSortStruct.id;
                    objectDecisionStruct[isObject].name = tempSortStruct.name;
                }
            }
            else {
                //don't do anything if objects don't match
            }
        }
    }
}

void swapSortedObjects(struct Context *tempSortStruct, int isScore, int minScoreAt) {
    //add current object context to temp struct
    tempSortStruct->object_id = contextDecisionStruct[isScore].object_id;
    tempSortStruct->object_name = contextDecisionStruct[isScore].object_name;
    tempSortStruct->object_confidence = contextDecisionStruct[isScore].object_confidence;
    tempSortStruct->object_detected = contextDecisionStruct[isScore].object_detected;

    tempSortStruct->object_weighting = contextDecisionStruct[isScore].object_weighting;
    tempSortStruct->object_uniqueness = contextDecisionStruct[isScore].object_uniqueness;
    tempSortStruct->object_score = contextDecisionStruct[isScore].object_score;
    tempSortStruct->object_instances = contextDecisionStruct[isScore].object_instances;

    //add minimum score object in current place
    contextDecisionStruct[isScore].object_id = contextDecisionStruct[minScoreAt].object_id;
    contextDecisionStruct[isScore].object_name = contextDecisionStruct[minScoreAt].object_name;
    contextDecisionStruct[isScore].object_confidence = contextDecisionStruct[minScoreAt].object_confidence;
    contextDecisionStruct[isScore].object_detected = contextDecisionStruct[minScoreAt].object_detected;

    contextDecisionStruct[isScore].object_weighting = contextDecisionStruct[minScoreAt].object_weighting;
    contextDecisionStruct[isScore].object_uniqueness = contextDecisionStruct[minScoreAt].object_uniqueness;
    contextDecisionStruct[isScore].object_score = contextDecisionStruct[minScoreAt].object_score;
    contextDecisionStruct[isScore].object_instances = contextDecisionStruct[minScoreAt].object_instances;

    //add temp struct object in its place
    contextDecisionStruct[minScoreAt].object_id = tempSortStruct->object_id;
    contextDecisionStruct[minScoreAt].object_name = tempSortStruct->object_name;
    contextDecisionStruct[minScoreAt].object_confidence = tempSortStruct->object_confidence;
    contextDecisionStruct[minScoreAt].object_detected = tempSortStruct->object_detected;

    contextDecisionStruct[minScoreAt].object_weighting = tempSortStruct->object_weighting;
    contextDecisionStruct[minScoreAt].object_uniqueness = tempSortStruct->object_uniqueness;
    contextDecisionStruct[minScoreAt].object_score = tempSortStruct->object_score;
    contextDecisionStruct[minScoreAt].object_instances = tempSortStruct->object_instances;
}

void selectionSortContext() {
    //sort elements in context array
    int isScore, nextScore, minScoreAt, minScoreID = 0;
    float minScore;
    for (isScore = 0; isScore < (totalContextDecisionStruct - 1); isScore++) { //iterate through entire array until last element
        minScoreAt = isScore; //current pos is set as min, unless ...
        minScoreID = contextDecisionStruct[isScore].object_id; //get current ID
        minScore = contextDecisionStruct[isScore].object_score; //get current score
        for (nextScore = (isScore + 1); nextScore < totalContextDecisionStruct; nextScore++) {
            if (minScore < contextDecisionStruct[nextScore].object_score) {
                minScoreAt = nextScore; //the position of the min element
                minScoreID = contextDecisionStruct[nextScore].object_id; //set object id of min score
                minScore = contextDecisionStruct[nextScore].object_score; //set object min score
            }
        }
        struct Context tempSortStruct; //temporary storage struct for temp sort object
        swapSortedObjects(&tempSortStruct, isScore, minScoreAt); //swap function for objects
    }

    sortDecisionStruct(); //sort decision struct to match order of context struct array
}

/**
 * Function to send navigateToDecision struct to move_base goal topic for executing to path planner.
 */
void sendToMovebase() {
    cout << "navigating to " << navigateToDecision[navigateToDecisionTimes].id << ", " << navigateToDecision[navigateToDecisionTimes].name << endl;
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
    chosenGoal.goal.target_pose.pose.position.x = navigateToDecision[navigateToDecisionTimes].point_x;
    chosenGoal.goal.target_pose.pose.position.y = navigateToDecision[navigateToDecisionTimes].point_y;
    chosenGoal.goal.target_pose.pose.position.z = navigateToDecision[navigateToDecisionTimes].point_z;
    chosenGoal.goal.target_pose.pose.orientation.x = navigateToDecision[navigateToDecisionTimes].quat_x;
    chosenGoal.goal.target_pose.pose.orientation.y = navigateToDecision[navigateToDecisionTimes].quat_y;
    chosenGoal.goal.target_pose.pose.orientation.z = navigateToDecision[navigateToDecisionTimes].quat_z;
    chosenGoal.goal.target_pose.pose.orientation.w = navigateToDecision[navigateToDecisionTimes].quat_w;

    ptr_movebaseGoal_pub->publish(chosenGoal);
    navigateToDecisionTimes++; //iterate to next position for decision struct
    cout << "published goal\n";
}

/**
 * Function for navigating to object with room information 
 *
 * @return set to 1 if successfully allocated an object to navigate towards
 */
int navigateToObjectWithRoom() {
    printSeparator(0);
    int madeDecision = 0;

    //object location data in objectDecisionStruct
    //object context data in contextDecisionStruct
    selectionSortContext(); //sort via context score

    //would probably be more useful to sort from highest to lowest score
    //get the highest score
    float largestScore = 0;
    int largestScoreID = 0;
    std::string largestScoreName;
    int largestPosScore = 0;
    largestScore = contextDecisionStruct[0].object_score;
    for (int contextDec = 0; contextDec < totalContextDecisionStruct; contextDec++) {
        cout << largestScore << endl;
        if (largestScore < contextDecisionStruct[contextDec].object_score) {
            largestScore = contextDecisionStruct[contextDec].object_score;
            largestScoreID = contextDecisionStruct[contextDec].object_id;
            largestScoreName = contextDecisionStruct[contextDec].object_name;
            largestPosScore = contextDec;
            if (DEBUG_navigateToObjectWithRoom) {
                cout << "found new largest score" << endl;
            }
        }
    }
    if (DEBUG_navigateToObjectWithRoom) {
        cout << "largest score is " << largestScore << " at context position " << largestPosScore << endl;
    }
    //times the score with the confidence - less confident detections will be lower on the decision making scale

    navigateToDecision[navigateToDecisionTimes].id = largestScoreID;
    navigateToDecision[navigateToDecisionTimes].name = largestScoreName;
    for (int isNavDecision = 0; isNavDecision < totalObjectDecisionStruct; isNavDecision++) {
        if (largestScoreID == objectDecisionStruct[isNavDecision].id) {
            //if IDs match
            navigateToDecision[navigateToDecisionTimes].confidence = objectDecisionStruct[isNavDecision].confidence;
            
            navigateToDecision[navigateToDecisionTimes].point_x = objectDecisionStruct[isNavDecision].point_x;
            navigateToDecision[navigateToDecisionTimes].point_y = objectDecisionStruct[isNavDecision].point_y;
            navigateToDecision[navigateToDecisionTimes].point_z = objectDecisionStruct[isNavDecision].point_z;

            navigateToDecision[navigateToDecisionTimes].quat_x = objectDecisionStruct[isNavDecision].quat_x;
            navigateToDecision[navigateToDecisionTimes].quat_y = objectDecisionStruct[isNavDecision].quat_y;
            navigateToDecision[navigateToDecisionTimes].quat_z = objectDecisionStruct[isNavDecision].quat_z;
            navigateToDecision[navigateToDecisionTimes].quat_w = objectDecisionStruct[isNavDecision].quat_w;

            madeDecision = 1; //successfully allocated an object to navigate towards
        }
    }
    return madeDecision;
}

/**
 * Function for navigating to object without room information 
 *
 * @return set to 1 if successfully allocated an object to navigate towards
 */
int navigateToObjectWithoutRoom() {
    static const bool DEBUG_navigateToObjectWithoutRoom_1 = 0;
    static const bool DEBUG_navigateToObjectWithoutRoom_2 = 0;
    int madeDecision = 0;

    
    //object location data in objectDecisionStruct
    //object context data in contextDecisionStruct

    //sort from highest to lowest score - get the highest score
    //get history - take room from previous object
    //get current room - from the room detection node

    return madeDecision;
}

void createContextDecisionStruct() {
    static const bool DEBUG_createContextDecisionStruct_1 = 1;
    static const bool DEBUG_createContextDecisionStruct_2 = 1;

    //add first part of context decision struct assignment to this function
    int objectCount = 0;
    for (int isDecision = 0; isDecision < totalObjectDecisionStruct; isDecision++) {
        //run through entire decision struct
        int isDecisionID = objectDecisionStruct[isDecision].id; //get current object ID
        if (DEBUG_createContextDecisionStruct_1) {
            cout << "in obj pos " << isDecision << " with ID " << isDecisionID << endl;
        }
        for (int isContext = 0; isContext < totalObjectContextStruct; isContext++) {
            //run through entire context struct
            int isContextID = objectContext[isContext].object_id; //get current object ID
            if (DEBUG_createContextDecisionStruct_1) {
                cout << "in ctx pos " << isContext << " with ID " << isContextID << endl;
            }
            if (isDecisionID == isContextID) { //if decision object is equal to object in context struct
                if (DEBUG_createContextDecisionStruct_2) {
                    cout << "found object match" << endl;
                }
                //add found context data to new struct
                contextDecisionStruct[objectCount].object_id = objectContext[isContext].object_id; //set object ID
                contextDecisionStruct[objectCount].object_name = objectContext[isContext].object_name; //set object name
                contextDecisionStruct[objectCount].object_confidence = objectContext[isContext].object_confidence; //set object mobilenet confidence
                contextDecisionStruct[objectCount].object_detected = objectContext[isContext].object_detected; //set times object has been detected

                contextDecisionStruct[objectCount].object_weighting = objectContext[isContext].object_weighting; //set object context weighting
                contextDecisionStruct[objectCount].object_uniqueness = objectContext[isContext].object_uniqueness; //set object context uniqueness
                contextDecisionStruct[objectCount].object_score = objectContext[isContext].object_score; //set object score from context node
                contextDecisionStruct[objectCount].object_instances = objectContext[isContext].object_instances; //set object instances in env
                if (DEBUG_createContextDecisionStruct_2) { //print out current object info from decision struct
                    //print out object location and info
                    cout <<
                    objectDecisionStruct[isDecision].id << ", " <<
                    objectDecisionStruct[isDecision].name << ", " <<
                    objectDecisionStruct[isDecision].confidence << ", " <<

                    objectDecisionStruct[isDecision].point_x << ", " <<
                    objectDecisionStruct[isDecision].point_y << ", " <<
                    objectDecisionStruct[isDecision].point_z << ", " <<

                    objectDecisionStruct[isDecision].quat_x << ", " <<
                    objectDecisionStruct[isDecision].quat_y << ", " <<
                    objectDecisionStruct[isDecision].quat_z << ", " <<
                    objectDecisionStruct[isDecision].quat_w << endl;

                    //print out object context and info
                    cout <<
                    contextDecisionStruct[objectCount].object_id << ", " <<
                    contextDecisionStruct[objectCount].object_name << ", " <<
                    contextDecisionStruct[objectCount].object_confidence << ", " <<
                    contextDecisionStruct[objectCount].object_detected << ", " <<

                    contextDecisionStruct[objectCount].object_weighting << ", " <<
                    contextDecisionStruct[objectCount].object_uniqueness << ", " <<
                    contextDecisionStruct[objectCount].object_score << ", " <<
                    contextDecisionStruct[objectCount].object_instances << endl;
                }
                objectCount++; //iterate to the next object
            }
        }
    }
    totalContextDecisionStruct = objectCount;
    cout << "total context decision struct is " << totalContextDecisionStruct << endl;
    //object location data in objectDecisionStruct
    //object context data in contextDecisionStruct
}

/**
 * Function for starting decision making process to navigate to an object 
 *
 * @param parameter 'navigateToState' is an int of the navigation mode from the finObjectAndRoom function
 *        param belongs to int
 */
void startDecidingGoal(int navigateToState) {
    int madeDecision = 0;
    switch (navigateToState) {
        case 1: //navigate to object - no room info available
            if (DEBUG_startDecidingGoal) {
                cout << "navigate to object, no room info available" << endl;
                cout << "total objects decided " << totalObjectDecisionStruct << endl;
            }
            {
            createContextDecisionStruct(); //add context data to decision struct
            madeDecision = navigateToObjectWithoutRoom();
            if (madeDecision) {
                cout << "successfully made goal decision towards object" << endl;
            }
            }
            break;
        case 2: //navigate to object with room information
            if (DEBUG_startDecidingGoal) {
                cout << "navigate to object with room information" << endl;
                cout << "total rooms decided " << totalRoomDecisionStruct << endl;
                cout << "total objects and room decided " << totalObjectDecisionStruct << endl;
            }
            //start adding room and object decision code here
            {
            createContextDecisionStruct(); //add context data to decision struct
            madeDecision = navigateToObjectWithRoom(); //make decision and return true if successful
            //assign decision to navigate to decision struct
            if (madeDecision) {
                cout << "successfully made goal decision towards room and object" << endl;
            }
            }
            break;
        case 3: //navigate to a room - no object info available
            if (DEBUG_startDecidingGoal) {
                cout << "navigate to a room, no object info available" << endl;
                cout << "total rooms decided " << totalRoomDecisionStruct << endl;
            }
            for (int isRoom = 0; isRoom < totalRoomDecisionStruct; isRoom++) {
                navigateToDecision[navigateToDecisionTimes].id = roomDecisionStruct[isRoom].id;
                navigateToDecision[navigateToDecisionTimes].name = roomDecisionStruct[isRoom].name;
                //navigateToDecision[navigateToDecisionTimes].confidence; //no confidence available for room

                navigateToDecision[navigateToDecisionTimes].point_x = roomDecisionStruct[isRoom].point_x;
                navigateToDecision[navigateToDecisionTimes].point_y = roomDecisionStruct[isRoom].point_y;
                navigateToDecision[navigateToDecisionTimes].point_z = roomDecisionStruct[isRoom].point_z;

                navigateToDecision[navigateToDecisionTimes].quat_x = roomDecisionStruct[isRoom].quat_x;
                navigateToDecision[navigateToDecisionTimes].quat_y = roomDecisionStruct[isRoom].quat_y;
                navigateToDecision[navigateToDecisionTimes].quat_z = roomDecisionStruct[isRoom].quat_z;
                navigateToDecision[navigateToDecisionTimes].quat_w = roomDecisionStruct[isRoom].quat_w;
            }
            break;
    }
    cout << "finished goal decision function" << endl;
    if (madeDecision) {
        sendToMovebase();
    }
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
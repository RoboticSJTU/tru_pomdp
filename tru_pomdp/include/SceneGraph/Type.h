#pragma once
#include <iostream>
#include <vector>
#include <string>

// NodeType is an enum class that represents the type of a Node
// Node with type NODE_ROBOT is defined in Robot.h
// Others are defined in Node.h
enum NodeType
{
    NODE_NONE, // for base Node class
    // NODE_POSE,
    // NODE_FURNITURE,
    NODE_AREA,
    NODE_OBJECT,
    NODE_ROBOT
};
const static std::vector<std::string> NodeNameMap = {"NODE_NONE",
                                                    //  "NODE_POSE",
                                                    //  "NODE_FURNITURE",
                                                     "NODE_AREA",
                                                     "NODE_OBJECT",
                                                     "NODE_ROBOT"};

// output NodeType through ostream
std::ostream &operator<<(std::ostream &os, const NodeType &node_type);

enum ActionType
{
    ACTION_NONE,
    // ACTION_MOVE,
    ACTION_OPEN,
    ACTION_PICK,
    ACTION_PLACE,
    ACTION_DONE
};

std::ostream &operator<<(std::ostream &os, const ActionType &action_type);

std::string ActionTypeToString(ActionType action_type);

static const int COST_OPEN = 5;
static const int COST_PICK = 5;
static const int COST_PLACE = 5;

static const int COST_OTHERS = 25;

static const int GOAL_REWARD = 500;


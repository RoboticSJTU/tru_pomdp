#pragma once
#include <SceneGraph/Node.h>
#include <SceneGraph/Exceptions.h>

// Robot is a special Node in SceneGraph
// The child of Robot is a ObjectNode represents the object in hand
// The parent of Robot is a AreaNode represents the location of the robot
class Robot : public Node
{
public:
    Robot(const std::string &name) : Node(name, NODE_ROBOT) {}
    Robot(): Node("robot", NODE_ROBOT) {}
    Robot(const Robot &robot) = default;
    std::string ToJsonString() const override ;
    ~Robot() override = default;
    // friend std::ostream &operator<<(std::ostream &os, const Robot &robot);
};
#include <SceneGraph/Type.h>

// output function of NodeType
// output NodeType::NODE_POSE as Pose
std::ostream& operator<<(std::ostream& os, const NodeType& node_type)
{
    switch (node_type)
    {
    case NODE_AREA:
        os << "Area";
        break;
    case NODE_OBJECT:
        os << "Object";
        break;
    case NODE_ROBOT:
        os << "Robot";
        break;
    default:
        os << "None";
        break;
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const ActionType& action_type)
{
    switch (action_type)
    {
    case ACTION_OPEN:
        os << "Open";
        break;
    case ACTION_PICK:
        os << "Pick";
        break;
    case ACTION_PLACE:
        os << "Place";
        break;
    default:
        os << "None";
        break;
    }
    return os;
}

std::string ActionTypeToString(ActionType action_type)
{
    switch (action_type)
    {
    case ACTION_OPEN:
        return "Open";
    case ACTION_PICK:
        return "Pick";
    case ACTION_PLACE:
        return "Place";
    default:
        return "None";
    }
}

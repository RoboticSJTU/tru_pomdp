#include <SceneGraph/Node.h>
#include <SceneGraph/Exceptions.h>
#include <nlohmann/json.hpp>
#include <stdexcept>
using json = nlohmann::json;

// definition of Node::DEBUG_NODE_MEMORY_LEAK
// std::set<Node*> Node::DEBUG_NODE_MEMORY_LEAK;

Node::Node(const std::string &name)
    :_name(name), Type(NODE_NONE)
{
    //DEBUG_NODE_MEMORY_LEAK.insert(this);  
}

Node::Node(const std::string &name, NodeType type) 
: _name(name), Type(type)
{
    //DEBUG_NODE_MEMORY_LEAK.insert(this);  
}

Node::~Node() {
    //DEBUG_NODE_MEMORY_LEAK.erase(this);
    // std::cerr << "Call Node destructor: " << _name << std::endl;
}

std::string Node::GetName() const {
    return _name;
}

void Node::SetName(const std::string &name) {
    _name = name;
}

std::string Node::ToJsonString() const {
    json j;
    j["name"] = _name;
    j["type"] = NodeNameMap[Type];
    return j.dump(4);
}

std::ostream& operator<<(std::ostream& os, const Node& node) {
    os << "Node: " << node._name << " Type: " << NodeNameMap[node.Type];
    return os;
}

// ============= AreaNode
AreaNode::AreaNode(const std::string& name, bool is_open)
    : Node(name, NODE_AREA){
    _name = name;
    _is_open = is_open;
}


std::string AreaNode::ToJsonString() const {
    json j;
    j["name"] = _name;
    j["type"] = NodeNameMap[Type];
    j["is_open"] = _is_open;

    return j.dump(4);
}

// ============= ObjectNode
ObjectNode::ObjectNode(const std::string& name)
    : Node(name, NODE_OBJECT) {
}

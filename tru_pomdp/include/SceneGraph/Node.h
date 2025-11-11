#pragma once
#include <SceneGraph/Type.h>
#include <vector>
#include <string>
#include <utility>
#include <map>
#include <set>
#include <memory>

class Node
{
protected:
    std::string _name;
    bool _is_open = false;

    // avoid calling node function from a freed scene graph.
    bool _is_freed = false;
public:
    const NodeType Type = NODE_NONE;    

    // constructor
    // Node(const Node &node);
    Node(const std::string &name);
    Node(const std::string &name, NodeType type);
    Node(const Node &node) = default;

    // Setter and Getter
    bool IsOpen() const {
        return _is_open;
    }
    void Open(){
        _is_open = true;
    }


    std::string GetName() const;
    void SetName(const std::string &name);

    //get type
    NodeType GetType() const{
        return Type;
    }

    virtual std::string ToJsonString() const ;
    virtual ~Node();



    // friend
    friend std::ostream& operator<<(std::ostream& os, const Node& node);
    //operator =
    Node& operator=(const Node &node){
        _name = node._name;
        _is_open = node._is_open;
        return *this;
    }
    friend class SceneGraph;

};

/*
 * @brief an AreaNode has property is_open and a set of objects in the area.
 */

class AreaNode : public Node
{
public:
    // constructor
    AreaNode(const std::string& name, bool is_open);
    // AreaNode(const AreaNode &area) = delete;
    AreaNode(const AreaNode &area) = default;

    // override
    ~AreaNode() override = default;

    // bool IsOpen() const{
    //     return _is_open;
    // }

    // std::shared_ptr<Node> Copy(NodeSet& recorder, bool with_invisable) const override;
    std::string ToJsonString() const override ;
    //override =
    AreaNode& operator=(const AreaNode &area){
        _name = area._name;
        _is_open = area._is_open;
        return *this;
    }
};

/*
 * @brief an ObjectNode has only a name as and a parent area.
 */
class ObjectNode : public Node
{
public:
    ObjectNode(const std::string& name);
    // ObjectNode(const ObjectNode &object) = delete;
    ObjectNode(const ObjectNode &object) = default;
    // override
    ~ObjectNode() override = default;
    // std::shared_ptr<Node> Copy(NodeSet& recorder, bool with_invisable) const override;
};

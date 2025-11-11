#pragma once
#include <exception>
#include <string>

// CHECK_NULL_NODE(node) will check if node is nullptr
// If node is nullptr, throw NullNodeException
// These is optional message argument for NullNodeException
// which will be appended to the message "Node is null"
#define CHECK_NULL_NODE(node, message) \
    if (node == nullptr) { \
        throw NullNodeException(std::string("Node is null: ") + message); \
    }

#define CHECK_NODE_TYPE(node, type) \
    if (node->Type != type) { \
        throw InvalidNodeTypeException("Node is not of type " + std::to_string(type)); \
    }

#define CHECK_NODE_FREED(node) \
    if (node->_is_freed) { \
        throw NodeAlreadyFreedException(); \
    }


// Exceptions used in the SceneGraph library
// Exception class for invalid node type
class InvalidNodeTypeException : public std::exception {
public:
    InvalidNodeTypeException(const std::string &message) : _message(message) {}
    const char *what() const noexcept override {
        return _message.c_str();
    }
private:
    std::string _message;
};

// Exception class for node from different SceneGraph
class DifferentSceneGraphException : public std::exception {
public:
    DifferentSceneGraphException(const std::string &message) : _message(message) {}
    const char *what() const noexcept override {
        return _message.c_str();
    }
private:
    std::string _message;
};

class NullNodeException : public std::exception {
public:
    NullNodeException(const std::string &message) : _message(message) {}
    const char *what() const noexcept override {
        return _message.c_str();
    }
private:
    std::string _message;
};

class SceneAlreadyInitializedException : public std::exception {
public:
    SceneAlreadyInitializedException() {}
    const char *what() const noexcept override {
        return "Scene is already initialized";
    }
};

class ProblemAlreadyInitializedException: public std::exception {
public:
    ProblemAlreadyInitializedException() {}
    const char *what() const noexcept override {
        return "Problem is already initialized";
    }
};

class SceneNotInitializedException: public std::exception {
    public:
        SceneNotInitializedException() {}
        const char *what() const noexcept override {
            return "Scene is not initialized";
        }
};

class NodeAlreadyFreedException: public std::exception {
    public:
        NodeAlreadyFreedException() {}
        const char *what() const noexcept override {
            return "Call node function from a freed scene graph. \
            Please make sure that your scene graph is not freed before calling node functions. \
            The easiest way is keep scene graph instance in a variable instead of using method chaining.\n \
            For example, instead of \n`simulator.get_observation().get_area_node(\"node_name\").children`\n, use\n \
            `graph = simulator.get_observation() \ngraph.get_area_node(\"node_name\").children`";
        }
};
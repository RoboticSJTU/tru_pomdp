#pragma once
#include <SceneGraph/Type.h>
#include <vector>
#include <set>
#include <functional>  // Include this for hash
#include <cstring>

using namespace std;
class Action
{
public:
    ActionType Type;
    vector<string> Args;

    Action() {
        Type = ActionType::ACTION_NONE;
    }
    Action(ActionType type, const vector<string>& args) {
        Type = type;
        Args = args;
    }

    // Copy constructor
    Action(const Action& action) {
        Type = action.Type;
        Args = action.Args;
    }

    ~Action() {
    }

    string ToString() const {
        string result = ActionTypeToString(Type) + "(";
        for (size_t i = 0; i < Args.size(); ++i) {
            result += Args[i];
            result += ", ";
        }
        result += ")";
        return result;
    }

    // Output function with operator<<
    friend ostream& operator<<(ostream& os, const Action& action) {
        os << action.Type;
        os << "(";
        for (const auto& arg : action.Args) {
            os << arg << ", ";
        }
        os << ")";
        return os;
    }

    // Overload the equality operator
    bool operator==(const Action& other) const {
        if (Type != other.Type) {
            return false;
        }
        if (Args.size() != other.Args.size()) {
            return false;
        }
        for (size_t i = 0; i < Args.size(); ++i) {
            if (Args[i] != other.Args[i]) {
                return false;
            }
        }
        return true;
    }

    // Overload the inequality operator
    bool operator!=(const Action& other) const {
        return !(*this == other);
    }
};

// Specialize hash for Action
namespace std {
    template <>
    struct hash<Action> {
        size_t operator()(const Action& action) const {
            size_t h = 0;
            
            // Combine hash of Type (assuming ActionType is hashable)
            h ^= hash<ActionType>{}(action.Type) + 0x9e3779b9 + (h << 6) + (h >> 2);

            // Combine hash of Args (using hash for vector)
            for (const auto& arg : action.Args) {
                h ^= hash<string>{}(arg) + 0x9e3779b9 + (h << 6) + (h >> 2);
            }

            return h;
        }
    };
}

class ActionSimple{
public:
    // ActionType Type;
    int Args[3];

    ActionSimple(){
        // Type = ActionType::ACTION_NONE;
        // Args[0] = -1;
        // Args[1] = -1;
        memset(Args, -1, sizeof(Args));
        Args[2] = ActionType::ACTION_NONE;
    }

    ActionSimple(ActionType type, int arg1, int arg2){
        // Type = type;
        Args[0] = arg1;
        Args[1] = arg2;
        Args[2] = type;
    }

    ActionSimple(int args[3]){
        memcpy(Args, args, sizeof(Args));
    }

    ActionSimple(const ActionSimple& action){
        memcpy(Args, action.Args, sizeof(Args));
    }

    ~ActionSimple(){
        // No dynamic memory allocation, so no specific cleanup is needed.
        // Destructor can remain empty.
    }
    ActionType Type() const{
        return static_cast<ActionType>(Args[2]);
    }

    friend ostream& operator<<(ostream& os, const ActionSimple& action){
        os << action.Type();
        os << "(";
        os << action.Args[0] << ", " << action.Args[1] << ")";
        return os;
    }

    bool operator==(const ActionSimple& other) const{
        // if (Type() != other.Type()){
        //     return false;
        // }
        // if (Args[0] != other.Args[0] || Args[1] != other.Args[1]){
        //     return false;
        // }
        // return true;
        return memcmp(Args, other.Args, sizeof(Args)) == 0;
    }

    bool operator!=(const ActionSimple& other) const{
        return !(*this == other);
    }

    Action ToAction(vector<string> area_names, vector<string> object_names) const{
        vector<string> args;

        if(Args[2] == ActionType::ACTION_NONE){
            return Action(ActionType::ACTION_NONE, {});
        }
        
        args.push_back(area_names[Args[0]]);
        if (Type() == ActionType::ACTION_PICK){
            args.push_back(object_names[Args[1]]);
        }
        return Action(Type(), args);
        // return Action(Type, args);

    }
    // = operator
    ActionSimple& operator=(const ActionSimple& other){
        // Args = other.Type();
        memcpy(Args, other.Args, sizeof(Args));
        return *this;
    }
};

namespace std{
    template <>
    struct hash<ActionSimple>{
        size_t operator()(const ActionSimple& action) const{
            size_t h = 0;
            h = action.Args[0] + action.Args[1] * 100 + action.Args[2] * 10000;
            return h;
        }
    };
}


#pragma once

#include <ostream>
#include <string>
#include <vector>
#include <set>
#include <SceneGraph/SceneGraph.h>

using namespace std;

class Task
{
private:
	string _language_description;
    // goal list is a list of pair of (area_name, object_name)
    int _ideal_cost = 0;

public:
    vector<vector<pair<string, string>>> _area_has_obj_list;
	vector<pair<string, string>> _area_has_obj;

    // static set<Task*> DEBUG_TASK_MEMORY_LEAK;
	Task(){
        //DEBUG_TASK_MEMORY_LEAK.insert(this);
    };
    Task(const string &language_description);
    // copy constructor
    Task(const Task &task) {
        _language_description = task._language_description;
        _area_has_obj = task._area_has_obj;
        _ideal_cost = task._ideal_cost;
        // DEBUG_TASK_MEMORY_LEAK.insert(this);
    }
    ~Task() {
        // DEBUG_TASK_MEMORY_LEAK.erase(this); 
    }
    bool LoadProblemJson(const string &json_str);
	string GetLanguageDescription() const {return _language_description;};
    void SetLanguageDescription(const string &language_description) { _language_description = language_description; }
	vector<pair<string, string>> GetListSpecification() const {return _area_has_obj;};
    void RemoveGoal(const string &area_name, const string &object_name) {
        for (auto it = _area_has_obj.begin(); it != _area_has_obj.end(); it++) {
            if (it->first == area_name && it->second == object_name) {
                _area_has_obj.erase(it);
                break;
            }
        }
    }
    void AddGoal(const string &area_name, const string &object_name) {
         _area_has_obj.push_back(make_pair(area_name, object_name)); 
    }
    int GetIdealCost() const { return _ideal_cost; }
    // iterator of goal list
    vector<pair<string, string>>::iterator begin() { return _area_has_obj.begin(); }
    vector<pair<string, string>>::iterator end() { return _area_has_obj.end(); }
    // const iterator of goal list
    vector<pair<string, string>>::const_iterator begin() const { return _area_has_obj.begin(); }
    vector<pair<string, string>>::const_iterator end() const { return _area_has_obj.end(); }
    string ToJsonString() const;
    friend ostream& operator<<(ostream& os, const Task& task) {
        os << "Task: " << task._language_description << endl;
        os << "Goal List: " << endl;
        for (auto &goal : task._area_has_obj) {
            os << "  " << goal.first << " has " << goal.second << endl;
        }
        os << "Ideal Cost: " << task._ideal_cost;
        return os;
    }
};

class TaskSimple{
// private:
public:
    // int _area_has_obj[10][2];
    array<array<int, 2>, 10> _area_has_obj;
    int _goal_num;
    TaskSimple();
    TaskSimple(const Task &task, vector<string> area_names, vector<string> object_names);

    ~TaskSimple(){
        // No dynamic memory allocation, so no specific cleanup is needed.
        // Destructor can remain empty.
    }
    
    void AddGoal(int area_id, int object_id);
    void AddGoalFromName(const string &area_name, const string &object_name, vector<string> area_names, vector<string> object_names);
    void RemoveGoal(int area_id, int object_id);
    void RemoveGoalFromName(const string &area_name, const string &object_name, vector<string> area_names, vector<string> object_names);
    void Reset(){
        for(int i = 0; i < _goal_num; i++){
            _area_has_obj[i][0] = -1;
            _area_has_obj[i][1] = -1;
        }
        _goal_num = 0;
    }

    // int GetGoalNum() const { return _goal_num; }

    //= operator
    TaskSimple& operator=(const TaskSimple &task){
        memcpy(_area_has_obj.data(), task._area_has_obj.data(), sizeof(_area_has_obj));
        _goal_num = task._goal_num;
        return *this;
    }
    //== operator
    bool operator==(const TaskSimple &task) const {
        if(_goal_num != task._goal_num){
            return false;
        }
        //the order of goals may not be the same but the content should be the same
        bool same = true;
        for (int i = 0; i < _goal_num; i++)
        {
            bool find_match = false;
            for (int j = 0; j < task._goal_num; j++)
            {
                if(_area_has_obj[i][0] == task._area_has_obj[j][0] && _area_has_obj[i][1] == task._area_has_obj[j][1]){
                    find_match = true;
                    break;
                }
            }
            if(!find_match){
                same = false;
                break;
            }
        }
        return same;
    }

    friend class SceneGraphSimple;
};

class TaskSimpleExp{

public:
    array<int, 10> _objects;
    int _goal_num;

    TaskSimpleExp();
    TaskSimpleExp(const Task &task, vector<string> object_names);

    void AddGoal(int object_id);
    void AddGoalFromName(const string &object_name, vector<string> object_names);
    void RemoveGoal(int object_id);
    void RemoveGoalFromName(const string &object_name, vector<string> object_names);
    void Reset(){
        for(int i = 0; i < _goal_num; i++){
            _objects[i] = -1;
        }
        _goal_num = 0;
    }

    //= operator
    TaskSimpleExp& operator=(const TaskSimpleExp &task){
        memcpy(_objects.data(), task._objects.data(), sizeof(_objects));
        _goal_num = task._goal_num;
        return *this;
    }

    //== operator
    bool operator==(const TaskSimpleExp &task) const {
        if(_goal_num != task._goal_num){
            return false;
        }
        //the order of goals may not be the same but the content should be the same
        bool same = true;
        for (int i = 0; i < _goal_num; i++)
        {
            bool find_match = false;
            for (int j = 0; j < task._goal_num; j++)
            {
                if(_objects[i] == task._objects[j]){
                    find_match = true;
                    break;
                }
            }
            if(!find_match){
                same = false;
                break;
            }
        }
        return same;
    }
    friend class SceneGraphSimple;

};

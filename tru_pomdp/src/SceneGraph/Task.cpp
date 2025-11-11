#include <SceneGraph/Task.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
using json = nlohmann::json;

// define DEBUG_TASK_MEMORY_LEAK
// std::set<Task*> Task::DEBUG_TASK_MEMORY_LEAK;

Task::Task(const std::string &language_description) : _language_description(language_description) {
    // DEBUG_TASK_MEMORY_LEAK.insert(this);
}
bool Task::LoadProblemJson(const std::string &json_str) {
    json j;
    try {
        j = json::parse(json_str);
    } catch (json::parse_error &e) {
        std::cerr << "Failed to parse json: " << e.what() << std::endl;
        return false;
    }
    auto task_js = j["task"];
    _language_description = task_js["description"].get<std::string>();

    //if ["goal_set"] is found, use it to initialize the task
    /****
        "goal_set": [
            [
                {
                    "target_area": "human_hand",
                    "placed_object": "apple"
                },
                {
                    "target_area": "human_hand",
                    "placed_object": "wine"
                }
            ]
        ]
    ******/
   if(task_js.find("goal_set") != task_js.end()){
       for(auto &goal_set : task_js["goal_set"]){
           std::vector<std::pair<std::string, std::string>> goal_list;
           for(auto &goal : goal_set){
               goal_list.push_back(std::make_pair(goal["target_area"].get<std::string>(), goal["placed_object"].get<std::string>()));
           }
           _area_has_obj_list.push_back(goal_list);
       }
    }
    else{
        for (auto &goal : task_js["goal_list"]) {
            _area_has_obj.push_back(std::make_pair(goal["target_area"].get<std::string>(), goal["placed_object"].get<std::string>()));
        }
    }

    // for (auto &goal : task_js["goal_list"]) {
    //     _area_has_obj.push_back(std::make_pair(goal["target_area"].get<std::string>(), goal["placed_object"].get<std::string>()));
    // }
    // check the existence of ideal_cost
    if(task_js.find("ideal_cost") == task_js.end()){
        std::cerr << "ideal_cost not found in task" << std::endl;
        return false;
    }
    task_js["ideal_cost"].get_to(_ideal_cost);
    return true;
} 
std::string Task::ToJsonString() const {
    json j;
    j["description"] = _language_description;
    for(auto &area_obj : _area_has_obj){
        json goal;
        goal["target_area"] = area_obj.first;
        goal["placed_object"] = area_obj.second;
        
        if(j.find("goal_list") == j.end()){
            j["goal_list"] = json::array();
        }
        // j["goal_list"].push_back(goal);
    }
    j["ideal_cost"] = _ideal_cost;
    return j.dump(4);
}

/********************
 * TaskSimple
 *******************/

TaskSimple::TaskSimple(){
    for(int i = 0; i < 10; i++){
        _area_has_obj[i][0] = -1;
        _area_has_obj[i][1] = -1;
    }
    _goal_num = 0;
}

TaskSimple::TaskSimple(const Task &task, vector<string> area_names, vector<string> object_names){
    int count = 0;
    for(int i = 0; i < 10; i++){
        _area_has_obj[i][0] = -1;
        _area_has_obj[i][1] = -1;
    }
    _goal_num = 0;
    for(auto &goal : task.GetListSpecification()){
        int area_id = distance(area_names.begin(), find(area_names.begin(), area_names.end(), goal.first));
        int object_id = distance(object_names.begin(), find(object_names.begin(), object_names.end(), goal.second));
        if(area_id != area_names.size() && object_id != object_names.size()){
            _area_has_obj[count][0] = area_id;
            _area_has_obj[count][1] = object_id;
            count++;
        }
        else{
            std::cerr << "Area " << goal.first << " or Object " << goal.second << " does not exist" << std::endl;
        }
    }
    _goal_num = count;
}

void TaskSimple::AddGoal(int area_id, int object_id){

    // for(int i = 0; i < 50; i++){
    //     if(_area_has_obj[i][0] == -1){
    //         _area_has_obj[i][0] = area_id;
    //         _area_has_obj[i][1] = object_id;
    //         _goal_num++;
    //         break;
    //     }
    // }
    _area_has_obj[_goal_num][0] = area_id;
    _area_has_obj[_goal_num][1] = object_id;
    _goal_num++;
}

void TaskSimple::AddGoalFromName(const string &area_name, const string &object_name, vector<string> area_names, vector<string> object_names){
    int area_id = distance(area_names.begin(), find(area_names.begin(), area_names.end(), area_name));
    int object_id = distance(object_names.begin(), find(object_names.begin(), object_names.end(), object_name));
    if(area_id != area_names.size() && object_id != object_names.size()){
        AddGoal(area_id, object_id);
    }
    else{
        std::cerr << "Area " << area_name << " or Object " << object_name << " does not exist" << std::endl;
    }
}

void TaskSimple::RemoveGoal(int area_id, int object_id){
    for(int i = 0; i < 10; i++){
        if(_area_has_obj[i][0] == area_id && _area_has_obj[i][1] == object_id){
            //move the rest goals forward
            // for(int j = i; j < 9; j++){
            //     // _area_has_obj[j][0] = _area_has_obj[j+1][0];
            //     // _area_has_obj[j][1] = _area_has_obj[j+1][1];
            // }
            memmove(&_area_has_obj[i], &_area_has_obj[i+1], sizeof(array<int, 2>)*(9-i));
            _goal_num--;
            break;
        }
    }
}

void TaskSimple::RemoveGoalFromName(const string &area_name, const string &object_name, vector<string> area_names, vector<string> object_names){
    int area_id = distance(area_names.begin(), find(area_names.begin(), area_names.end(), area_name));
    int object_id = distance(object_names.begin(), find(object_names.begin(), object_names.end(), object_name));
    if(area_id != area_names.size() && object_id != object_names.size()){
        RemoveGoal(area_id, object_id);
    }
    else{
        std::cerr << "Area " << area_name << " or Object " << object_name << " does not exist" << std::endl;
    }
}

/********************
 * TaskSimpleExp
 *******************/

TaskSimpleExp::TaskSimpleExp(){
    for (int i = 0; i < 10; i++)
    {
        _objects[i] = -1;
    }
    _goal_num = 0;
}

TaskSimpleExp::TaskSimpleExp(const Task &task, vector<string> object_names){
    int count = 0;
    for(auto &goal : task.GetListSpecification()){
        int object_id = distance(object_names.begin(), find(object_names.begin(), object_names.end(), goal.second));
        if(object_id != object_names.size()){
            _objects[count] = object_id;
            count++;
        }
        else{
            std::cerr << "Object " << goal.second << " does not exist" << std::endl;
        }
    }
    _goal_num = count;
}

void TaskSimpleExp::AddGoal(int object_id){
    _objects[_goal_num] = object_id;
    _goal_num++;
}

void TaskSimpleExp::AddGoalFromName(const string &object_name, vector<string> object_names){
    int object_id = distance(object_names.begin(), find(object_names.begin(), object_names.end(), object_name));
    if(object_id != object_names.size()){
        AddGoal(object_id);
    }
    else{
        std::cerr << "Object " << object_name << " does not exist" << std::endl;
    }
}

void TaskSimpleExp::RemoveGoal(int object_id){
    for(int i = 0; i < 10; i++){
        if(_objects[i] == object_id){
            //move the rest goals forward
            // for(int j = i; j < 9; j++){
            //     _objects[j] = _objects[j+1];
            // }
            memmove(&_objects[i], &_objects[i+1], sizeof(int)*(9-i));
            _goal_num--;
            break;
        }
    }
}

void TaskSimpleExp::RemoveGoalFromName(const string &object_name, vector<string> object_names){
    int object_id = distance(object_names.begin(), find(object_names.begin(), object_names.end(), object_name));
    if(object_id != object_names.size()){
        RemoveGoal(object_id);
    }
    else{
        std::cerr << "Object " << object_name << " does not exist" << std::endl;
    }
}


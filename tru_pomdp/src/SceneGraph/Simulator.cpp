#include <SceneGraph/Simulator.h>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <cstdlib>
#include <vector>
#include <string>
#include <algorithm>
#include <unordered_set>


using json = nlohmann::json;
using namespace std;

// definition of Simulator::DEBUG_SIMULATOR_MEMORY_LEAK
// set<Simulator*> Simulator::DEBUG_SIMULATOR_MEMORY_LEAK;

unique_ptr<SceneGraph> Simulator::GetObservation() const {
    // cout<<"start copy observation"<<endl;
    return _graph.Copy(false);
}


Simulator::Simulator(const string& json_scene_str, const string &json_problem_str) {
    _graph.LoadSceneJsonFull(json_scene_str);
    _graph.LoadProblemJson(json_problem_str);
    _task.LoadProblemJson(json_problem_str);
    // DEBUG_SIMULATOR_MEMORY_LEAK.insert(this);

}

bool Simulator::GoalTestExp() {

    if (_task._area_has_obj_list.size() != 0) {
        vector<string> objects;
        for (const auto& goal: _task.GetListSpecification()) {
            objects.push_back(goal.second);
        }
        if (_graph.CheckObjectsExistsAndAreasOpen(objects)) {
            return true;
        }
    }
    return false;
}

vector<string> Simulator::target_objects_in_goal_set(const vector<string> target_objects) {
    vector<string> sorted_target_objects = target_objects;
    sort(sorted_target_objects.begin(), sorted_target_objects.end());
    
    if (_task._area_has_obj_list.size() != 0) {
        for (const auto& goals: _task._area_has_obj_list){
            vector<string> objects;
            for (const auto& goal: goals) {
                objects.push_back(goal.second);
            }
            // unordered_multiset<string> sorted_objects(objects.begin(), objects.end());
            // unordered_multiset<string> sorted_target_objects(target_objects.begin(), target_objects.end());

            // if (sorted_objects == sorted_target_objects) {
            //     return true;
            // }
            vector<string> sorted_objects = objects;
            sort(sorted_objects.begin(), sorted_objects.end());

            if (includes(sorted_target_objects.begin(), sorted_target_objects.end(), sorted_objects.begin(), sorted_objects.end())){
                return sorted_objects;
            }
        }
    }
    return {};
}

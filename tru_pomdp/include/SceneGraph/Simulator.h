#pragma once
#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/Task.h>
#include <SceneGraph/Action.h>

using namespace std;
class Simulator
{
private:
	SceneGraph _graph;
	Task _task;
	unsigned long _total_cost = 0;
public:
    // static set<Simulator*> DEBUG_SIMULATOR_MEMORY_LEAK;
    // Simulator should be constructed with json strings
	// Simulator() = delete; // todo : copy from default scene description
    // Simulator(const Simulator&) = delete;
    Simulator() {
    }

	Simulator(const string& json_scene_str, const string &json_problem_str);
	~Simulator() {
        // DEBUG_SIMULATOR_MEMORY_LEAK.erase(this);
    };
    vector<Action> GetLegalActions() {
        return _graph.GetLegalActions();
    }
	ActionResult TakeAction(const Action& action) {
        auto result = _graph.Forward(action);
        _total_cost += result.GetStepCost();
        return result;
    }
    // copy current graph and agent to state
    unsigned long GetTotalCost() const { return _total_cost; }
    Task GetTask() const { return _task; }
    bool GoalTest() {
        bool result = _graph.GoalTest(_task);
        return result;
    }
    vector<string>ObjectsReachGoal(){
        return _graph.ObjectsReachGoal(_task);
    }
    bool GoalTestExp();

    vector<string> target_objects_in_goal_set(const vector<string> target_objects);

    vector<string> all_objects_in_goal_set(){
        vector<string> objects;
        for(const auto& goals: _task._area_has_obj_list) {
            for (const auto& goal: goals) {
                string object = goal.second;
                if(find(objects.begin(), objects.end(), object) == objects.end()){
                    objects.push_back(object);
                }
            }
        }
        return objects;
    }
    
    unique_ptr<SceneGraph> GetObservation() const ;
    unique_ptr<SceneGraph> Copy() const {
        return _graph.Copy(true);
    }
    //copy the simulator

    unique_ptr<Simulator> CopySimulator() const {
        unique_ptr<Simulator> simulator = make_unique<Simulator>();
        simulator->_graph = *(_graph.Copy(true));
        simulator->_task = _task;
        return simulator;
    }
};
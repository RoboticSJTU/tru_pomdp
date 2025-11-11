#ifndef TRU_POMDP_WORLD_H
#define TRU_POMDP_WORLD_H

#include <tru_pomdp_task.h>
#include <utils.h>

#include <despot/interface/world.h>
#include <despot/core/pomdp_world.h>
#include <SceneGraph/Simulator.h>
#include <SceneGraph/Action.h>

#include <iostream>
#include <fstream>

using namespace despot;
using namespace std;

class TruPomdpWorld : public POMDPWorld
{
public:
    unique_ptr<Simulator> simulator_;
    vector<Action> task_all_actions_;
    vector<Action> *task_all_actions_ptr_;

    vector<string> *area_name_list_ptr_;
    vector<string> *object_name_list_ptr_;

    TruPomdpWorld(DSPOMDP *model, int seed) : POMDPWorld(model, seed) {
        task_all_actions_ptr_ = new vector<Action>();
        task_all_actions_ = {};

        area_name_list_ptr_ = new vector<string>();
        object_name_list_ptr_ = new vector<string>();
        vector<string> area_name_list = *area_name_list_ptr_;
        vector<string> object_name_list = *object_name_list_ptr_;
    }
    ~TruPomdpWorld() {
        delete task_all_actions_ptr_;
        delete area_name_list_ptr_;
        delete object_name_list_ptr_;
        cout << "delete trupomdpworld" << endl;
    }

    virtual bool Connect();
    virtual State *Initialize();
    virtual State *Initialize(const string problem_file_path, const string scene_file_path);
    // vector<string> *Initialize(const string problem_file_path, const string scene_file_path);
    virtual State *GetCurrentState() {
        return nullptr;
    };
    State *GetCurrentState(vector<ActionSimple> *all_actions_ptr, unordered_map<size_t, int> *action_index_map);
    virtual bool ExecuteAction(ACT_TYPE action, OBS_TYPE &obs);
    virtual bool ExecuteAction_save_data(ACT_TYPE action, OBS_TYPE &obs, int step);
};

#endif
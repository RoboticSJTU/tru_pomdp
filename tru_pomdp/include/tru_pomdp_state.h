#ifndef TRU_POMDP_STATE_H
#define TRU_POMDP_STATE_H

#include <despot/interface/pomdp.h>
#include <despot/core/particle_belief.h>
#include <despot/core/globals.h>

#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/Task.h>
#include <SceneGraph/Action.h>
#include <utils.h>

using namespace std;
namespace despot{

/* ==============================================================================
 * TruPomdpState class
 * ==============================================================================*/

class TruPomdpState: public State{
public:
    // unique_ptr<SceneGraph> scene_graph_;

    SceneGraphSimple scene_graph_;

    TaskSimple task_;
    // vector<string> action_strs_;
    // vector<string>* all_actions_ptr_;
    unique_ptr<array<ActionSimple, 200>> legal_actions_;
    int legal_action_num_;

    vector<ActionSimple>* all_actions_ptr_;
    unordered_map<size_t, int>* action_index_map_;
    TruPomdpState();
    TruPomdpState(const SceneGraphSimple scene_graph, const TaskSimple &task, vector<ActionSimple> *all_actions_ptr, unordered_map<size_t, int> *action_index_map);
    TruPomdpState(const SceneGraphSimple scene_graph, const TaskSimple& task, vector<ActionSimple>* all_actions_ptr, unordered_map<size_t, int> *action_index_map ,double weight, int state_id = 1, int scenario_id = 1);

    ~TruPomdpState(){
        // delete all_actions_ptr_;
        // delete action_index_map_;
    }

    virtual void UpdateActions();

    SceneGraphSimple scene_graph() const;
    void set_scene_graph(const SceneGraphSimple scene_graph);
    virtual void set_task(const TaskSimple& task);
    virtual string text() const;
    virtual State* Copy() const {
        // unique_ptr<SceneGraph> scene_graph_copy = scene_graph_->Copy();
        // SceneGraphSimple scene_graph_copy = scene_graph_.Copy();
        return new TruPomdpState(scene_graph_, task_, all_actions_ptr_, action_index_map_, weight, state_id, scenario_id);                                                      
    }
    // void Print(ostream& out = cout) const;

};

}//namespace despot

#endif


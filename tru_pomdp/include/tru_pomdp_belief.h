#ifndef TRU_POMDP_BELIEF_H
#define TRU_POMDP_BELIEF_H

#include <despot/interface/pomdp.h>
#include <despot/core/particle_belief.h>
#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/Task.h>
#include <SceneGraph/Action.h>

#include <tru_pomdp_task.h>
#include <tru_pomdp_state.h>
#include <tree_of_hypothesis.h>
#include <utils.h>

#include <sstream>
#include <random>
#include <algorithm>
#include <exception>
#include <unordered_set>

//json
#include <nlohmann/json.hpp>

using namespace std;
namespace despot{

class TruPomdpBelief:  public ParticleBelief{
// private:
    // vector<State*> particles_;

public:
    const DSPOMDP* model_;
    int unique_belief_num_;
    vector<int> unique_belief_list_;

    int step_;
    vector<pair<string, string>> object_matching_pairs_;
    int prompt_tokens_;
    int completion_tokens_;

    vector<string>* area_name_list_ptr_;
    vector<string>* object_name_list_ptr_;
    string language_instruction_;

    vector<TaskSimple> wrong_tasks_;

    vector<pair<TaskSimple, double>> partial_update_hypothesis_;

    int _reached_goal_task_num;

    // vector<State *> Sample(int num) const;
    TruPomdpBelief(vector<State*> particles, const DSPOMDP* model, vector<int> unique_belief_list);
    ~TruPomdpBelief(){
        for (int i = 0; i < particles_.size(); i++){
            delete particles_[i];
        }
        // //delete model
        // delete model_;
        cout << "delete trupomdpbelief" << endl;
    }
    //void UpdateActions(const vector<State*> particles);
    virtual vector<int> Update_return(ACT_TYPE action, OBS_TYPE obs);
    virtual void Update_SceneGraph(ACT_TYPE action, unique_ptr<SceneGraph> obs_scene_graph);
    virtual void ReConstructBelief(
        string language_instruction,
        SceneGraphSimple start_scene_graph,
        vector<ActionSimple> *all_actions_ptr,
        unordered_map<size_t, int> *action_index_map);
    vector<State*> Sample(int num, vector<State*> belief,
        const DSPOMDP* model) const;

    string generate_belief_prompt();
};
} // namespace despot

#endif
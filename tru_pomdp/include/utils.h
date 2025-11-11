#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <string>
#include <vector>
#include <sstream>  
#include <unordered_map>
#include <queue>
#include <climits>
#include <set>
#include <regex>

#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/Action.h>
#include <SceneGraph/Node.h>
#include <string>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <tree_of_hypothesis.h>
#include <unordered_set>

#include <tru_pomdp_task.h>
#include <tru_pomdp_belief.h>

#include <cstdlib>



using namespace std;

namespace despot{
    class TruPomdpTask;
    class State;
    class TruPomdpState;
    class TruPomdpExpTask;
    class TruPomdpExpState;
    class TruPomdpComplTask;
}

Action get_true_action(const string &action_str, const unique_ptr<SceneGraph> &scene_graph);

void generate_all_actions(const vector<string> &area_name_list, vector<string> &object_name_list, vector<ActionSimple> *all_actions_ptr, unordered_map<size_t, int> *action_index_map);

void get_target_object_related_actions(unique_ptr<array<ActionSimple, 200>> &actions, int &action_num, const vector<int> &target_object_names);

vector<Action> generate_task_all_actions(const vector<string> &area_name_list, vector<string> &object_name_list);

string generate_observation_prompt(SceneGraphSimple, const vector<string> area_names, const vector<string> object_names);

string generate_wrong_tasks_prompt(vector<TaskSimple> wrong_tasks, const vector<string> area_names, const vector<string> object_names);

nlohmann::json pare_llm_answer_to_json(string llm_answer);

bool encode_observation(SceneGraphSimple scene_graph, uint64_t &obs);

string get_clean_object_name(const string &object_name);

string call_llm_api(const string& url, const string& api_key, const nlohmann::json& root);

string modify_place_action(string action_sequence, string current_action);

namespace despot{
    void get_object_matching_pairs_from_tree_of_hypothesis(vector<pair<string, string>> &object_matching_pairs, vector<pair<string, string>> &current_object_matching_pairs,
                                                        const vector<vector<string>>& object_list_from_tree_of_hypothesis,
                                                        const vector<string> &all_objects);
    void generate_particles(vector<State *> &particles,
                            const vector<pair<map<string, pair<string, string>>, double>> &tree_of_hypothesis,
                            const SceneGraphSimple &start_scene_graph,
                            vector<string> *area_name_list_ptr,
                            vector<string> *object_name_list_ptr,
                            vector<ActionSimple> *all_actions_ptr,
                            unordered_map<size_t, int> *action_index_map_,
                            const vector<pair<string, string>> &current_object_matching_pairs,
                            int &unique_belief_num,
                            vector<int> &unique_belief_list,
                            const TruPomdpTask *task_ptr, 
                            const vector<TaskSimple> &wrong_tasks_ = {}
                            );
}
#endif
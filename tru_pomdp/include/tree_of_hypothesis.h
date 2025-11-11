#ifndef TREE_OF_HYPOTHESIS_H
#define TREE_OF_HYPOTHESIS_H

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <regex>
#include <thread>
#include <future>
#include <exception>
#include "openai/openai.hpp"  // Assuming openai-cpp library is included
#include <sstream>  // 需要引入
#include <fstream>
#include <cstdlib>  // setenv
#include <utils.h>

using namespace std;



class TreeOfHypothesis{
private:
    string llm_model_;
    string api_key_;
    string base_url_;
    int k_;
    // openai::OpenAI client_;

    string objects_of_interest_system_prompt;
    string objects_of_interest_example_prompt;
    string initial_areas_system_prompt;
    string initial_areas_example_prompt;
    string target_areas_system_prompt;
    string target_areas_example_prompt;

    string no_tree_system_prompt;
    string no_tree_example_prompt;

    string obj_of_interest_and_target_area_system_prompt;
    string obj_of_interest_and_target_area_example_prompt;

    string initial_areas_set_system_prompt;
    string initial_areas_set_example_prompt;


public:
    int prompt_tokens;
    int completion_tokens;
    bool exp_;

    TreeOfHypothesis(const string &model, const string &api_key, const string &base_url, int k);
    string query_llm(const string& prompt, string module);

    void RecursiveCombinations(
        const vector<string>& objects,
        const map<string, vector<pair<string, double>>>& initial_areas_map,
        const map<string, vector<pair<string, double>>>& target_areas_map,
        size_t idx,
        vector<pair<map<string, pair<string, string>>, double>>& all_combinations,
        map<string, pair<string, string>>& current_combination,
        double current_probability);

    void RecursiveCombinationsOneArea(
        const vector<string> &objects,
        const map<string, vector<pair<string, double>>> &initial_areas_map,
        size_t idx,
        vector<pair<map<string,string>, double>> &all_combinations,
        map<string, string> &current_combination,
        double current_probability);

    vector<pair<map<string, pair<string, string>>, double>>
    generate_tree_of_hypothesis_obj_of_interest_target_areas(
        const string &language_instruction, 
        const string &observation_prompt, 
        const string &wrong_states_prompt = "",
        const vector<pair<map<string, string>, double>> partial_update_hypothesis = {},
        const double total_probability = 1.0);

};

#endif
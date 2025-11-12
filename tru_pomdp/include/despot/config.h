#ifndef CONFIG_H
#define CONFIG_H

#include <string>

namespace despot {

struct Config {
	double time_per_move;  // CPU time available to construct the search tree
	int sim_len; // The number of simulation steps for each episode.
	int num_scenarios; // The number of scenarios usedto generate the DESPOT tree
	int search_depth; // The maximum depth of the search tree
	int max_policy_sim_len; // Maximum number of steps for simulating the default policy (rollout). Note that the depth of rollouts won't exceed the maximum search depth.
	double discount; // The discount factor
	double pruning_constant; // The pruning constant attached to each node for regularization purpose
	double xi; // xi * gap(root) is the target uncertainty at the root.
	unsigned int root_seed;
	std::string default_action;
	double noise;
	bool silence; // toggle logging

	int task_parsing_port;
	int belief_placement_port;
	bool save_data;
	std::string current_root_dir;
	int current_step;

	std::string llm_model;
	std::string api_key;
	std::string base_url;
	std::string proxy_url;  // Proxy URL, set to "None" to disable proxy
	int k;

	bool cot;
	
	std::string scene_graph;

	int total_llm_calls;
	double total_llm_query_time;

	int tree_of_hypothesis_llm_calls;
	double tree_of_hypothesis_llm_query_time;

	double total_search_time;

	std::string action_sequence;
	std::vector<std::string> objects_in_goal_list;
	std::string objects_in_goal;


	int tree_of_hypothesis_calls;
	int input_tokens;
	int output_tokens;

	Config() : time_per_move(1),
			   sim_len(90),
			   num_scenarios(500),
			   search_depth(90),
			   max_policy_sim_len(90),
			   discount(0.95),
			   pruning_constant(0),
			   xi(0.95),
			   root_seed(42),
			   default_action(""),
			   noise(0.1),
			   silence(false),
			   task_parsing_port(23100),
			   belief_placement_port(23200),
			   save_data(false),
			   current_root_dir("/home/ubuntu/projects/llm_pomdp/data/"),
			   current_step(0),

			   total_llm_calls(0),
			   total_llm_query_time(0),
			   tree_of_hypothesis_llm_calls(0),
			   tree_of_hypothesis_llm_query_time(0),
			   total_search_time(0),
			   action_sequence(""),
			   objects_in_goal_list({}),
			   objects_in_goal(""),
			   input_tokens(0),
			   output_tokens(0)
			   

	{
	}
};

} // namespace despot

#endif

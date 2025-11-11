#include <tru_pomdp_task.h>

using namespace std;
namespace despot
{
/* ==============================================================================
 * TruPomdpTask class
 * ==============================================================================*/

    bool TruPomdpTask::Step(State &s, double random_num, ACT_TYPE action, double &reward) const
    {
        // cout<<"start step"<<endl;
        TruPomdpState &state = static_cast<TruPomdpState &>(s);

        // vector<ActionSimple> all_actions = *state.all_actions_ptr_;
        ActionSimple true_action;
        try
        {
            true_action = (*state.all_actions_ptr_)[action];
        }
        catch(const exception& e){
            cout << "Caught an exception: " << e.what() << endl;
            reward = -100;
            return false;
        }
        // cout<<"true action: "<<true_action<<endl;

        if(true_action.Type() == ActionType::ACTION_NONE){
            reward = -100;
            return false;
        }
        // cout<<"not none action"<<endl;

        // SceneGraphSimple scene_graph = state.scene_graph_.Copy();
        reward = 0;
        // cout<<"copy scene graph"<<endl;

        vector<bool> subtask_reach_before_step = {};
        TaskSimple current_task;
        for (int i = 0; i < state.task_._goal_num; i++)
        {
            array<int, 2> goal = state.task_._area_has_obj[i];
            current_task.AddGoal(goal[0], goal[1]);
            subtask_reach_before_step.push_back(state.scene_graph_.GoalTest(current_task));
            current_task.Reset();
        }
        // cout<<"calculate subtask reach before step"<<endl;
        //find true action in legal actions
        if(find(state.legal_actions_->begin(), state.legal_actions_->begin() + state.legal_action_num_, true_action) == state.legal_actions_->begin() + state.legal_action_num_){
            reward = -100;
            return false;
        }
        // cout<<"find true action in legal actions"<<endl;

        bool goal_test_before_action = state.scene_graph_.GoalTest(state.task_);

        ActionResult action_result = state.scene_graph_.Forward(true_action);
        if (!action_result._is_success)
        {
            reward = -100;
            // state.UpdateActions();
            return false;
        }
        // cout<<"action success"<<endl;

        vector<bool> subtask_reach_after_step = {};
        // TaskSimple current_task;

        for(int i = 0; i < state.task_._goal_num; i++){
            array<int, 2> goal = state.task_._area_has_obj[i];
            current_task.AddGoal(goal[0], goal[1]);
            subtask_reach_after_step.push_back(state.scene_graph_.GoalTest(current_task));
            current_task.Reset();
        }
        // cout<<"calculate subtask reach after step"<<endl;

        // reward += 100 * (count(subtask_reach_after_step.begin(), subtask_reach_after_step.end(), true) - count(subtask_reach_before_step.begin(), subtask_reach_before_step.end(), true));
        int subtask_reach_before_step_count = count(subtask_reach_before_step.begin(), subtask_reach_before_step.end(), true);
        int subtask_reach_after_step_count = count(subtask_reach_after_step.begin(), subtask_reach_after_step.end(), true);

        if (subtask_reach_after_step_count > subtask_reach_before_step_count){
            reward += 200;
        }
        else if (subtask_reach_after_step_count < subtask_reach_before_step_count){
            reward -= 250;
        }
        // if (subtask_reach_after_step_count < subtask_reach_before_step_count){
        //     reward -= 300;
        // }

        // cout<<"action step cost: "<<action_result._step_cost<<endl;
        reward -= action_result._step_cost;
        // cout<<"reward: "<<reward<<endl;
        bool goal_test_after_action = state.scene_graph_.GoalTest(state.task_);
        if ((!goal_test_before_action) && goal_test_after_action){
            reward += 200;
            return true;
        }

        state.UpdateActions();
        // cout<<"goal test"<<endl;
        return false;
    }

    bool TruPomdpTask::Step(State &s, double random_num, ACT_TYPE action, double &reward, OBS_TYPE &obs) const
    {
        // cout<<"start step"<<endl;
        bool terminate = Step(s, random_num, action, reward);
        TruPomdpState &state = static_cast<TruPomdpState &>(s);
        // SceneGraphSimple observation_scene_graph = state.scene_graph_.Copy(false);
        encode_observation(state.scene_graph_.Copy(false), obs);
        // cout<<"encode observation"<<endl;
        return terminate;
    }

    State *TruPomdpTask::Copy(const State *particle) const
    {
        const TruPomdpState *objState = static_cast<const TruPomdpState *>(particle);
        TruPomdpState *state = static_cast<TruPomdpState *>(Allocate(objState->state_id, objState->weight));
        state->scene_graph_ = objState->scene_graph_.Copy();
        state->task_ = objState->task_;
        // state->legal_actions_ = objState->legal_actions_;
        //deep copy legal actions
        for (int i = 0; i < objState->legal_action_num_; i++)
        {
            state->legal_actions_->at(i) = objState->legal_actions_->at(i);
        }        
        state->legal_action_num_ = objState->legal_action_num_;
        state->all_actions_ptr_ = objState->all_actions_ptr_;
        state->action_index_map_ = objState->action_index_map_;
        state->weight = objState->weight;
        state->state_id = objState->state_id;
        state->scenario_id = objState->scenario_id;
        // state->UpdateActions();
        state->SetAllocated();
        return state;
    }

    vector<State *> TruPomdpTask::Copy(const vector<State *> particles) const
    {
        vector<State *> copied_particles;
        for (int i = 0; i < particles.size(); i++)
        {
            copied_particles.push_back(Copy(particles[i]));
        }
        return copied_particles;
    }


    Belief *TruPomdpTask::InitialBelief(const State *start, string type) const
    {
    }


    Belief *TruPomdpTask::InitialBelief_SceneGraph(
        const SceneGraphSimple start_scene_graph, 
        string language_instruction, 
        vector<string>*area_name_list_ptr,
        vector<string>* object_name_list_ptr,
        vector<ActionSimple>*all_actions_ptr) const
    {   

        vector<string> area_names = *area_name_list_ptr;
        vector<string> object_names = *object_name_list_ptr;

        string observation_prompt = generate_observation_prompt(start_scene_graph, *area_name_list_ptr, *object_name_list_ptr);

        string llm_model = Globals::config.llm_model;
        string api_key = Globals::config.api_key;
        string base_url = Globals::config.base_url;
        int k = Globals::config.k;


        vector<State *> particles;
        TreeOfHypothesis tree_of_hypothesis_ = TreeOfHypothesis(
            llm_model,
            api_key,
            base_url,
            k);
        int unique_belief_num = 0;
        vector<int> unique_belief_list;
        vector<pair<string, string>> object_matching_pairs;

        int try_time = 0;
        while(particles.size() == 0 && try_time <=3){
            try_time++;
            vector<pair<map<string, pair<string, string>>, double>> tree_of_hypothesis;
            for (int i = 0; i < 3; i++)
            {
                try{
                    tree_of_hypothesis = tree_of_hypothesis_.generate_tree_of_hypothesis_obj_of_interest_target_areas(
                        language_instruction,
                        observation_prompt);                    
                }
                catch(const exception& e){
                    cout << "Caught an exception: " << e.what() << endl;
                    continue;
                }
                if (tree_of_hypothesis.size() > 0)
                {
                    break;
                }
            }

            if (tree_of_hypothesis.size() == 0)
            {
                cout << "no hypothesis generated" << endl;
                // exit(1);
            }
            //normalize
            double total_prob = 0;
            for (const auto & [combination, prob]: tree_of_hypothesis)total_prob += prob;
            for (auto &[combination, prob] : tree_of_hypothesis)prob = prob / total_prob;

            vector<pair<string, string>> current_object_matching_pairs = {};
            SceneGraphSimple scene_graph_copy = start_scene_graph.Copy();
            vector<string> all_objects = scene_graph_copy.all_objects(*object_name_list_ptr);

            vector<vector<string>> object_list_from_tree_of_hypothesis;

            for (const auto &[combination, prob] : tree_of_hypothesis)
            {
                vector<string> objects;
                for (const auto &[object_name, area_pair] : combination)
                {
                    objects.push_back(object_name);
                }
                object_list_from_tree_of_hypothesis.push_back(objects);
            }

            get_object_matching_pairs_from_tree_of_hypothesis(object_matching_pairs, current_object_matching_pairs, object_list_from_tree_of_hypothesis, all_objects);
            //current_object_matching_pairs contains none
            //object_matching_pairs only contains matched objects

            unique_belief_num = 0;
            unique_belief_list.clear();
            generate_particles(particles, 
                tree_of_hypothesis, 
                start_scene_graph, 
                area_name_list_ptr, 
                object_name_list_ptr, 
                all_actions_ptr, 
                action_index_map_, 
                current_object_matching_pairs, 
                unique_belief_num, unique_belief_list,
                this);
        }

        if (particles.size() == 0){
            cout<<"No valid belief is generated"<<endl;
            throw runtime_error("No valid belief is generated");
        }

        //recalculate the weights
        double total_weight = 0;
        for (int i = 0; i<particles.size();i++){
            total_weight += particles[i]->weight;
        }
        for (int i = 0; i<particles.size();i++){
            particles[i]->weight /=total_weight;
        }
        cout<<"prompt_tokens: "<<tree_of_hypothesis_.prompt_tokens<<endl;
        cout<<"completion_tokens: "<<tree_of_hypothesis_.completion_tokens<<endl;
        TruPomdpBelief *belief = new TruPomdpBelief(particles, this, unique_belief_list);
        
        belief->prompt_tokens_ = tree_of_hypothesis_.prompt_tokens;
        belief->completion_tokens_ = tree_of_hypothesis_.completion_tokens;

        Globals::config.input_tokens += tree_of_hypothesis_.prompt_tokens;
        Globals::config.output_tokens += tree_of_hypothesis_.completion_tokens;
        Globals::config.tree_of_hypothesis_calls += 1;
        
        belief->object_matching_pairs_ = object_matching_pairs;
        belief->area_name_list_ptr_ = area_name_list_ptr;
        belief->object_name_list_ptr_ = object_name_list_ptr;
        belief->language_instruction_ = language_instruction;

        generate_all_actions(*area_name_list_ptr, *object_name_list_ptr, all_actions_ptr, action_index_map_);
        return belief;
    }

    ParticleLowerBound *TruPomdpTask::CreateParticleLowerBound(string name) const
    {
        return new TruPomdpLowerBound(this);
    }

    ScenarioLowerBound *TruPomdpTask::CreateScenarioLowerBound(string name, string particle_bound_name) const
    {
        // return new TruPomdpLowerBound(this);
        DSPOMDP *model = const_cast<TruPomdpTask *>(this);
        return new TruPomdpDefaultPolicy(model, new TruPomdpLowerBound(model));
    }

    ParticleUpperBound *TruPomdpTask::CreateParticleUpperBound(string name) const
    {
        return new TruPomdpUpperBound();
    }

    ScenarioUpperBound *TruPomdpTask::CreateScenarioUpperBound(string name, string particle_bound_name) const
    {
        return new TruPomdpUpperBound();
    }

    void TruPomdpTask::PrintState(const State &state, ostream &out) const
    {
        const TruPomdpState &objState = static_cast<const TruPomdpState &>(state);
        out << objState.text();
    }

    void TruPomdpTask::PrintBelief(const Belief &belief, ostream &out) const
    {
        const TruPomdpBelief &simple_belief = static_cast<const TruPomdpBelief &>(belief);
        out << simple_belief.text();
    }

    void TruPomdpTask::PrintAction(ACT_TYPE action, ostream &out) const
    {
        vector<ActionSimple> all_actions = *all_actions_ptr_;
        ActionSimple action_str = all_actions[action];

        out << action_str;
    }

    State *TruPomdpTask::Allocate(int state_id, double weight) const
    {
        // return new TruPomdpState();
        // TruPomdpState *state = new TruPomdpState();
        TruPomdpState *state = memory_pool_.Allocate();
        state->state_id = state_id;
        state->weight = weight;
        return state;
    }

    void TruPomdpTask::Free(State *particle) const
    {
        // delete particle;
        memory_pool_.Free(static_cast<TruPomdpState *>(particle));
    }

    int TruPomdpTask::NumActiveParticles() const
    {
        cout << "num_allocated: " << memory_pool_.num_allocated() << endl;
        return memory_pool_.num_allocated();
    }

/* ==============================================================================
 * TruPomdpUpperBound class
 * ==============================================================================*/

    TruPomdpUpperBound::TruPomdpUpperBound()
    {
    }

    double TruPomdpUpperBound::Value(const State &state) const
    {
        const TruPomdpState *objState = static_cast<const TruPomdpState *>(&state);
        int num_goals = objState->task_._goal_num;
        // double upper_bound = (num_goals) * 200 + 500;
        double upper_bound = (num_goals) * 200 + 200;
        return upper_bound;
        // return 500;
    }

    double TruPomdpUpperBound::Value(const vector<State*> &particles) const
    {   
        int max_num_goals = 0;
        for(const auto& particle: particles){
            const TruPomdpState *objState = static_cast<const TruPomdpState *>(particle);
            int num_goals = objState->task_._goal_num;
            if(num_goals > max_num_goals){
                max_num_goals = num_goals;
            }
        }
        double uppder_bound = (max_num_goals) * 200 + 200;
        return uppder_bound;
        // return 500;
    }

    double TruPomdpUpperBound::Value(const vector<State*>& particles, RandomStreams& streams, History& history) const{
        return Value(particles);
    }


/* ==============================================================================
 * TruPomdpLowerBound class
 * ==============================================================================*/

    TruPomdpLowerBound::TruPomdpLowerBound(const DSPOMDP* model) : ParticleLowerBound(model)
    {
    }

    //random policy
    ValuedAction TruPomdpLowerBound::Value_random(const State &s) const
    {
        const TruPomdpState* state = static_cast<const TruPomdpState*>(&s);
        ActionSimple action;
        if (state->legal_action_num_ <= 1){
            cerr <<"state legal action num" << state->legal_action_num_<<endl;
            action = (*state->all_actions_ptr_)[0];
        }
        else{
            random_device rd;
            mt19937 gen(rd());
            uniform_int_distribution<> dis(0, state->legal_action_num_ - 1);
            int random_index = dis(gen);

            action = (*state->legal_actions_)[random_index];
        }
		//action_int is the index of the action in all_actions
		// ACT_TYPE action_int = find(all_actions.begin(), all_actions.end(), action) - all_actions.begin();
        size_t action_key = hash<ActionSimple>{}(action);
        ACT_TYPE action_int = (*state->action_index_map_) [action_key];

        SceneGraphSimple scene_graph = state->scene_graph_.Copy();
        ActionResult action_result = scene_graph.Forward(action);     

        return ValuedAction(action_int, action_result._step_cost);
    }

    ValuedAction TruPomdpLowerBound::Value(const State &s) const{
        return ValuedAction(0, 0);
    }

    ValuedAction TruPomdpLowerBound::Value(const vector<State*>& particles) const{
        return ValuedAction(0, 0);
    }

    ACT_TYPE TruPomdpDefaultPolicy::Action_random(const State &s) const
    {
        const TruPomdpState* state = static_cast<const TruPomdpState*>(&s);
        ActionSimple action;
        if (state->legal_action_num_ <= 1){
            cerr <<"state legal action num" << state->legal_action_num_<<endl;
            action = (*state->all_actions_ptr_)[0];
        }
        else{
            random_device rd;
            mt19937 gen(rd());
            uniform_int_distribution<> dis(0, state->legal_action_num_ - 1);
            int random_index = dis(gen);

            action = (*state->legal_actions_)[random_index];
        }
        size_t action_key = hash<ActionSimple>{}(action);
        ACT_TYPE action_int = (*state->action_index_map_) [action_key];

        return action_int;
    }

    ACT_TYPE TruPomdpDefaultPolicy::Action(const vector<State *> &particles,
                    RandomStreams &streams, History &history) const{
        
        map<int, int> action_count;
        TaskSimple task;
        TaskSimple temp_task;
        for (int i = 0; i < particles.size(); i++)
        {
            TruPomdpState* objstate = static_cast<TruPomdpState*>(particles[i]);
            task = objstate->task_;
            vector<array<int, 2>> temp_unreached_goals= {};

            for(int goal_index = 0; goal_index < task._goal_num; goal_index++){
                array<int, 2> goal = task._area_has_obj[goal_index];
                temp_task.AddGoal(goal[0], goal[1]);
                bool goal_test = objstate->scene_graph_.GoalTest(temp_task);
                if (!goal_test){
                    temp_unreached_goals.push_back(goal);
                }
                temp_task.Reset();
            }

            // ActionSimple action = NextAction(temp_unreached_goals, objstate->scene_graph_);
            // size_t action_key = hash<ActionSimple>{}(action);   
            // ACT_TYPE action_int = (*objstate->action_index_map_) [action_key];

            // if (find(objstate->legal_actions_->begin(), objstate->legal_actions_->begin() + objstate->legal_action_num_, action) == objstate->legal_actions_->begin() + objstate->legal_action_num_){
            ACT_TYPE action_int = Action_random(*objstate);
            // }
            if (action_count.find(action_int) == action_count.end()){
                action_count[action_int] = 1;
            }
            else{
                action_count[action_int] += 1;
            }
        }
        int max_action = -1;
        int max_count = 0;
        for (const auto& entry : action_count)
        {
            int action = entry.first;
            int count = entry.second;

            if (count > max_count) {
                max_count = count;
                max_action = action;
            }
        }
        return max_action;

    }

} // namespace despot

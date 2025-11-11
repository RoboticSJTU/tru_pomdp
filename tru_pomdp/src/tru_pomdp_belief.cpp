#include <tru_pomdp_belief.h>

using namespace std;

namespace despot{

/* =============================================================================
 * TruPomdpBelief class
 * =============================================================================*/

    TruPomdpBelief::TruPomdpBelief(vector<State*> particles, const DSPOMDP* model, vector<int> unique_belief_list):
    ParticleBelief(particles, model, nullptr, false)
    {
        // particles_ = particles;
        model_ = model;
        unique_belief_list_  = unique_belief_list;
        unique_belief_num_ = unique_belief_list.size();
        step_ = 0;
        // tree_of_hypothesis_ = tree_of_hypothesis;
         prompt_tokens_ = 0;
        completion_tokens_ = 0;
        _reached_goal_task_num = 0;

    }

    vector<int> TruPomdpBelief::Update_return(ACT_TYPE action, OBS_TYPE obs)
    {
        step_++;
        TruPomdpState *state = static_cast<TruPomdpState *>(particles_[0]);

        vector<ActionSimple> all_actions = *state->all_actions_ptr_;
        ActionSimple true_action = all_actions[action];
        vector<int> wrong_believes = {};

        for (int i = 0; i < particles_.size(); i++)
        {
            TruPomdpState *state = static_cast<TruPomdpState *>(particles_[i]);

            // unique_ptr<SceneGraph> scene_graph = state->scene_graph_->Copy();

            ActionResult action_result = state->scene_graph_.Forward(true_action);
            if (! action_result._is_success){
                // cout<<"action failed"<<endl;
                // cout<<"action: "<<action_str<<endl;
                // cout<<"failed reason: "<<action_result._feedback_message<<endl;
                wrong_believes.push_back(state->state_id);
            }
            state->UpdateActions();
            particles_[i] = state;
        }
        return wrong_believes;
    }

    void TruPomdpBelief::Update_SceneGraph(ACT_TYPE action, unique_ptr<SceneGraph> obs_scene_graph)
    {

        for(const auto& objet_node: obs_scene_graph->all_objects())
        {
            string object_name = objet_node.GetName();
            if(find(object_name_list_ptr_->begin(), object_name_list_ptr_->end(), object_name) == object_name_list_ptr_->end())
            {
                object_name_list_ptr_->push_back(object_name);
            }
        }

        vector<string> object_names = *object_name_list_ptr_;

        SceneGraphSimple obs_scene_graph_simple;
        unique_ptr<SceneGraph> obs_scene_graph_ptr = obs_scene_graph->Copy();
        obs_scene_graph_simple = obs_scene_graph_simple.CopyFromSceneGraph(move(obs_scene_graph_ptr), *area_name_list_ptr_, *object_name_list_ptr_);

        vector<int> wrong_believes_update = Update_return(action, 0);
        vector<pair<string, string>> current_object_matching_pairs = object_matching_pairs_; //include none
        string language_instruction = language_instruction_;
        vector<ActionSimple>* all_actions_ptr = static_cast<TruPomdpState *>(particles_[0])->all_actions_ptr_;
        unordered_map<size_t, int>* action_index_map = static_cast<TruPomdpState *>(particles_[0])->action_index_map_;

        vector<string> unmatched_objects_in_current_state;
        string unmatched_objects_string = "(";

        vector<string> observed_objects;
        string observed_objects_string = "(";

        for (int i = 0; i < particles_.size(); i++)
        {
            TruPomdpState *state = static_cast<TruPomdpState *>(particles_[i]);

            for (const auto& object: state->scene_graph_.all_objects(*object_name_list_ptr_))
            {
                string object_name = get_clean_object_name(object);
                //step 1: check if the object is in current_object_matching_pairs_
                bool object_in_current_object_matching_pairs = false;
                for (const auto& pair: current_object_matching_pairs)
                {
                    if (pair.first == object_name)
                    {
                        object_in_current_object_matching_pairs = true;
                        break;
                    }
                }
                bool object_in_unmatched_objects = false;
                for (const auto &unmatched_object : unmatched_objects_in_current_state)
                {
                    if (unmatched_object == object_name)
                    {
                        object_in_unmatched_objects = true;
                        break;
                    }
                }
                
                //step 2: check if the object is in current observation
                bool object_in_observation = false;
                for (const auto& obs_object: obs_scene_graph_simple.all_objects(*object_name_list_ptr_))
                {
                    string obs_object_name = get_clean_object_name(obs_object);
                    if (object_name == obs_object_name)
                    {
                        object_in_observation = true;
                        if(!object_in_current_object_matching_pairs)
                        {
                            current_object_matching_pairs.push_back(make_pair(object_name, obs_object_name));
                            object_matching_pairs_.push_back(make_pair(object_name, obs_object_name));
                        }
                        // current_object_matching_pairs.push_back(make_pair(object_name, obs_object_name));
                        // object_matching_pairs_.push_back(make_pair(object_name, obs_object_name));
                        break;
                    }
                }
                if(!object_in_current_object_matching_pairs && !object_in_unmatched_objects && !object_in_observation)
                {
                    unmatched_objects_in_current_state.push_back(object_name);
                    unmatched_objects_string += object_name + " ";
                }
            }
        }
        double wrong_belief_probability = 0;
        double all_probability = 0;
        double new_particles_weight = 0;
        vector<State* > new_particles = {};
        vector<int> old_unique_belief_list = unique_belief_list_;

        vector<State* > partial_update_particles; //particles that only need to update the initial areas

        // for (const auto& belief_id: unique_belief_list_)

        // int reached_goal_task_num = 0;
        for (int index = 0; index < unique_belief_num_; index++)
        {
            //if belief_id is in wrong_believes_update
            // cout<<"check one belief"<<endl;
            int belief_id = old_unique_belief_list[index];
            // if (find(wrong_believes_update.begin(), wrong_believes_update.end(), belief_id) != wrong_believes_update.end()){
            //     // wrong_believes.push_back(belief_id);
            //     unique_belief_list_.erase(remove(unique_belief_list_.begin(), unique_belief_list_.end(), belief_id), unique_belief_list_.end());
            //     continue;
            // }
            TruPomdpState *state;

            for (int i = 0; i < particles_.size(); i++){
                int state_id = particles_[i]->state_id;
                if (state_id == belief_id){
                    state = static_cast<TruPomdpState *>(particles_[i]);
                    break;
                }
            }
            int wrong_belief = false;
            TaskSimple task = state->task_;
            SceneGraphSimple belief_scene_graph = state->scene_graph_.Copy();
            vector<string> belief_object_names = belief_scene_graph.all_objects(*object_name_list_ptr_);

            SceneGraphSimple new_scene_graph = obs_scene_graph_simple.Copy();
            if (belief_scene_graph.GoalTest(task)){

                bool in_wrong_tasks = false;
                for (const auto & wrong_task: wrong_tasks_){
                    if (task == wrong_task){
                        in_wrong_tasks = true;
                        break;
                    }
                }
                if (!in_wrong_tasks) 
                {
                    wrong_tasks_.push_back(task);
                }
                unique_belief_list_.erase(remove(unique_belief_list_.begin(), unique_belief_list_.end(), state->state_id), unique_belief_list_.end());
                _reached_goal_task_num +=1;
                continue;
            }
            
            //todo

            array<array<int, 2>, 10> area_has_obj = task._area_has_obj;
            int goal_num = task._goal_num;

            for (int goal_index = 0; goal_index < goal_num; goal_index++){
                array<int, 2> goal = area_has_obj[goal_index];

                TaskSimple current_task;
                current_task.AddGoal(goal[0], goal[1]);
                if (new_scene_graph.GoalTest(current_task)){
                    continue;
                }
                int target_object_id = goal[1];
                string target_object_name = object_name_list_ptr_->at(target_object_id);
                if (find(belief_object_names.begin(), belief_object_names.end(), target_object_name) == belief_object_names.end())
                {
                    // wrong_believes.push_back(belief_id);
                    wrong_belief = true;
                    break;
                }
                string matching_object_name = "";
                for (const auto& pair: current_object_matching_pairs)
                {
                    if (pair.first == get_clean_object_name(target_object_name))
                    {
                        matching_object_name = pair.second;
                        break;
                    }
                }
                if (matching_object_name == "None"  || matching_object_name == "")
                {
                    // wrong_believes.push_back(belief_id);
                    
                    string belief_target_object_name = get_clean_object_name(target_object_name);
                    string belief_target_object_initial_area_name = belief_scene_graph.GetObjectParentName(belief_target_object_name, *object_name_list_ptr_, *area_name_list_ptr_);
                    bool belief_target_object_partent_open = belief_scene_graph.GetAreaOpenFromName(belief_target_object_initial_area_name, *area_name_list_ptr_);
                    if(belief_target_object_partent_open)
                    {
                        // wrong_believes.push_back(belief_id);
                        partial_update_particles.push_back(state);
                        wrong_belief = true;
                        break;
                    }
                    else{
                        target_object_name = get_clean_object_name(target_object_name);
                        if(belief_target_object_initial_area_name == "robot"){
                            
                            //check whether target_object_name is in object_name_list_ptr_
                            if(find(object_name_list_ptr_->begin(), object_name_list_ptr_->end(), target_object_name) == object_name_list_ptr_->end())
                            {
                                object_name_list_ptr_->push_back(target_object_name);
                            }

                            new_scene_graph.AddObjectInHandFromName(target_object_name, *object_name_list_ptr_);
                        }
                        else{
                            if(find(object_name_list_ptr_->begin(), object_name_list_ptr_->end(), target_object_name) == object_name_list_ptr_->end())
                            {
                                object_name_list_ptr_->push_back(target_object_name);
                            }
                            new_scene_graph.AddObjectInAreaFromName(belief_target_object_initial_area_name, target_object_name, *area_name_list_ptr_, *object_name_list_ptr_);
                        }
                    }
                }
                else{
                    // string belief_object_initial_area_name = state_scene_graph->GetObject(target_object_name)->GetParent()->GetName();
                    // shared_ptr<AreaNode> true_object_initial_area_node = new_scene_graph->GetArea(belief_object_initial_area_name);
                    const string belief_object_initial_area_name = belief_scene_graph.GetObjectParentName(target_object_name, *object_name_list_ptr_, *area_name_list_ptr_);
                    if(belief_object_initial_area_name == "robot"){

                        string robot_hand_object_name = new_scene_graph.GetObjectInHandName(*object_name_list_ptr_);
                        string true_object_name_adjust = get_clean_object_name(robot_hand_object_name);
                        if (true_object_name_adjust != matching_object_name)
                        {
                            // wrong_believes.push_back(belief_id);
                            partial_update_particles.push_back(state);
                            wrong_belief = true;
                            break;
                        }
                        vector<string> area_names = *area_name_list_ptr_;
                        string target_area_name = area_names[goal[0]];
                        task.RemoveGoalFromName(target_area_name, robot_hand_object_name, *area_name_list_ptr_, *object_name_list_ptr_);
                        task.AddGoalFromName(target_area_name, robot_hand_object_name, *area_name_list_ptr_, *object_name_list_ptr_);
                    }
                    else{
                        bool matching_object_in_initial_area = false;
                        // shared_ptr<AreaNode> true_object_initial_area_node = new_scene_graph->GetArea(belief_object_initial_area_name);
                        string true_object_name;
                        for (const auto &object : new_scene_graph.GetObjectsInArea(belief_object_initial_area_name, *object_name_list_ptr_, *area_name_list_ptr_))
                        {
                            string true_object_name_adjust = get_clean_object_name(object);
                            if (true_object_name_adjust == matching_object_name)
                            {
                                true_object_name = object;
                                matching_object_in_initial_area = true;
                                break;
                            }
                        }
                        if (!matching_object_in_initial_area)
                        {
                            // wrong_believes.push_back(belief_id);
                            partial_update_particles.push_back(state);
                            wrong_belief = true;
                            break;
                        }
                        else{
                            // string target_area_name = *area_name_list_ptr_[goal[0]];
                            vector<string> area_names = *area_name_list_ptr_;
                            string target_area_name = area_names[goal[0]];
                            task.RemoveGoalFromName(target_area_name, target_object_name, *area_name_list_ptr_, *object_name_list_ptr_);
                            task.AddGoalFromName(target_area_name, true_object_name, *area_name_list_ptr_, *object_name_list_ptr_);

                        }
                    }
                }
            }

            if (!wrong_belief){
                state->scene_graph_ = new_scene_graph.Copy();
                state->task_ = task;
                state->UpdateActions();               
                new_particles.push_back(state);
                new_particles_weight += state->weight;
            }
            else{
                wrong_belief_probability+=state->weight;
                unique_belief_list_.erase(remove(unique_belief_list_.begin(), unique_belief_list_.end(), state->state_id), unique_belief_list_.end());
            }
            all_probability += state->weight;
        }

        particles_ = new_particles;
        unique_belief_num_ = unique_belief_list_.size();

        // for (const auto &partial_update_state: partial_update_particles){
        //     TruPomdpState *state = static_cast<TruPomdpState *>(partial_update_state);
        //     TaskSimple task = state->task_;
            
        //     bool in_partial_update_hypothesis = false;
        //     for (int i = 0; i < partial_update_hypothesis_.size(); i++){
        //         if (partial_update_hypothesis_[i].first == task){
        //             partial_update_hypothesis_[i].second += state->weight;
        //             in_partial_update_hypothesis = true;
        //             break;
        //         }
        //     }
        //     // if(!in_partial_update_hypothesis){
        //     //     partial_update_hypothesis_.push_back(make_pair(task, state->weight));
        //     // }
        // }        

        if (wrong_belief_probability >= 0.7 or new_particles_weight < 0.3 or _reached_goal_task_num >=2)
        {
            _reached_goal_task_num = 0;
            cout<<"new_particles_weight: "<<new_particles_weight<<endl;
            for(int try_time = 0; try_time < 3; try_time++){
                ReConstructBelief(language_instruction, obs_scene_graph_simple.Copy() , all_actions_ptr, action_index_map);
                if(particles_.size() > 0){
                    break;
                }
            }
            if (particles_.size() == 0){
                throw runtime_error("No valid belief is generated");
            }
        }
        double total_prob = 0;
        vector<double> weight_list = {};
        for (const auto& belief: particles_)
        {
            total_prob += belief->weight;
            weight_list.push_back(belief->weight);
        }
        cout<<"total_prob: "<<total_prob<<endl;
        generate_all_actions(*area_name_list_ptr_, *object_name_list_ptr_, all_actions_ptr, action_index_map);
    }

    void TruPomdpBelief::ReConstructBelief(
        string language_instruction, 
        SceneGraphSimple start_scene_graph_simple, 
        vector<ActionSimple>* all_actions_ptr, 
        unordered_map<size_t, int>* action_index_map)
    {
        cout<<"start ReConstructBelief"<<endl;
        // SceneGraphSimple start_scene_graph_simple = start_scene_graph;
        // start_scene_graph_simple = start_scene_graph_simple.CopyFromSceneGraph(move(start_scene_graph), *area_name_list_ptr_, *object_name_list_ptr_);
        string observation_prompt = generate_observation_prompt(start_scene_graph_simple, *area_name_list_ptr_, *object_name_list_ptr_);

        string wrong_tasks_prompt = generate_wrong_tasks_prompt(wrong_tasks_, *area_name_list_ptr_, *object_name_list_ptr_);

        TreeOfHypothesis tree_of_hypothesis_ = TreeOfHypothesis(
            Globals::config.llm_model,
            Globals::config.api_key,
            Globals::config.base_url,
            Globals::config.k);

        // vector<pair<map<string, string>, double>> partial_update_hypothesis;
        // for (const auto & partial_update_task : partial_update_hypothesis_){
        //     TaskSimple task = partial_update_task.first;
        //     double prob = partial_update_task.second;
        //     map<string, string> partial_update_combination;
        //     for (int goal_index = 0; goal_index < task._goal_num; goal_index++){
        //         array<int, 2> goal = task._area_has_obj[goal_index];
        //         string target_object_name = object_name_list_ptr_->at(goal[1]);
        //         string target_area_name = area_name_list_ptr_->at(goal[0]);
        //         partial_update_combination[target_object_name] = target_area_name;
        //     }
        //     partial_update_hypothesis.push_back(make_pair(partial_update_combination, prob));
        // }

        double new_particles_total_probability;
        for (const auto& belief: particles_)
        {
            new_particles_total_probability += belief->weight;
        }

        vector<pair<map<string, pair<string, string>>, double>> tree_of_hypothesis;
        try{
            tree_of_hypothesis = tree_of_hypothesis_.generate_tree_of_hypothesis_obj_of_interest_target_areas(
                language_instruction,
                observation_prompt,
                wrong_tasks_prompt);
        }
        catch (const exception& e){
            cout<<e.what()<<endl;
        }

        double current_total_probability = 0;

        for (const auto &belief: particles_)
        {
            current_total_probability += belief->weight;
        }

        int max_state_id = 0;
        for (const auto& belief_id: unique_belief_list_)
        {
            if (belief_id > max_state_id)
            {
                max_state_id = belief_id;
            }
        }        

        int unique_belief_num = max_state_id + 1;
        vector<pair<string, string>> current_object_matching_pairs = object_matching_pairs_;
        vector<string> unmatched_objects_in_current_state;
        string unmatched_objects_string = "(";

        vector<string> observed_objects;
        string observed_objects_string = "(";

        const TruPomdpTask* model = static_cast<const TruPomdpTask*>(model_);
        SceneGraphSimple scene_graph_copy = start_scene_graph_simple.Copy();
        vector<string> all_objects = scene_graph_copy.all_objects(*object_name_list_ptr_);

        vector<vector<string>> object_list_from_tree_of_hypothesis;

        for (const auto &[combination, prob] : tree_of_hypothesis)
        {
            vector<string> objects;
            for (const auto &[object_name, obs_object_name]: combination)
            {
                objects.push_back(object_name);
            }
            object_list_from_tree_of_hypothesis.push_back(objects);
        }

        get_object_matching_pairs_from_tree_of_hypothesis(
            object_matching_pairs_, 
            current_object_matching_pairs, 
            object_list_from_tree_of_hypothesis , 
            all_objects);
        

        vector<State* > new_particles = {};

        generate_particles(
            new_particles, 
            tree_of_hypothesis, 
            start_scene_graph_simple, 
            area_name_list_ptr_, 
            object_name_list_ptr_, 
            all_actions_ptr, 
            action_index_map, 
            current_object_matching_pairs, 
            unique_belief_num, 
            unique_belief_list_, 
            model,
            wrong_tasks_);

        unique_belief_num_  = unique_belief_list_.size();
        if ((particles_.size() + new_particles.size()) == 0 )
        {
            cout<<"no valid hypothesis"<<endl;
        }
        else{
            double new_particles_weight = {};
            for (const auto& belief: new_particles)
            {
                new_particles_weight += belief->weight;
            }
            for (const auto& belief: new_particles)
            {
                belief->weight = belief->weight / new_particles_weight * (1 - current_total_probability);
                particles_.push_back(belief);
            }

        }

        prompt_tokens_+=tree_of_hypothesis_.prompt_tokens;
        completion_tokens_+=tree_of_hypothesis_.completion_tokens;
        cout<<"prompt_tokens: "<<prompt_tokens_<<endl;
        cout<<"completion_tokens: "<<completion_tokens_<<endl;

        Globals::config.input_tokens += tree_of_hypothesis_.prompt_tokens;
        Globals::config.output_tokens += tree_of_hypothesis_.completion_tokens;
        Globals::config.tree_of_hypothesis_calls += 1;

        //delete all new particles
        // for (const auto& belief: new_particles)
        // {
        //     delete belief;
        // }
    }



    // vector<State*> TruPomdpBelief::Sample(int num) const {
    //     return Sample(num, particles_, model_);
    // }

    vector<State*> TruPomdpBelief::Sample(int num, vector<State*> particles, const DSPOMDP* model) const {
        vector<State*> sample;

        // 当 particles 的数量小于 num 时，直接根据每个粒子的权重分配粒子
        cout<<"current particle num: " << particles.size() <<endl;
        if (particles.size() < num) {
            double total_weight = 0.0;
            for (State* p : particles) {
                total_weight += p->weight;  // 计算所有粒子的权重总和
            }

            // 根据每个粒子的权重来分配新的粒子
            for (State* p : particles) {
                int count = static_cast<int>(num * (p->weight / total_weight));  // 计算该粒子应该贡献的粒子数

                // 复制粒子，并根据需要的数量添加到样本中
                for (int i = 0; i < count; i++) {
                    State* particle = model->Copy(p);  // 复制粒子
                    particle->weight = 1.0 / num;  // 新粒子的权重均为 1/num
                    sample.push_back(particle);
                }
            }

            // 如果样本数量不足 num，补充粒子（均匀补充）
            while (sample.size() < num) {
                State* particle = model->Copy(particles[0]);  // 复制第一个粒子（可以选择任何粒子作为补充）
                particle->weight = 1.0 / num;
                sample.push_back(particle);
            }
        }
        else { // 当粒子数量大于等于 num 时，使用当前的采样逻辑
            //set random seed to 0
            // cout<<"start sample"<<endl;
            double unit = 1.0 / num;
            vector<double> weight_list;
            for (int i = 0; i < particles.size(); i++)
            {
                weight_list.push_back(particles[i]->weight);
            }

            // double seed = 0.0;
            mt19937 gen(42);
            discrete_distribution<> dist(weight_list.begin(), weight_list.end());

            // vector<int> sampled_ids;
            // sampled_ids.resize(num);
            for (int i = 0; i < num; i++) 
            {
                int id = dist(gen);
                State* particle = model->Copy(particles[id]);
                particle->weight = unit;
                sample.push_back(particle);
            }
        }

        // 随机打乱采样的粒子，增加多样性
        random_shuffle(sample.begin(), sample.end());
        // cout<<"complete sample"<<endl;
        // logd << "[ParticleBelief::Sample] Sampled " << sample.size() << " particles" << endl;
        // for (int i = 0; i < sample.size(); i++) {
        //     logv << " " << i << " = " << *sample[i] << endl;
        // }

        return sample;
    }

    string TruPomdpBelief::generate_belief_prompt(){
        
        vector<pair<map<string, pair<string, string>> , double>> belief_list;
        vector<TaskSimple> task_list;
        
        for (const auto& belief: particles_)
        {
            TruPomdpState *state = static_cast<TruPomdpState *>(belief);
            SceneGraphSimple scene_graph = state->scene_graph_;
            TaskSimple task = state->task_;
            //check whether the task already exists in the task_list
            bool task_exist = false;
            int task_index = -1;
            for (int i = 0; i < task_list.size(); i++)
            {
                if (task == task_list[i])
                {
                    task_exist = true;
                    task_index = i;
                    break;
                }
            }
            if (task_exist)
            {
                belief_list[task_index].second += state->weight;
                continue;
            }

            task_list.push_back(task);
            //get the objects of interest and target areas from the task
            vector<string> objects_of_interest_names;
            vector<string> target_area_names;
            for (int goal_index = 0; goal_index < task._goal_num; goal_index++)
            {
                array<int, 2> goal = task._area_has_obj[goal_index];
                string object_name = object_name_list_ptr_->at(goal[1]);
                string area_name = area_name_list_ptr_->at(goal[0]);
                objects_of_interest_names.push_back(object_name);
                target_area_names.push_back(area_name);
            }

            //get the objects initial area in the scene graph
            vector<string> objects_of_interest_initial_areas;
            for (int i = 0; i < objects_of_interest_names.size(); i++)
            {
                string object_name = objects_of_interest_names[i];
                string initial_area_name = scene_graph.GetObjectParentName(object_name, *object_name_list_ptr_, *area_name_list_ptr_);
                objects_of_interest_initial_areas.push_back(initial_area_name);
            }
            
            map<string, pair<string, string>> belief_combination;
            for (int i = 0; i < objects_of_interest_names.size(); i++)
            {
                belief_combination[objects_of_interest_names[i]] = make_pair(objects_of_interest_initial_areas[i], target_area_names[i]);
            }

            belief_list.push_back(make_pair(belief_combination, state->weight));            
        }

        //sort the belief_list by the weight from large to small
        sort(belief_list.begin(), belief_list.end(), [](const pair<map<string, pair<string, string>> , double>& a, const pair<map<string, pair<string, string>> , double>& b){
            return a.second > b.second;
        });
        
        cout<< "start generate belief prompt"<<endl;

        nlohmann::json belief_json_prompt;
        for (const auto& belief: belief_list)
        {
            map<string, pair<string, string>> belief_combination = belief.first;
            double belief_weight = belief.second;
            nlohmann::json belief_json;
            nlohmann::json belief_content = nlohmann::json::array();  // 这是装 object_json 的数组
            
            for (const auto& [object_name, pair]: belief_combination)
            {
                string initial_area_name = pair.first;
                string target_area_name = pair.second;
                nlohmann::json object_json;
                object_json["object_name"] = object_name;
                object_json["initial_area_name"] = initial_area_name;
                object_json["target_area_name"] = target_area_name;
                belief_content.push_back(object_json);
            }
            
            // belief_json 是一个对象，里面有两个 key：belief 和 belief_weight
            belief_json["belief"] = belief_content;
            belief_json["belief_weight"] = belief_weight;
            belief_json_prompt.push_back(belief_json);
        }
        string belief_prompt;
        belief_prompt = "```json\n" + belief_json_prompt.dump(4) + "\n```";
        return belief_prompt;
    }
} // namespace despot
#include <tru_pomdp_planner.h>

using namespace std;
using namespace despot;

/*==================================================
* TruPomdpPlanner class
==================================================*/

DSPOMDP* TruPomdpPlanner::InitializeModel(option::Option* options) {
    DSPOMDP* model = new TruPomdpTask(3);
    return model;
}

World* TruPomdpPlanner::InitializeWorld(string& world_type, DSPOMDP* model, option::Option* options, string problem_file_path, string scene_file_path)  {
    
    TruPomdpWorld* world = new TruPomdpWorld(model, 0);
    
    world->Initialize(problem_file_path, scene_file_path);        
    // TruPomdpTask* object_model = static_cast<TruPomdpTask*>(model);
    return world;
}

pair<bool, int> TruPomdpPlanner::PlanningLoop_Return(Solver *&solver, World *world, Logger *logger) {
    TruPomdpWorld* object_world = static_cast<TruPomdpWorld*>(world);
    vector<string> area_name_list = *object_world->area_name_list_ptr_;
    vector<string> object_name_list = *object_world->object_name_list_ptr_;        

    pair<bool, int>result;
    for (int i = 0; i < Globals::config.sim_len; i++)
    {
        pair<bool, bool> terminal = RunStep_return(solver, world, logger);
        if (terminal.first){
            if(!terminal.second){
                result.first = false;
                result.second = i+1;
                return result;
            } 
            result.first = true;
            result.second = i+1;
            return result;
            }
        
        double real_time = get_time_second() - EvalLog::curr_inst_start_time;
        if (real_time >= 600) {
            // cout << "Time limit reached. Exiting..." << endl;
            result.first = false;
            result.second = i+1;
            return result;
        }

    }
    result.first = false;
    result.second = Globals::config.sim_len;
    return result;
}

pair<bool, bool> TruPomdpPlanner::RunStep_return(Solver* solver, World* world, Logger* logger) {
    logger->CheckTargetTime();
    double step_start_t = get_time_second();
    double start_t = get_time_second();
    DESPOT* despot_solver = static_cast<DESPOT*>(solver);
    ACT_TYPE action = solver->Search().action;
    double end_t = get_time_second();
    double search_time = (end_t - start_t);
    // cout<<"logging level: "<<logging::level()<<endl;
    logi << "[RunStep] Time spent in " << typeid(*solver).name()
            << "::Search(): " << search_time << endl;
    cout << "[RunStep] Time spent in " << typeid(*solver).name()
            << "::Search(): " << search_time << endl;

    Globals::config.total_search_time += search_time;

    OBS_TYPE obs;

    unique_ptr<SceneGraph> obs_scene_graph;

    start_t = get_time_second();
    // bool terminal = world->ExecuteAction(action, obs);
    step_++;
    Globals::config.current_step = step_;
    //创建文件夹
    if (Globals::config.save_data){
        string step_dir = Globals::config.current_root_dir + "/step_" + to_string(step_);
        string command = "mkdir " + step_dir;
        system(command.c_str());
    }
    TruPomdpWorld* object_world = static_cast<TruPomdpWorld*>(world);
    bool terminal = object_world->ExecuteAction_save_data(action, obs, step_);
    obs_scene_graph = object_world->simulator_->GetObservation();
    // obs_scene_graph = object_world->simulator_->Copy();
    
    if (Globals::config.save_data){

        // ostringstream oss;
        // oss << *obs_scene_graph;
        // string scene_graph_json_string = oss.str();
        string scene_graph_json_string = obs_scene_graph->ToJsonString();
        string step_dir = Globals::config.current_root_dir + "/step_" + to_string(step_);
        string scene_graph_file_path = step_dir + "/scene_graph.json";
        //write the observation scene graph to json file
        ofstream scene_graph_file;
        scene_graph_file.open(scene_graph_file_path, ios::out|ios::app);
        scene_graph_file << scene_graph_json_string;
        scene_graph_file.close();
    }

    end_t = get_time_second();
    double execute_time = (end_t - start_t);
    logi << "[RunStep] Time spent in ExecuteAction(): " << execute_time << endl;
    cout << "[RunStep] Time spent in ExecuteAction(): " << execute_time << endl;

    start_t = get_time_second();
    // solver->BeliefUpdate(action, obs);
    if(!terminal){
        try{
            despot_solver->BeliefUpdate(action, move(obs_scene_graph));
        }catch(const std::exception& e){
            cout << "Caught an exception: " << e.what() << endl;
            TruPomdpWorld* object_world = static_cast<TruPomdpWorld*>(world);
            object_world->step_reward_ -= 0;
            terminal = true;
            end_t = get_time_second();
            double update_time = (end_t - start_t);
            logi << "[RunStep] Time spent in Update(): " << update_time << endl;
            cout << "[RunStep] Time spent in Update(): " << update_time << endl;

            return make_pair(logger->SummarizeStep(step_, round_, terminal, action, obs,
                step_start_t), false);
        }
        // despot_solver->BeliefUpdate(action, move(obs_scene_graph));
    }
    // solver->BeliefUpdate(action, move(obs_scene_graph));
    end_t = get_time_second();
    double update_time = (end_t - start_t);
    logi << "[RunStep] Time spent in Update(): " << update_time << endl;
    cout << "[RunStep] Time spent in Update(): " << update_time << endl;

    return make_pair(logger->SummarizeStep(step_, round_, terminal, action, obs,
                step_start_t), true);
}

int TruPomdpPlanner::RunPlanning_write_result(int argc, char *argv[], string problem_file_path, string scene_file_path, string result_file_path) {

    logging::level(3);
    if (Globals::config.save_data){
        string step_0_dir = Globals::config.current_root_dir + "/step_0";
        //创建文件夹
        string command = "mkdir " + step_0_dir;
        system(command.c_str());
    }
    /* =========================
    * initialize parameters
    * =========================*/
    string solver_type = ChooseSolver(); //"DESPOT";
    bool search_solver;
    int num_runs = 1;
    string world_type = "pomdp";
    string belief_type = "DEFAULT";
    int time_limit = -1;

    option::Option *options = InitializeParamers(argc, argv, solver_type,
            search_solver, num_runs, world_type, belief_type, time_limit);
    if(options==NULL)
        return 0;
    clock_t main_clock_start = clock();

    /* =========================
    * initialize model
    * =========================*/
    DSPOMDP *model = InitializeModel(options);
    assert(model != NULL);

    /* =========================
    * initialize world
    * =========================*/
    
    World *world = InitializeWorld(world_type, model, options, problem_file_path, scene_file_path);
    assert(world != NULL);

    /* =========================
    * initialize belief
    * =========================*/
    
    TruPomdpWorld* object_world = static_cast<TruPomdpWorld*>(world);
    

    vector<string>* area_name_list_ptr = object_world->area_name_list_ptr_;
    vector<string>* object_name_list_ptr = object_world->object_name_list_ptr_;
    SceneGraphSimple start_scene_graph;
    unique_ptr<SceneGraph> start_scene_graph_ptr = object_world->simulator_->GetObservation();

    for(const auto& area_node: start_scene_graph_ptr->all_areas()){
        string area_name = area_node.GetName();
        if (find(area_name_list_ptr->begin(), area_name_list_ptr->end(), area_name) == area_name_list_ptr->end()){
            area_name_list_ptr->push_back(area_name);
        }
    }
    for(const auto& object_node: start_scene_graph_ptr->all_objects()){
        string object_name = object_node.GetName();
        if (find(object_name_list_ptr->begin(), object_name_list_ptr->end(), object_name) == object_name_list_ptr->end()){
            object_name_list_ptr->push_back(object_name);
        }
    }

    start_scene_graph = start_scene_graph.CopyFromSceneGraph(move(start_scene_graph_ptr), *area_name_list_ptr, *object_name_list_ptr);
    string language_instruction = object_world->simulator_->GetTask().GetLanguageDescription();

    vector<string> area_names = *area_name_list_ptr;
    vector<string> object_names = *object_name_list_ptr;

    // unique_ptr<SceneGraph> scene_graph = object_world->simulator_->GetObservation();
    // string observation_prompt = generate_observation_prompt(scene_graph);
    // cout << observation_prompt << endl;

    if (Globals::config.save_data){
        string step_0_dir = Globals::config.current_root_dir + "/step_0";

        unique_ptr<SceneGraph> scene_graph = object_world->simulator_->GetObservation();
        ostringstream oss;
        oss << *scene_graph;
        string scene_graph_json_string = scene_graph->ToJsonString();
        // nlohmann::json scene_graph_json = nlohmann::json::parse(scene_graph_json_string);
        string scene_graph_file_path = step_0_dir + "/scene_graph.json";
        ofstream scene_graph_file;
        scene_graph_file.open(scene_graph_file_path, ios::app);
        scene_graph_file << scene_graph_json_string;
        scene_graph_file.close();
    }
    TruPomdpTask *object_model = static_cast<TruPomdpTask *>(model);
    vector<ActionSimple>* all_actions_ptr = object_model->all_actions_ptr_;
    unordered_map<size_t, int>* action_index_map = object_model->action_index_map_;
    Belief* belief;
    try{
        belief = object_model->InitialBelief_SceneGraph(start_scene_graph, language_instruction, area_name_list_ptr, object_name_list_ptr, all_actions_ptr);
    }catch(const std::exception& e){
        //write the results
        double real_time = get_time_second() - EvalLog::curr_inst_start_time;
        ofstream outfile;
        outfile.open(result_file_path, ios::app);
        outfile << "Is reach goal: " << 0 << " ";
        outfile << "Step Num: " << 0 << " ";
        
        outfile << "Total time: Total / Search = "
                 << real_time << " / " << Globals::config.total_search_time << "s " <<endl;
        outfile << "Total llm query calls / time = "
                << Globals::config.total_llm_calls << " / " << Globals::config.total_llm_query_time << "s " <<endl;
        outfile << "Tree of hypothesis llm query calls / time = "
                << Globals::config.tree_of_hypothesis_llm_calls << " / " << Globals::config.tree_of_hypothesis_llm_query_time << "s " <<endl;
        outfile << "Tree of hypothesis calls = " << Globals::config.tree_of_hypothesis_calls << " Input tokens = " << Globals::config.input_tokens << " Output tokens = " << Globals::config.output_tokens << endl;
        outfile << "Total discounted / undiscounted reward = "
                << 0 << " / "
                << 0
                << endl;
        outfile.close();

        if (Globals::config.save_data){
            string settings_file_path = Globals::config.current_root_dir + "/result.txt";
            ofstream settings_file;
            settings_file.open(settings_file_path, ios::app);
            settings_file << "Is reach goal: " << 0 << endl;
            settings_file << "Step Num: " << 0 << endl;
            settings_file << "Total time: Total / Search = "
                    << real_time << " / " << Globals::config.total_search_time << "s " <<endl;
            settings_file << "Total llm query calls / time = "
                    << Globals::config.total_llm_calls << " / " << Globals::config.total_llm_query_time << "s " <<endl;
            settings_file << "Tree of hypothesis llm query calls / time = "
                    << Globals::config.tree_of_hypothesis_llm_calls << " / " << Globals::config.tree_of_hypothesis_llm_query_time << "s " <<endl;
            settings_file << "Tree of hypothesis calls = " << Globals::config.tree_of_hypothesis_calls << " Input tokens = " << Globals::config.input_tokens << " Output tokens = " << Globals::config.output_tokens << endl;
            settings_file << "Total discounted / undiscounted reward = "
                    << 0 << " / "
                    << 0
                    << endl;
            settings_file.close();
        }

        return 0;
    }
    
    assert(belief != NULL);

    /* =========================
    * initialize solver
    * =========================*/
    Solver *solver = InitializeSolver(model, belief, solver_type, options);
    
    /* =========================
    * initialize logger
    * =========================*/
    Logger *logger = NULL;
    InitializeLogger(logger, options, model, belief, solver, num_runs,
            main_clock_start, object_world, world_type, time_limit, solver_type);
    //world->world_seed(world_seed);

    /* =========================
    * Display parameters
    * =========================*/
    DisplayParameters(options, model);

    /* =========================
    * run planning
    * =========================*/

    // vector<ActionSimple>* all_actions_ptr = object_model->all_actions_ptr_;

    State* start_state = object_world->GetCurrentState(all_actions_ptr, action_index_map);
    logger->InitRound(start_state);
    delete start_state;
    round_=0; step_=0;
    pair<bool, int> result;

    result = PlanningLoop_Return(solver, object_world, logger);

    logger->EndRound();

    int is_reach_goal = result.first;
    int step_num = result.second;

    PrintResult(step_num, logger, main_clock_start);
    double real_time = get_time_second() - EvalLog::curr_inst_start_time;

    //write the results
    ofstream outfile;
    outfile.open(result_file_path, ios::app);
    outfile << "Is reach goal: " << is_reach_goal << " ";
    outfile << "Step Num: " << step_num << " ";
    outfile << "Total time: Total / Search = "
                << real_time << " / " << Globals::config.total_search_time << "s " <<endl;
    outfile << "Total llm query calls / time = "
            << Globals::config.total_llm_calls << " / " << Globals::config.total_llm_query_time << "s " <<endl;
    outfile << "Tree of hypothesis llm query calls / time = "
            << Globals::config.tree_of_hypothesis_llm_calls << " / " << Globals::config.tree_of_hypothesis_llm_query_time << "s " <<endl;
    outfile << "Tree of hypothesis calls = " << Globals::config.tree_of_hypothesis_calls << " Input tokens = " << Globals::config.input_tokens << " Output tokens = " << Globals::config.output_tokens << endl;
    outfile << "Total discounted / undiscounted reward = "
            << logger->GetTotalDiscountedReward() << " / "
            << logger->GetTotalUndiscountedReward()
            << endl;
    outfile.close();

    if (Globals::config.save_data){
        string settings_file_path = Globals::config.current_root_dir + "/result.txt";
        ofstream settings_file;
        settings_file.open(settings_file_path, ios::app);
        settings_file << "Is reach goal: " << is_reach_goal << endl;
        settings_file << "Step Num: " << step_num << endl;
        settings_file << "Total time: Total / Search = "
                << real_time << " / " << Globals::config.total_search_time << "s " <<endl;
        settings_file << "Total llm query calls / time = "
                << Globals::config.total_llm_calls << " / " << Globals::config.total_llm_query_time << "s " <<endl;
        settings_file << "Tree of hypothesis llm query calls / time = "
                << Globals::config.tree_of_hypothesis_llm_calls << " / " << Globals::config.tree_of_hypothesis_llm_query_time << "s " <<endl;
        settings_file << "Tree of hypothesis calls = " << Globals::config.tree_of_hypothesis_calls << " Input tokens = " << Globals::config.input_tokens << " Output tokens = " << Globals::config.output_tokens << endl;
        settings_file << "Total discounted / undiscounted reward = "
                << logger->GetTotalDiscountedReward() << " / "
                << logger->GetTotalUndiscountedReward()
                << endl;
        settings_file.close();
    }

    return step_num;
}

void TruPomdpPlanner::PrintResult(int num_runs, Logger *logger, clock_t main_clock_start){
    cout << "Step Num: " << num_runs << endl;   
    cout << "Total llm query calls / time = "
            << Globals::config.total_llm_calls << " / " << Globals::config.total_llm_query_time << "s " <<endl;
    cout << "Tree of hypothesis llm query calls / time = "
            << Globals::config.tree_of_hypothesis_llm_calls << " / " << Globals::config.tree_of_hypothesis_llm_query_time << "s " <<endl;
    cout << "Total discounted / undiscounted reward = "
            << endl;
}


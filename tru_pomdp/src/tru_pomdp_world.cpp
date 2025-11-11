#include <tru_pomdp_world.h>

using namespace despot;

bool TruPomdpWorld::Connect()
{
    return true;
}

State *TruPomdpWorld::Initialize()
{
    return nullptr;
}

State *TruPomdpWorld::Initialize(const std::string problem_file_path, const std::string scene_file_path)
{
    std::ifstream file(problem_file_path);
    std::string problem_str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    std::ifstream file2(scene_file_path);
    std::string scene_str((std::istreambuf_iterator<char>(file2)), std::istreambuf_iterator<char>());

    simulator_ = std::make_unique<Simulator>(scene_str, problem_str);

    unique_ptr<SceneGraph> scene_graph = simulator_->Copy();
    vector<AreaNode> all_areas = scene_graph->all_areas();
    vector<string> areas;
    for (const auto& area : all_areas) {
        areas.push_back(area.GetName());
    }
    Task task = simulator_->GetTask();
    vector<string> objects;

    for(const auto& goal: task.GetListSpecification()) {
        objects.push_back(goal.second);
    }
    vector<ObjectNode> all_objects = scene_graph->all_objects();
    for (const auto& object : all_objects) {
        if (find(objects.begin(), objects.end(), object.GetName()) == objects.end()) {
            objects.push_back(object.GetName());
        }
    }

    //generate all legal actions
    vector<Action> task_all_actions = generate_task_all_actions(areas, objects);
    task_all_actions_ = task_all_actions;
    //get the pointer of all_actions
    vector<Action>* task_all_actions_ptr = &task_all_actions_;
    task_all_actions_ptr_ = task_all_actions_ptr;

    std::unique_ptr<SceneGraph> observation = simulator_->GetObservation();
    for (const auto& area : observation->all_areas()) {
        area_name_list_ptr_->push_back(area.GetName());
    }
    for (const auto& object : observation->all_objects()) {
        object_name_list_ptr_->push_back(object.GetName());
    }
    // cout<<"complete InitializeWorld"<<endl;
    // return GetCurrentState();
    return nullptr;
}


State *TruPomdpWorld::GetCurrentState(vector<ActionSimple> *all_actions_ptr, unordered_map<size_t, int> *action_index_map)
{
    // cout<<"start GetCurrentState"<<endl;
    std::unique_ptr<SceneGraph> observation = simulator_->GetObservation();
    SceneGraphSimple scene_graph_simple;
    scene_graph_simple = scene_graph_simple.CopyFromSceneGraph(move(observation), *area_name_list_ptr_, *object_name_list_ptr_);

    TaskSimple task;
    return new TruPomdpState(scene_graph_simple, task, all_actions_ptr, action_index_map);
    // cout<<"complete GetCurrentState"<<endl;
}

bool TruPomdpWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE &obs)
{
    return false;
}

bool TruPomdpWorld::ExecuteAction_save_data(ACT_TYPE action, OBS_TYPE &obs, int step)
{   
    
    TruPomdpTask* object_model = static_cast<TruPomdpTask *>(model_);

    vector<ActionSimple> all_actions = *object_model->all_actions_ptr_;
    ActionSimple true_action_simple = all_actions[action];

    Action true_action = true_action_simple.ToAction(*area_name_list_ptr_, *object_name_list_ptr_);
    cout << "---------------------------------------------------------" << endl;
    cout << "action_simple: " << true_action_simple << endl;
    cout << "action: " << true_action << endl;

    string modified_action = modify_place_action(Globals::config.action_sequence, true_action.ToString());


    Globals::config.action_sequence += modified_action+", ";
    // cout<<"action sequence: "<<Globals::config.action_sequence<<endl;
    

    //if "PICK" in action_str, then check if the object is reachable 
    if (true_action.Type == ActionType::ACTION_PICK)

    {
        for (const auto & task_action: task_all_actions_){
            if(task_action.Type == ActionType::ACTION_PICK){
                string clean_true_object_name = get_clean_object_name(true_action.Args[1]);
                string clean_task_object_name = get_clean_object_name(task_action.Args[1]);

                string true_area_name = true_action.Args[0];
                string task_area_name = task_action.Args[0];
                if(clean_true_object_name == clean_task_object_name && true_area_name == task_area_name){
                    true_action = task_action;
                    ActionResult action_result = simulator_->TakeAction(true_action);
                    if (Globals::config.save_data){
                        string step_dir = Globals::config.current_root_dir + "/step_" + std::to_string(step);
                        string action_file_path = step_dir + "/action.txt";
                        std::ofstream action_file;
                        action_file.open(action_file_path, std::ios::app);
                        action_file << "action: " << true_action << endl;
                        //action result
                        action_file << "action success: " << action_result._is_success << endl;
                        action_file << "action feedback: " << action_result._feedback_message << endl;
                        action_file << "action step cost: " << action_result._step_cost << endl;
                        action_file.close();
                    }

                    if (action_result._is_success){
                        bool goal_test = simulator_->GoalTest();
                        unique_ptr<SceneGraph> observation = simulator_->GetObservation();
                        SceneGraphSimple secne_graph_simple;
                        secne_graph_simple.CopyFromSceneGraph(move(observation), *area_name_list_ptr_, *object_name_list_ptr_);
                        // encode_observation(move(observation), obs);
                        encode_observation(secne_graph_simple, obs);
                        //show object  in the robot's hand
                        observation = simulator_->GetObservation();
                        const Robot* robot = observation->GetRobot();
                        const ObjectNode* object_in_hand = observation->GetObjectInHand();
                        // if (object_in_hand != nullptr){
                        cout << "object in hand: " << object_in_hand->GetName() << endl;
                        // }

                        //modify step_reward_ in its parent class
                        // step_reward_
                        // bool goal_test = simulator_->GoalTest();
                        step_reward_ = simulator_->GoalTest() * 100.0 -(action_result._step_cost);
                        return goal_test;
                    }
                }
            }
        }
    }


    ActionResult action_result = simulator_->TakeAction(true_action);

    if (true_action.Type == ActionType::ACTION_PLACE){
        Task task = simulator_->GetTask();
        vector<string> object_in_goal = simulator_->ObjectsReachGoal();
        Globals::config.objects_in_goal_list = object_in_goal;

        Globals::config.objects_in_goal = "Objects already in target areas: ";
        for (const auto& object: object_in_goal){
            string clean_object_name = get_clean_object_name(object);

            Globals::config.objects_in_goal += clean_object_name + ", ";
        }
    }
    cout<<Globals::config.objects_in_goal<<endl;

    if (Globals::config.save_data){
        string step_dir = Globals::config.current_root_dir + "/step_" + std::to_string(step);
        string action_file_path = step_dir + "/action.txt";
        std::ofstream action_file;
        action_file.open(action_file_path, std::ios::app);
        action_file << "action: " << true_action << endl;
        //action result
        action_file << "action success: " << action_result._is_success << endl;
        action_file << "action feedback: " << action_result._feedback_message << endl;
        action_file << "action step cost: " << action_result._step_cost << endl;
        action_file.close();
    }
    if (!action_result._is_success)
    {
        unique_ptr<SceneGraph> observation = simulator_->GetObservation();
        SceneGraphSimple secne_graph_simple;
        secne_graph_simple.CopyFromSceneGraph(move(observation), *area_name_list_ptr_, *object_name_list_ptr_);
        encode_observation(secne_graph_simple, obs);
        // encode_observation(move(observation), obs);
        step_reward_ = -20;
        cout << "action failed" << endl;
        cout << "failed reason: " << action_result._feedback_message << endl;
        return false;
    }

    bool goal_test = simulator_->GoalTest();
    unique_ptr<SceneGraph> observation = simulator_->GetObservation();

    for (const auto& object: observation->all_objects()){
        string object_name = object.GetName();
        //if object name isn't in object_names_ptr_, add it
        if (find(object_name_list_ptr_->begin(), object_name_list_ptr_->end(), object_name) == object_name_list_ptr_->end()){
            // object_names.push_back(object_name);
            object_name_list_ptr_->push_back(object_name);
        }
    }
    SceneGraphSimple secne_graph_simple;
    secne_graph_simple.CopyFromSceneGraph(move(observation), *area_name_list_ptr_, *object_name_list_ptr_);
    encode_observation(secne_graph_simple, obs);
    step_reward_ = goal_test * 500.0 -(action_result._step_cost);
    return goal_test;
}
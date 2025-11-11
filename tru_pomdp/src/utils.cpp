#include <utils.h>

Action get_true_action(const string& action_str, const unique_ptr<SceneGraph>& scene_graph) {
    // Get the true action from the action string
    regex re(R"(\((\w+) ([\w\s-]+)\))");
    smatch match;
    if (regex_match(action_str, match, re)) {
        string action_type = match[1];
        string args_str = match[2];
        vector<string> args;
        regex re_args(R"(\w+)");
        sregex_iterator it(args_str.begin(), args_str.end(), re_args);
        sregex_iterator end;
        for (; it != end; ++it) {
            args.push_back(it->str());
        }
        if (action_type == "PLACE") {
            return Action(ActionType::ACTION_PLACE, args);
        } else if (action_type == "OPEN") {
            return Action(ActionType::ACTION_OPEN, args);
        }
    }
    //pick action need different matching
    //example: (PICK storage_and_inventory_station_cabinet boxed-drink)
    // cout<<"action_str: "<<action_str<<endl;
    regex re_pick(R"(\((\w+) ([\w\s-]+) ([\w\s-]+)\))");
    smatch match_pick;
    if (regex_match(action_str, match_pick, re_pick)) {
        string action_type = match_pick[1];
        // string args_str = match_pick[2] + " " + match_pick[3];
        vector<string> args;
        args.push_back(match_pick[2]);
        args.push_back(match_pick[3]);
        if (action_type == "PICK") {
            return Action(ActionType::ACTION_PICK, args);
        }
    }

    return Action(ActionType::ACTION_NONE, {});
}

void generate_all_actions(const vector<string> &area_name_list, vector<string> &object_name_list, vector<ActionSimple> *all_actions_ptr, unordered_map<size_t, int> *action_index_map){
    // vector<ActionSimple> all_actions;
    //clear all_actions_ptr and action_index_map
    all_actions_ptr->clear();
    action_index_map->clear();
    int none_action_args[3] = {-1, -1, ActionType::ACTION_NONE};
    ActionSimple none_action(none_action_args);
    all_actions_ptr->emplace_back(none_action);
    int key_none = hash<ActionSimple>{}(none_action);
    action_index_map->insert({key_none, 0});

    for (int i = 0; i < area_name_list.size(); i++) {
        for (int j = 0; j < object_name_list.size(); j++) {
            int args[3] = {i, j, ActionType::ACTION_PICK};
            // all_actions.emplace_back(args);
            all_actions_ptr->emplace_back(args);
            size_t key = hash<ActionSimple>{}(ActionSimple(args));
            action_index_map->insert({key, all_actions_ptr->size() - 1});
        }
        //open
        int args_open[3] = {i, -1, ActionType::ACTION_OPEN};
        // all_actions.emplace_back(args_open);
        all_actions_ptr->emplace_back(args_open);
        size_t key_open = hash<ActionSimple>{}(ActionSimple(args_open));
        action_index_map->insert({key_open, all_actions_ptr->size() - 1});
        //place
        int args_place[3] = {i, -1, ActionType::ACTION_PLACE};
        // all_actions.emplace_back(args_place);
        all_actions_ptr->emplace_back(args_place);
        size_t key_place = hash<ActionSimple>{}(ActionSimple(args_place));
        action_index_map->insert({key_place, all_actions_ptr->size() - 1});
    }
    vector<ActionSimple> all_actions = *all_actions_ptr;
    unordered_map<size_t, int> action_map = *action_index_map;
    return;
}

void get_target_object_related_actions(unique_ptr<array<ActionSimple, 200>> &actions, int &action_num, const vector<int> &target_object_names) {
    // 使用 unordered_set 来快速查找目标对象
    unordered_set<int> target_object_set(target_object_names.begin(), target_object_names.end());
    int new_index = 0;  // 用于指示新的数组索引位置
    // ActionSimple action;
    for (int i = 0; i < action_num; i++)
    {
        // action = (*actions)[i];
        
        // 只处理 ACTION_PICK 类型的动作
        if ((*actions)[i].Type() == ActionType::ACTION_PICK) {
            if (target_object_set.find((*actions)[i].Args[1]) == target_object_set.end()) {
                // 该动作不需要保留，跳过
                continue;
            }
        }
        
        // 将需要保留的动作放入新位置
        if (i != new_index) {
            (*actions)[new_index] = (*actions)[i];
        }
        new_index++;
    }
    // 更新动作数量
    action_num = new_index;
    if (action_num <= 1){
        cerr<<"no legal actions"<<endl;
    }
}

vector<Action> generate_task_all_actions(const vector<string>& area_name_list, vector<string>& object_name_list) {

    vector<Action> all_actions;
    for (const auto& object_name : object_name_list) {
        for (const auto& area_name : area_name_list) {
            all_actions.push_back(Action(ActionType::ACTION_PICK, {area_name, object_name}));
        }
    }
    for (const auto& area_name : area_name_list) {
        all_actions.push_back(Action(ActionType::ACTION_PLACE, {area_name}));
    }
    for (const auto& area_name : area_name_list) {
        all_actions.push_back(Action(ActionType::ACTION_OPEN, {area_name}));
    }
    return all_actions;
}

string generate_observation_prompt(SceneGraphSimple scene_graph, const vector<string> area_names, const vector<string> object_names){
    string observation_prompt = "";
    //closed areas and open areas
    // vector<AreaNode> area_nodes = scene_graph->all_areas();

    vector<string> closed_area_names = {};
    vector<string> open_area_names = {};
    for (const auto& area_name : scene_graph.all_areas(area_names)){
        bool is_open = scene_graph.GetAreaOpenFromName(area_name, area_names);
        if (is_open){
            open_area_names.push_back(area_name);
        }
        else{
            closed_area_names.push_back(area_name);
        }
    }
    //all observed objects and their parent areas
    map<string, string> observed_objects = {};
    for (const auto& object_name : scene_graph.all_objects(object_names)){
        const string parent_area_name = scene_graph.GetObjectParentName(object_name, object_names, area_names);
        if (parent_area_name == "robot"){
            observed_objects[object_name] = "robot's hand";
            //in the prompt, need to tell llm that if the object is in the robot's hand, return robot
        }
        observed_objects[object_name] = parent_area_name;
    }

    //generate observation prompt
    observation_prompt += "The closed areas are: ";
    for (const auto& area_name : closed_area_names){
        observation_prompt += area_name + ", ";
    }
    observation_prompt += "\n";
    observation_prompt += "The open areas are: ";
    for (const auto& area_name : open_area_names){
        observation_prompt += area_name + ", ";
    }
    observation_prompt += "\n";
    observation_prompt += "The observed objects and their initial areas are: ";
    for (auto& [object_name, area_name] : observed_objects){
        //如果areaname中有surface，则用on，其他用in
        // string object_name_str = object_name.substr(0, object_name.find_last_of("_"));
        string object_name_str = get_clean_object_name(object_name);

        if (area_name.find("surface") != string::npos){
            observation_prompt += object_name_str + " is on " + area_name + ", ";
        }
        else if(area_name.find("robot") != string::npos){
            observation_prompt += object_name_str + " is in  robot's hand, ";
        }
        else{
            observation_prompt += object_name_str + " is in " + area_name + ", ";
        }
    }
    observation_prompt += "\n";
    return observation_prompt;
}

nlohmann::json pare_llm_answer_to_json(std::string llm_answer) {
    // cout<<llm_answer<<endl;
    // 找到 JSON 部分的开始和结束
    size_t json_start = llm_answer.find("```json");
    size_t json_end = llm_answer.find_last_of("```");

    if (json_start == std::string::npos || json_end == std::string::npos || json_start >= json_end) {
        cout<<"Invalid JSON format in the answer."<<endl;
        throw std::runtime_error("Invalid JSON format in the answer.");
    }

    // 截取 JSON 字符串
    std::string json_str = llm_answer.substr(json_start + 8, json_end - json_start - 8);

    // 输出查看截取的 JSON 字符串
    // std::cout << "Original JSON part: [" << json_str << "]" << std::endl;

    // 去除字符串两端的空格
    json_str = json_str.substr(json_str.find_first_not_of(" \t\r\n"), 
                               json_str.find_last_not_of(" \t\r\n") - json_str.find_first_not_of(" \t\r\n") + 1);

    // 输出清理后的 JSON 字符串
    // std::cout << "Trimmed JSON part: [" << json_str << "]" << std::endl;

    // 自动修正 JSON 字符串：
    // 1. 将单引号替换为双引号
    std::replace(json_str.begin(), json_str.end(), '\'', '\"');

    // 2. 删除所有非法字符（例如注释、非法换行符等）
    json_str = std::regex_replace(json_str, std::regex(R"(//.*)"), "");  // 删除注释
    json_str = std::regex_replace(json_str, std::regex(R"(\n|\r)"), ""); // 删除换行符
    json_str = std::regex_replace(json_str, std::regex("`"), ""); // 多余的反引号

    // 3. 处理转义字符
    // 这一步一般情况下不需要，但如果输入中有未正确转义的特殊字符，可以加一个步骤
    json_str = std::regex_replace(json_str, std::regex(R"(\\)"), "\\\\"); // 处理反斜杠

    // 2. 修复不正确的 JSON 格式：
    //     a. 修复对象外面错误的方括号 []
    if (json_str.front() == '[' && json_str.back() == ']') {
        json_str = json_str.substr(1, json_str.size() - 2);
    }

    // 输出修正后的 JSON 字符串
    // std::cout << "Corrected JSON part: " << json_str  << std::endl;

    // 解析 JSON 字符串
    nlohmann::json json_obj = nlohmann::json::parse(json_str);
    // cout<<"complete parse json"<<endl;
    return json_obj;
}

string get_clean_object_name(const string &object_name){
    //check if the last character is a number
    if (isdigit(object_name.back())){
        string new_object_name = object_name.substr(0, object_name.find_last_of("_"));
        return new_object_name;
    }
    return object_name;
}

string generate_wrong_tasks_prompt(vector<TaskSimple> wrong_tasks, const vector<string> area_names, const vector<string> object_names){
    string wrong_tasks_prompt = "";
    // cout<<"wrong tasks size: "<<wrong_tasks.size();
    for (int i = 0; i < wrong_tasks.size(); i++){
        TaskSimple wrong_task = wrong_tasks[i];
        wrong_tasks_prompt += to_string(i+1);
        wrong_tasks_prompt += ". ";
        for (int j = 0; j < wrong_task._goal_num; j++){
            array<int, 2> goal = wrong_task._area_has_obj[j];
            string object_name = object_names[goal[1]];
            object_name = get_clean_object_name(object_name);
            string area_name = area_names[goal[0]];
            wrong_tasks_prompt += object_name + " in " + area_name + ", ";
        }
        //change the last , to .
        wrong_tasks_prompt = wrong_tasks_prompt.substr(0, wrong_tasks_prompt.size() - 2) + ".\n";
    }
    // cout<<"wrong_tasks_prompt: "<<wrong_tasks_prompt<<endl;
    return wrong_tasks_prompt;
}

string modify_place_action(string action_sequence, string current_action){

    //find Place in current_action，if no, return current_action
    size_t place_pos = current_action.find("Place");
    if (place_pos == string::npos){
        return current_action;
    }

    //find the last "Pick" in the action_sequence
    size_t last_pick_pos = action_sequence.find_last_of("Pick");
    //Pick(<area>, <object>, )， get the object
    string pick_action = action_sequence.substr(last_pick_pos, action_sequence.find(")", last_pick_pos) - last_pick_pos + 1);
    //get the object name
    string object_name = pick_action.substr(pick_action.find(",") + 1, pick_action.find(")") - pick_action.find(",") - 1);

    //Place(<area>, )->Place(<area>, <object>)
    string place_area = current_action.substr(place_pos + 6, current_action.find(",", place_pos) - place_pos - 6);
    string new_place_action = "Place(" + place_area + ", " + object_name + ", )";
    return new_place_action;
}

namespace despot{

    void get_object_matching_pairs_from_tree_of_hypothesis(vector<pair<string, string>>& object_matching_pairs, vector<pair<string, string>>& current_object_matching_pairs,
        // const std::vector<std::pair<std::map<std::string, std::pair<std::string, std::string>>, double>>& tree_of_hypothesis,
        const vector<vector<string>>& object_list_from_tree_of_hypothesis,
        const vector<string> &all_objects ){
            
        vector<string> unmatched_objects_in_current_state;
        string unmatched_objects_string = "(";

        vector<string> observed_objects;
        string observed_objects_string = "(";     

        //find the unmatched objects in all observations
        for (auto &combination : object_list_from_tree_of_hypothesis)
        {
            for (const auto &goal: combination)
            {
                string object_name = get_clean_object_name(goal);
                // string initial_area = goal.second.first;
                // string target_area = goal.second.second;
                
                bool object_in_unmatched_objects = false;
                //step 1: check if the object has been checked
                for (const auto &unmatched_object : unmatched_objects_in_current_state)
                {
                    if (unmatched_object == object_name)
                    {
                        object_in_unmatched_objects = true;
                        break;
                    }
                }

                for (const auto &matched_object_pair : object_matching_pairs ){
                    if (matched_object_pair.
                    first == object_name){
                        break;
                    }
                }
                //step 2: check if the object is in current observation
                bool object_in_observation = false;
                
                for (const auto& obs_object: all_objects)
                {
                    string obs_object_name = get_clean_object_name(obs_object);
                    if (object_name == obs_object_name)
                    {
                        object_in_observation = true;
                        if(find(current_object_matching_pairs.begin(), current_object_matching_pairs.end(), make_pair(object_name, obs_object_name)) == current_object_matching_pairs.end())
                        {
                            current_object_matching_pairs.push_back(make_pair(object_name, obs_object_name));
                            object_matching_pairs.push_back(make_pair(object_name, obs_object_name));
                        }
                        break;
                    }
                }
                if(!object_in_unmatched_objects && !object_in_observation)
                {
                    unmatched_objects_in_current_state.push_back(object_name);
                    unmatched_objects_string += object_name + " ";
                }
            }
        }
    }

    void generate_particles(vector<State*>& particles, 
        const vector<pair<map<string, pair<string, string>>, double>>& tree_of_hypothesis, 
        const SceneGraphSimple& start_scene_graph, 
        vector<string>* area_name_list_ptr, 
        vector<string>* object_name_list_ptr,
        vector<ActionSimple>* all_actions_ptr,
        unordered_map<size_t, int>* action_index_map_,
        const vector<pair<string, string>>& current_object_matching_pairs,
        int& unique_belief_num,
        vector<int>& unique_belief_list,
        const TruPomdpTask* task_ptr,
        const vector<TaskSimple> &wrong_tasks_
        ){
            
        for (const auto &[combination, prob] : tree_of_hypothesis){
            // unique_ptr<SceneGraph> scene_graph = scene_graph_copy->Copy();
            SceneGraphSimple scene_graph = start_scene_graph.Copy();
            TaskSimple* current_task = new TaskSimple();
            bool wrong_combination = false;
            for(const auto& goal: combination){
                string object_name = goal.first;
                string initial_area_name = goal.second.first;
                string target_area_name = goal.second.second;
                //find the objects' name in despot::Globals::config.objects_in_goal_list
                bool object_in_goal_list = false;
                for (const auto& object_in_goal: despot::Globals::config.objects_in_goal_list)
                {
                    if (get_clean_object_name(object_name) == get_clean_object_name(object_in_goal))
                    {
                        object_in_goal_list = true;
                        break;
                    }
                }
                if (object_in_goal_list)
                {
                    continue;
                }

                //check if initial area is in the observation
                if (initial_area_name != "robot")
                {
                    // const AreaNode* initial_area_node = scene_graph->GetArea(initial_area_name);
                    if (find(area_name_list_ptr->begin(), area_name_list_ptr->end(), initial_area_name) == area_name_list_ptr->end())
                    {
                        // wrong_combination = true;
                        // break;
                        continue;
                    }
                }
                //check if target area is in the observation
                if (find(area_name_list_ptr->begin(), area_name_list_ptr->end(), target_area_name) == area_name_list_ptr->end())
                {
                    // wrong_combination = true;
                    // break;
                    continue;
                }
                bool object_has_pair = false;
                for (const auto& matching_pair: current_object_matching_pairs)
                {   string matching_object_name = "";
                    if (matching_pair.first == get_clean_object_name(object_name))
                    {
                        matching_object_name = matching_pair.second;
                    }
                    if (matching_object_name != "")
                    {
                        if(matching_object_name == "None")
                        {
                            break;
                        }
                        if (initial_area_name == "robot")
                        {
                            string true_object_name = scene_graph.GetObjectInHandName(*object_name_list_ptr);
                            string true_object_name_adjust = get_clean_object_name(true_object_name);
                            if (true_object_name_adjust == matching_object_name)
                            {
                                object_name = true_object_name;
                                object_has_pair = true;
                                break;
                            }
                            else{
                                wrong_combination = true;
                                break;
                            }
                        }
                        else
                        {
                            for (const auto& object_node: scene_graph.GetObjectsInArea(initial_area_name, *object_name_list_ptr, *area_name_list_ptr))
                            {
                                string true_object_name = object_node;
                                string true_object_name_adjust = get_clean_object_name(true_object_name);
                                if (true_object_name_adjust == matching_object_name)
                                {
                                    object_name = true_object_name;
                                    object_has_pair = true;
                                    break;
                                }
                            }
                            if(!object_has_pair){
                                wrong_combination = true;
                                break;
                            }
                        }
                    }
                }
                if (!object_has_pair)
                {
                    object_name = get_clean_object_name(object_name);
                    if(initial_area_name == "robot")
                    {
                        // wrong_combination = true;
                        // break;
                        continue;
                    }
                    else
                    {
                        if(scene_graph.GetAreaOpenFromName(initial_area_name, *area_name_list_ptr)){
                            // wrong_combination = true;
                            // break;
                            continue;
                        }
                        if(find(object_name_list_ptr->begin(), object_name_list_ptr->end(), object_name) == object_name_list_ptr->end())
                        {
                            object_name_list_ptr->push_back(object_name);
                        }
                        scene_graph.AddObjectInAreaFromName(initial_area_name, object_name, *area_name_list_ptr, *object_name_list_ptr);
                    }
                }
                current_task->AddGoalFromName(target_area_name, object_name, *area_name_list_ptr, *object_name_list_ptr); 

            }
            //check the lenghth of the current_task
            
            if (current_task->_goal_num == 0)
            {
                delete current_task;
                continue;
            }

            if(wrong_combination){
                continue;
            }

            if(scene_graph.GoalTest(*current_task)){
                continue;
            }

            //check if the current_task is in the wrong_tasks
            bool in_wrong_tasks = false;
            for (const auto& wrong_task: wrong_tasks_)
            {
                if (*current_task == wrong_task) 
                {
                    in_wrong_tasks = true;
                    break;
                }
            }
            if (in_wrong_tasks)
            {
                continue;
            }

            TruPomdpState* object_state = static_cast<TruPomdpState*>(task_ptr->Allocate(-1, 1.0));           
            object_state->scene_graph_ = scene_graph;
            object_state->task_ = *current_task;
            object_state->all_actions_ptr_ = all_actions_ptr;
            object_state->action_index_map_ = action_index_map_;
            object_state->weight = prob;
            object_state->state_id = unique_belief_num;
            object_state->scenario_id = 0;
            object_state->UpdateActions();

            particles.push_back(object_state);
            unique_belief_list.push_back(unique_belief_num);
            unique_belief_num += 1;
            object_state->SetAllocated();

            delete current_task;
        }
    }       


} // namespace despot


// 回调函数,用于接收服务器的响应数据
size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    ((string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

// 发起API请求
string call_llm_api(const string& url, const string& api_key, const nlohmann::json& root) {
    CURL* curl;
    CURLcode res;
    const int max_retries = 5;  // 最大重试次数
    int retries = 0;
    string read_buffer;

    // 将 JSON 对象转换为字符串
    string json_data;
    try {
        if (root.is_null()) {
            throw invalid_argument("JSON root is null!");
        }
        json_data = root.dump();  // 转换为 JSON 字符串
    } catch (const exception& e) {
        cerr << "Error serializing JSON: " << e.what() << endl;
        json_data = "{}";  // 使用一个空的 JSON 对象作为 fallback
    }

    while (retries < max_retries) {
        // 初始化 cURL
        curl_global_init(CURL_GLOBAL_DEFAULT);
        curl = curl_easy_init();

        if (curl) {
            // 设置 cURL 请求头
            struct curl_slist* headers = NULL;
            headers = curl_slist_append(headers, "Content-Type: application/json");
            //if find azure in url
            if (url.find("deployments") != string::npos) {
                headers = curl_slist_append(headers, ("api-key: " + api_key).c_str());
            } else {
                headers = curl_slist_append(headers, ("Authorization: Bearer " + api_key).c_str());
            }
            // headers = curl_slist_append(headers, ("Authorization: Bearer " + api_key).c_str());

            // 设置请求的 URL 和数据
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_data.c_str());

            // 用来保存响应的变量
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &read_buffer);

            // curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L); // 打开详细的 cURL 日志

            // 发起请求并获取响应
            res = curl_easy_perform(curl);

            // 如果请求成功,退出重试循环
            if (res == CURLE_OK) {
                curl_easy_cleanup(curl);
                curl_slist_free_all(headers);
                curl_global_cleanup();
                return read_buffer;  // 返回 API 响应内容
            } else {
                // 如果发生错误,记录错误并进行重试
                cerr << "Request failed (Attempt " << retries + 1 << "): "
                          << curl_easy_strerror(res) << endl;

                // 增加重试次数并等待
                retries++;
                if (retries < max_retries) {
                    cerr << "Retrying in 2 seconds..." << endl;
                    this_thread::sleep_for(chrono::seconds(2));  // 等待 2 秒
                }
            }

            // 清理 cURL
            curl_easy_cleanup(curl);
            curl_slist_free_all(headers);
        }

        curl_global_cleanup();
    }

    // 如果超过最大重试次数仍然失败,退出并输出错误
    cerr << "Error: Reached maximum retry attempts. Exiting..." << endl;
    return "";  // 返回空字符串表示失败
}

bool murmurHash64A(const std::string &key, uint64_t &hash) {
    if (key.empty()) {
        // 如果输入的字符串为空，返回失败
        return false;
    }

    const uint64_t c1 = 0x87c37b91114253d5;
    const uint64_t c2 = 0x4cf5ad432745937f;

    hash = 0;
    uint64_t k = 0;

    // Process each 8-byte chunk of the string
    size_t len = key.length();
    size_t i = 0;

    while (i + 8 <= len) {
        k = *(reinterpret_cast<const uint64_t*>(key.data() + i));
        i += 8;

        k *= c1;
        k = (k << 31) | (k >> (64 - 31));
        k *= c2;

        hash ^= k;
        hash = (hash << 27) | (hash >> (64 - 27));
        hash = hash * 5 + 0x52dce729;
    }

    // Handle the remaining bytes (if any)
    k = 0;
    size_t remaining = len - i;
    if (remaining > 0) {
        for (size_t j = 0; j < remaining; ++j) {
            k ^= (uint64_t)(key[i + j]) << (j * 8);
        }

        k *= c1;
        k = (k << 31) | (k >> (64 - 31));
        k *= c2;

        hash ^= k;
    }

    // Finalize the hash
    hash ^= len;
    hash = hash ^ (hash >> 33);
    hash = hash * 0xff51afd7ed558ccd;
    hash = hash ^ (hash >> 33);
    hash = hash * 0xc4ceb9fe1a85ec53;
    hash = hash ^ (hash >> 33);

    return true; // Hashing successful
}


bool encode_observation(SceneGraphSimple scene_graph, uint64_t &obs){
    const string obs_str = scene_graph.GetObsStr();

    return murmurHash64A(obs_str, obs);
}



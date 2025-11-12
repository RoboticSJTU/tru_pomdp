#include <tree_of_hypothesis.h>
#include <despot/core/globals.h>

#include <cstdlib>

using namespace std;

TreeOfHypothesis::TreeOfHypothesis(const string &model, const string &api_key, const string &base_url, int k) {
    llm_model_ = model;
    api_key_ = api_key;
    base_url_ = base_url;
    k_ = k;

    prompt_tokens = 0;
    completion_tokens = 0;


    //change to relative path
    string root_path = getenv("ROOT_PATH");
    string all_objects_file = root_path + "/all_obj_types.txt";
    string all_areas_file = root_path + "/all_areas/" + "/all_areas_" + despot::Globals::config.scene_graph + ".txt";

    bool cot = despot::Globals::config.cot;

    bool objects_of_interest_cot = true;
    bool initial_areas_cot = despot::Globals::config.cot;
    bool target_areas_cot = despot::Globals::config.cot;


    if(initial_areas_cot){
        initial_areas_system_prompt =
            "# Role \n"
            "You are an expert assistant specialized in object relocation within household kitchens. \n"
            "You'll receive: \n"
            "   1. the language instruction of the task.\n"
            "   2. the description of current observation of the environment, listing all open areas, closed areas and observed objects with their placements. If the object is inside a closed area, it is not visible.\n"
            "   3. the name of the object of interest you should now focus on.\n"
            "# Task \n"
            "You need to identify up to " + to_string(k_) + " most probable initial_areas for every missing object and their probability. \n\n"
            
            "# Guidelines \n"
            "## For the Initial_areas"
            "1. The object's placement is consistent with common sense.  \n"
            "2. The initial_areas can only be selected from the areas explicitly mentioned in the observation. \n"

            "## For the final answer: \n"
            "1. You **must** give out your reasoning process first. Then, you **must** give your final answer in json format same as the example json answer. \n\n"            "Now, carefully read the following requirements, then step by step give your reasoning, and finally, generate your answer in JSON format.\n"
            
            "# Requirements: Your reasoning process should include the following steps. For each step, you should explicitly give out your reasoning and the phased results. \n"
            "## **Step 1: Object Visibility Check** \n"
            "   - Check whether the **current object of interest** is **visible in the observation**. \n"
            "   - If the object is visible, set the probability of the object placed in the area to 1.0, and jump to step 4 and give out the final answer. \n"
            "   - If the objects current area is robot's hand, the selected area should be 'robot'. \n"
            "## **Step 2: Object initial_area Guess** \n"
            "   - List all the **closed areas in the observation**. \n"
            "   - Reason/Guess the up to " + to_string(k_) +  " possible initial_areas for the the **current object of interest** from the **closed areas in the observation** and corresponding probability using common sense.\n"
            "## **Step 3: Object Initial_Area Double Check** \n"
            "   - Double check the initial_areas you proposed: \n"
            "       1. **closed areas in the observation**. \n"
            "       2. The probability sum for all candidate areas must sum to 1.0 for each object. \n"
            "   - Return to step 2 if the double check fails. \n"
            "## **Step 4: Final Answer** \n"
            "   - Give out your final JSON answer in the same format of Example Json Answer. \n\n";

        initial_areas_example_prompt =
            // "# Example"
            "       - Example Json Answer: \n"
            "```json\n"
            "{\n"
            "    \"answer\": [\n"
            "        {\n"
            "            \"initial_area\": \"storage_and_inventory_station_surface\",\n"
            "            \"probability\": 0.7\n"
            "        },\n"
            "        {\n"
            "            \"initial_area\": \"cooler_layer\",\n"
            "            \"probability\": 0.3\n"
            "        }\n"
            "    ]\n"
            "}\n"
            "```";

    }
    else{
        initial_areas_system_prompt =
            "You are an assistant to solve an object rearrangement task in a household kitchen environment. \n"
            "You'll receive the description of current observation of the environment, and the current object of interest's name. \n"
            "The current observation contains the closed areas, open areas, and the observed objects.\n"
            "The description of current observation only contains the objects that are visible to the robot. If the object is placed in a closed area, the robot cannot see it. \n"
            "You need to select up to " + to_string(k_) + " most possible initial placing areas for the object, and the probability for object placed in this area. The object's placement is consistent with common sense.  \n"
            "If the object is seen in the observation, the probability of the object placed in the area is 1.0. \n"
            "If you cannot see the object in the observation, you shouldn't consider the current open areas and robot as the possible initial area of this object. You should only consider the closed areas.\n"
            "The name of the object may be ambiguous, you need to determine whether the current object of interest is in observed objects. \n"
            "If an area is open and the object is not seen in this area in the observation, the probability of the object placed in this area is 0.0, and you shouldn't choose it as the possible initial area of this object. This means that if the object isn't in the observed object, you should choose its initial area from closed areas.\n"
            "If you believe that the object is in robot's hand, the selected area should be 'robot'.\n"
            "If you are very confident with your answer, you can give less than " + to_string(k_) + " answers. \n"
            "You should generate at least one possible initial area for the object. \n";


        initial_areas_example_prompt =
            "Give your json answer between ```json\\n and ```\\n. \n"
            "You should only give the answer json file in the following format, and don't generate anything else: \n"
            "Example json answer: \n"
            "```json\n"
            "{\n"
            "    \"answer\": [\n"
            "        {\n"
            "            \"initial area\": \"storage_and_inventory_station_surface\",\n"
            "            \"probability\": 0.7\n"
            "        },\n"
            "        {\n"
            "            \"initial area\": \"cooler_layer\",\n"
            "            \"probability\": 0.3\n"
            "        }\n"
            "    ]\n"
            "}\n"
            "```";
    }

    //read the area list from file
    vector <string> area_list;
    ifstream all_areas_file_stream(all_areas_file);
    string area_name;
    while (getline(all_areas_file_stream, area_name)) {
        area_list.push_back(area_name);
    }


    // 创建 area_list_string
    ostringstream oss;
    for (size_t i = 0; i < area_list.size(); ++i) {
        oss << area_list[i];
        if (i != area_list.size() - 1) {
            oss << ", ";  // 不是最后一个元素,添加逗号
        }
    }
    string area_list_string = oss.str();


    obj_of_interest_and_target_area_system_prompt = 
        "# Role \n"
         "You are an assistant to solve an object rearrangement task in a household kitchen environment. \n"
        "You'll receive: \n"
        "   1. the language instruction of the task, including the objects_of_interest and their target_areas.\n"
        "   2. the description of current observation of the environment, listing all open areas, closed areas and observed objects with their placements. If the object is inside a closed area, it is not visible.\n"
        "   3. A list of previously attempted(wrong) goal states, each containing combinations of objects and target areas that failed to achieve the goal. \n"
        // "   3. An action history list that contains the actions that have been taken. \n"
        "   4. a list of objects that have been already placed in the target areas. \n"
        "# Task \n"
        "You need to:"
        "1. Identify the correct objects of interest based strictly on the instruction and observation. Only select objects can help complete the task. \n"
        "2. Select valid target areas for those objects based on the task instruction, using areas explicitly mentioned in the observation.  \n"
        "3. Provide up to " + to_string(k_) + " possible valid combinations of objects and their target areas. \n\n"

        "# Guidelines \n"
        "## For objects of interest: \n"        
        "1. The instruction is ambiguous. You need to infer the intent of the language instruction and use common sense. \n"
        "2. Consider both visual objects and, more importantly, unseen objects of interest in the closed areas. \n"
        "3. Use '_' to connect multi-word object names (e.g., 'bell_pepper', not 'bell pepper'). \n"
        "4. For the observed objects, use its full name appeared the observation. \n"

        "## For target areas: \n"
        "1. The instruction is ambiguous. You need to infer the intent of the language instruction and use common sense. \n"
        "2. The target areas can only be selected from the areas explicitly mentioned in the observation. \n"

        // "## For action history: \n"
        // "1. The action history contains the actions that have been taken. \n"
        // "2. Action types: \n"
        // "   - Open(<area>, ): open the area \n"
        // "   - Pick(<area>, <object>, ): pick up the object from the area \n"
        // "   - Place(<area>, <object>, ): put the object in robot's hand to the area. \n"

        "## For objects already in target areas: \n"
        "1. These objects are checked by human that have been already placed in the target areas. \n"
        "2. You should totally ignore these objects!!!!! \n"

        "## For the final answer: \n"
        "1. You **must** give out your reasoning process first. Then, you **must** give your final answer in json format same as the example json answer. \n\n"


        "# **Critical Rules (Must Read First)**\n"
        "1. **Ignore objects already in target areas**  \n"
        "   - (Elaborated in Section 3 → Step1. If an object's current area in the observation equals the designated target area, discard it from consideration.)\n"
        "2. **Cycle Control**\n"
        "   - When returning to Section 1 for re-processing, **do not** reconsider objects that have already been blacklisted or discarded.\n"
        "\n"
        "# Requirements: Your reasoning process should include the following sections (Section 1 to 5) and steps in each section. For each step, you should explicitly give out your reasoning and the phased results, and give out your final JSON answer at last. \n"
        "## **Section 1: OBJECTS OF INTEREST IDENTIFICATION***\n"
        "\n"
        "### **Step1: OBJECTS OF INTEREST IDENTIFICATION**\n"
        "- **Generate up to **10** objects of interest.\n"
        "- **Focus on “fresh” objects** not in the blacklist.\n"
        "- Pay less attention on objects in **wrong goal states**.\n"
        "   - The more time the object appeared in wrong goal states, the less attention you should pay to it, and the more probability you should add it to the blacklist. \n"
        "   - **Example**: wrong goal states are: 1. chamomile_tea in Spice_Storage_Drawer. Then, pay less attention to chamomile_tea, and consider adding it to the blacklist. \n"
        // "- **Ignore** objects you have already placed in the target areas in the action history.\n"
        // "   - The more time the object appeared in action history, the less attention you should pay to it, and the more probability you should add it to the blacklist. \n"
        // "   - **Example**: action history contains: Pick(Coffee_Station_Surface, chamomile_tea, ), Place(Spice_Storage_Drawer, chamomile_tea, ), and you believe that Spice_Storage_Drawer is the target area of chamomile_tea. Then, adding chamomile_tea to the blacklist. \n"
        "- Totally ignore the objects that have been already placed in the target areas. \n"
        "   - **Example**: Objects already in target areas: chamomile_tea, then you should totally ignore chamomile_tea, and move it to the blacklist. \n"


        "- **Object Deficit Resolution Protocol**:\n"
        "  - If the observed objects are **insufficient** (≤ instruction requirements),\n"
        "    → **Generate hypothetical (unseen) objects** that:\n"
        "      1. Fit the instruction’s context and patterns.\n"
        "      2. Pass semantic coherence checks.\n"
        "      3. Comply with resource constraints.\n"
        "      4. Do **not** duplicate blacklisted properties.\n"
        "      5. Fulfill missing capabilities in the current object pool.\n"
        "\n"
        "## **Section 2: TARGET AREA IDENTIFICATION**\n"
        "\n"
        "### **Step1: CONTEXTUAL TARGETING**\n"
        "- **Select the most probable target_area** for each object.\n"
        "- **Target_area Identification Protocol **:\n"
        "   - **Fit the instruction’s context and patterns**.\n"
        "   - **Reject common-sense conflicts** (e.g., placing trash in the refrigerator).\n"
        //consider surfaces and Human_Hand first if the instruction is asked to put the objects in a easy-to-reach place or similar intentions.
        "   - **Consider Human_Hand first** if the instruction is asked for easy access objects or similar intentions.\n"
        "       - Example intentions: prepare for use, easy to reach, etc.\n"
        "\n"
        "## **Section 3: VALIDATION LOOP**\n"
        "\n"
        "### **Step1: TARGET_AREA CHECK**\n"
        "- For each candidate object visible in the observation:\n"
        "  - If object in list that have been already placed in the target areas → **Discard** this object entirely.\n"
        "  - If `current area in observation == target_area` → **Discard** this object entirely.\n"
        // "      - **Pay attention to the action histoy**. If the object has been placed in the target area in the action history, discard it.\n"
        "  - Otherwise, keep it in working memory.\n"
        "\n"
        "### **Step2: COMPLETENESS TEST**\n"
        "- If the **remaining objects** after discarding cannot fulfill the instruction:\n"
        "  - **Add current candidates to the blacklist**\n"
        "  - **Return to Section 1** but **exclude** blacklisted objects in the next iteration.\n"
        "\n"
        "## **Section 4: COMBINATION GENERATION**\n"
        "\n"
        "### **Step1: OBJECT-CENTRIC COMBINATION ENGINE**\n"
        "- **Generate up to " + to_string(k_) + "** object-only combinations from the pool of valid objects.\n"
        "- **No target_area assignments yet.**\n"
        "- **Each combination must contain no more than 4 objects.**\n"
        "- **Prioritize logical groupings** and **auto-prune duplicates** or redundant patterns.\n"
        "\n"
        "### **Step2: POST-HOC TARGET_AREA ASSIGNMENT**\n"
        "- For **each** combination from Step1:\n"
        "  1. **Per-object resolution**\n"
        "     - Select the highest-validity target_area option (per Section 2).\n"
        "  2. **Cross-combination locking**\n"
        "     - The first assignment chosen for an object → target_area **locks** that mapping.\n"
        "     - Subsequent combinations **must** reuse the same mapping.\n"
        "\n"

        "### **Step3: CROSS-MATRIX VALIDATION**\n"
        "- **Consistency Audit**\n"
        "  - Check that every object consistently uses the **same** target_area in **all** generated combinations.\n"
        "- **Failure Modes**\n"
        "  - If any target_area mismatch is detected, **remove all** conflicting combinations.\n"
        "  - If an object conflict arises, revisit Section 4 step 1 with penalty weighting.\n"
        "\n"
        "#### **Final Safeguards (Section 4)**\n"
        "1. **Sequential Locking Protocol**\n"
        "   - The first valid combination’s object→target_area assignments bind subsequent combinations.\n"
        "2. **Retroactive Consistency**\n"
        "   - Any new combinations must respect existing locked mappings.\n"
        "3. **Combination Quarantine**\n"
        "   - Combinations involving any unvalidated object-target pair are kept aside until validated.\n"

        "#### **Example(Section 4)**\n"
        "**Expected** combinations: \n"
        "  - combination 1: object: blender, target_area: Coffee_Station_Surface; object: cheese_grater, target_area:  Flex_Workspace_Surface \n"
        "  - combination 2: object: blender, target_area: Coffee_Station_Surface; object: potato_peeler, target_area:  Daily_Dish_Shelf\n"
        "Explanation: the **same object** in 2 combinations (blender) has the **same target_area** (Coffee_Station_Surface). The **combination of objects** in 2 combinations are different (blender and cheese_grater, blender and potato_peeler) \n"

        "**Unexpected** combinations: \n"
        "  - combination 1: object: blender, target_area: Coffee_Station_Surface; object: cheese_grater, target_area:  Flex_Workspace_Surface \n"
        "  - combination 2: object: blender, target_area: Daily_Dish_Shelf; object: cheese_grater, target_area:  Flex_Workspace_Surface\n"
        "Explanation: the **same object** in 2 combinations (blender) has the **different target_area** (Coffee_Station_Surface and Daily_Dish_Shelf). The **combination of objects** in 2 combinations are the same (blender and cheese_grater) \n"
        "\n"
        "## **Section 5: FINAL ANSWER**\n"
        "\n"
        "### **Step1: RESULT AGGREGATION & VALIDATION**\n"
        "- Combine **all** validated combinations.\n"
        "- Explicitly present the **final set** of object→target_area mappings.\n"
        "- Ensure **100% target-area consistency** with the locked pairs.\n"
        "\n"
        "### **Step2: FINAL OUTPUT CERTIFICATION**\n"
        "- Only execute after **successful** validation of sections 1–4.\n"
        "- Output the **final JSON answer** if:\n"
        "  1. All rules in sections 1–4 are satisfied.\n"
        "  2. Resource allocations remain within bounds.\n"
        "- The **final JSON answer** should be the same format with Example output JSON data: \n";
        // "\n";

    obj_of_interest_and_target_area_example_prompt =
        "       - Put your json data between ```json\\n and ```\\n. \n"

        "       - Example output JSON data: \n"
        "```json\n"
        "{\n"
        "    \"answer\": [\n"
        "        {\n"
        "            \"objects\": [\n"
        "               {\n"
        "                   \"object\": \"apple\",\n"
        "                   \"target_area\": \"Human_Hand\"\n"
        "               },\n"
        "               {\n"
        "                   \"object\": \"banana\",\n"
        "                   \"target_area\": \"robot\"\n"
        "               }\n"
        "            ],\n"
        "            \"probability\": 0.7\n"
        "        },\n"
        "        {\n"
        "            \"objects\": [\n"
        "               {\n"
        "                   \"object\": \"orange\",\n"
        "                   \"target_area\": \"Human_Hand\"\n"
        "               },\n"
        "               {\n"
        "                   \"object\": \"banana\",\n"
        "                   \"target_area\": \"robot\"\n"
        "               }\n"
        "            ],\n"
        "            \"probability\": 0.3\n"
        "        }\n"
        "    ]\n"
        "}\n"
        "```";
}       

string TreeOfHypothesis::query_llm(const string& prompt, string module){
    string system_prompt;
    if (module == "initial_areas"){

        system_prompt = initial_areas_system_prompt + initial_areas_example_prompt;

    }
    else if (module == "obj_of_interest_and_target_area"){
        cout<<"obj_of_interest_and_target_area"<<endl;
        system_prompt = obj_of_interest_and_target_area_system_prompt + obj_of_interest_and_target_area_example_prompt;
    }
    else
    {
        cout << "module is not recognized" << endl;
        exit(1);
    }

    if(setenv("OPENAI_API_KEY", api_key_.c_str(), 1) != 0){
        cout << "setenv failed" << endl;
        exit(1);
    }
    if (base_url_ != ""){
        if(setenv("OPENAI_API_BASE", base_url_.c_str(), 1) != 0){
            cout << "setenv failed" << endl;
            exit(1);
        }
    }

    // Set proxy from config if not "None"
    string proxy_url = despot::Globals::config.proxy_url;
    if (!proxy_url.empty() && proxy_url != "None") {
        if(setenv("https_proxy", proxy_url.c_str(), 1) != 0){
            cout << "setenv https_proxy failed" << endl;
            exit(1);
        }
        if(setenv("http_proxy", proxy_url.c_str(), 1) != 0){
            cout << "setenv http_proxy failed" << endl;
            exit(1);
        }
        // Extract port for socks5 proxy (assuming format http://host:port)
        size_t last_colon = proxy_url.find_last_of(':');
        if (last_colon != string::npos) {
            string socks_proxy = "socks5://" + proxy_url.substr(7, last_colon - 7) + ":7891";
            if(setenv("all_proxy", socks_proxy.c_str(), 1) != 0){
                cout << "setenv all_proxy failed" << endl;
                exit(1);
            }
        }
    }

    string total_prompt = system_prompt + prompt;
    nlohmann::json request;
    
    // Check if using Azure OpenAI (URL contains "deployments")
    bool is_azure = (base_url_.find("deployments") != string::npos);
    
    if (llm_model_ == "o3-mini"){
        if (is_azure) {
            // Azure doesn't need model field in request body
            request = {
                {"messages", nlohmann::json::array({
                    {
                        {"role", "system"},
                        {"content", system_prompt}
                    },
                    {
                        {"role", "user"},
                        {"content", prompt}
                    }})
                }
            };
        } else {
            request = {
                {"model", llm_model_},
                {"messages", nlohmann::json::array({
                    {
                        {"role", "system"},
                        {"content", system_prompt}
                    },
                    {
                        {"role", "user"},
                        {"content", prompt}
                    }})
                }
            };
        }
    }
    else{
        if (is_azure) {
            // Azure doesn't need model field in request body
            request = {
                {"messages", nlohmann::json::array({
                    {
                        {"role", "system"},
                        {"content", system_prompt}
                    },
                    {
                        {"role", "user"},
                        {"content", prompt}
                    }})
                },
                {"temperature", 0.1},
                {"max_tokens", 4000}
            };
        } else {
            request = {
                {"model", llm_model_},
                {"messages", nlohmann::json::array({
                    {
                        {"role", "system"},
                        {"content", system_prompt}
                    },
                    {
                        {"role", "user"},
                        {"content", prompt}
                    }})
                },
                {"temperature", 0.1},
                {"max_tokens", 4000}
            };
        }
    }

    // 发起 API 请求
    //try 10 times
    string completion;
    nlohmann::json response;
    string content;
    for (int i = 0; i < 10; i++)
    {
        try{
            
            despot::Globals::config.total_llm_calls++;
            despot::Globals::config.tree_of_hypothesis_llm_calls++;
            //start time
            // auto start = chrono::steady_clock::now();
            completion = call_llm_api(base_url_, api_key_, request);
            //end time
            // auto end = chrono::steady_clock::now();

            // despot::Globals::config.total_llm_query_time += chrono::duration<double>(end - start).count();    
            // despot::Globals::config.tree_of_hypothesis_llm_query_time += chrono::duration<double>(end - start).count();    
            // cout<<completion<<endl;
            response = nlohmann::json::parse(completion);
            // cout << "response: " << response << endl;

            // cout <<"input tokens number:" << response["useage"]["prompt_tokens"] << endl;
            // cout <<"output tokens number:" << response["useage"]["completion_tokens"] << endl;
        
            content = response["choices"][0]["message"]["content"];

            //change json to int
            prompt_tokens += response["usage"]["prompt_tokens"].get<int>();  
            completion_tokens += response["usage"]["completion_tokens"].get<int>();

            // cout<<"response: "<<completion<<endl;
            break;
            // return completion;
        }
        catch(const exception& e){
            cout<<"error: "<<e.what()<<endl;
            cout<<"retrying..."<<endl;
            //sleep for 2 seconds
            this_thread::sleep_for(chrono::seconds(2));
        }
    }
    // cout<< content << endl;
    return content;

}

void TreeOfHypothesis::RecursiveCombinations(
    const vector<string>& objects,
    const map<string, vector<pair<string, double>>>& initial_areas_map,
    const map<string, vector<pair<string, double>>>& target_areas_map,
    size_t idx,
    vector<pair<map<string, pair<string, string>>, double>>& all_combinations,
    map<string, pair<string, string>>& current_combination,
    double current_probability) {

    if (idx == objects.size()) {
        // 递归的终止条件,已经处理完所有对象
        all_combinations.push_back({current_combination, current_probability});
        return;
    }

    const string& object = objects[idx];
    
    if (initial_areas_map.find(object) != initial_areas_map.end() && target_areas_map.find(object) != target_areas_map.end()) {
        // 获取当前对象的所有initial_area和target_area组合
        for (const auto& initial_area : initial_areas_map.at(object)) {
            for (const auto& target_area : target_areas_map.at(object)) {
                // 当前组合
                current_combination[object] = {initial_area.first, target_area.first};
                // 递归计算下一个对象的组合
                RecursiveCombinations(objects, initial_areas_map, target_areas_map, idx + 1, all_combinations, current_combination,
                                      current_probability * initial_area.second * target_area.second);
            }
        }
    }
}
   

void TreeOfHypothesis::RecursiveCombinationsOneArea(
    const vector<string> &objects,
    const map<string, vector<pair<string, double>>> &area_map,
    size_t idx,
    vector<pair<map<string,string>, double>> &all_combinations,
    map<string, string> &current_combination,
    double current_probability){
        
    if (idx == objects.size()){
        // 递归的终止条件,已经处理完所有对象
        all_combinations.push_back({current_combination, current_probability});
        return;
    }

    const string &object = objects[idx];

    if (area_map.find(object) != area_map.end()){
        // 获取当前对象的所有initial_area组合
        for (const auto &area : area_map.at(object)){
            // 当前组合
            current_combination[object] = area.first;
            // 递归计算下一个对象的组合
            RecursiveCombinationsOneArea(objects, area_map, idx + 1, all_combinations, current_combination, current_probability * area.second);
        }
    }
}

vector<pair<map<string, pair<string, string>>, double>>
TreeOfHypothesis::generate_tree_of_hypothesis_obj_of_interest_target_areas(
    const string &language_instruction, 
    const string &observation_prompt, 
    const string &wrong_states_prompt,
    const vector<pair<map<string, string>, double>> partial_update_hypothesis,
    const double total_probability
    )
{
    string global_prompt = "Instructions: "+language_instruction+"\n";
    global_prompt += "Current Observation: "+observation_prompt+"\n";

    string wrong_states_prompt_with_heading = "The wrong goal states are: \n" + wrong_states_prompt + "\n";
    
    wrong_states_prompt_with_heading +=  despot::Globals::config.objects_in_goal + "\n";
    // cout<< "wrong_states_prompt_with_heading: " << wrong_states_prompt_with_heading << endl;

    vector<string> all_object_names = {};
    vector<pair<map<string, string>, double>> obj_of_interest_and_target_areas_results;

    string objects_of_interest_question_prompt = global_prompt + wrong_states_prompt_with_heading + "Now give your answer.\n";

    //start time
    auto start = chrono::steady_clock::now();
    string object_of_interests_rough_answer = query_llm(objects_of_interest_question_prompt, "obj_of_interest_and_target_area");
    //parse the objects of interests
    //end time
    auto end = chrono::steady_clock::now();
    despot::Globals::config.total_llm_query_time += chrono::duration<double>(end - start).count();
    despot::Globals::config.tree_of_hypothesis_llm_query_time += chrono::duration<double>(end - start).count();

    if (despot::Globals::config.save_data){
        string step_dir = despot::Globals::config.current_root_dir + "/step_" + to_string(despot::Globals::config.current_step);
        //create a directory for all llm_query results
        string llm_dir = step_dir + "/llm_query_results";
        string command = "mkdir -p " + llm_dir;
        system(command.c_str());
        string file_name = llm_dir + "/object_of_interests.txt";
        //write the prompt and answer to the file
        ofstream file;
        file.open(file_name, ios::out|ios::app);
        file << "prompt: " << objects_of_interest_question_prompt << endl;
        file << "answer: " << object_of_interests_rough_answer << endl;
    }

    nlohmann::json object_of_interests_json = pare_llm_answer_to_json(object_of_interests_rough_answer);

    // cout<<"object_of_interests_json: "<<object_of_interests_json.dump(2)<<endl;

    //get the objects of interest and target areas
    for (const auto& answer : object_of_interests_json["answer"]){
        map<string, string> combination;
        for(const auto& object: answer["objects"]){
            string object_name = object["object"];
            string target_area = object["target_area"];
            combination[object_name] = target_area;
            if (find(all_object_names.begin(), all_object_names.end(), object_name) == all_object_names.end()){
                all_object_names.push_back(object_name);
            }
        }
        double probability = answer["probability"];
        obj_of_interest_and_target_areas_results.push_back(make_pair(combination, probability));
        cout<<"object of interest and target areas: ";
        for (const auto & [object_name, target_area]: combination){
            cout<< object_name<<": "<<target_area<<", ";
        }
        cout<<"probability: "<<probability<<endl;
    }

    //get the initial areas
    vector<pair<string, string>> initial_areas_problem_prompts;
    for(const auto& object_name: all_object_names){
        string initial_areas_prompt = "Current Observation: " + observation_prompt + "\n" + "What are the initial areas of " + object_name + "?";
        initial_areas_problem_prompts.push_back(make_pair(initial_areas_prompt, "initial_areas"));
    }

    vector<future<void>> all_responses;
    map<string, vector<pair<string, double>>> initial_areas_all_results;

    mutex mtx;  // 用于同步

    start = chrono::steady_clock::now();
    // 启动异步任务
    for (size_t i = 0; i < initial_areas_problem_prompts.size(); ++i) {
        all_responses.push_back(async(launch::async, [&, i]() {
            // 执行查询
            // cout<<"query_llm: "<<initial_areas_problem_prompts[i].first<<endl;
            string response_rough_str = query_llm(initial_areas_problem_prompts[i].first, initial_areas_problem_prompts[i].second);
            nlohmann::json response_json = pare_llm_answer_to_json(response_rough_str);

            // 使用互斥锁保证线程安全地访问 response_answers
            {
                lock_guard<mutex> lock(mtx);  // 加锁保护对共享数据的访问
                // 判断并修改 response_answers
                string object_name = all_object_names[i];
                vector<pair<string, double>> area_info;
                for(const auto& initial_area_pair: response_json["answer"]){
                    string initial_area_name = initial_area_pair["initial_area"];
                    double initial_area_probability = initial_area_pair["probability"];
                    area_info.push_back(make_pair(initial_area_name, initial_area_probability));
                }
                initial_areas_all_results[object_name] = area_info;
                
                if (despot::Globals::config.save_data){
                    string step_dir = despot::Globals::config.current_root_dir + "/step_" + to_string(despot::Globals::config.current_step);
                    string llm_dir = step_dir + "/llm_query_results";
                    string file_name = llm_dir + "/initial_areas_" + object_name + ".txt";
                    //write the prompt and answer to the file
                    ofstream file;
                    file.open(file_name, ios::out|ios::app);
                    file << "prompt: " << initial_areas_problem_prompts[i].first << endl;
                    file << "answer: " << response_rough_str << endl;
                }

            }
        }));
    }

    // 等待所有异步任务完成
    for (auto& future : all_responses) {
        future.get();
    }

    end = chrono::steady_clock::now();
    despot::Globals::config.total_llm_query_time += chrono::duration<double>(end - start).count();
    despot::Globals::config.tree_of_hypothesis_llm_query_time += chrono::duration<double>(end - start).count();

    for (const auto& [key, value] : initial_areas_all_results) {
        cout << key << ": ";
        for (const auto& area : value) {
            cout << area.first << " " << area.second << ", ";
        }
        cout << endl;
    }

    vector<pair<map<string, pair<string, string>>, double>> all_results;

    double new_particles_probability = total_probability;

    if (new_particles_probability > 0){
        for(const auto& object_of_interest_result: obj_of_interest_and_target_areas_results){
            
            const map<string, string>& hypothesis = object_of_interest_result.first;
            vector<string> object_names;
            for (const auto &[object_name, target_area_name] : hypothesis)
            {
                object_names.push_back(object_name);
            }
            map<string, vector<pair<string, double>>> initial_areas_map = initial_areas_all_results;

            //calculate combination of initial areas
            vector<pair<map<string, string>, double>> all_combinations;
            map<string, string> current_combination_initial_areas;
            RecursiveCombinationsOneArea(object_names, initial_areas_all_results, 0, all_combinations, current_combination_initial_areas, object_of_interest_result.second);
            //add target areas    
            for (const auto& [combination, probability]: all_combinations){
                map<string, pair<string, string>> new_combination;
                for (const auto& [object_name, initial_area_name]: combination){
                    //find the target area
                    string target_area_name = hypothesis.at(object_name);
                    new_combination[object_name] = make_pair(initial_area_name, target_area_name);
                }
                all_results.push_back({new_combination, probability});
            }
        }
    }

    //normalize the probability to 1
    double sum_probability = 0.0;
    for(const auto& [combination, probability]: all_results){
        sum_probability += probability;
    }

    for(auto& [combination, probability]: all_results){
        probability = probability / sum_probability;
    }

    //write all_results to file
    if (despot::Globals::config.save_data){
        string step_dir = despot::Globals::config.current_root_dir + "/step_" + to_string(despot::Globals::config.current_step);
        string llm_dir = step_dir + "/llm_query_results";
        string file_name = llm_dir + "/all_results_from_tree_of_hypothesis.txt";
        //write the prompt and answer to the file
        ofstream file;
        file.open(file_name, ios::out|ios::app);
        for(const auto& [combination, probability]: all_results){
            file << "combination: ";
            for(const auto& [object_name, area_pair]: combination){
                file << object_name << ": " << area_pair.first << " " << area_pair.second << ", ";
            }
            file << "probability: " << probability << endl;
        }
    }

    return all_results;

}

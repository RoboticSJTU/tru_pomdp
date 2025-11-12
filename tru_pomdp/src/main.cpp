#include <tru_pomdp_world.h>
#include <tru_pomdp_task.h>
#include <tru_pomdp_planner.h>

#include <despot/planner.h>
#include <despot/core/globals.h>
#include <utils.h>

#include<cstdlib>


using namespace despot;
using namespace std;

int main(int argc, char *argv[]) {

    string scene_graph = argv[1];

    string root_path = getenv("ROOT_PATH");
    string problem_file_path = root_path + "/tasks/" + scene_graph+ "/" +argv[2];

    cout << "problem_file_path: " << problem_file_path << endl;
    // string scene_file_path = argv[2];
    string scene_file_path = root_path+ "/scenegraphs/kitchen_" + scene_graph + ".json";
    cout << "scene_file_path: " << scene_file_path<<endl;
    int num_scenarios = stoi(argv[3]);
    cout << "num_scenarios: " << num_scenarios << endl;
    int sim_len = stoi(argv[4]);
    cout << "sim_len: " << sim_len << endl;
    int max_policy_sim_len = (argc > 4) ? stoi(argv[5]) : sim_len; // Default to sim_len if not provided
    cout << "max_policy_sim_len: " << max_policy_sim_len << endl;
    string result_file_path = root_path + "/results/" + argv[6];
    cout << "result_file_path: " << result_file_path << endl;
    ofstream outfile;
    outfile.open(result_file_path, ios::app);
    outfile << argv[2]<< endl;
    outfile.close();

    string llm_model = argv[7];
    cout << "llm_model: " << llm_model << endl;
    string api_key = argv[8];
    cout << "api_key: " << api_key << endl;
    string base_url = argv[9];
    cout << "base_url: " << base_url << endl;
    int k = stoi(argv[10]);
    cout << "k: " << k << endl;
    string proxy_url = (argc > 11) ? argv[11] : "None";
    cout << "proxy_url: " << proxy_url << endl;

    string COT_str = (argc > 12)? (argv[12]) : "false";
    cout << "COT: " << COT_str << endl;
    if(COT_str == "true"){
        Globals::config.cot = true;
    }else{
        Globals::config.cot = false;
    }

    string save_data_str = (argc > 13)? (argv[13]) : "false";
    cout << "save_data: " << save_data_str << endl;
    if(save_data_str == "true"){
        Globals::config.save_data = true;
    }else{
        Globals::config.save_data = false;
    }

    // string scene_file_path = scene_file_path;
    Globals::config.num_scenarios = num_scenarios;
    Globals::config.sim_len = sim_len;
    Globals::config.search_depth = 20;
    Globals::config.max_policy_sim_len = max_policy_sim_len;

    Globals::config.time_per_move = 1.0;
    
    Globals::config.llm_model = llm_model;
    Globals::config.api_key = api_key;
    Globals::config.base_url = base_url;
    Globals::config.proxy_url = proxy_url;
    Globals::config.k = k;
    Globals::config.scene_graph = scene_graph;

    if (Globals::config.save_data){
        //获得当前时间戳
        time_t now = time(0);
        //根据时间戳和argv[1]创建文件夹
        string dir_name = root_path + "/data/" + to_string(now);
        Globals::config.current_root_dir = dir_name;
        string command = "mkdir " + dir_name;
        system(command.c_str());

        //创建文件： settings.txt
        string settings_file_path = dir_name + "/settings.txt";
        ofstream settings_file;
        settings_file.open(settings_file_path, ios::app);
        if (!settings_file.is_open())
        {
            cerr << "Error: Failed to open settings file." << endl;
            return 1;
        }
        settings_file << "problem_file_path: " << problem_file_path << endl;
        //todo: default policy 版本
        //Global::config
        settings_file << "scene_graph: " << scene_graph << endl;
        settings_file << "problem: " << argv[2] << endl;
        settings_file << "num_scenarios: " << num_scenarios << endl;
        settings_file << "sim_len: " << sim_len << endl;
        settings_file << "search_depth: " << Globals::config.search_depth << endl;
        settings_file << "max_policy_sim_len: " << Globals::config.max_policy_sim_len << endl;
        settings_file << "time_per_move: " << Globals::config.time_per_move << endl;
        settings_file << "llm_model: " << llm_model << endl;
        settings_file << "k: " << k << endl;
        settings_file << "proxy_url: " << proxy_url << endl;

        settings_file.close();
        cout << "settings_file_path: " << settings_file_path << endl;        
    }

    TruPomdpPlanner planner;
    int step_num = planner.RunPlanning_write_result(argc, argv, problem_file_path, scene_file_path, result_file_path);
    cout<<"Complete for "<< problem_file_path<< endl;
}
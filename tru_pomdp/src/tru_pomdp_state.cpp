#include<tru_pomdp_state.h>

using namespace std;
namespace despot
{

    /* ==============================================================================
     * TruPomdpState class
     * ==============================================================================*/

    TruPomdpState::TruPomdpState()
    {
        weight = 1.0;
        legal_actions_ = make_unique<array<ActionSimple, 200>>();
    }

    TruPomdpState::TruPomdpState(
        const SceneGraphSimple scene_graph, 
        const TaskSimple &task, 
        std::vector<ActionSimple> *all_actions_ptr,
        unordered_map<size_t, int> *action_index_map)
    {
        // scene_graph_ = move(scene_graph->Copy());
        scene_graph_ = scene_graph.Copy();
        task_ = task;
        weight = 1.0;

        all_actions_ptr_ = all_actions_ptr;
        action_index_map_ = action_index_map;
        legal_actions_ = make_unique<array<ActionSimple, 200>>();
        UpdateActions();
    }

    TruPomdpState::TruPomdpState(
        const SceneGraphSimple scene_graph, 
        const TaskSimple &task, 
        std::vector<ActionSimple> *all_actions_ptr,
        unordered_map<size_t, int> *action_index_map,
        double w, 
        int s_id, 
        int sc_id)
    {
        scene_graph_ = scene_graph.Copy();
        task_ = task;
        weight = w;
        state_id = s_id;
        scenario_id = sc_id;

        all_actions_ptr_ = all_actions_ptr;
        // vector<string> all_actions = *all_actions_ptr_;
        action_index_map_ = action_index_map;
        legal_actions_ = make_unique<array<ActionSimple, 200>>();
        UpdateActions();
    }

    // TruPomdpState::~TruPomdpState()
    // {
    // }

    string TruPomdpState::text() const
    {
        //todo
        ostringstream oss;
        return oss.str();
    }

    void TruPomdpState::set_scene_graph(const SceneGraphSimple scene_graph)
    {
        scene_graph_ = scene_graph.Copy();
        UpdateActions();
    }

    void TruPomdpState::set_task(const TaskSimple &task)
    {
        task_ = task;
    }


    void TruPomdpState::UpdateActions()
    {
        scene_graph_.GetLegalActions(legal_actions_, legal_action_num_);
        // legal_actions_ = temp.first;
        // legal_action_num_ = temp.second;

        // vector<AreaNode> all_area_list = state->scene_graph_->all_areas();

        vector<int> object_id_list;
        TaskSimple current_task;
        // for (const auto &goal : task_._area_has_obj)
        for (int i =0; i< task_._goal_num; i++)
        {
            const auto goal = task_._area_has_obj[i];
            // TaskSimple current_task = TaskSimple();
            current_task.AddGoal(goal[0], goal[1]);
            bool goal_test = scene_graph_.GoalTest(current_task);
            if (goal_test){
                continue;
            }
            object_id_list.push_back(goal[1]);
            current_task.Reset();
        }
        if (object_id_list.size()==0){
            cerr<<"all goals are reached!"<<endl;
        }
        // vector<string> action_strs = get_str_actions(legal_actions, object_name_list);
        get_target_object_related_actions(legal_actions_, legal_action_num_, object_id_list);
        if(legal_action_num_ <=1){
            cerr<<"no legal actions"<<endl;
        }
    }
} // namespace despot
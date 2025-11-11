#pragma once
#include <vector>
#include <map>
#include <SceneGraph/Node.h>
#include <SceneGraph/Robot.h>
#include <SceneGraph/Action.h>
#include <SceneGraph/Task.h>
#include <SceneGraph/ActionResult.h>
#include <memory>
#include <unordered_map>
#include <queue>
#include <limits>
#include <algorithm>

using namespace std;

class SceneGraphSimple;
class Task;
class TaskSimple;
class TaskSimpleExp;
class SceneGraph
{
private:

    vector<string> _area_names;
    vector<AreaNode> _area_nodes;
    vector<string> _object_names;
    vector<ObjectNode> _object_nodes;
    Robot _robot;

    vector<pair<int, int>> _objects_in_areas;
    int _robot_location = -1;
    int _object_in_robot_hand = -1;

    bool _scene_initialized = false;
    bool _problem_initialized = false;

    //navigation map, store the distance between each two areas
    //use a 2d vector to store the distance
    vector<double> _navigation_map;
    int _area_num;

    // void build_quick_index();

public:
    SceneGraph();
    SceneGraph(int area_num, int object_num){
        //生成一个空scenegraph，预留area_num个区域和object_num个物体的空间
        //area和object的名字长度预留50个字符
        _area_names.reserve(area_num);
        _area_nodes.reserve(area_num);
        _object_names.reserve(object_num);
        _object_nodes.reserve(object_num);
        _objects_in_areas.reserve(object_num);
        _navigation_map.reserve(area_num*area_num);
        area_num = area_num;
    }
    // SceneGraph(const SceneGraph&) = delete;
    // delete operator =
    // SceneGraph& operator=(const SceneGraph&) = delete;
    virtual ~SceneGraph();
    Node AllocNode(const string &name, NodeType type);
    bool LoadScene();
    bool LoadSceneJson(const string &json_str);
    //load scene from full json file
    bool LoadSceneJsonFull(const string &file_content);
    // string DumpJson() const;
    bool LoadProblemJson(const string &file_content);

    // operator << for debug output
    friend ostream &operator<<(ostream &os, const SceneGraph &graph);

    // get a copy of scene graph where only observable part is copied.
    unique_ptr<SceneGraph> Copy(bool with_invisible=true) const;
    const Node* GetNode(const string &name) const;
    vector<AreaNode> all_areas() const;
    vector<ObjectNode> all_objects() const;
    const AreaNode* GetArea(const string &name) const;
    const ObjectNode* GetObject(const string &name) const;
    const Robot* GetRobot() const;

    ActionResult Forward(const Action &action);
    vector<Action> GetLegalActions();
    bool GoalTest(const Task &task);
    vector<string> ObjectsReachGoal(const Task &task);
    // void TakeActionImpl_Move(const vector<string> &args, ActionResult &result);
    void TakeActionImpl_Pick(const vector<string> &args, ActionResult &result);
    void TakeActionImpl_Place(const vector<string> &args, ActionResult &result);
    void TakeActionImpl_Open(const vector<string> &args, ActionResult &result);


    string ToJsonString() const;
    int GetReward(const Action &action, const Task &task) const;

    // change scene graph
    bool AddObjectInArea(const string& area_name, const string& object_name);
    bool AddObjectInHand(const string& object_name);
    bool RemoveObject(const string& object_name);
    bool SetRobotLocation(const string& area_name);
    bool SetAreaIsOpen(const string& area_name, bool is_open);

    // get address of this
    uintptr_t DEBUG_GET_ADDRESS() const {
        return reinterpret_cast<uintptr_t>(this);
    }

    const AreaNode* GetRobotLocation();
    bool CheckObjectInArea(const string& object_name, const string& area_name);
    void RemoveObjectFromArea(const string& object_name, const string& area_name);

    const Node* GetObjectParentNode(const string& object_name);
    const ObjectNode* GetObjectInHand();
    const vector<ObjectNode*> GetObjectsInArea(const string& area_name); 

    bool CheckObjectsExistsAndAreasOpen(const vector<string> &object_names);

    friend class SceneGraphSimple;
};

//simple version of SceneGraph
//no string
class SceneGraphSimple{
private:
    array<int, 50> _area_ids;
    int _area_num = 0;
    array<int, 50> _object_ids;
    int _object_num = 0;
    array<int, 50> _area_opens;
    array<int, 50> _objects_in_areas;
    int _robot_location = -1;
    // int _object_in_robot_hand = -1;

    array<int, 2500> _navigation_map;

public:
    SceneGraphSimple();
    virtual ~SceneGraphSimple();

    friend ostream &operator<<(ostream &os, const SceneGraph &graph);
    SceneGraphSimple Copy(bool with_invisible=true) const; 
    SceneGraphSimple CopyFromSceneGraph(unique_ptr<SceneGraph> scene_graph, vector <string> area_names, vector <string> object_names) const;
    
    bool CheckObjectInScene(const int object_id) const{
        return distance(_object_ids.begin(), find(_object_ids.begin(), _object_ids.begin()+_object_num, object_id)) != _object_num;
    }
    bool CheckAreaInScene(const int area_id) const{
        return distance(_area_ids.begin(), find(_area_ids.begin(), _area_ids.begin()+_area_num, area_id)) != _area_num;
    }
    const vector<string> all_areas(vector<string> area_names) const;
    const vector<string> all_objects(vector<string> object_names) const;
    const pair<string, bool> GetArea(int id, vector<string> area_names) const;
    const string GetObject(int id, vector<string> object_names) const;
    const bool GetAreaOpen(int id) const;
    const bool GetAreaOpenFromId(int id) const;
    const bool GetAreaOpenFromName(const string& area_name, vector<string> area_names) const;

    ActionResult Forward(const ActionSimple &action);
    pair<array<ActionSimple, 200>, int> GetLegalActions();
    void GetLegalActions(unique_ptr<array<ActionSimple, 200>> &legal_actions, int &action_num);
    bool GoalTest(TaskSimple &task)const;
    void TakeActionImpl_Pick(const ActionSimple &action, ActionResult &result);
    void TakeActionImpl_Place(const ActionSimple &action, ActionResult &result);
    void TakeActionImpl_Open(const ActionSimple &action, ActionResult &result);

    int GetReward(const ActionSimple &action, const TaskSimple &task) const;

    bool AddObjectInArea(int area_id, int object_id);
    bool AddObjectInHand(int object_id);
    bool RemoveObject(int object_id);
    bool SetRobotLocation(int area_id);
    bool SetAreaIsOpen(int area_id, bool is_open);

    bool AddObjectInAreaFromName(const string& area_name, const string& object_name, vector<string> area_names, vector<string> object_names);
    bool AddObjectInHandFromName(const string& object_name, vector<string> object_names);
    bool RemoveObjectFromName(const string& object_name, vector<string> object_names);
    bool SetRobotLocationFromName(const string& area_name, vector<string> area_names);
    bool SetAreaIsOpenFromName(const string& area_name, bool is_open, vector<string> area_names);

    const int GetRobotLocation() const;
    const string GetRobotLocationName(vector<string> area_names);

    const int GetObjectParent(int object_id) const{
        int object_index = distance(_object_ids.begin(), find(_object_ids.begin(), _object_ids.begin()+_object_num, object_id));
        if (object_index >= 0 && object_index < _object_num)
        {
            return _objects_in_areas[object_index];
        }
        return -1;
    };
    const string GetObjectParentName(const string& object_name, vector<string> object_names, vector<string> area_names);
    const string GetObjectInHandName(vector<string> object_names);
    const vector<string> GetObjectsInArea(const string& area_name, vector<string> object_names, vector<string> area_names);

    const string GetObsStr();
    const int GetObjectInHand() const{
        int object_index = distance(_objects_in_areas.begin(), find(_objects_in_areas.begin(), _objects_in_areas.begin()+_object_num, 50));
        if(object_index < _object_num){
            return _object_ids[object_index];
        }
        return -1;
    }

    //重构 = 操作符
    SceneGraphSimple &operator=(const SceneGraphSimple &scene_graph);
    
    //functions for exploration process
    ActionResult ForwardExp(const ActionSimple &action);
    void GetLegalActionsExp(unique_ptr<array<ActionSimple, 200>> &legal_actions, int &action_num);
    bool GoalTestExp(TaskSimpleExp &task)const;


};

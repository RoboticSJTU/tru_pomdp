#include <SceneGraph/ActionResult.h>
#include <SceneGraph/Exceptions.h>
#include <SceneGraph/SceneGraph.h>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <random>

using json = nlohmann::json;
using namespace std;

// 结构体来表示一个边
// 结构体表示边
struct Edge {
    string to;  // 目标区域
    double cost;  // 目标区域的移动成本
};

// 计算最短路径的函数：Dijkstra算法
vector<double> dijkstra(const unordered_map<string, vector<Edge>>& graph, const string& start, const vector<string>& areas) {
    // 存储每个区域的最短路径，初始值为无穷大
    unordered_map<string, double> dist;
    for (const auto& area : areas) {
        dist[area] = numeric_limits<double>::infinity();
    }

    dist[start] = 0;

    // 优先队列，按最短路径从小到大排序
    priority_queue<pair<double, string>, vector<pair<double, string>>, greater<>> pq;
    pq.push({0, start});

    while (!pq.empty()) {
        auto [current_dist, current_area] = pq.top();
        pq.pop();

        // 如果当前区域的最短路径已经大于当前距离，跳过
        if (current_dist > dist[current_area]) {
            continue;
        }

        // 遍历所有相邻区域
        // for (const auto& edge : graph.at(current_area)) {
        for (int i = 0; i < graph.at(current_area).size(); i++) {
            Edge edge = graph.at(current_area)[i];
            double new_dist = current_dist + edge.cost;

            // 如果找到更短的路径，更新最短路径并将新状态加入队列
            if (new_dist < dist[edge.to]) {
                dist[edge.to] = new_dist;
                pq.push({new_dist, edge.to});
            }
        }
    }

    // 将最短路径结果按区域顺序存入vector
    vector<double> result;
    for (const auto& area : areas) {
        result.push_back(dist[area]);
    }

    return result;
}

// 从 JSON 解析获取 pose 到 area 的映射关系
unordered_map<string, unordered_map<string, string>> buildPoseToAreaMap(const json& j) {
    unordered_map<string, unordered_map<string, string>> pose_to_area;
    
    for (auto pose = j["room"].begin(); pose != j["room"].end(); pose++)
    {
        auto furnitures = pose.value();
        for (auto furniture = furnitures.begin(); furniture != furnitures.end(); furniture++)
        {
            auto areas = furniture.value();
            for (auto area = areas.begin(); area != areas.end(); area++)
            {
                string pose_name = pose.key();
                string area_name = area.key();
                string furniture_name = furniture.key();
                pose_to_area[pose_name][area_name] = furniture_name;
            }
        }
    }
    return pose_to_area;
}

// 计算所有 area 之间的最短路径并存储在一个矩阵中
vector<double> computeShortestPaths(const json& scene_graph, const vector<string>& area_names) {
    unordered_map<string, vector<Edge>> graph;
    unordered_map<string, unordered_map<string, string>> pose_to_area = buildPoseToAreaMap(scene_graph);

    // 构建图（基于 pose 之间的边）
    for (const auto& edge : scene_graph["edges"]) {
        string pose_from = edge["link"][0];
        string pose_to = edge["link"][1];
        double cost = edge["cost"];

        // 获取area名称
        unordered_map<string, string>& areas_from = pose_to_area[pose_from];
        unordered_map<string, string>& areas_to = pose_to_area[pose_to];

        // 如果从两个 pose 之间有连接，添加边
        for (const auto& area_from : areas_from) {
            for (const auto& area_to : areas_to) {
                if (area_from.second != area_to.second) {  // 不同的 furniture 之间，才有 cost
                    graph[area_from.first].push_back({area_to.first, cost});
                    graph[area_to.first].push_back({area_from.first, cost});  // 双向图
                }
            }
        }
    }

    // 同一 furniture 中的 area 之间的 cost 为 0
    for (const auto& pose_pair : pose_to_area) {
        for (const auto& area_pair : pose_pair.second) {
            for (const auto& other_area_pair : pose_pair.second) {
                if (area_pair.first != other_area_pair.first) {
                    graph[area_pair.first].push_back({other_area_pair.first, 0});
                    graph[other_area_pair.first].push_back({area_pair.first, 0});  // 双向图
                }
            }
        }
    }

    // 结果矩阵
    vector<double> matrix(area_names.size() * area_names.size(), numeric_limits<double>::infinity());

    // 使用 Dijkstra 计算每对 area 之间的最短路径
    for (int i = 0; i < area_names.size(); ++i) {
        string start = area_names[i];
        vector<double> dist = dijkstra(graph, start, area_names);

        // 将计算的结果填入矩阵（按行优先顺序）
        for (int j = 0; j < area_names.size(); ++j) {
            matrix[i * area_names.size() + j] = dist[j];
        }
    }

    cout<<"max distance: "<<*max_element(matrix.begin(), matrix.end())<<endl;
    return matrix;
}

SceneGraph::SceneGraph()
{
    // create a root node
    // _root = AllocNode("root", NODE_NONE);
    // DEBUG_SCENE_GRAPH_MEMORY_LEAK.insert(this);
}

SceneGraph::~SceneGraph()
{
    // // reset all shared_ptrs in _all_nodes.
    // // reset all of their parent and children.
    // // cerr << "DEBUG: SceneGraph destructor" << endl;
    // for (auto node : _area_nodes)
    // {
    //     node.reset();
    // }
    // _root.reset();
}

Node SceneGraph::AllocNode(const string &name,
                           NodeType type)
{
    // shared_ptr<Node> new_node = nullptr;
    // Node new_node;

    switch (type)
    {
        //   case NODE_POSE:
        //     new_node = make_shared<PoseNode>(name);
        //     break;
    case NODE_AREA:
        // AreaNode new_node(name, false);
        return AreaNode(name, false);
        break;
    case NODE_OBJECT:
        // new_node = make_shared<ObjectNode>(name);
        return ObjectNode(name);
        break;
        //   case NODE_FURNITURE:
        //     new_node = make_shared<FurnitureNode>(name);
        //     break;
    case NODE_ROBOT:
        // new_node = make_shared<Robot>();
        return Robot();
        break;
    default:
        // new_node = make_shared<Node>(name, type);
        return Node(name, type);
    }
}

// void SceneGraph::build_quick_index()
// {
//     for (auto node : _all_nodes)
//     {
//         _map_name_node[node->GetName()] = node;
//         if (node->Type == NODE_AREA)
//         {
//             // _map_name_area[node->GetName()] = dynamic_cast<AreaNode *>(node);
//             _map_name_area[node->GetName()] =
//                 dynamic_pointer_cast<AreaNode>(node);
//         }
//         else if (node->Type == NODE_OBJECT)
//         {
//             // _map_name_object[node->GetName()] = dynamic_cast<ObjectNode *>(node);
//             _map_name_object[node->GetName()] =
//                 dynamic_pointer_cast<ObjectNode>(node);
//         }
//         else if (node->Type == NODE_ROBOT)
//         {
//             // _robot = dynamic_cast<Robot *>(node);
//             _robot = dynamic_pointer_cast<Robot>(node);
//         }
//     }
// }

// bool SceneGraph::LoadScene() { return LoadSceneJson(JSON_SCENE_TEST); }

bool SceneGraph::LoadSceneJson(const string &json_str)
{

    // only need to load area
    if (_scene_initialized)
    {
        throw SceneAlreadyInitializedException();
    }

    json j = json::parse(json_str);
    for (auto area = j["room"].begin(); area != j["room"].end(); area++)
    {
        AreaNode area_node(area.key(), false);
        area_node._is_open = area.value().at("is_open").get<bool>();

        _area_names.push_back(area.key());
        _area_nodes.push_back(area_node);

        // Node::LinkNodes(_root, area_node);
    }

    // build_quick_index();
    _scene_initialized = true;
    return true;
}

bool SceneGraph::LoadSceneJsonFull(const string &file_content)
{
    if (_scene_initialized)
    {
        throw SceneAlreadyInitializedException();
    }

    json j = json::parse(file_content);
    for (auto pose = j["room"].begin(); pose != j["room"].end(); pose++)
    {
        auto furnitures = pose.value();
        for (auto furniture = furnitures.begin(); furniture != furnitures.end(); furniture++)
        {
            auto areas = furniture.value();
            for (auto area = areas.begin(); area != areas.end(); area++)
            {
                // cout << "... Loading area " << area.key() << endl;
                bool is_open = area.value().at("is_open").get<bool>();
                // shared_ptr<AreaNode> area_node =
                //     dynamic_pointer_cast<AreaNode>(AllocNode(area.key(), NODE_AREA));
                // area_node->_is_open = is_open;
                // Node::LinkNodes(_root, area_node);

                AreaNode area_node(area.key(), is_open);
                _area_names.push_back(area.key());
                _area_nodes.push_back(area_node);
                // cout << "area name " << area.key() << " " <<  area_node._name << endl;
            }
        }
    }

    // build_quick_index();
    _scene_initialized = true;
    //calculate the navigation map
    _navigation_map = computeShortestPaths(j, _area_names);
    _area_num = _area_names.size();
    return true;
}

bool SceneGraph::LoadProblemJson(const string &file_content)
{
    if (!_scene_initialized)
    {
        throw SceneNotInitializedException();
    }
    if (_problem_initialized)
    {
        throw ProblemAlreadyInitializedException();
    }

    json j = json::parse(file_content);
    // load object placement
    for (auto placement = j["placement"].begin();
         placement != j["placement"].end(); placement++)
    {
        auto area_name = placement.value().at("area").get<string>();
        auto object_name = placement.value().at("placed_object").get<string>();
        // add object to area
        // auto area = _map_name_area[area_name];
        // auto area = GetArea(area_name);
        // CHECK_NULL_NODE(area, area_name);
        ObjectNode object(object_name);
        _object_names.push_back(object_name);
        _object_nodes.push_back(object);
        int area_index = distance(_area_names.begin(),
                                       find(_area_names.begin(), _area_names.end(), area_name));

        if (area_index > _area_names.size() - 1)
        {
            throw runtime_error("Area " + area_name + " does not exist");
        }
        _objects_in_areas.push_back(make_pair(_object_names.size() - 1, area_index));
    }

    auto robot_js = j["robot"];
    auto robot_location_str = robot_js["location"].get<string>();
    auto robot_obj_in_hand_str = robot_js["object_in_hand"].get<string>();
    auto robot_location = GetArea(robot_location_str);

    // check if robot has a valid location
    CHECK_NULL_NODE(robot_location, robot_location_str);
    // _robot = dynamic_cast<Robot *>(AllocNode("robot", NODE_ROBOT));
    // _robot = dynamic_pointer_cast<Robot>(AllocNode("robot", NODE_ROBOT));
    // Node::LinkNodes(robot_location, _robot);
    // _map_name_node[_robot->GetName()] = _robot;

    _robot_location = distance(_area_names.begin(),
                                    find(_area_names.begin(), _area_names.end(), robot_location_str));
    // set object_in_hand as child of robot
    if (robot_obj_in_hand_str != "")
    {
        // auto object_in_hand = GetObject(robot_obj_in_hand_str);
        // CHECK_NULL_NODE(object_in_hand, robot_obj_in_hand_str);
        ObjectNode object_in_hand(robot_obj_in_hand_str);
        _object_names.push_back(robot_obj_in_hand_str);
        _object_nodes.push_back(object_in_hand);
        _object_in_robot_hand = _object_names.size() - 1;
    }

    _problem_initialized = true;
    return true;
}

ostream &operator<<(ostream &os, const SceneGraph &graph)
{
    // unsigned long indent = 0;
    // function<void(const Node &)> print_node = [&](const Node &node)
    // {
    //     for (unsigned long i = 0; i < indent; i++)
    //         os << "  ";
    //     os << node << endl;
    //     indent++;
    //     // iterate use node.children_begin(), node.children_end()
    //     for (auto child : node.GetChildren())
    //     {
    //         print_node(*child);
    //     }
    //     indent--;
    // };
    // print_node(*graph._root);
    // return os;

    // print all areas
    os << "Areas: " << endl;
    for (const auto &area : graph._area_nodes)
    {
        os << area << endl;
    }

    // print all objects
    os << "Objects: " << endl;
    for (const auto &object : graph._object_nodes)
    {
        os << object << endl;
    }

    // print robot
    os << "Robot: " << endl;
    os << graph.GetRobot() << endl;

    // todo: print all relationships

    return os;
}

unique_ptr<SceneGraph> SceneGraph::Copy(bool with_invisible) const
{
    // auto result = make_shared<SceneGraph>();
    auto result = make_unique<SceneGraph>();
    // result->_root = _root->Copy(result->_all_nodes, with_invisible);
    // result->build_quick_index();
    // return result;

    // deep copy all areas
    result->_area_names = _area_names;
    // result->_area_nodes = _area_nodes;
    for (int i = 0; i < _area_nodes.size(); i++)
    {
        AreaNode area = _area_nodes[i];
        result->_area_nodes.push_back(area);
        // cout << "area name " << _area_nodes[i]._name << " " << area._name << endl;
    }
    // deep copy all objects
    if (with_invisible)
    {
        result->_object_names = _object_names;

        //copy one by one
        for (int i = 0; i < _object_nodes.size(); i++)
        {
            ObjectNode object = _object_nodes[i];
            result->_object_nodes.push_back(object);
        }

        // result->_object_nodes = _object_nodes;
        result->_objects_in_areas = _objects_in_areas;
        result->_object_in_robot_hand = _object_in_robot_hand;
        result->_robot = _robot;
        result->_robot_location = _robot_location;
        // result_->

    }
    else
    {
        // only copy visible objects
        // for (const auto &area_of_object_pair : _objects_in_areas)

        // cout << "object num" << _object_names.size() << " "<< _object_nodes.size() <<endl;
        // cout << "area num" << _area_names.size()<< " " << _area_nodes.size() << endl;

        for (int i = 0; i < _objects_in_areas.size(); i++)
        {
            // cout<< _objects_in_areas[i].first <<" "<< _objects_in_areas[i].second << endl;
            // cout << _area_names[area_of_object_pair.second] << endl;

            string area_name = _area_names[_objects_in_areas[i].second];
            AreaNode area = _area_nodes[_objects_in_areas[i].second];
            // cout << "area name " << area_name << " " << area._name << endl;

            if (area._is_open)
            {
                // cout << "copy object name" << _object_names[_objects_in_areas[i].first] << endl;
                result->_object_names.push_back(_object_names[_objects_in_areas[i].first]);
                // cout << "copy object node" <<endl;
                result->_object_nodes.push_back(_object_nodes[_objects_in_areas[i].first]);
                // cout << "copy index" <<endl;
                result->_objects_in_areas.push_back(make_pair(result->_object_names.size() - 1, _objects_in_areas[i].second));
                // cout << "copy done" <<endl;
            }
        }
        // deep copy robot
        // cout << "copy robot" << endl;
        result->_robot_location = _robot_location;
        string object_in_robot_hand = _object_in_robot_hand == -1 ? "" : _object_names[_object_in_robot_hand];
        if (object_in_robot_hand != "")
        {
            result->_object_names.push_back(object_in_robot_hand);
            ObjectNode object_in_hand_node = _object_nodes[_object_in_robot_hand];
            result->_object_nodes.push_back(object_in_hand_node);
            result->_object_in_robot_hand = result->_object_names.size() - 1;
        }
        else{
            result->_object_in_robot_hand = -1;
        }
    }
    // cout << "copy done" << endl;
    result->_scene_initialized = _scene_initialized;
    result->_problem_initialized = _problem_initialized;

    // copy navigation map
    result->_navigation_map = _navigation_map;
    result->_area_num = _area_num;

    return result;
}

// shared_ptr<Node> SceneGraph::Root() const { return _root; };
const Node* SceneGraph::GetNode(const string &name) const
{
    // find the node index in the vector
    auto area_iter = find(_area_names.begin(), _area_names.end(), name);
    if (area_iter != _area_names.end())
    {
        int index = distance(_area_names.begin(), area_iter);
        return &_area_nodes[index];
    }

    auto object_iter = find(_object_names.begin(), _object_names.end(), name);
    if (object_iter != _object_names.end())
    {
        int index = distance(_object_names.begin(), object_iter);
        ObjectNode* node = const_cast<ObjectNode*>(&_object_nodes[index]);
        return node;
    }
    return nullptr;
}

vector<AreaNode> SceneGraph::all_areas() const
{
    // cout << "area num" << _area_nodes.size() << endl;
    vector<AreaNode> area_nodes_new;
    // for (const auto &area : _area_nodes)
    for (int i = 0; i < _area_nodes.size(); i++)
    {
        // cout << "area name " <<  _area_nodes[i]._name << endl;
        AreaNode area = _area_nodes[i];
        // cout << "area name " << _area_nodes[i]._name << " " << area._name << endl;
        area_nodes_new.push_back(area);
    }
    // cout<< "area num" << area_nodes_new.size() << endl;
    return area_nodes_new;
}

vector<ObjectNode> SceneGraph::all_objects() const
{
    return _object_nodes;
}

const AreaNode* SceneGraph::GetArea(const string &name) const
{

    auto area_iter = find(_area_names.begin(), _area_names.end(), name);
    if (area_iter != _area_names.end())
    {
        int index = distance(_area_names.begin(), area_iter);
        return &_area_nodes[index];
    }
    else
    {
        return nullptr;
    }
}
const ObjectNode* SceneGraph::GetObject(const string &name) const
{

    auto object_iter = find(_object_names.begin(), _object_names.end(), name);
    if (object_iter != _object_names.end())
    {
        int index = distance(_object_names.begin(), object_iter);
        return &_object_nodes[index];
    }
    else
    {
        return nullptr;
    }
}

const Robot* SceneGraph::GetRobot() const
{
    return &_robot;
}

ActionResult SceneGraph::Forward(const Action &action)
{
    ActionResult result;
    switch (action.Type)
    {
    case ActionType::ACTION_PICK:
        TakeActionImpl_Pick(action.Args, result);
        break;
    case ActionType::ACTION_PLACE:
        TakeActionImpl_Place(action.Args, result);
        break;
    case ActionType::ACTION_OPEN:
        TakeActionImpl_Open(action.Args, result);
        break;
    case ActionType::ACTION_NONE:
        result._is_success = true;
        result._feedback_message = "No action";
        result._step_cost = COST_OTHERS;
        break;
    case ActionType::ACTION_DONE:
        result._is_success = true;
        result._feedback_message = "Declare done";
        result._step_cost = COST_OTHERS;
        break;
    default:
        throw runtime_error("Unknown action type");
        break;
    }
    return result;
}

const AreaNode *SceneGraph::GetRobotLocation()
{
    if (_robot_location == -1)
    {
        return nullptr;
    }
    return &_area_nodes[_robot_location];
}

vector<Action> SceneGraph::GetLegalActions()
{
    vector<Action> legal_actions;
    const AreaNode *robot_location = GetRobotLocation();
    if (robot_location == nullptr)
    {
        cerr << "Robot has no location" << endl;
        return legal_actions;
    }

    // add all open action
    for (int area_index = 0; area_index < _area_nodes.size(); area_index++)
    {
        if (!_area_nodes[area_index]._is_open)
        {
            legal_actions.push_back(Action(ActionType::ACTION_OPEN, {_area_nodes[area_index]._name}));
        }
    }

    if (_object_in_robot_hand == -1)
    {
        // add all pick action
        for (int pair_index = 0; pair_index < _objects_in_areas.size(); pair_index++)
        {
            //if area is open
            if (_area_nodes[_objects_in_areas[pair_index].second]._is_open)
            {
                legal_actions.push_back(Action(ActionType::ACTION_PICK,
                                               {_area_nodes[_objects_in_areas[pair_index].second]._name,
                                                _object_names[_objects_in_areas[pair_index].first]}));
            }
        }
    }
    else
    {
        // add all place action
        for (int area_index = 0; area_index < _area_nodes.size(); area_index++)
        {
            if (_area_nodes[area_index]._is_open)
            {
                legal_actions.push_back(Action(ActionType::ACTION_PLACE, {_area_nodes[area_index]._name}));
            }
        }
    }

    return legal_actions;

    //if no objects in hand: add all open actions and all pick actions
    // if (_object_in_robot_hand == -1)
    // {
    //     for (int area_index = 0; area_index < _area_nodes.size(); area_index++)
    //     {
    //         if (!_area_nodes[area_index]._is_open)
    //         {
    //             legal_actions.push_back(Action(ActionType::ACTION_OPEN, {_area_nodes[area_index]._name}));
    //         }
    //     }
        
    //     // add all pick action
    //     for (int pair_index = 0; pair_index < _objects_in_areas.size(); pair_index++)
    //     {
    //         //if area is open
    //         if (_area_nodes[_objects_in_areas[pair_index].second]._is_open)
    //         {
    //             legal_actions.push_back(Action(ActionType::ACTION_PICK,
    //                                            {_area_nodes[_objects_in_areas[pair_index].second]._name,
    //                                             _object_names[_objects_in_areas[pair_index].first]}));
    //         }
    //     }

    // }
    // else
    // {
    //     // add all place action
    //     for (int area_index = 0; area_index < _area_nodes.size(); area_index++)
    //     {
    //         if (_area_nodes[area_index]._is_open)
    //         {
    //             legal_actions.push_back(Action(ActionType::ACTION_PLACE, {_area_nodes[area_index]._name}));
    //         }
    //     }
    // }
}

void SceneGraph::TakeActionImpl_Open(const vector<string> &args,
                                     ActionResult &result)
{
    string area_name = args[0];
    // PoseNode *robot_location = _robot->GetLocation();
    // shared_ptr<AreaNode> robot_location = _robot->GetLocation();
    const AreaNode *robot_location = GetRobotLocation();
    // check if robot has a valid location
    if (robot_location == nullptr)
    {
        result._is_success = false;
        result._feedback_message = "Robot has no location";
        result._step_cost = COST_OTHERS;
        return;
    }

    //check if robot is holding an object
    // if (_object_in_robot_hand != -1)
    // {
    //     result._is_success = false;
    //     result._feedback_message = "Robot's hand is not free";
    //     result._step_cost = COST_OTHERS;
    //     return;
    // }

    // check if area is open
    // AreaNode *area_node = GetArea(area_name);
    int area_index = distance(_area_names.begin(),
                                   find(_area_names.begin(), _area_names.end(), area_name));
    AreaNode* area_node = &_area_nodes[area_index];
    if (area_node == nullptr)
    {
        // return make_shared<ActionResult>(false, "Area " + area_name + " does
        // not exist", 0);
        result._is_success = false;
        result._feedback_message = "Area " + area_name + " does not exist";
        result._step_cost = COST_OTHERS;
        return;
    }

    if (area_node->IsOpen())
    {
        // return make_shared<ActionResult>(false, area_name + " is already
        // open", 0);
        result._is_success = false;
        result._feedback_message = area_name + " is already open";
        result._step_cost = COST_OTHERS;
        return;
    }

    // open the area
    int robot_old_location_index = _robot_location;
    SetRobotLocation(area_name);
    int robot_new_location_index = _robot_location;

    double distance_cost = _navigation_map[robot_old_location_index * _area_num + robot_new_location_index];

    area_node->Open();
    // return make_shared<ActionResult>(true, "Robot opened " + area_name,
    // COST_OPEN);
    result._is_success = true;
    result._feedback_message = "Robot opened " + area_name;
    result._step_cost = distance_cost+COST_OPEN;
    return;
}

bool SceneGraph::CheckObjectInArea(const string &object_name, const string &area_name)
{
    for (const auto &object_area_pair : _objects_in_areas)
    {
        if (_object_names[object_area_pair.first] == object_name && _area_names[object_area_pair.second] == area_name)
        {
            return true;
        }
    }
    return false;
}

void SceneGraph::RemoveObjectFromArea(const string &object_name, const string &area_name)
{
    for (int i = 0; i < _objects_in_areas.size(); i++)
    {
        if (_object_names[_objects_in_areas[i].first] == object_name && _area_names[_objects_in_areas[i].second] == area_name)
        {
            _objects_in_areas.erase(_objects_in_areas.begin() + i);
            // _object_names.erase(_object_names.begin() + i);
            break;
        }
    }
}

void SceneGraph::TakeActionImpl_Pick(const vector<string> &args,
                                     ActionResult &result)
{
    // cerr << "DEBUG Pick action: " << args[0] << "\t" << args[1] << endl;
    string area_name = args[0];
    string obj_name = args[1];
    // PoseNode *robot_location = _robot->GetLocation();
    // shared_ptr<AreaNode> robot_location = _robot->GetLocation();
    const AreaNode *robot_location = GetRobotLocation();
    // check if robot has a valid location
    if (robot_location == nullptr)
    {
        result._is_success = false;
        result._feedback_message = "Robot has no location";
        result._step_cost = COST_OTHERS;
        return;
    }
    // cerr << "DEBUG Pick action robot at : " << *robot_location << endl;
    // check if robot is hand free
    if (_object_in_robot_hand != -1)
    {
        result._is_success = false;
        result._feedback_message = "Robot's hand is not free";
        result._step_cost = COST_OTHERS;
        return;
    }

    // check if area is open
    // AreaNode *area_node = GetArea(area_name);
    // shared_ptr<AreaNode> area_node = GetArea(area_name);
    int area_index = distance(_area_names.begin(),
                                   find(_area_names.begin(), _area_names.end(), area_name));
    AreaNode* area_node = &_area_nodes[area_index];

    if (area_node == nullptr)
    {
        // return make_shared<ActionResult>(false, "Area " + area_name + " does
        // not exist", 0);
        result._is_success = false;
        result._feedback_message = "Area " + area_name + " does not exist";
        result._step_cost = COST_OTHERS;
        return;
    }
    // cerr << "DEBUG Pick action find area : " << *area_node << endl;
    if (!area_node->IsOpen())
    {
        // return make_shared<ActionResult>(false, area_name + " is not open",
        // 0);
        result._is_success = false;
        result._feedback_message = area_name + " is not open";
        result._step_cost = COST_OTHERS;
        return;
    }

    // check if object is in area
    // Node *target_object = area_node->GetObject(obj_name);
    // shared_ptr<Node> target_object = area_node->GetChild(obj_name);
    if (CheckObjectInArea(obj_name, area_name) == false)
    {
        // return make_shared<ActionResult>(false, obj_name + " does not exist
        // in " + area_name, 0);
        result._is_success = false;
        result._feedback_message = obj_name + " does not exist in " + area_name;
        result._step_cost = COST_OTHERS;
        return;
    }

    // pick up the object
    // _robot->AddChild(dynamic_pointer_cast<ObjectNode>(target_object));
    // target_object->ResetParent();
    // Node::LinkNodes(_robot, target_object);

    int robot_old_location_index = _robot_location;
    SetRobotLocation(area_name);
    int robot_new_location_index = _robot_location;

    double distance_cost = _navigation_map[robot_old_location_index * _area_num + robot_new_location_index];

    // Node::LinkNodes(_robot, target_object);
    _object_in_robot_hand = distance(_object_names.begin(),
                                          find(_object_names.begin(), _object_names.end(), obj_name));

    // remove object from area
    // area_node->RemoveChild(target_object);
    RemoveObjectFromArea(obj_name, area_name);

    // return make_shared<ActionResult>(true, "Robot picked up " + obj_name,
    // COST_PICK);
    result._is_success = true;
    result._feedback_message = "Robot picked up " + obj_name;
    result._step_cost = distance_cost + COST_PICK;
    return;
}

void SceneGraph::TakeActionImpl_Place(const vector<string> &args,
                                      ActionResult &result)
{
    string area_name = args[0];
    // PoseNode *robot_location = _robot->GetLocation();
    //   shared_ptr<AreaNode> robot_location = _robot->GetLocation();
    const AreaNode *robot_location = GetRobotLocation();
    // check if robot has a valid location
    if (robot_location == nullptr)
    {
        result._is_success = false;
        result._feedback_message = "Robot has no location";
        result._step_cost = COST_OTHERS;
        return;
    }

    // check if robot is holding an object
    if (_object_in_robot_hand == -1)
    {
        // return make_shared<ActionResult>(false, "Robot is not holding any
        // object", 0);
        result._is_success = false;
        result._feedback_message = "Robot is not holding any object";
        result._step_cost = COST_OTHERS;
        return;
    }

    // check if area is open
    // AreaNode *area_node = GetArea(area_name);
    // shared_ptr<AreaNode> area_node = GetArea(area_name);

    int area_index = distance(_area_names.begin(),
                                   find(_area_names.begin(), _area_names.end(), area_name));
    AreaNode* area_node = &_area_nodes[area_index];

    if (area_node == nullptr)
    {
        // return make_shared<ActionResult>(false, "Area " + area_name + " does
        // not exist", 0);
        result._is_success = false;
        result._feedback_message = "Area " + area_name + " does not exist";
        result._step_cost = COST_OTHERS;
        return;
    }
    if (!area_node->IsOpen())
    {
        // return make_shared<ActionResult>(false, area_name + " is not open",
        // 0);
        result._is_success = false;
        result._feedback_message = area_name + " is not open";
        result._step_cost = COST_OTHERS;
        return;
    }

    // place the object
    int robot_old_location_index = _robot_location;
    SetRobotLocation(area_name);
    int robot_new_location_index = _robot_location;

    double distance_cost = _navigation_map[robot_old_location_index * _area_num + robot_new_location_index];

    _objects_in_areas.push_back(make_pair(_object_in_robot_hand, area_index));

    // return make_shared<ActionResult>(true, "Robot placed " +
    // target_object->GetName() + " in " + area_name, COST_PLACE);
    result._is_success = true;
    result._feedback_message =
        "Robot placed " + _object_names[_object_in_robot_hand] + " in " + area_name;
    result._step_cost = distance_cost+COST_PLACE;

    _object_in_robot_hand = -1;

    return;
}

bool SceneGraph::GoalTest(const Task &task)
{
    // iterate all goals in task

    vector<vector<pair<string, string>>> goals = task._area_has_obj_list;
    if (goals.size() != 0)
    {
        for (auto goal : goals)
        {
            bool goal_test = true;
            for (auto area_obj = goal.begin(); area_obj != goal.end(); area_obj++)
            {
                string area_name = area_obj->first;
                string obj_name = area_obj->second;

                if (CheckObjectInArea(obj_name, area_name) == false)
                {
                    goal_test = false;
                    break;
                }
            }
            if (goal_test)
            {
                return true;
            }
        }
        return false;
    }
    else
    {
        
        vector<pair<string, string>> goal = task._area_has_obj;
        // for (auto area_obj = task.begin(); area_obj != task.end(); area_obj++)
        for (auto area_obj : goal)
        {
            string area_name = area_obj.first;
            string obj_name = area_obj.second;
            // get area node
            if(CheckObjectInArea(obj_name, area_name) == false)
            {
                return false;
            }
        }
        return true;
    }
}

vector<string> SceneGraph::ObjectsReachGoal(const Task &task){
    //get all objects in the scenegraph and their target areas

    vector<string> objects_in_goal;
    vector<vector<pair<string, string>>> goals = task._area_has_obj_list;
    if (goals.size() != 0)
    {
        for (auto goal : goals)
        {
            bool goal_test = true;
            for (auto area_obj = goal.begin(); area_obj != goal.end(); area_obj++)
            {
                string area_name = area_obj->first;
                string obj_name = area_obj->second;

                if (CheckObjectInArea(obj_name, area_name) == true)
                {
                    //find obj_name in objects_in_goal
                    auto it = find(objects_in_goal.begin(), objects_in_goal.end(), obj_name);
                    if (it == objects_in_goal.end())
                    {
                        // cout<<"obj_name: "<<obj_name<<endl;
                        objects_in_goal.push_back(obj_name);
                    }
                }
            }
        }
        return objects_in_goal;
    }
    return objects_in_goal;

}

string SceneGraph::ToJsonString() const
{
    json j;
    
    for (int i = 0;i < _area_nodes.size();i++)
    {
        auto area = _area_nodes[i];
        j["room"][area.GetName()]["is_open"] = area.IsOpen();
        for (const auto &object_area_pair : _objects_in_areas)
        {
            if (object_area_pair.second == i)
            {
                j["placement"].push_back({{"area", area.GetName()}, {"placed_object", _object_names[object_area_pair.first]}});
            }
        }
    }

    j["robot"]["location"] = _area_names[_robot_location];
    if (_object_in_robot_hand != -1)
    {
        j["robot"]["object_in_hand"] = _object_names[_object_in_robot_hand];
    }

    return j.dump(4);
}

// bool SceneGraph::IsInFrontOf(const AreaNode &area) const {
//   auto robot_location = _robot->GetLocation();
//   return area.GetName() == robot_location->GetName();
// }
// bool SceneGraph::IsGraspable(const ObjectNode &object) const
// {
//     // return object.GetParent() != nullptr && IsInFrontOf(*dynamic_cast<AreaNode
//     // *>(object.GetParent()));
//     return object.GetParent() != nullptr &&
//            object.GetParent()->IsOpen();
// }

bool SceneGraph::AddObjectInArea(const string &area_name,
                                 const string &object_name)
{
    int area_index = distance(_area_names.begin(),
                                   find(_area_names.begin(), _area_names.end(), area_name));
    if (area_index == _area_names.size())
    {
        cerr << "Area " << area_name << " does not exist" << endl;
        return false;
    }
    // int object_index = distance(_object_names.begin(),
    //                                  find(_object_names.begin(), _object_names.end(), object_name));
    // if(object_index != _object_names.size())
    // {
    //     cerr << "Object " << object_name << " already exists" << endl;
    //     return false;
    // }

    _object_names.push_back(object_name);
    _object_nodes.push_back(ObjectNode(object_name));
    _objects_in_areas.push_back(make_pair(_object_names.size() - 1, area_index));
    return true;
}

bool SceneGraph::AddObjectInHand(const string &object_name)
{
    if (_object_in_robot_hand != -1)
    {
        cerr << "Robot's hand is not free" << endl;
        return false;
    }

    _object_names.push_back(object_name);
    _object_nodes.push_back(ObjectNode(object_name));
    _object_in_robot_hand = _object_names.size() - 1;

    return true;
}

bool SceneGraph::SetRobotLocation(const string &area_name)
{
    int area_index = distance(_area_names.begin(),
                                   find(_area_names.begin(), _area_names.end(), area_name));
    if (area_index == _area_names.size())
    {
        cerr << "Area " << area_name << " does not exist" << endl;
        return false;
    }
    _robot_location = area_index;

    return true;
}

bool SceneGraph::SetAreaIsOpen(const string &area_name, bool is_open)
{
    int area_index = distance(_area_names.begin(),
                                   find(_area_names.begin(), _area_names.end(), area_name));
    if (area_index == _area_names.size())
    // if (area == nullptr)
    {
        cerr << "Area " << area_name << " does not exist" << endl;
        return false;
    }
    _area_nodes[area_index]._is_open = is_open;
    // area->_is_open = is_open;
    return true;
}

bool SceneGraph::RemoveObject(const string &object_name)
{

    for (int i = 0; i < _objects_in_areas.size(); i++)
    {
        if (_object_names[_objects_in_areas[i].first] == object_name)
        {
            _objects_in_areas.erase(_objects_in_areas.begin() + i);
            _object_names.erase(_object_names.begin() + _objects_in_areas[i].first);
            _object_nodes.erase(_object_nodes.begin() + _objects_in_areas[i].first);
            return true;
        }
    }

    cerr << "Object " << object_name << " does not exist" << endl;

    return false;
}

const Node* SceneGraph::GetObjectParentNode(const string &object_name)
{
    for (const auto &object_area_pair : _objects_in_areas)
    {
        if (_object_names[object_area_pair.first] == object_name)
        {
            return &_area_nodes[object_area_pair.second];
        }
    }
    if(_object_in_robot_hand != -1 && _object_names[_object_in_robot_hand] == object_name)
    {
        return &_robot;
    }
    return nullptr;
}
const ObjectNode* SceneGraph::GetObjectInHand()
{
    if (_object_in_robot_hand == -1)
    {
        return nullptr;
    }
    ObjectNode object_in_hand = _object_nodes[_object_in_robot_hand];
    // cout<<"object in hand: "<<object_in_hand<<endl;
    return &_object_nodes[_object_in_robot_hand];
}

const vector<ObjectNode*> SceneGraph::GetObjectsInArea(const string &area_name)
{
    vector<ObjectNode*> objects;
    for (const auto &object_area_pair : _objects_in_areas)
    {
        if (_area_names[object_area_pair.second] == area_name)
        {
            objects.push_back(&_object_nodes[object_area_pair.first]);
        }
    }
    return objects;
}


bool SceneGraph::CheckObjectsExistsAndAreasOpen(const vector<string> &object_names)
{
    for (const string &object_name : object_names)
    {
        if (GetObject(object_name) == nullptr)
        {
            cerr << "Object " << object_name << " does not exist" << endl;
            return false;
        }
    }
    for (const string &object_name : object_names)
    {
        const Node* parent_node = GetObjectParentNode(object_name);
        if (parent_node == nullptr)
        {
            cerr << "Object " << object_name << " does not have a parent" << endl;
            return false;
        }
        if (parent_node->Type != NODE_AREA)
        {
            cerr << "Object " << object_name << " is not in an area" << endl;
            return false;
        }
        const AreaNode* area_node = dynamic_cast<const AreaNode*>(parent_node);
        if (!area_node->IsOpen())
        {
            cerr << "Area " << area_node->GetName() << " is not open" << endl;
            return false;
        }

    }
    return true;
}

/*********************************
 * SceneGraphSimple
*********************************/

SceneGraphSimple::SceneGraphSimple()
{
    //set all ids to -1
    // _area_ids.fill(-1);
    // _object_ids.fill(-1);
    // _area_opens.fill(-1);
    // _objects_in_areas.fill(-1);
    // _navigation_map.fill(-1);
    // _robot_location = -1;
    // _area_num = 0;
}

SceneGraphSimple::~SceneGraphSimple()
{
}

SceneGraphSimple SceneGraphSimple::Copy(bool with_invisible) const{
    SceneGraphSimple result;
    if (with_invisible)
    {
        memcpy(result._area_ids.data(), _area_ids.data(), 50 * sizeof(int));
        memcpy(result._object_ids.data(), _object_ids.data(), 50 * sizeof(int));
        memcpy(result._area_opens.data(), _area_opens.data(), 50 * sizeof(int));
        memcpy(result._objects_in_areas.data(), _objects_in_areas.data(), 50 * sizeof(int));
        result._robot_location = _robot_location;
        // result._object_in_robot_hand = _object_in_robot_hand;
        memcpy(result._navigation_map.data(), _navigation_map.data(), 2500 * sizeof(int));
        result._area_num = _area_num;
        result._object_num = _object_num;
        return result; // Fix: Add a return statement
    }
    else
    {
        memcpy(result._area_ids.data(), _area_ids.data(), 50 * sizeof(int));
        memcpy(result._area_opens.data(), _area_opens.data(), 50 * sizeof(int));
        result._robot_location = _robot_location;
        result._area_num = _area_num;
        memcpy(result._navigation_map.data(), _navigation_map.data(), 2500 * sizeof(int));

        int count = 0;
        for (int i = 0; i < _object_num; i++)
        {
            if (_object_ids[i] == -1)
            {
                break;
            }
            else
            {   
                int area_id = _objects_in_areas[i];
                int area_index = distance(_area_ids.begin(), find(_area_ids.begin(), _area_ids.begin()+_area_num, area_id));
                bool is_open;
                if (area_id == 50) // robot
                {
                    is_open = true;
                }
                else
                {
                    is_open = _area_opens[area_index];
                }
                if (is_open)
                {
                    result._object_ids[count] = _object_ids[i];
                    result._objects_in_areas[count] = area_index;
                    count++;
                }
            }
        }
        result._object_num = count; 
        return result;
    }
}

SceneGraphSimple SceneGraphSimple::CopyFromSceneGraph(unique_ptr<SceneGraph> scene_graph, vector<string> area_names, vector<string> object_names) const
{
    SceneGraphSimple result;


    for (int i = 0; i < scene_graph->_area_names.size(); i++)
    {
        string area_name = scene_graph->_area_names[i];
        bool is_open = scene_graph->_area_nodes[i].IsOpen();
        int area_id = distance(area_names.begin(),
                                   find(area_names.begin(), area_names.end(), scene_graph->_area_names[i]));
        
        result._area_ids[i] = area_id;
        result._area_opens[i] = scene_graph->_area_nodes[i].IsOpen();
    }
    // result._area_num = scene_graph->_area_names.size();


    // copy object names
    int count_object = 0;
    for (const string &object_name : scene_graph->_object_names)
    {
        int object_id = distance(object_names.begin(),
                                   find(object_names.begin(), object_names.end(), object_name));
        if (object_id != object_names.size()){
            result._object_ids[count_object] = object_id;
            count_object++;
        }
        else{
            cerr << "Object " << object_name << " does not exist" << endl;
        }
    }
    result._object_num = count_object;

    // copy objects in areas

    for (const auto &object_area_pair : scene_graph->_objects_in_areas)
    {
        string object_name = scene_graph->_object_names[object_area_pair.first];
        string area_name = scene_graph->_area_names[object_area_pair.second];
        int object_id = distance(object_names.begin(),
                                   find(object_names.begin(), object_names.end(), object_name));
        int object_index = distance(result._object_ids.begin(), find(result._object_ids.begin(), result._object_ids.end(), object_id));
        int area_id = distance(area_names.begin(),
                                      find(area_names.begin(), area_names.end(), area_name));
        int area_index = distance(result._area_ids.begin(), find(result._area_ids.begin(), result._area_ids.end(), area_id));
                       
        if (object_id != object_names.size() && area_id != area_names.size()){
            result._objects_in_areas[object_index] = area_id;
        }
        else{
            cerr << "Object " << object_name << " or Area " << area_name << " does not exist" << endl;
        }
    }

    // copy robot location
    result._robot_location = scene_graph->_robot_location;

    // copy object in robot hand
    if (scene_graph->_object_in_robot_hand != -1)
    {
        string object_name = scene_graph->_object_names[scene_graph->_object_in_robot_hand];
        int object_id = distance(object_names.begin(),
                                   find(object_names.begin(), object_names.end(), object_name));
        // int object_index = distance(result._object_ids.begin(), find(result._object_ids.begin(), result._object_ids.begin()+_object_num, object_id));
        int object_index;
        for (int i = 0; i < result._object_num; i++)
        {
            if (result._object_ids[i] == object_id)
            {
                object_index = i;
                break;
            }
        }
        if (object_id != object_names.size())
        {
            result._objects_in_areas[object_index] = 50;//robot
        }
        else{
            cerr << "Object " << object_name << " does not exist" << endl;
        }
    }

    // copy navigation map
    for (int i = 0; i < scene_graph->_area_num; i++)
    {
        for (int j = 0; j < scene_graph->_area_num; j++)
        {
            result._navigation_map[i * scene_graph->_area_num + j] = scene_graph->_navigation_map[i * scene_graph->_area_num + j];
        }
    }
    result._area_num = scene_graph->_area_num;

    return result;
}

const vector<string> SceneGraphSimple::all_areas(vector<string> area_names) const
{
    vector<string> result;
    for (int i = 0; i < _area_num; i++)
    {
        if (_area_ids[i] != -1)
        {
            result.push_back(area_names[_area_ids[i]]);
        }
    }
    return result;
}

const vector<string> SceneGraphSimple::all_objects(vector<string> object_names) const
{
    vector<string> result;
    for (int i = 0; i < _object_num; i++)
    {
        if (_object_ids[i] != -1)
        {
            result.push_back(object_names[_object_ids[i]]);
        }
    }
    return result;
}

const pair<string, bool> SceneGraphSimple::GetArea(int index, vector<string> area_names) const
{
    if (index == -1)
    {
        return make_pair("", false);
    }
    string area_name = area_names[index];
    bool is_open = _area_opens[index];
    return make_pair(area_name, is_open);
}

const string SceneGraphSimple::GetObject(int index, vector<string> object_names) const
{
    if (index == -1)
    {
        return "";
    }
    return object_names[_object_ids[index]];
}

const bool SceneGraphSimple::GetAreaOpen(int index) const
{
    return _area_opens[index];
}

const bool SceneGraphSimple::GetAreaOpenFromId(int id) const
{
    int index = distance(_area_ids.begin(), find(_area_ids.begin(), _area_ids.begin()+_area_num, id));
    return _area_opens[index];
}

const bool SceneGraphSimple::GetAreaOpenFromName(const string &name, vector<string> area_names) const
{
    int id = distance(area_names.begin(),
                                   find(area_names.begin(), area_names.end(), name));
    int index = distance(_area_ids.begin(), find(_area_ids.begin(), _area_ids.begin()+_area_num, id));
    return _area_opens[index];
}

ActionResult SceneGraphSimple::Forward(const ActionSimple &action)
{
    ActionResult result;
    switch (action.Type())
    {
    case ActionType::ACTION_PICK:
        TakeActionImpl_Pick(action, result);
        break;
    case ActionType::ACTION_PLACE:
        TakeActionImpl_Place(action, result);
        break;
    case ActionType::ACTION_OPEN:
        TakeActionImpl_Open(action, result);
        break;
    case ActionType::ACTION_NONE:
        result._is_success = true;
        result._feedback_message = "No action";
        result._step_cost = COST_OTHERS;
        break;
    case ActionType::ACTION_DONE:
        result._is_success = true;
        result._feedback_message = "Declare done";
        result._step_cost = COST_OTHERS;
        break;
    }
    return result;
}

pair<array<ActionSimple, 200>, int> SceneGraphSimple::GetLegalActions()
{
    // vector<ActionSimple> legal_actions;
    // legal_actions.reserve(_area_num * 5); // Preallocate memory based on maximum possible actions
    array<ActionSimple, 200> legal_actions;

    if (_robot_location == -1)
    {
        cerr << "Robot has no location" << endl;
        return make_pair(legal_actions, 0);
    }

    // Add all open actions
    int count = 0;
    for (int i = 0; i < _area_num; ++i)
    {
        if (!_area_opens[i])
        {
            // legal_actions.emplace_back(ActionType::ACTION_OPEN, _area_ids[i], -1);
            int args[3] = {_area_ids[i], -1, ActionType::ACTION_OPEN};
            ActionSimple action(args);
            legal_actions[count] = action;
            count++;
        }
    }
}
void SceneGraphSimple::GetLegalActions(unique_ptr<array<ActionSimple, 200>> &legal_actions, int &count)
{
    // 清空 legal actions
    count = 1;

    // 提前检查机器人的位置
    if (_robot_location == -1)
    {
        cerr << "Robot has no location" << endl;
        return;
    }

    // 添加所有打开操作
    for (int i = 0; i < _area_num; ++i)
    {
        if (!_area_opens[i])
        {
            int args[3] = {_area_ids[i], -1, ActionType::ACTION_OPEN};
            (*legal_actions)[count] = ActionSimple(args);
            count++;
        }
    }

    // 通过局部缓存来避免重复查找
    bool isRobotHoldingObject = find(_objects_in_areas.begin(), _objects_in_areas.begin() + _object_num, 50) != _objects_in_areas.begin() + _object_num;

    if (!isRobotHoldingObject)
    {
        // 添加所有拾取操作
        for (int i = 0; i < _object_num; ++i)
        {
            int area_id = _objects_in_areas[i];
            int area_index = distance(_area_ids.begin(), find(_area_ids.begin(), _area_ids.begin()+_area_num, area_id));
            if (_area_opens[area_index])
            {
                int args[3] = {area_id, _object_ids[i], ActionType::ACTION_PICK};
                (*legal_actions)[count] = ActionSimple(args);
                count++;
            }
        }
    }
    else
    {
        for (int i = 0; i < _area_num; ++i)
        {
            if (_area_opens[i])
            {
                if(_area_ids[i] ==_robot_location){
                    continue;
                }
                int args[3] = {_area_ids[i], -1, ActionType::ACTION_PLACE};
                (*legal_actions)[count] = ActionSimple(args);
                count++;
            }
        }
    }

    if (count <= 1){
        cerr<<"no legal actions"<<endl;
    }

    //if robot is not holding an object, add all open and pick actions
    //if robot is holding an object, add all place actions

    // bool isRobotHoldingObject = find(_objects_in_areas.begin(), _objects_in_areas.begin() + _object_num, 50) != _objects_in_areas.begin() + _object_num;
    // if (!isRobotHoldingObject)
    // {
    //     // Add all open actions
    //     for (int i = 0; i < _area_num; ++i)
    //     {
    //         if (!_area_opens[i])
    //         {
    //             int args[3] = {_area_ids[i], -1, ActionType::ACTION_OPEN};
    //             (*legal_actions)[count] = ActionSimple(args);
    //             count++;
    //         }
    //     }

    //     // Add all pick actions
    //     for (int i = 0; i < _object_num; ++i)
    //     {
    //         int area_id = _objects_in_areas[i];
    //         int area_index = distance(_area_ids.begin(), find(_area_ids.begin(), _area_ids.begin()+_area_num, area_id));
    //         if (_area_opens[area_index])
    //         {
    //             int args[3] = {area_id, _object_ids[i], ActionType::ACTION_PICK};
    //             (*legal_actions)[count] = ActionSimple(args);
    //             count++;
    //         }
    //     }
    // }
    // else
    // {
    //     // Add all place actions
    //     for (int i = 0; i < _area_num; ++i)
    //     {
    //         if (_area_opens[i])
    //         {
    //             if(_area_ids[i] ==_robot_location){
    //                 continue;
    //             }
    //             int args[3] = {_area_ids[i], -1, ActionType::ACTION_PLACE};
    //             (*legal_actions)[count] = ActionSimple(args);
    //             count++;
    //         }
    //     }
    // }
    // if (count <= 1){
    //     cerr<<"no legal actions"<<endl;

}

void SceneGraphSimple::TakeActionImpl_Open(const ActionSimple &action,
                                     ActionResult &result)
{

    // //check if robot is holding an object
    // if (find(_objects_in_areas.begin(), _objects_in_areas.begin()+_object_num, 50) != _objects_in_areas.begin()+_object_num)
    // {
    //     result._is_success = false;
    //     result._feedback_message = "Robot's hand is not free";
    //     result._step_cost = COST_OTHERS;
    //     return;
    // }

    int area_id= action.Args[0];
    int area_index = distance(_area_ids.begin(),
                                   find(_area_ids.begin(), _area_ids.begin()+_area_num, area_id));
    if (area_index == _area_num)
    {
        result._is_success = false;
        result._feedback_message = "Area does not exist";
        result._step_cost = COST_OTHERS;
        return;
    }
    if (_area_opens[area_index])
    {
        result._is_success = false;
        result._feedback_message = "Area is already open";
        result._step_cost = COST_OTHERS;
        return;
    }

    //move robot to area
    int old_robot_location = _robot_location;
    int old_robot_location_index = distance(_area_ids.begin(), find(_area_ids.begin(), _area_ids.begin()+_area_num, old_robot_location));

    _robot_location = area_id;
    double distance_cost = _navigation_map[old_robot_location_index * _area_num + area_index];

    _area_opens[area_index] = true;
    result._is_success = true;
    result._feedback_message = "Area is opened";
    result._step_cost = old_robot_location + COST_OPEN;
    return;
}

void SceneGraphSimple::TakeActionImpl_Pick(const ActionSimple &action,
                                     ActionResult &result)
{
    //find the id of area and object from _area_ids and _object_ids
    int area_id = action.Args[0];
    int object_id = action.Args[1];

    int area_index = distance(_area_ids.begin(),
                                   find(_area_ids.begin(), _area_ids.begin()+_area_num, area_id));
    int object_index = distance(_object_ids.begin(),
                                   find(_object_ids.begin(), _object_ids.begin()+_object_num, object_id));

    if (area_index == _area_num)
    {
        result._is_success = false;
        result._feedback_message = "Area does not exist";
        result._step_cost = COST_OTHERS;
        return;
    }
    if (object_index == _object_num)
    {
        result._is_success = false;
        result._feedback_message = "Object does not exist";
        result._step_cost = COST_OTHERS;
        return;
    }
    // robot id: 50
    if (find(_objects_in_areas.begin(), _objects_in_areas.begin()+_object_num, 50) != _objects_in_areas.begin()+_object_num)
    {
        result._is_success = false;
        result._feedback_message = "Robot's hand is not free";
        result._step_cost = COST_OTHERS;
        return;
    }
    if (!_area_opens[area_index])
    {
        result._is_success = false;
        result._feedback_message = "Area is not open";
        result._step_cost = COST_OTHERS;
        return;
    }
    if (_objects_in_areas[object_index] != area_id)
    {
        result._is_success = false;
        result._feedback_message = "Object does not exist in area";
        result._step_cost = COST_OTHERS;
        return;
    }

    //move robot to area
    int old_robot_location = _robot_location;
    int old_robot_location_index = distance(_area_ids.begin(), find(_area_ids.begin(), _area_ids.begin()+_area_num, old_robot_location));

    _robot_location = area_id;
    double distance_cost = _navigation_map[old_robot_location_index * _area_num + area_index];

    // _object_in_robot_hand = object_id;
    _objects_in_areas[object_index] = 50;
    result._is_success = true;
    result._feedback_message = "Object is picked";
    result._step_cost = distance_cost + COST_PICK;
    return;
}

void SceneGraphSimple::TakeActionImpl_Place(const ActionSimple &action,
                                      ActionResult &result)
{
    int area_id = action.Args[0];
    int area_index = distance(_area_ids.begin(),
                                   find(_area_ids.begin(), _area_ids.begin()+_area_num, area_id));
    if (area_index == _area_num)
    {
        result._is_success = false;
        result._feedback_message = "Area does not exist";
        result._step_cost = COST_OTHERS;
        return;
    }
    if (find(_objects_in_areas.begin(), _objects_in_areas.begin() + _object_num, 50) == _objects_in_areas.begin() + _object_num)
    {
        result._is_success = false;
        result._feedback_message = "Robot is not holding any object";
        result._step_cost = COST_OTHERS;
        return;
    }
    if (!_area_opens[area_index])
    {
        result._is_success = false;
        result._feedback_message = "Area is not open";
        result._step_cost = COST_OTHERS;
        return;
    }

    //move robot to area
    int old_robot_location = _robot_location;
    int old_robot_location_index = distance(_area_ids.begin(), find(_area_ids.begin(), _area_ids.begin()+_area_num, old_robot_location));

    _robot_location = area_id;
    double distance_cost = _navigation_map[old_robot_location_index * _area_num + area_index];

    int objet_index = distance(_objects_in_areas.begin(), find(_objects_in_areas.begin(), _objects_in_areas.begin()+_object_num, 50));
    _objects_in_areas[objet_index] = area_id;
    // _object_in_robot_hand = -1;
    result._is_success = true;
    result._feedback_message = "Object is placed";
    result._step_cost = distance_cost + COST_PLACE;
    return;
}

bool SceneGraphSimple::GoalTest(TaskSimple &task) const {
    // array<array<int, 2>, 50> goals;
    // memcpy(goals.data(), task._area_has_obj.data(), sizeof(goals)* sizeof(array<int, 2>));
    bool goal_test = true;
    for (int i = 0; i < task._goal_num; i++)
    {
        // if (task._area_has_obj[i][0] == -1)
        // {
        //     break;
        // }
        bool subgoal_test = true;

        int object_id = task._area_has_obj[i][1];
        int area_id = task._area_has_obj[i][0];

        int object_index = distance(_object_ids.begin(), find(_object_ids.begin(), _object_ids.begin()+_object_num, object_id));
        if (object_index >= _object_num)
        {
            goal_test = false;
            break;
        }
        int area_index = distance(_area_ids.begin(), find(_area_ids.begin(), _area_ids.begin()+ _area_num, area_id));
        if (area_index >= _area_num)
        {
            goal_test = false;
            break;
        }

        if (_objects_in_areas[object_index] != area_index)
        {
            goal_test = false;
            break;
        }
    }
    return goal_test;
}

bool SceneGraphSimple::AddObjectInArea(int area_id, int object_id){
    int area_index = distance(_area_ids.begin(),
                                   find(_area_ids.begin(), _area_ids.begin()+ _area_num, area_id));
    int object_index = distance(_object_ids.begin(),
                                   find(_object_ids.begin(), _object_ids.begin()+_object_num, object_id));

    if (area_index == _area_num)
    {
        cerr << "Area does not exist" << endl;
        return false;
    }
    if (object_index != _object_num)
    {
        cerr << "Object already exists" << endl;
        return false;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
    }

    // int object_last_index = distance(_object_ids.begin(), find(_object_ids.begin(), _object_ids.end(), -1));
    int object_last_index = _object_num;
    if (object_last_index == 50)
    {
        cerr << "Object list is full" << endl;
        return false;
    }
    _object_ids[object_last_index] = object_id;
    _object_num += 1;
    _objects_in_areas[object_last_index] = area_id;
    return true;
}

bool SceneGraphSimple::AddObjectInHand(int object_id){
    int object_index = distance(_object_ids.begin(),
                                   find(_object_ids.begin(), _object_ids.begin()+_object_num, object_id));
    if (object_index != _object_num)
    {
        cerr << "Object already exists" << endl;
        return false;
    }
    if (find(_objects_in_areas.begin(), _objects_in_areas.begin()+_object_num, 50) != _objects_in_areas.begin()+_object_num)
    {
        cerr << "Robot's hand is not free" << endl;
        return false;
    }
    // int object_last_index = distance(_object_ids.begin(), find(_object_ids.begin(), _object_ids.end(), -1));
    int object_last_index = _object_num;
    _object_ids[object_last_index] = object_id;
    _object_num += 1;
    // _object_in_robot_hand = object_id;
    _objects_in_areas[object_last_index] = 50;
    return true;
}

bool SceneGraphSimple::RemoveObject(int object_id){
    int object_index = distance(_object_ids.begin(),
                                   find(_object_ids.begin(), _object_ids.begin()+_object_num, object_id));
    if (object_index == _object_num)
    {
        cerr << "Object does not exist" << endl;
        return false;
    }
        //remove from _object_ids and _objects_in_areas
        //copy the rest of the array to the left
    for (int i = object_index; i < _object_num -1; i++)
    {
        _object_ids[i] = _object_ids[i + 1];
        _objects_in_areas[i] = _objects_in_areas[i + 1];
        // if (_object_ids[i] == -1)
        // {
        //     break;
        // }
    }
    // _object_ids[49] = -1;
    _object_num -= 1;
    return true;
}

bool SceneGraphSimple::SetRobotLocation(int area_id){
    int area_index = distance(_area_ids.begin(),
                                   find(_area_ids.begin(), _area_ids.begin()+_area_num, area_id));
    if (area_index == _area_num)
    {
        cerr << "Area does not exist" << endl;
        return false;
    }
    _robot_location = area_index;
    return true;
}

bool SceneGraphSimple::SetAreaIsOpen(int area_id, bool is_open){
    int area_index = distance(_area_ids.begin(),
                                   find(_area_ids.begin(), _area_ids.begin()+_area_num, area_id));
    if (area_index == _area_num)
    {
        cerr << "Area does not exist" << endl;
        return false;
    }
    _area_opens[area_index] = is_open;
    return true;
}

bool SceneGraphSimple::AddObjectInAreaFromName(const string &area_name, const string &object_name, vector<string> area_names, vector<string> object_names){
    int area_id = distance(area_names.begin(),
                                   find(area_names.begin(), area_names.end(), area_name));
    int object_id = distance(object_names.begin(),
                                   find(object_names.begin(), object_names.end(), object_name));
    if (area_id == area_names.size())
    {
        cerr << "Area " << area_name << " does not exist" << endl;
        return false;
    }
    if (object_id == object_names.size())
    {
        cerr << "Object " << object_name << " does not exist" << endl;
        return false;
    }
    return AddObjectInArea(area_id, object_id);
}

bool SceneGraphSimple::AddObjectInHandFromName(const string &object_name, vector<string> object_names){
    int object_id = distance(object_names.begin(),
                                   find(object_names.begin(), object_names.end(), object_name));
    if (object_id == object_names.size())
    {
        cerr << "Object " << object_name << " does not exist" << endl;
        return false;
    }
    return AddObjectInHand(object_id);
}

bool SceneGraphSimple::RemoveObjectFromName(const string &object_name, vector<string> object_names){
    int object_id = distance(object_names.begin(),
                                   find(object_names.begin(), object_names.end(), object_name));
    if (object_id == object_names.size())
    {
        cerr << "Object " << object_name << " does not exist" << endl;
        return false;
    }
    return RemoveObject(object_id);
}

bool SceneGraphSimple::SetRobotLocationFromName(const string &area_name, vector<string> area_names){
    int area_id = distance(area_names.begin(),
                                   find(area_names.begin(), area_names.end(), area_name));
    if (area_id == area_names.size())
    {
        cerr << "Area " << area_name << " does not exist" << endl;
        return false;
    }
    return SetRobotLocation(area_id);
}

bool SceneGraphSimple::SetAreaIsOpenFromName(const string &area_name, bool is_open, vector<string> area_names){
    int area_id = distance(area_names.begin(),
                                   find(area_names.begin(), area_names.end(), area_name));
    if (area_id == area_names.size())
    {
        cerr << "Area " << area_name << " does not exist" << endl;
        return false;
    }
    return SetAreaIsOpen(area_id, is_open);
}

const int SceneGraphSimple::GetRobotLocation() const{
    return _robot_location;
}

const string SceneGraphSimple::GetRobotLocationName(vector<string> area_names) {
    if (_robot_location == -1)
    {
        return "";
    }
    return area_names[_area_ids[_robot_location]];
}

const string SceneGraphSimple::GetObjectParentName(const string& object_name, vector<string> object_names, vector<string> area_names){
    int object_id = distance(object_names.begin(),
                                   find(object_names.begin(), object_names.end(), object_name));
    if (object_id == object_names.size())
    {
        cerr << "Object " << object_name << " does not exist" << endl;
        return "";
    }
    int object_index = distance(_object_ids.begin(), find(_object_ids.begin(), _object_ids.begin()+_object_num, object_id));
    if (object_index >= 0 && object_index < _object_num)
    {
        int area_id = _objects_in_areas[object_index];
        if (area_id == -1)
        {
            return "";
        }
        if (area_id == 50)
        {
            return "robot";
        }
        return area_names[area_id];
    }
    else
    {
        cerr << "Object " << object_name << " does not exist" << endl;
        return "";
    }
}

const string SceneGraphSimple::GetObjectInHandName(vector<string> object_names){
    // if (_object_in_robot_hand == -1)
    // {
    //     return "";
    // }
    int object_index = distance(_objects_in_areas.begin(), find(_objects_in_areas.begin(), _objects_in_areas.begin()+_object_num, 50));
    if (object_index == _object_num)
    {
        return "";
    }
    return object_names[_object_ids[object_index]];
}

const vector<string> SceneGraphSimple::GetObjectsInArea(const string &area_name, vector<string> object_names, vector<string> area_names){
    int area_id = distance(area_names.begin(),
                                   find(area_names.begin(), area_names.end(), area_name));
    if (area_id == area_names.size())
    {
        cerr << "Area " << area_name << " does not exist" << endl;
        return vector<string>();
    }
    vector<string> objects;
    for (int i = 0; i < _object_num; i++)
    {
        if (_objects_in_areas[i] == area_id)
        {
            objects.push_back(object_names[_object_ids[i]]);
        }
    }
    return objects;
}


const string SceneGraphSimple::GetObsStr(){
    std::ostringstream oss;
    // 将所有数组内容转为字符串
    for (int i = 0; i < 50; ++i) {
        oss << _object_ids[i] << " ";
    }
    oss << "| ";
    for (int i = 0; i < 50; ++i) {
        oss << _area_opens[i] << " ";
    }
    oss << "| ";
    for (int i = 0; i < 50; ++i) {
        oss << _objects_in_areas[i] << " ";
    }
    oss << "| ";
    oss << _robot_location << " | ";

    // 返回构造好的字符串
    return oss.str();
}

//= operator overloading
SceneGraphSimple &SceneGraphSimple::operator=(const SceneGraphSimple &scene_graph)
{
    if (this == &scene_graph)
    {
        return *this;
    }
    memcpy(_area_ids.data(), scene_graph._area_ids.data(), 50 * sizeof(int));
    memcpy(_object_ids.data(), scene_graph._object_ids.data(), 50 * sizeof(int));
    memcpy(_area_opens.data(), scene_graph._area_opens.data(), 50 * sizeof(int));
    memcpy(_objects_in_areas.data(), scene_graph._objects_in_areas.data(), 50 * sizeof(int));
    _robot_location = scene_graph._robot_location;
    // _object_in_robot_hand = scene_graph._object_in_robot_hand;
    memcpy(_navigation_map.data(), scene_graph._navigation_map.data(), 2500 * sizeof(int));
    _area_num = scene_graph._area_num;
    _object_num = scene_graph._object_num;
    return *this;
}

ActionResult SceneGraphSimple::ForwardExp(const ActionSimple &action)
{
    ActionResult result;
    switch (action.Type())
    {
    case ActionType::ACTION_OPEN:
        TakeActionImpl_Open(action, result);
        break;
    case ActionType::ACTION_NONE:
        result._is_success = true;
        result._feedback_message = "No action";
        result._step_cost = 0;
        break;
    case ActionType::ACTION_DONE:
        result._is_success = true;
        result._feedback_message = "Declare done";
        result._step_cost = 0;
        break;
    default:
        result._is_success = false;
        result._feedback_message = "Invalid action";
        result._step_cost = 0;
        break;
    }
    return result;
}

void SceneGraphSimple::GetLegalActionsExp(unique_ptr<array<ActionSimple, 200>> &legal_actions, int &count)
{
    // 清空 legal actions
    count = 1;

    // 提前检查机器人的位置
    if (_robot_location == -1)
    {
        cerr << "Robot has no location" << endl;
        return;
    }

    // 添加所有打开操作
    for (int i = 0; i < _area_num; ++i)
    {
        if (!_area_opens[i])
        {
            int args[3] = {_area_ids[i], -1, ActionType::ACTION_OPEN};
            (*legal_actions)[count] = ActionSimple(args);
            count++;
        }
    }

    if (count <= 1){
        cerr<<"no legal actions"<<endl;
    }
}

bool SceneGraphSimple::GoalTestExp(TaskSimpleExp &task) const {
    bool goal_test = true;
    for (int i = 0; i < task._goal_num; i++)
    {
        bool subgoal_test = true;
        int object_id = task._objects[i];
        //check if all the objects exsists in the scene, and their parent areas are open

        int object_index = distance(_object_ids.begin(), find(_object_ids.begin(), _object_ids.begin()+_object_num, object_id));
        if (object_index >= _object_num)
        {
            goal_test = false;
            break;
        }
        int area_id = _objects_in_areas[object_index];
        if (area_id == -1)
        {
            goal_test = false;
            break;
        }
        if (area_id == 50)
        {
            continue;
        }
        int area_index = distance(_area_ids.begin(), find(_area_ids.begin(), _area_ids.begin()+_area_num, area_id));
        if (!_area_opens[area_index])
        {
            goal_test = false;
            break;
        }
    }
    return goal_test;
}
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <SceneGraph/Action.h>
#include <SceneGraph/ActionResult.h>
#include <SceneGraph/Node.h>
#include <SceneGraph/Robot.h>
#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/Simulator.h>
#include <SceneGraph/Task.h>
#include <SceneGraph/Type.h>


namespace py = pybind11;

PYBIND11_MODULE(_kitchen_env, m)
{

    py::enum_<NodeType> node_type(m, "NodeType");
    node_type
        .value("NODE_NONE", NodeType::NODE_NONE)
        .value("NODE_AREA", NodeType::NODE_AREA)
        .value("NODE_OBJECT", NodeType::NODE_OBJECT)
        .value("NODE_ROBOT", NodeType::NODE_ROBOT)
        .export_values();

    py::enum_<ActionType> action_type(m, "ActionType");
    action_type
        .value("ACTION_NONE", ActionType::ACTION_NONE)
        .value("ACTION_DONE", ActionType::ACTION_DONE)
        .value("ACTION_OPEN", ActionType::ACTION_OPEN)
        .value("ACTION_PICK", ActionType::ACTION_PICK)
        .value("ACTION_PLACE", ActionType::ACTION_PLACE)
        .export_values();

    py::class_<Node>(m, "Node")
        .def("type", &Node::GetType)
        .def("name", &Node::GetName);

    py::class_<AreaNode, Node>(m, "AreaNode")
        .def("is_open", &AreaNode::IsOpen);

    py::class_<ObjectNode, Node>(m, "ObjectNode");

    py::class_<Robot, Node>(m, "Robot");

    py::class_<SceneGraph>(m, "SceneGraph")
        .def(py::init<>())
        .def("copy", &SceneGraph::Copy, py::return_value_policy::take_ownership)
        .def("get_node", &SceneGraph::GetNode, py::return_value_policy::reference_internal)
        .def("get_area", &SceneGraph::GetArea, py::return_value_policy::reference_internal)
        .def("get_object", &SceneGraph::GetObject, py::return_value_policy::reference_internal)
        .def("get_robot", &SceneGraph::GetRobot, py::return_value_policy::reference_internal)
        .def("forward", &SceneGraph::Forward, py::return_value_policy::reference_internal)
        .def("get_legal_actions", &SceneGraph::GetLegalActions, py::return_value_policy::reference_internal)
        .def("goal_test", &SceneGraph::GoalTest, py::return_value_policy::reference_internal)

        // .def("get_reward", &SceneGraph::GetReward)
        .def("add_object_in_area", &SceneGraph::AddObjectInArea)
        .def("add_object_in_hand", &SceneGraph::AddObjectInHand)
        .def("remove_object", &SceneGraph::RemoveObject)
        .def("set_robot_location", &SceneGraph::SetRobotLocation)
        .def("set_area_is_open", &SceneGraph::SetAreaIsOpen)

        .def("get_robot_location", &SceneGraph::GetRobotLocation, py::return_value_policy::reference_internal)
        .def("check_object_in_area", &SceneGraph::CheckObjectInArea)
        .def("remove_object_from_area", &SceneGraph::RemoveObjectFromArea)
        .def("get_object_parent_node", &SceneGraph::GetObjectParentNode, py::return_value_policy::reference_internal)
        .def("get_object_in_hand", &SceneGraph::GetObjectInHand, py::return_value_policy::reference_internal)
        .def("get_objects_in_area", &SceneGraph::GetObjectsInArea, py::return_value_policy::reference_internal)
        .def("check_objects_exists_and_areas_open", &SceneGraph::CheckObjectsExistsAndAreasOpen)
        .def("all_areas", &SceneGraph::all_areas, py::return_value_policy::reference_internal)
        .def("all_objects", &SceneGraph::all_objects, py::return_value_policy::reference_internal)
        .def("to_json", &SceneGraph::ToJsonString);

    py::class_<Simulator, std::shared_ptr<Simulator>>(m, "Simulator")
        .def(py::init<const string&, const string&>())
        .def("get_legal_actions", &Simulator::GetLegalActions)
        .def("take_action", &Simulator::TakeAction)
        .def("get_total_cost", &Simulator::GetTotalCost)
        .def("get_task", &Simulator::GetTask)
        .def("goal_test", &Simulator::GoalTest)
        .def("get_observation", &Simulator::GetObservation, py::return_value_policy::take_ownership)
        .def("get_objects_reach_goal", &Simulator::ObjectsReachGoal);

    py::class_<Task>(m, "Task")
        .def(py::init<>())
        .def(py::init<const std::string &>())
        .def("get_language_description", &Task::GetLanguageDescription)
        .def("set_language_description", &Task::SetLanguageDescription)
        .def("remove_goal", &Task::RemoveGoal)
        .def("add_goal", &Task::AddGoal);

    py::class_<ActionResult>(m, "ActionResult")
        .def("get_is_success", &ActionResult::GetIsSuccess)
        .def("get_feedback_message", &ActionResult::GetFeedbackMessage)
        .def("get_step_cost", &ActionResult::GetStepCost);

    py::class_<Action>(m, "Action")
        .def(py::init<>())
        .def(py::init<ActionType, const std::vector<std::string> &>())
        .def_readwrite("type", &Action::Type)
        .def_readwrite("args", &Action::Args);

        m.attr("REBUILD_HELPER") = "helper10";


}

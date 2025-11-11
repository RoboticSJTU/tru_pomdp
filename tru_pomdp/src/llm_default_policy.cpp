#include <llm_default_policy.h>

ActionSimple NextAction(
    const std::vector<std::array<int, 2>>& unreached_goals,
    const SceneGraphSimple& current_scene_graph
) {
    // If no goals remain, return no-op
    if (unreached_goals.empty()) {
        return ActionSimple();
    }

    // Check each goal
    for (const auto& goal : unreached_goals) {
        int goal_area = goal[0];
        int goal_object = goal[1];

        // Skip if the area or object is not in the scene
        if (!current_scene_graph.CheckAreaInScene(goal_area) ||
            !current_scene_graph.CheckObjectInScene(goal_object)) {
            continue;
        }

        // If goal is already satisfied, skip it
        int parent_of_goal = current_scene_graph.GetObjectParent(goal_object);
        if (parent_of_goal == goal_area) {
            continue;
        }

        // Check what the robot is currently holding
        int object_in_hand = current_scene_graph.GetObjectInHand();

        if (object_in_hand == goal_object) {
            // Need to place it in goal_area
            if (!current_scene_graph.GetAreaOpenFromId(goal_area)) {
                // Open the goal area first
                return ActionSimple(ActionType::ACTION_OPEN, goal_area, -1);
            } else {
                // Place the object
                return ActionSimple(ActionType::ACTION_PLACE, goal_area, -1);
            }
        } 
        else if (object_in_hand != -1) {
            // Holding a different object; place it back in its parent
            int parent_of_held = current_scene_graph.GetObjectParent(object_in_hand);
            if (parent_of_held == -1) {
                // No known place to put it; do nothing
                return ActionSimple();
            }
            if (!current_scene_graph.GetAreaOpenFromId(parent_of_held)) {
                return ActionSimple(ActionType::ACTION_OPEN, parent_of_held, -1);
            } else {
                return ActionSimple(ActionType::ACTION_PLACE, parent_of_held, -1);
            }
        } 
        else {
            // Holding nothing; pick up the goal object
            if (parent_of_goal == -1) {
                // No parent area known; do nothing
                return ActionSimple();
            }
            if (!current_scene_graph.GetAreaOpenFromId(parent_of_goal)) {
                // Open the parent's area first
                return ActionSimple(ActionType::ACTION_OPEN, parent_of_goal, -1);
            } else {
                // Then pick the object
                return ActionSimple(ActionType::ACTION_PICK, parent_of_goal, goal_object);
            }
        }
    }

    // If all goals are satisfied or no action can be deduced, return no-op
    return ActionSimple();



}


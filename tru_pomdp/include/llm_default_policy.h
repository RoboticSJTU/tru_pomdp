#ifndef LLM_DEFAULT_POLICY_H
#define LLM_DEFAULT_POLICY_H

#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/Action.h>
#include <utils.h>
#include <memory>

ActionSimple NextAction(
    const vector<array<int, 2>> &unreached_goals,
    const SceneGraphSimple &current_scene_graph);

#endif
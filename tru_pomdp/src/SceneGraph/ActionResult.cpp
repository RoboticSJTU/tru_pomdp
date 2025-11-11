#include <SceneGraph/ActionResult.h>

// definition of ActionResult::DEBUG_ACTION_RESULT_MEMORY_LEAK
// std::set<ActionResult *> ActionResult::DEBUG_ACTION_RESULT_MEMORY_LEAK;

ActionResult::ActionResult(bool is_success, const std::string &feedback_message, unsigned long step_cost)
{
	_is_success = is_success;
	_feedback_message = feedback_message;
	_step_cost = step_cost;
    // DEBUG_ACTION_RESULT_MEMORY_LEAK.insert(this);
}

ActionResult::ActionResult()
{
    _is_success = false;
    _feedback_message = "";
    _step_cost = 0;
    // DEBUG_ACTION_RESULT_MEMORY_LEAK.insert(this);
}

bool ActionResult::GetIsSuccess() const
{
	return _is_success;
}

std::string ActionResult::GetFeedbackMessage() const
{
	return _feedback_message;
}

unsigned long ActionResult::GetStepCost() const
{
	return _step_cost;
}

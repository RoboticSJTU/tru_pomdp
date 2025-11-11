#pragma once
#include <string>
#include <ostream>
#include <set>

class ActionResult
{
public:
    bool _is_success;
	std::string _feedback_message;
	unsigned long _step_cost;

    // static std::set<ActionResult*> DEBUG_ACTION_RESULT_MEMORY_LEAK;

	ActionResult(bool is_success, const std::string &feedback_message, unsigned long step_cost);
    ActionResult();
    // copy constructor
    ActionResult(const ActionResult& result) {
        _is_success = result._is_success;
        _feedback_message = result._feedback_message;
        _step_cost = result._step_cost;
        // DEBUG_ACTION_RESULT_MEMORY_LEAK.insert(this);
    }
    ~ActionResult() {
        // DEBUG_ACTION_RESULT_MEMORY_LEAK.erase(this);
    }
	bool GetIsSuccess() const;
	std::string GetFeedbackMessage() const;
	unsigned long GetStepCost() const;
    // operator << for debug output
    friend std::ostream &operator<<(std::ostream &os, const ActionResult &result) {
        os << "ActionResult:\t";
        if (result._is_success) {
            os << "Success" << std::endl;
        } else {
            os << "Fail" << std::endl;
        }
        os << "Cost:\t" << result._step_cost << std::endl;
        os << "Reason:\t" << result._feedback_message;
        return os;
    }
};

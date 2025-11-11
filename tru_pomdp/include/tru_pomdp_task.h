#ifndef TRU_POMDP_TASK_H
#define TRU_POMDP_TASK_H

#include <despot/interface/pomdp.h>
#include <despot/core/particle_belief.h>
#include <despot/core/globals.h>

#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/Task.h>
#include <SceneGraph/Action.h>

#include <tru_pomdp_belief.h>
#include <tru_pomdp_state.h>
#include <llm_default_policy.h>
#include <utils.h>

#include <random>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <random>
#include <algorithm>

using namespace std;
namespace despot{

    class TruPomdpState;

    /* ==============================================================================
     * TruPomdpTask class
     * ==============================================================================*/

    class TruPomdpTask : public DSPOMDP
    {
        friend class TruPomdpState;
        friend class TruPomdpLowerBound;
        friend class TruPomdpUpperBound;
        friend class TruPomdpBelief;

    public:
        // Task task_;
        // TruPomdpTask(Task task);
        int num_k_;
        vector<ActionSimple> *all_actions_ptr_;
        unordered_map<size_t, int> *action_index_map_;
        TruPomdpTask()
        {
            num_k_ = 3;
            // vector<ActionSimple> all_actions = {};
            all_actions_ptr_ = new vector<ActionSimple>();
            action_index_map_ = new unordered_map<size_t, int>();
            int none_action[3] = {-1, -1, ActionType::ACTION_NONE};
            all_actions_ptr_->emplace_back(none_action);
            int key = hash<ActionSimple>{}(ActionSimple(none_action));
            action_index_map_->insert({key, 0});

        }
        TruPomdpTask(int num_k = 3)
        {
            num_k_ = num_k;
            vector<ActionSimple> all_actions = {};
            all_actions_ptr_ = new vector<ActionSimple>();
            action_index_map_ = new unordered_map<size_t, int>();
            int none_action[3] = {-1, -1, ActionType::ACTION_NONE};
            all_actions_ptr_->emplace_back(none_action);
            int key = hash<ActionSimple>{}(ActionSimple(none_action));
            action_index_map_->insert({key, 0});

        }

        ~TruPomdpTask()
        {
            cout << "delete trupomdptask" << endl;
            delete all_actions_ptr_;
            delete action_index_map_;
        }

        virtual bool Step(State &s, double random_num, ACT_TYPE action, double &reward) const;
        virtual bool Step(State &s, double random_num, ACT_TYPE action, double &reward, OBS_TYPE &obs) const;

        virtual State *Copy(const State *particle) const;
        virtual vector<State *> Copy(const vector<State *> particles) const;
        virtual Belief *InitialBelief(const State *start, string type = "DEFAULT") const;
        virtual Belief *InitialBelief_SceneGraph(
            const SceneGraphSimple start_scene_graph,
            string language_instruction,
            vector<string> *area_name_list_ptr,
            vector<string> *object_name_list_ptr,
            vector<ActionSimple> *all_actions_ptr) const;
        // Belief *InitialBelief(vector<State *> particles, const DSPOMDP *model, const int num_k) const;
        virtual ParticleLowerBound *CreateParticleLowerBound(string name = "DEFAULT") const;
        virtual ScenarioLowerBound *CreateScenarioLowerBound(string name = "DEFAULT", string particle_bound_name = "DEFAULT") const;
        virtual ParticleUpperBound *CreateParticleUpperBound(string name = "DEFAULT") const;
        virtual ScenarioUpperBound *CreateScenarioUpperBound(string name = "DEFAULT", string particle_bound_name = "DEFAULT") const;

        virtual void PrintState(const State &state, ostream &out = cout) const;
        virtual void PrintBelief(const Belief &belief, ostream &out = cout) const;
        virtual void PrintObs(const State &state, OBS_TYPE obs, ostream &out = cout) const
        {
            PrintState(state, out);
        };
        virtual void PrintAction(ACT_TYPE action, ostream &out = cout) const;

        virtual State *Allocate(int state_id, double weight) const;
        virtual void Free(State *particle) const;
        virtual int NumActiveParticles() const;

        virtual int NumActions() const
        {
            // 必须重载，但要改掉
            return 26;
        }

        virtual double ObsProb(OBS_TYPE obs, const State &state, ACT_TYPE action) const
        {
            // 必须重载，但不会使用
            return 1.0;
        }

        virtual double GetMaxReward() const
        {
            return 100;
        }

        virtual ValuedAction GetBestAction() const
        {
            // 必须重载，但不会使用
            return ValuedAction(0, 0);
        }

        mutable MemoryPool<TruPomdpState> memory_pool_;
    
};

/* ==============================================================================
 * TruPomdpUpperBound class
 * ==============================================================================*/

    class TruPomdpUpperBound: public ParticleUpperBound{
    public:
        TruPomdpUpperBound();
        double Value(const State &state) const;
        double Value(const vector<State*> &particles) const;
        double Value(const vector<State*>& particles, RandomStreams& streams, History& history) const;
    };

/* ==============================================================================
 * TruPomdpLowerBound class
 * ==============================================================================*/

    class TruPomdpLowerBound: public ParticleLowerBound{
    public:
        TruPomdpLowerBound(const DSPOMDP* model);
        ValuedAction Value_random(const State& state) const;
        ValuedAction Value(const State& state) const;
        ValuedAction Value(const vector<State*>& particles) const;

    };

    class TruPomdpDefaultPolicy: public DefaultPolicy{
    public: 
        TruPomdpDefaultPolicy(DSPOMDP* model, ParticleLowerBound* bound)
                : DefaultPolicy(model, bound){}

        ACT_TYPE Action(const vector<State *> &particles,
                        RandomStreams &streams, History &history) const;
        ACT_TYPE Action_random(const State &s) const;
    };

} // namespace despot
#endif
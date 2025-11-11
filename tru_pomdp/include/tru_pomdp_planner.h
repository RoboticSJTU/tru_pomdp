#ifndef TRU_POMDP_PLANNER_H
#define TRU_POMDP_PLANNER_H

#include <tru_pomdp_world.h>
#include <tru_pomdp_task.h>

#include <despot/planner.h>
#include <despot/core/globals.h>
#include <utils.h>

#include <SceneGraph/Action.h>
#include <SceneGraph/SceneGraph.h>

#include<cstdlib>
using namespace despot;
using namespace std;
class TruPomdpPlanner: public Planner{
    
public:
    TruPomdpPlanner(){}
    virtual DSPOMDP *InitializeModel(option::Option *options);
    virtual World* InitializeWorld(string& world_type, DSPOMDP* model, option::Option* options){
        return nullptr;
    }
    virtual World *InitializeWorld(string &world_type, DSPOMDP *model, option::Option *options, string problem_file_path, string scene_file_path);
    virtual void InitializeDefaultParameters(){}
    string ChooseSolver(){
        return "DESPOT";
    }
    virtual pair<bool, int> PlanningLoop_Return(Solver *&solver, World *world, Logger *logger);
    virtual pair<bool, bool> RunStep_return(Solver *solver, World *world, Logger *logger);
    virtual int RunPlanning_write_result(int argc, char *argv[], string problem_file_path, string scene_file_path, string result_file_path);
    virtual void PrintResult(int num_runs, Logger *logger, clock_t main_clock_start);

};

#endif
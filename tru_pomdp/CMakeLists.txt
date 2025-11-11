cmake_minimum_required(VERSION 2.8.3)
project(tru_pomdp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmax-include-depth=500")

# find_package(Despot REQUIRED COMPONENTS)
# find_package(SceneGraph REQUIRED COMPONENTS)
find_package(CURL REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

include_directories(
    include
    # "${DESPOT_INCLUDE_DIRS}"
    # "${SceneGraph_INCLUDE_DIRS}"
)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer -g")

# target_link_libraries(tru_pomdp PRIVATE despot)
# target_link_libraries(tru_pomdp PRIVATE SceneGraph)

# set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_options(
    -O3
    # -Ofast
    # -flto
    # -march=native
)
add_definitions(-w)

add_library("${PROJECT_NAME}_lib" SHARED
    src/tru_pomdp_task.cpp
    src/tru_pomdp_world.cpp
    src/utils.cpp
    src/tru_pomdp_belief.cpp
    src/despot/interface/pomdp.cpp
    src/despot/interface/world.cpp
    src/despot/interface/belief.cpp
    src/despot/interface/lower_bound.cpp
    src/despot/interface/upper_bound.cpp
    src/despot/interface/default_policy.cpp
    src/despot/core/particle_belief.cpp
    src/despot/core/globals.cpp
    src/despot/core/builtin_lower_bounds.cpp
    src/despot/core/mdp.cpp
    src/despot/core/node.cpp
    src/despot/core/builtin_policy.cpp
    src/despot/core/pomdp_world.cpp
    src/despot/core/solver.cpp
    src/despot/core/builtin_upper_bounds.cpp
    src/despot/logger.cpp
    src/despot/planner.cpp
    src/despot/evaluator.cpp
    src/despot/pomdpx/parser/function.cpp
    src/despot/pomdpx/parser/parser.cpp
    src/despot/pomdpx/parser/variable.cpp
    src/despot/pomdpx/pomdpx.cpp
    src/despot/random_streams.cpp
    src/despot/plannerbase.cpp
    src/despot/solver/aems.cpp
    src/despot/solver/despot.cpp
    src/despot/solver/pomcp.cpp
    src/despot/solver/baseline_solver.cpp
    src/despot/util/coord.cpp
    src/despot/util/dirichlet.cpp
    src/despot/util/exec_tracker.cpp
    src/despot/util/floor.cpp
    src/despot/util/gamma.cpp
    src/despot/util/logging.cpp
    src/despot/util/random.cpp
    src/despot/util/seeds.cpp
    src/despot/util/util.cpp
    src/despot/util/tinyxml/tinystr.cpp
    src/despot/util/tinyxml/tinyxml.cpp
    src/despot/util/tinyxml/tinyxmlerror.cpp
    src/despot/util/tinyxml/tinyxmlparser.cpp
    src/SceneGraph/Robot.cpp
    src/SceneGraph/ActionResult.cpp
    src/SceneGraph/Simulator.cpp
    src/SceneGraph/Type.cpp
    src/SceneGraph/SceneGraph.cpp
    src/SceneGraph/Task.cpp
    src/SceneGraph/Action.cpp
    src/SceneGraph/Node.cpp
    src/tree_of_hypothesis.cpp
    src/tru_pomdp_state.cpp
    src/llm_default_policy.cpp
    src/tru_pomdp_planner.cpp

)

add_executable("${PROJECT_NAME}" 
    src/main.cpp
)
target_link_libraries("${PROJECT_NAME}" "${PROJECT_NAME}_lib")

target_link_libraries("${PROJECT_NAME}_lib" CURL::libcurl)
target_link_libraries("${PROJECT_NAME}_lib" pthread)

pybind11_add_module(_kitchen_env 
    python_binding/KitchenEnvPy.cpp
)

target_link_libraries(_kitchen_env PRIVATE ${PROJECT_NAME}_lib)
target_include_directories(_kitchen_env PRIVATE include)





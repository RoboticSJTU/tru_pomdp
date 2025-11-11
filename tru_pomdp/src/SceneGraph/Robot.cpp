#include <SceneGraph/Robot.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

std::string Robot::ToJsonString() const {
    json j;
    j["name"] = _name;
    j["type"] = NodeNameMap[Type];
    return j.dump(4);
}

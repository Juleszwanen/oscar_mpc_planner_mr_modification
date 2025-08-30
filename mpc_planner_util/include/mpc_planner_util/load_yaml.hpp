#ifndef LOAD_YAML_H
#define LOAD_YAML_H

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <string>

// JULES: get the abs file path of x and go up to the parent, go up one more go into the /config/ directory and get the file with .yaml
#define SYSTEM_CONFIG_PATH(x, filename) std::filesystem::path(x).parent_path().string() + "/../config/" + filename + ".yaml"
#define SYSTEM_CONFIG_PATH_INCLUDE(x, filename) std::filesystem::path(x).parent_path().parent_path().string() + "/../config/" + filename + ".yaml"

inline void loadConfigYaml(const std::string &file, YAML::Node &_yaml_out)
{
    _yaml_out = YAML::LoadFile(file);
}

#endif // LOAD_YAML_H
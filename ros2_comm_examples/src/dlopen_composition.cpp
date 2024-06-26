// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <vector>

#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"
#include "rcutils/logging.h"

#define DLOPEN_COMPOSITION_LOGGER_NAME "dlopen_composition"

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (argc < 2) {
    fprintf(stderr, "Requires at least one argument to be passed with the library to load\n");
    return 1;
  }
  rclcpp::init(argc, argv);
  rclcpp::Logger logger = rclcpp::get_logger(DLOPEN_COMPOSITION_LOGGER_NAME);
  rclcpp::executors::SingleThreadedExecutor exec;
  // rclcpp::executors::StaticSingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  std::vector<class_loader::ClassLoader *> loaders;
  std::vector<rclcpp_components::NodeInstanceWrapper> node_wrappers;

  std::vector<std::string> libraries;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--ros-args") {
      continue;
    }
    if (std::string(argv[i]) == "--log-level") {
      i++;
      continue;
    }
    libraries.push_back(argv[i]);
  }
  if (libraries.empty()) {
    RCLCPP_INFO(logger, "No libraries to load");
    return 0;
  }
  for (auto library : libraries) {
    RCLCPP_INFO(logger, "Load library %s", library.c_str());
    auto loader = new class_loader::ClassLoader(library);
    auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
    for (auto clazz : classes) {
      RCLCPP_INFO(logger, "Instantiate class %s", clazz.c_str());
      auto node_factory = loader->createInstance<rclcpp_components::NodeFactory>(clazz);
      /**
       * 可以通过指定options参数修改node name，例如：
       * 
            static int count = 0;
            std::string node_name = "node_" + std::to_string(count++);
            std::vector<std::string> remap_rules;
            remap_rules.push_back("--ros-args");
            remap_rules.push_back("-r");
            remap_rules.push_back("__node:=" + node_name);
            options.arguments(remap_rules);
      *        
      */
      auto wrapper = node_factory->create_node_instance(options);
      auto node = wrapper.get_node_base_interface();
      node_wrappers.push_back(wrapper);
      exec.add_node(node);
    }
    loaders.push_back(loader);
  }

  exec.spin();

  for (auto wrapper : node_wrappers) {
    exec.remove_node(wrapper.get_node_base_interface());
  }
  node_wrappers.clear();

  rclcpp::shutdown();

  return 0;
}

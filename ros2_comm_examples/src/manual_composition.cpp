#include <include/publisher.h>
#include "include/subscriber.h"
#include <rclcpp/executor.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  int sub_node_num = 1;
  char* env = getenv("SUB_NUM");
  if (env) {
    sub_node_num = atoi(env);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "input sub_node_num: %d", sub_node_num);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
      "using default sub_node_num: %d, which is set with [export SUB_NUM=val].",
      sub_node_num);
  }

  RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "sub_node_num: %d", sub_node_num);
  if (sub_node_num < 1 || sub_node_num > 10) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),
    "invalid sub_node_num: "
    << sub_node_num);
    return -1;
  }

  enum class ExecutorType {
    SINGLE_THREADED = 0,
    MULTI_THREADED,
    STATIC_SINGLE_THREADED,
    INVALID
  };

  int executor_type = 0;
  env = getenv("EXE_TYPE");
  if (env) {
    executor_type = atoi(env);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "input executor_type: %d", executor_type);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
      "using default executor_type: %d, which is set with [export EXE_TYPE=val].",
      executor_type);
  }
  RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
    "executor_type(0: single threaded, 1: multi threaded, 2: static single threaded): %d",
    executor_type);


  int use_ipc = 0;
  env = getenv("USE_IPC");
  if (env) {
    use_ipc = atoi(env);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "input use_ipc: %d", use_ipc);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
      "using default use_ipc: %d, which is set with [export USE_IPC=val].",
      use_ipc);
  }
  RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
    "use_ipc(0: disable ipc, 1: enable ipc): %d",
    use_ipc);

  std::shared_ptr<rclcpp::Executor> exec_ptr = nullptr;
  if (static_cast<int>(ExecutorType::SINGLE_THREADED) == executor_type) {
    exec_ptr = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  } else if (static_cast<int>(ExecutorType::MULTI_THREADED) == executor_type) {
    exec_ptr = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  } else if (static_cast<int>(ExecutorType::STATIC_SINGLE_THREADED) == executor_type) {
    exec_ptr = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),
      "invalid executor_type: "
      << executor_type);
    return -1;
  }

  auto node_opt = rclcpp::NodeOptions();
  node_opt.use_intra_process_comms(use_ipc == 1? true : false);
  auto pub_node = std::make_shared<ros2_demos::MinimalPublisher>(node_opt);
  exec_ptr->add_node(pub_node);

  std::vector<std::shared_ptr<ros2_demos::MinimalSubscriber>> sub_nodes;
  for (auto idx = 0; idx < sub_node_num; ++idx) {
    auto sub_node = std::make_shared<ros2_demos::MinimalSubscriber>(node_opt);
    sub_nodes.push_back(sub_node);
    RCLCPP_WARN_STREAM(rclcpp::get_logger("minimal_subscriber"),
      "Add sub node[" << reinterpret_cast<std::uintptr_t>(sub_node.get()) << "]");
    exec_ptr->add_node(sub_node);
  }
  exec_ptr->spin();

  rclcpp::shutdown();
  return 0;
}
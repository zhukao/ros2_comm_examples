#include "include/subscriber.h"

#ifdef BUILD_EXE
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto exec = rclcpp::executors::StaticSingleThreadedExecutor();
  auto node = std::make_shared<ros2_demos::MinimalSubscriber>(rclcpp::NodeOptions());
  exec.add_node(node);
  exec.spin();
  
  rclcpp::shutdown();
  return 0;
}
#else
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_demos::MinimalSubscriber)
#endif

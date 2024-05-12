cmake_minimum_required(VERSION 3.8)
project(ros2_comm_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options(-fPIC)
  add_compile_options(-O2)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sharedmem_msgs REQUIRED)


include_directories(
  include
  ${PROJECT_SOURCE_DIR}
)

add_library(talker SHARED
  src/publisher.cpp
)
target_compile_definitions(talker
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(talker
  rclcpp
  rclcpp_components
  sharedmem_msgs
)

rclcpp_components_register_nodes(talker "ros2_demos::MinimalPublisher")
set(node_plugins "${node_plugins}ros2_demos::MinimalPublisher;$<TARGET_FILE:talker>\n")

add_library(listener SHARED
  src/subscriber.cpp
)
target_compile_definitions(listener
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(listener
  rclcpp
  rclcpp_components
  sharedmem_msgs
)

rclcpp_components_register_nodes(listener "ros2_demos::MinimalSubscriber")
set(node_plugins "${node_plugins}ros2_demos::MinimalSubscriber;$<TARGET_FILE:listener>\n")

add_executable(demo_listener src/subscriber.cpp)
target_compile_definitions(demo_listener PUBLIC -DBUILD_EXE)
ament_target_dependencies(demo_listener
  rclcpp
  sharedmem_msgs
  )

add_executable(demo_talker src/publisher.cpp)
target_compile_definitions(demo_talker PUBLIC -DBUILD_EXE)
ament_target_dependencies(demo_talker
  rclcpp
  sharedmem_msgs
  )

add_executable(demo_talker_loan src/publisher.cpp)
target_compile_definitions(demo_talker_loan PUBLIC -DBUILD_EXE -DUSING_LOAN_MSG)
ament_target_dependencies(demo_talker_loan
  rclcpp
  sharedmem_msgs
  )
  
add_executable(demo_manual_composition src/manual_composition.cpp)
ament_target_dependencies(demo_manual_composition
  rclcpp
  sharedmem_msgs
  )
target_link_libraries(demo_manual_composition 
  listener
  talker
  )
  
add_executable(demo_dlopen_composition src/dlopen_composition.cpp)
ament_target_dependencies(demo_dlopen_composition
  "class_loader"
  "rclcpp"
  "rclcpp_components"
  )

install(TARGETS
  listener
  DESTINATION lib/)

install(TARGETS
  talker
  DESTINATION lib/)

install(TARGETS
  demo_talker
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS
  demo_talker_loan
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS
  demo_listener
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  demo_manual_composition
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  demo_dlopen_composition
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

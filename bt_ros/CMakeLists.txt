cmake_minimum_required(VERSION 3.10)
project(bt_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PKG_DEPS
  actionlib
  actionlib_msgs
  behaviortree_cpp_v3
  bt_ros_msgs
  message_generation
  roscpp
  rosfmt
  xmlrpcpp
)

find_package(catkin REQUIRED COMPONENTS
  ${PKG_DEPS}
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  ${PKG_DEPS}
  DEPENDS
  Boost
)

# Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/executor_bt.cpp
  src/controls/executor_node.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(executor_tree src/executor_node.cpp)
target_link_libraries(executor_tree
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
)

# Install
install(
  TARGETS ${PROJECT_NAME} executor_tree
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Tests
add_library(dummy_nodes_dyn SHARED test/src/register_plugins.cpp)
target_link_libraries(dummy_nodes_dyn PRIVATE ${PROJECT_NAME})
target_compile_definitions(dummy_nodes_dyn PRIVATE BT_PLUGIN_EXPORT)

add_executable(simple_execute_ref_server test/src/simple_execute_ref_server.cpp)
target_link_libraries(simple_execute_ref_server ${catkin_LIBRARIES})

find_package(rostest REQUIRED)

add_rostest_gtest(simple_action_1
  test/simple_action_1.test
  test/simple_action_1_test.cpp
)
target_link_libraries(simple_action_1
  ${PROJECT_NAME}
)

add_rostest_gtest(service_node
  test/service_node.test
  test/service_node_test.cpp
)
target_link_libraries(service_node
  ${PROJECT_NAME}
)

add_rostest_gtest(simple_server_1
  test/simple_server_1.test
  test/simple_server_1_test.cpp
)
target_link_libraries(simple_server_1
  ${PROJECT_NAME}
)

add_rostest_gtest(subscriber_test
  test/subscriber.test
  test/subscriber_test.cpp
)
target_link_libraries(subscriber_test
  ${PROJECT_NAME}
)

add_rostest_gtest(executor_test
  test/executor.test
  test/executor_test.cpp
)
target_link_libraries(executor_test
  ${PROJECT_NAME}
)

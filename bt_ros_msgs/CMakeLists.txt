cmake_minimum_required(VERSION 3.10)
project(bt_ros_msgs)

set(MSG_DEPS
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS message_generation ${MSG_DEPS})

macro(subdirlist result curdir)
  file(GLOB children ${curdir})
  set(filelist "")

  foreach(child ${children})
    get_filename_component(child ${child} NAME)
    list(APPEND filelist ${child})
  endforeach()

  set(${result} ${filelist})
endmacro()

subdirlist(MSG_FILES "msg/*.msg")
subdirlist(SRV_FILES "srv/*.srv")

add_message_files(DIRECTORY msg FILES ${MSG_FILES})
add_service_files(DIRECTORY srv FILES ${SRV_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  ${MSG_DEPS}
)

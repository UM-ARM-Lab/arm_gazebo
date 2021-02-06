cmake_minimum_required(VERSION 3.5)
project(arm_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS moveit_msgs arm_gazebo_msgs arc_utilities)
find_package(gazebo REQUIRED)
catkin_package(LIBRARIES CATKIN_DEPENDS moveit_msgs arm_gazebo_msgs)

function(gazebo_plugin target_name)
    set(sources ${ARGN})
    add_library(${target_name} SHARED ${sources})
    target_compile_options(${target_name} PRIVATE ${FLAGS})
    target_include_directories(${target_name} PUBLIC include)
    target_include_directories(${target_name} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
    target_link_libraries(${target_name} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
    add_dependencies(${target_name} ${catkin_EXPORTED_TARGETS})
endfunction()

gazebo_plugin(gazebo_ros_moveit_planning_scene src/gazebo_ros_moveit_planning_scene.cpp)


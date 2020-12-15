cmake_minimum_required(VERSION 3.5)
project(arm_gazebo_msgs)

set(CATKIN_DEPENDENCIES std_msgs)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDENCIES} message_generation)

add_message_files(
        FILES
)

add_service_files(
        FILES
        ExcludeModels.srv
)

generate_messages(
        DEPENDENCIES
        ${CATKIN_DEPENDENCIES}
        )


catkin_package(CATKIN_DEPENDS ${CATKIN_DEPENDENCIES} message_runtime)

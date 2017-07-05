cmake_minimum_required(VERSION 2.8.3)
project(sara_moveit)
SET(CMAKE_CXX_STANDARD 11)
add_definitions(-std=c++11)
find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    moveit_core
    roscpp rospy
    std_msgs
    message_generation
    geometry_msgs
    std_srvs
)

add_service_files(
        DIRECTORY srv
        FILES
        #pick.srv
        move.srv
)

generate_messages(
        DEPENDENCIES geometry_msgs
)



add_executable( sara_moveit_move_server
        include/sara_moveit/move_server.h
        src/move_server.cpp
        )

#add_executable( sara_moveit_pick_server
#        include/sara_moveit/pick_server.h
#        src/pick_server.cpp
#        )



include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

target_link_libraries(sara_moveit_move_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

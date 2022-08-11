cmake_minimum_required(VERSION 3.0.2)
project(fleet_turtle)

find_package(catkin 
  REQUIRED 
    genmsg 
    actionlib_msgs 
    actionlib
  COMPONENTS
    rospy
    roscpp
    std_msgs
    message_generation
)

add_message_files(
  FILES
    Setpoint.msg
)

# add_service_files(
#   FILES
#   Service2.srv
# )

add_action_files(
  DIRECTORY 
    action 
  FILES 
    Setpoint.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    actionlib_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES navigationturt
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

catkin_install_python(PROGRAMS
  scripts/explorer.py
  scripts/navigator.py
  scripts/action_move_base.py
  scripts/initial_position.py
  scripts/mapper.py
  scripts/read_from_txt.py
  scripts/reference_listener.py
  scripts/uv_cleaner.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

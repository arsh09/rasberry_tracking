cmake_minimum_required(VERSION 2.8.3)
project(rasberry_tracking)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  ros_numpy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  message_generation
  rasberry_perception
  bayes_tracking  # Must be the version from the github.com/RaymondKirk/bayes_tracking repo
)

find_package(Boost REQUIRED COMPONENTS thread)

# ==============================================
# Python scripts setup
# ==============================================
if(DEFINED ENV{INSTALL_RASBERRY_TRACKING_PYTHON_FROM_PIP})
    # Install python dependancies (first try to install locally, then to user site_packages)
    find_package(PythonInterp REQUIRED)
    set(return_status "1")
    set(failed_commands "")
    foreach(flag "" "--user")
        set(command "pip install -r ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt ${flag}")
        execute_process (COMMAND bash -c "${command}" OUTPUT_VARIABLE output_var ERROR_VARIABLE error_var
                         RESULT_VARIABLE return_status)
        message(WARNING "${command}Not a warning just output of pip install dependancies:\n${output_var}")

        if(return_status EQUAL "0")
            break()
        endif()
        list(APPEND failed_commands "\n\t- '${command}'")
    endforeach()
    if(NOT return_status EQUAL "0")
        message(SEND_ERROR "${output_var}")
        message(SEND_ERROR "${error_var}")
        message(FATAL_ERROR "Failed to install python requirements with:${failed_commands}")
    endif()
endif()

catkin_python_setup()

# ==============================================
# Service/Message files setup
# ==============================================
# add_service_files(FILES)

# add_message_files(FILES)

# generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs visualization_msgs rasberry_tracking rasberry_perception)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf std_msgs sensor_msgs geometry_msgs visualization_msgs bayes_tracking rasberry_perception
)

# ==============================================
# Build
# ==============================================

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )
add_executable(${PROJECT_NAME} src/${PROJECT_NAME}/tracking.cpp )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

# ==============================================
# Install
# ==============================================

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.hpp" PATTERN ".git" EXCLUDE )

install(DIRECTORY "launch" "config" DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

file(GLOB python_scripts_DIR "scripts/*.py")
file(GLOB bash_scripts_DIR "scripts/*.sh")
catkin_install_python(PROGRAMS ${python_scripts_DIR} ${bash_scripts_DIR} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

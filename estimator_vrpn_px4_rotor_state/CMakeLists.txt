cmake_minimum_required(VERSION 3.0.2)
project(estimator_vrpn_px4_rotor_state)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  ros1_utils
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(xgc2_math REQUIRED CONFIG)
find_package(xgc2_state_machine REQUIRED CONFIG)

add_message_files(
  FILES
  RigidStateEstimate.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES estimator_vrpn_px4_rotor_state_core estimator_vrpn_px4_rotor_state_ros
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp ros1_utils sensor_msgs std_msgs
  DEPENDS EIGEN3
)

set(PRIVATE_GENERATED_INCLUDE_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/private_generated_include
)
set(PRIVATE_RIGID_STATE_HEADER
  ${PRIVATE_GENERATED_INCLUDE_DIR}/${PROJECT_NAME}/RigidStateEstimate.h
)

add_custom_command(
  OUTPUT ${PRIVATE_RIGID_STATE_HEADER}
  COMMAND ${CMAKE_COMMAND} -E make_directory
          ${PRIVATE_GENERATED_INCLUDE_DIR}/${PROJECT_NAME}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/RigidStateEstimate.h
          ${PRIVATE_RIGID_STATE_HEADER}
  DEPENDS ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/RigidStateEstimate.h
)
add_custom_target(${PROJECT_NAME}_private_generated_headers
  DEPENDS ${PRIVATE_RIGID_STATE_HEADER}
)
add_dependencies(${PROJECT_NAME}_private_generated_headers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(estimator_vrpn_px4_rotor_state_core
  src/vrpn_px4_rotor_state_estimator_runtime.cpp
  src/state_machine/coasting_state.cpp
  src/state_machine/fault_state.cpp
  src/state_machine/health_monitor_state.cpp
  src/state_machine/initializing_state.cpp
  src/state_machine/running_state.cpp
  src/state_machine/self_check_state.cpp
)
add_dependencies(estimator_vrpn_px4_rotor_state_core
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(estimator_vrpn_px4_rotor_state_core PUBLIC cxx_std_17)
target_link_libraries(estimator_vrpn_px4_rotor_state_core
  xgc2_math::math
  xgc2_state_machine::state_machine
)

add_library(estimator_vrpn_px4_rotor_state_ros
  src/vrpn_px4_rotor_state_estimator_node.cpp
  src/input/rigid_state_input_producer.cpp
  src/output/rigid_state_output_consumer.cpp
)
add_dependencies(estimator_vrpn_px4_rotor_state_ros
  ${PROJECT_NAME}_private_generated_headers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(estimator_vrpn_px4_rotor_state_ros PUBLIC cxx_std_17)
target_include_directories(estimator_vrpn_px4_rotor_state_ros
  BEFORE PRIVATE
  ${PRIVATE_GENERATED_INCLUDE_DIR}
)
target_link_libraries(estimator_vrpn_px4_rotor_state_ros
  estimator_vrpn_px4_rotor_state_core
  xgc2_state_machine::state_machine
  ${catkin_LIBRARIES}
)

add_executable(vrpn_px4_rotor_state_estimator_node
  src/vrpn_px4_rotor_state_estimator_main.cpp
)
add_dependencies(vrpn_px4_rotor_state_estimator_node
  ${PROJECT_NAME}_private_generated_headers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(vrpn_px4_rotor_state_estimator_node PUBLIC cxx_std_17)
target_include_directories(vrpn_px4_rotor_state_estimator_node
  BEFORE PRIVATE
  ${PRIVATE_GENERATED_INCLUDE_DIR}
)
target_link_libraries(vrpn_px4_rotor_state_estimator_node
  estimator_vrpn_px4_rotor_state_ros
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_core_test
    test/vrpn_px4_rotor_state_estimator_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_core_test)
    target_compile_features(${PROJECT_NAME}_core_test PUBLIC cxx_std_17)
    target_link_libraries(${PROJECT_NAME}_core_test
      estimator_vrpn_px4_rotor_state_core
      xgc2_math::math
      xgc2_state_machine::state_machine
      ${catkin_LIBRARIES}
    )
  endif()
endif()

install(TARGETS estimator_vrpn_px4_rotor_state_core estimator_vrpn_px4_rotor_state_ros vrpn_px4_rotor_state_estimator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)

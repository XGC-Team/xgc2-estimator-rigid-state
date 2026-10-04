cmake_minimum_required(VERSION 3.10)
project(RigidStateNativeWire VERSION 1.0.0 LANGUAGES NONE)
include(CMakePackageConfigHelpers)
set(CMAKE_INSTALL_INCLUDEDIR "include" CACHE PATH "Header installation directory")
set(CMAKE_INSTALL_DATADIR "share" CACHE PATH "Architecture-independent data directory")
add_library(rigid_state_native_wire INTERFACE)
add_library(RigidStateNative::Wire ALIAS rigid_state_native_wire)
set_target_properties(rigid_state_native_wire PROPERTIES EXPORT_NAME Wire)
target_include_directories(rigid_state_native_wire INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
install(FILES ../include/estimator_vrpn_px4_rotor_state/native/rigid_state_wire_v1.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/estimator_vrpn_px4_rotor_state/native)
install(TARGETS rigid_state_native_wire EXPORT RigidStateNativeWireTargets)
set(wire_config_dir ${CMAKE_INSTALL_DATADIR}/cmake/RigidStateNativeWire)
install(EXPORT RigidStateNativeWireTargets NAMESPACE RigidStateNative::
  DESTINATION ${wire_config_dir})
configure_package_config_file(RigidStateNativeWireConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/RigidStateNativeWireConfig.cmake
  INSTALL_DESTINATION ${wire_config_dir})
# CMake 3.10's original SameMajorVersion helper skips architecture checks when
# the producer pointer size is empty. Preserve that header-only contract even
# when this LANGUAGES NONE exporter is added by a compiled parent project.
set(_rigid_wire_pointer_size "${CMAKE_SIZEOF_VOID_P}")
set(CMAKE_SIZEOF_VOID_P "")
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/RigidStateNativeWireConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
set(CMAKE_SIZEOF_VOID_P "${_rigid_wire_pointer_size}")
unset(_rigid_wire_pointer_size)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/RigidStateNativeWireConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/RigidStateNativeWireConfigVersion.cmake
  DESTINATION ${wire_config_dir})

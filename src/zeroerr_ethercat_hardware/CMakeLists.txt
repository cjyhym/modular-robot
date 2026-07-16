cmake_minimum_required(VERSION 3.8)
project(zeroerr_ethercat_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# ============================================================
# ROS 2 dependencies
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ============================================================
# EtherLab
# ============================================================

find_path(
  ETHERCAT_INCLUDE_DIR
  NAMES ecrt.h
  PATHS /usr/local/etherlab/include
  NO_DEFAULT_PATH
)

find_library(
  ETHERCAT_LIBRARY
  NAMES ethercat
  PATHS /usr/local/etherlab/lib
  NO_DEFAULT_PATH
)

if(NOT ETHERCAT_INCLUDE_DIR)
  message(FATAL_ERROR
    "没有找到 ecrt.h，请检查 /usr/local/etherlab/include")
endif()

if(NOT ETHERCAT_LIBRARY)
  message(FATAL_ERROR
    "没有找到 libethercat.so，请检查 /usr/local/etherlab/lib")
endif()

message(STATUS "EtherLab include: ${ETHERCAT_INCLUDE_DIR}")
message(STATUS "EtherLab library: ${ETHERCAT_LIBRARY}")

# ============================================================
# Hardware plugin shared library
# ============================================================

add_library(
  ${PROJECT_NAME}
  SHARED
  src/zeroerr_system.cpp
)

target_compile_features(
  ${PROJECT_NAME}
  PUBLIC
  cxx_std_17
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${ETHERCAT_INCLUDE_DIR}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(
  ${PROJECT_NAME}
  ${ETHERCAT_LIBRARY}
  rt
)

# 让插件运行时能够找到 /usr/local/etherlab/lib/libethercat.so
set_target_properties(
  ${PROJECT_NAME}
  PROPERTIES
    BUILD_RPATH "/usr/local/etherlab/lib"
    INSTALL_RPATH "/usr/local/etherlab/lib"
)

# ============================================================
# pluginlib export
# ============================================================

pluginlib_export_plugin_description_file(
  hardware_interface
  zeroerr_hardware.xml
)

# ============================================================
# Install
# ============================================================

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES zeroerr_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)

ament_export_include_directories(include)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)


install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES config/controllers.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()

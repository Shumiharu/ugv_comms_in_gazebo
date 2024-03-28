# gz_add_system(rf-comms
#   SOURCES
#   RFComms.cc
#   PUBLIC_LINK_LIBS
#     ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
# )

cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-cmake2 REQUIRED)

project(RFComms_custom)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-gazebo6 REQUIRED)
add_library(RFComms_custom SHARED RFComms_custom.cc)
set_property(TARGET RFComms_custom PROPERTY CXX_STANDARD 17)
target_link_libraries(RFComms_custom
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo6::ignition-gazebo6)


cmake_minimum_required(VERSION 3.16)
project(wind_plugin)

# Find required packages for Gazebo Sim (Jazzy compatible)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-math7 REQUIRED)

# Create the wind plugin library
add_library(WindPlugin SHARED
  WindPlugin.cpp
)

# Link Gazebo libraries
target_link_libraries(WindPlugin
  PUBLIC
    gz-sim8::core
    gz-math7::core
)

# Set compiler flags for plugin
set_target_properties(WindPlugin PROPERTIES
  COMPILE_FLAGS "-fPIC"
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

cmake_minimum_required(VERSION 3.5)
project(pb_nav2_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(pb_back_up_frees_space_behavior SHARED
  src/behaviors/back_up_free_space.cpp
)

ament_auto_add_library(layers SHARED
  src/layers/intensity_obstacle_layer.cpp
  src/layers/intensity_voxel_layer.cpp
)

if(BUILD_TESTING)
set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

ament_auto_package()

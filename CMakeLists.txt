# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "${CMAKE_CURRENT_LIST_DIR}/components"
  "${CMAKE_CURRENT_LIST_DIR}/external/rammp-rtps/components"
)

set(
  COMPONENTS
  # NOTE: you can add additional esp-idf and espp components you want to use to
  #       the line below.
  #
  # Alternatively, If they exist in the component registry, then you can ignore
  # adding them here, and instead use call
  #
  # `idf.py add-dependency "<namespace>/<component>^<version>"`
  #
  # e.g.
  #
  # `idf.py add-dependency "espp/task>=1.0"`
  "main esptool_py"
  CACHE STRING
  "List of components to include"
  )

# TODO: update this with your project's name
project(rammp_x_omni_chair)

set(CMAKE_CXX_STANDARD 20)

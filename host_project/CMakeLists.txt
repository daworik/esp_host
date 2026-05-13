cmake_minimum_required(VERSION 3.16)

# Set the project name (this becomes the binary name)
set(PROJECT_VER "1.0")
set(PROJECT_VER_NUMBER 1)

set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(host_drone_controller)

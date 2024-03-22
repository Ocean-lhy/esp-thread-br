# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(ENV{EXTRA_CFLAGS} "-Werror -Werror=deprecated-declarations -Werror=unused-variable \
    -Werror=unused-but-set-variable -Werror=unused-function -Wstrict-prototypes")

set(ENV{EXTRA_CXXFLAGS} "-Werror -Werror=deprecated-declarations -Werror=unused-variable \
    -Werror=unused-but-set-variable -Werror=unused-function")

# (Not part of the boilerplate)
# This example uses an extra component for common functions such as Wi-Fi and Ethernet connection.
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_ot_br)

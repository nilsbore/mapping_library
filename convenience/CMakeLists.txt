project(convenience)

set(CMAKE_CXX_FLAGS "-O4 -g -Wall -fPIC -std=c++0x")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Main library
add_library(files src/files.cpp)
add_library(conversions src/conversions.cpp)

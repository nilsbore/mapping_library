project(ransac_primitives)

set(CMAKE_CXX_FLAGS "-O4 -g -Wall -fPIC -std=c++0x")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# PCL
set(PCL_DIR "/usr/share/pcl-1.7")
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV
include_directories(/opt/ros/hydro/include)
link_directories(/opt/ros/hydro/lib)

# MAIN LIBRARY
add_library(base_primitive src/base_primitive.cpp)
add_library(primitive_extractor src/primitive_extractor.cpp src/base_primitive.h)
add_library(primitive_visualizer src/primitive_visualizer.cpp)
add_library(plane_primitive src/plane_primitive.cpp)
add_library(sphere_primitive src/sphere_primitive.cpp)
add_library(cylinder_primitive src/cylinder_primitive.cpp)
add_library(primitive_leaf src/primitive_leaf.cpp)
add_library(primitive_octree src/primitive_octree.cpp)
add_executable(test_primitives src/test_primitives.cpp)
add_executable(test_find_point src/test_find_point.cpp)
#add_executable(test_sphere_components src/test_sphere_components.cpp)

target_link_libraries(base_primitive primitive_octree opencv_core opencv_imgproc opencv_highgui ${PCL_LIBRARIES})
target_link_libraries(plane_primitive base_primitive ${PCL_LIBRARIES})
target_link_libraries(sphere_primitive base_primitive ${PCL_LIBRARIES})
target_link_libraries(cylinder_primitive base_primitive ${PCL_LIBRARIES})
target_link_libraries(primitive_leaf ${PCL_LIBRARIES})
target_link_libraries(primitive_octree primitive_leaf base_primitive ${PCL_LIBRARIES})
target_link_libraries(primitive_extractor base_primitive primitive_visualizer primitive_octree ${PCL_LIBRARIES})
target_link_libraries(primitive_visualizer ${PCL_LIBRARIES})
target_link_libraries(test_primitives primitive_extractor plane_primitive primitive_visualizer ${PCL_LIBRARIES})
target_link_libraries(test_find_point primitive_extractor plane_primitive sphere_primitive cylinder_primitive primitive_visualizer ${PCL_LIBRARIES})
#target_link_libraries(test_sphere_components sphere_primitive)

project(graph_primitives)

set(CMAKE_CXX_FLAGS "-O4 -g -Wall -fPIC -std=c++0x")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# PCL
set(PCL_DIR "/opt/ros/groovy/share/pcl-1.6")
find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV
include_directories(/opt/ros/groovy/include/opencv2)
link_directories(/opt/ros/groovy/lib)

# spatial_primitives
include_directories(${PROJECT_SOURCE_DIR}/../ransac_primitives/src)
link_directories(${PROJECT_SOURCE_DIR}/../ransac_primitives/lib)

# convenience
include_directories(${PROJECT_SOURCE_DIR}/../convenience/src)
link_directories(${PROJECT_SOURCE_DIR}/../convenience/lib)

# MAIN LIBRARY
add_library(graph_extractor src/graph_extractor.cpp)
add_executable(test_graph src/test_graph.cpp)
add_executable(test_bgl src/test_bgl.cpp)
add_executable(test_plot_bgl src/test_plot_bgl.cpp)
add_executable(display_graph src/display_graph.cpp)

target_link_libraries(display_graph ${PCL_LIBRARIES} ${VTK_LIBRARIES})

target_link_libraries(graph_extractor conversions primitive_extractor primitive_octree primitive_leaf primitive_visualizer
base_primitive plane_primitive sphere_primitive cylinder_primitive opencv_core opencv_imgproc opencv_highgui ${PCL_LIBRARIES})

target_link_libraries(test_graph graph_extractor files primitive_extractor primitive_octree primitive_leaf primitive_visualizer
base_primitive plane_primitive sphere_primitive cylinder_primitive opencv_core opencv_imgproc opencv_highgui ${PCL_LIBRARIES})

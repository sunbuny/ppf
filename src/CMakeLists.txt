find_package(VTK REQUIRED)
include(${VTK_USE_FILE})


include_directories( ${OPENGL_INCLUDE_DIRS})

include_directories(/usr/include/eigen3/)

add_library(utils utils.cpp MurmurHash3.cpp)

add_executable(test_main test/main.cpp )
target_link_libraries(test_main utils)

add_executable(vis_main test/vis.cpp)
target_link_libraries(vis_main ${VTK_LIBRARIES} utils)

add_executable(test_simple test/test_simple.cpp)

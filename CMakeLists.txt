cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

project( 3DComplete )

find_package( PCL )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(progetto src/codice_progetto.cpp)
add_executable(coplanarity src/coplanarity.cpp)

target_link_libraries(progetto ${PCL_LIBRARIES})
target_link_libraries(coplanarity ${PCL_LIBRARIES})
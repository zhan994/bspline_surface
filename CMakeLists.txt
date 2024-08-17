cmake_minimum_required(VERSION 3.0.2)
project(bspline_surface)

# for Debug
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE "Release")
find_package(PCL 1.10 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

aux_source_directory(src/ SOURCES)
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
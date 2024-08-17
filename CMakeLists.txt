cmake_minimum_required(VERSION 3.0.2)
project(bspline_surface)


find_package(PCL 1.10 REQUIRED)

# for Debug
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE "Release")


include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)

file(GLOB SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable(${PROJECT_NAME} ${SOURCES})

# link_directories(${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/lib)

# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}  
)
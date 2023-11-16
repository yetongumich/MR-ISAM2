cmake_minimum_required(VERSION 3.0)
enable_testing()
project(mr_isam2 CXX C)

find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBS gtsam)

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIR})

file(GLOB sources "*.cpp")
file(GLOB headers "*.h")

include_directories(".")

add_library(mr_isam2 SHARED ${sources} ${headers})
target_link_libraries(mr_isam2 ${GTSAM_LIBS})
set_target_properties(mr_isam2 PROPERTIES LINKER_LANGUAGE CXX)


add_subdirectory(tests)
add_subdirectory(scripts)
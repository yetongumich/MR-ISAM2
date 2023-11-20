cmake_minimum_required(VERSION 3.0)
project(mr_isam2 CXX C)

set(CMAKE_CXX_STANDARD 17)

# System Dependencies
find_package(GTSAM REQUIRED)

# Build the MR-iSAM2 Library
file(GLOB_RECURSE mr_isam2_srcs "src/*.cpp" "include/*.h")
add_library(mr_isam2 SHARED ${mr_isam2_srcs})
target_link_libraries(mr_isam2 gtsam)
target_include_directories(mr_isam2 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
set_target_properties(mr_isam2 PROPERTIES LINKER_LANGUAGE CXX)

# Build the Tests
# Run with `make check` (GTSAM standard language) 
add_subdirectory(tests)
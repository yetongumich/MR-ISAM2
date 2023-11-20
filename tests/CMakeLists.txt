find_package(GTest REQUIRED)
find_package(GTSAMCMakeTools REQUIRED)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

gtsamAddTestsGlob(tests "test*.cpp" "" "mr_isam2")
enable_testing()
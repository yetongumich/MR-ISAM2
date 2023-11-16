gtsamAddTestsGlob(tests "test*.cpp" "" "mr_isam2")

option(DEBUG "Turn on debug mode to print out more info?" OFF)

if(DEBUG)
    add_definitions(-DDEBUG=1)
else()
    add_definitions(-DDEBUG=0)
endif()

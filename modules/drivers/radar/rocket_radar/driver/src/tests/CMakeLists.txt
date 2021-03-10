include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIBS_DIR})

if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-Wall -Wno-deprecated -pthread)
endif()
add_executable(rra-test gtestmain.cpp test-sensorgroup.cpp test-scan.cpp)
target_link_libraries(rra-test rrapi gtest ${CMAKE_THREAD_LIBS_INIT} ${PLATFORM_LIBS})

# vim: syntax=cmake tw=130

list(APPEND GTEST_FILES
   # ${CMAKE_CURRENT_SOURCE_DIR}/test-detfilter.cpp
   # ${CMAKE_CURRENT_SOURCE_DIR}/test-session.cpp
   # ${CMAKE_CURRENT_SOURCE_DIR}/test-serialize.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test-scan.cpp
    )
set(GTEST_FILES ${GTEST_FILES} PARENT_SCOPE)

# list(APPEND GTEST_NAMES detfilter session serialize scan)
list(APPEND GTEST_NAMES scan radar_connection radar_scanning)
set(GTEST_NAMES ${GTEST_NAMES} PARENT_SCOPE)

# vim: syntax=cmake tw=130

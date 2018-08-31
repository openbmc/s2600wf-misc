cmake_minimum_required (VERSION 3.6)
project (sel-logger CXX)

add_executable (sel-logger src/sel-logger.cpp)

target_include_directories (sel-logger PRIVATE ${CMAKE_SOURCE_DIR})

target_link_libraries (sel-logger systemd sdbusplus pthread)

install (TARGETS sel-logger
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib/static)

find_package (Boost 1.66 REQUIRED)
include_directories (${BOOST_SRC_DIR})

add_definitions (-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions (-DBOOST_SYSTEM_NO_DEPRECATED)
add_definitions (-DBOOST_ALL_NO_LIB)
add_definitions (-DBOOST_NO_RTTI)
add_definitions (-DBOOST_NO_TYPEID)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-rtti")

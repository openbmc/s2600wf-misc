cmake_minimum_required (VERSION 3.1 FATAL_ERROR)
set (BUILD_SHARED_LIBRARIES OFF)
include (ExternalProject)
set (CMAKE_CXX_STANDARD 23)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

project (hsbp-manager CXX)

set (
    CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -lstdc++fs \
    -Werror \
    -Wall \
    -Wextra \
    -Wshadow \
    -Wnon-virtual-dtor \
    -Wold-style-cast \
    -Wcast-align \
    -Wunused \
    -Woverloaded-virtual \
    -Wpedantic \
    -Wconversion \
    -Wmisleading-indentation \
    -Wduplicated-cond \
    -Wduplicated-branches \
    -Wlogical-op \
    -Wnull-dereference \
    -Wuseless-cast \
    -Wdouble-promotion \
    -Wformat=2 \
    -fno-rtti"
)

find_package(Boost REQUIRED)

add_definitions (-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions (-DBOOST_SYSTEM_NO_DEPRECATED)
add_definitions (-DBOOST_ALL_NO_LIB)
add_definitions (-DBOOST_NO_RTTI)
add_definitions (-DBOOST_NO_TYPEID)
add_definitions (-DBOOST_ASIO_DISABLE_THREADS)

include_directories (${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable (hsbp-manager src/hsbp_manager.cpp)

target_link_libraries (hsbp-manager -lsystemd)
target_link_libraries (hsbp-manager i2c)
target_link_libraries (hsbp-manager ${Boost_LIBRARIES})
target_link_libraries (hsbp-manager sdbusplus)
target_link_libraries (hsbp-manager stdc++fs)
target_link_libraries (hsbp-manager gpiodcxx)


find_package (PkgConfig REQUIRED)
pkg_get_variable(SYSTEMD_SYSTEM_UNIT_DIR systemd systemdsystemunitdir)
pkg_check_modules(SDBUSPLUSPLUS sdbusplus REQUIRED)

set (SERVICE_FILES ${PROJECT_SOURCE_DIR}/service_files/hsbp-manager.service)

install (TARGETS hsbp-manager DESTINATION bin)
install (FILES ${SERVICE_FILES} DESTINATION "${SYSTEMD_SYSTEM_UNIT_DIR}")

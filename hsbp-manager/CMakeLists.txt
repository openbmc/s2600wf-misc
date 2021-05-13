cmake_minimum_required (VERSION 3.1 FATAL_ERROR)
set (BUILD_SHARED_LIBRARIES OFF)
include (ExternalProject)
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
if (NOT YOCTO) # to download gtest
    include ("cmake/HunterGate.cmake")
    huntergate (URL "https://github.com/ruslo/hunter/archive/v0.18.64.tar.gz"
                SHA1 "baf9c8cc4f65306f0e442b5419967b4c4c04589a")
endif ()

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
    "
)
# Temporarily disable rtti
# See https://github.com/chriskohlhoff/asio/issues/533
#    -fno-rtti

option (YOCTO "Enable Building in Yocto" OFF)

if (NOT YOCTO)
    externalproject_add (
        Boost URL
        https://boostorg.jfrog.io/artifactory/main/release/1.76.0/source/boost_1_76_0.tar.gz
        URL_MD5 e425bf1f1d8c36a3cd464884e74f007a SOURCE_DIR
        "${CMAKE_BINARY_DIR}/boost-src" BINARY_DIR
        "${CMAKE_BINARY_DIR}/boost-build" CONFIGURE_COMMAND "" BUILD_COMMAND ""
        INSTALL_COMMAND mkdir -p "${CMAKE_BINARY_DIR}/prefix/include/" && cp -R
        ${CMAKE_BINARY_DIR}/boost-src/boost ${CMAKE_BINARY_DIR}/prefix/include
    ) # requires apt install autoconf-archive and autoconf
    externalproject_add (sdbusplus-project PREFIX
                         ${CMAKE_BINARY_DIR}/sdbusplus-project GIT_REPOSITORY
                         https://github.com/openbmc/sdbusplus.git GIT_TAG
                         fa3137a78e952134559bf7c41c4f29fa4afabe02 SOURCE_DIR
                         ${CMAKE_BINARY_DIR}/sdbusplus-src BINARY_DIR
                         ${CMAKE_BINARY_DIR}/sdbusplus-build CONFIGURE_COMMAND
                         "" BUILD_COMMAND cd ${CMAKE_BINARY_DIR}/sdbusplus-src
                         && meson build -Ddefault_library=static && ninja -C
                         build libsdbusplus.a INSTALL_COMMAND ""
                         LOG_DOWNLOAD ON)

    include_directories (SYSTEM ${CMAKE_BINARY_DIR}/sdbusplus-src)
    link_directories (${CMAKE_BINARY_DIR}/sdbusplus-src/build)

    include_directories (SYSTEM ${CMAKE_BINARY_DIR}/boost-src)
    set (CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR}/boost-src ${CMAKE_PREFIX_PATH})

    option (HUNTER_ENABLED "Enable hunter package pulling" ON)
    hunter_add_package (GTest)

    find_package (GTest CONFIG REQUIRED)
endif ()

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

if (NOT YOCTO)
    add_dependencies (hsbp-manager sdbusplus-project)
endif ()

set (SERVICE_FILES ${PROJECT_SOURCE_DIR}/service_files/hsbp-manager.service)

install (TARGETS hsbp-manager DESTINATION bin)
install (FILES ${SERVICE_FILES} DESTINATION /lib/systemd/system/)

cmake_minimum_required (VERSION 3.1 FATAL_ERROR)
set (BUILD_SHARED_LIBRARIES OFF)
include (ExternalProject)
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-rtti")

option (YOCTO "Enable Building in Yocto" OFF)

if (NOT YOCTO)
    externalproject_add (
        Boost URL
        https://dl.bintray.com/boostorg/release/1.71.0/source/boost_1_71_0.tar.gz
        URL_MD5 5f521b41b79bf8616582c4a8a2c10177 SOURCE_DIR
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

    include_directories (${CMAKE_BINARY_DIR}/sdbusplus-src)
    include_directories (${CMAKE_BINARY_DIR}/nlohmann/include)
    include_directories (${CMAKE_BINARY_DIR}/nlohmann/include/nlohmann)
    include_directories (${CMAKE_BINARY_DIR}/phosphor-dbus-interfaces/include)
    link_directories (${CMAKE_BINARY_DIR}/sdbusplus-src/build)

    include_directories (${CMAKE_BINARY_DIR}/boost-src)
    set (CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR}/boost-src ${CMAKE_PREFIX_PATH})
    option (ENABLE_TEST "Enable Google Test" OFF)
    if (ENABLE_TEST)
        hunter_add_package (GTest)
        find_package (GTest CONFIG REQUIRED)
        enable_testing ()
    endif ()
endif ()

add_definitions (-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions (-DBOOST_SYSTEM_NO_DEPRECATED)
add_definitions (-DBOOST_ALL_NO_LIB)
add_definitions (-DBOOST_NO_RTTI)
add_definitions (-DBOOST_NO_TYPEID)
add_definitions (-DBOOST_ASIO_DISABLE_THREADS)

include_directories (${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories (${Boost_INCLUDE_DIRS})
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable (callback-manager src/callback_manager.cpp)

target_link_libraries (callback-manager -lsystemd)
target_link_libraries (callback-manager stdc++fs)
target_link_libraries (callback-manager ${Boost_LIBRARIES})
target_link_libraries (callback-manager sdbusplus)

if (NOT YOCTO)
    add_dependencies (callback-manager sdbusplus-project)
endif ()

set (SERVICE_FILES ${PROJECT_SOURCE_DIR}/service_files/callback-manager.service)

install (TARGETS callback-manager DESTINATION sbin)
install (FILES ${SERVICE_FILES} DESTINATION /lib/systemd/system/)

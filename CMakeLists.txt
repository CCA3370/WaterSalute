cmake_minimum_required(VERSION 3.16)
project(WaterSalute VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Force x86_64 architecture on macOS for X-Plane 12 compatibility
if(APPLE)
    set(CMAKE_OSX_ARCHITECTURES "x86_64" CACHE STRING "Build architectures for macOS" FORCE)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "10.15" CACHE STRING "Minimum macOS deployment version" FORCE)
endif()

# X-Plane SDK paths
set(XPLM_SDK_PATH "${CMAKE_SOURCE_DIR}/SDK")

# Source files
set(SOURCES
    src/WaterSalute.cpp
    src/Common.cpp
    src/RoadNetwork.cpp
    src/PathPlanning.cpp
    src/FireTruck.cpp
)

# Platform detection
if(WIN32)
    set(XPLM_PLATFORM "Windows")
    set(XPLM_PLATFORM_DEFINE "IBM=1" "APL=0" "LIN=0")
    set(XPLM_LIBRARY_SUFFIX ".xpl")
elseif(APPLE)
    set(XPLM_PLATFORM "macOS")
    set(XPLM_PLATFORM_DEFINE "IBM=0" "APL=1" "LIN=0")
    set(XPLM_LIBRARY_SUFFIX ".xpl")
else()
    set(XPLM_PLATFORM "Linux")
    set(XPLM_PLATFORM_DEFINE "IBM=0" "APL=0" "LIN=1")
    set(XPLM_LIBRARY_SUFFIX ".xpl")
endif()

message(STATUS "Building for platform: ${XPLM_PLATFORM}")

# Create shared library
add_library(WaterSalute SHARED ${SOURCES})

# Include directories
target_include_directories(WaterSalute PRIVATE
    ${XPLM_SDK_PATH}/CHeaders/XPLM
    ${XPLM_SDK_PATH}/CHeaders/Widgets
)

# Compile definitions
target_compile_definitions(WaterSalute PRIVATE
    ${XPLM_PLATFORM_DEFINE}
    XPLM200=1
    XPLM210=1
    XPLM300=1
    XPLM301=1
    XPLM302=1
    XPLM303=1
    XPLM400=1
    XPLM410=1
    XPLM420=1
)

# Platform-specific settings
if(WIN32)
    target_compile_definitions(WaterSalute PRIVATE
        _CRT_SECURE_NO_WARNINGS
        WIN32_LEAN_AND_MEAN
        NOMINMAX
    )
    target_link_options(WaterSalute PRIVATE
        /DEF:${CMAKE_SOURCE_DIR}/src/WaterSalute.def
    )
    # Link against XPLM library
    target_link_libraries(WaterSalute PRIVATE
        ${XPLM_SDK_PATH}/Libraries/Win/XPLM_64.lib
    )
elseif(APPLE)
    target_compile_options(WaterSalute PRIVATE
        -fvisibility=hidden
        -fPIC
    )
    target_link_options(WaterSalute PRIVATE
        -flat_namespace
        -undefined suppress
    )
elseif(UNIX)
    target_compile_options(WaterSalute PRIVATE
        -fvisibility=hidden
        -fPIC
    )
    target_link_options(WaterSalute PRIVATE
        -Wl,--version-script=${CMAKE_SOURCE_DIR}/src/WaterSalute.sym
    )
endif()

# Output settings
set_target_properties(WaterSalute PROPERTIES
    PREFIX ""
    OUTPUT_NAME "WaterSalute"
    SUFFIX ${XPLM_LIBRARY_SUFFIX}
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/WaterSalute"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/WaterSalute"
)

# For multi-config generators (e.g. Visual Studio), ensure output goes directly to WaterSalute/
if(CMAKE_CONFIGURATION_TYPES)
    foreach(config ${CMAKE_CONFIGURATION_TYPES})
        string(TOUPPER ${config} config_upper)
        set_target_properties(WaterSalute PROPERTIES
            LIBRARY_OUTPUT_DIRECTORY_${config_upper} "${CMAKE_BINARY_DIR}/WaterSalute"
            RUNTIME_OUTPUT_DIRECTORY_${config_upper} "${CMAKE_BINARY_DIR}/WaterSalute"
        )
    endforeach()
endif()

# Copy resources to build directory
add_custom_command(TARGET WaterSalute POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/WaterSalute/resources"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/resources"
        "${CMAKE_BINARY_DIR}/WaterSalute/resources"
    COMMENT "Copying resources to build directory"
)

# Installation
install(TARGETS WaterSalute
    LIBRARY DESTINATION WaterSalute
    RUNTIME DESTINATION WaterSalute
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/resources/
    DESTINATION WaterSalute/resources
)

message(STATUS "WaterSalute plugin configuration complete")
message(STATUS "  SDK Path: ${XPLM_SDK_PATH}")
message(STATUS "  Output: ${CMAKE_BINARY_DIR}/WaterSalute/WaterSalute${XPLM_LIBRARY_SUFFIX}")

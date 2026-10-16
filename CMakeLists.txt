cmake_minimum_required(VERSION 3.24)

# Forma Motus �?" procedural locomotion pose warping.
# R4.5 detaches the former UpperBodyInteraction module from this plugin.
# R5.1: dual locomotion providers + stable bank, with an explicit build revision to prevent stale-object hybrid DLLs.
project(FormaMotus VERSION 0.5.13 LANGUAGES CXX)

# The implementation of Universal Kinematics is NOT compiled into this plugin.
# By default we consume the SDK from the already-separated sibling project shown
# by the user in the sibling project's sdk directory.
set(
    UNIVERSAL_KINEMATICS_SDK_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../05 - UK/sdk"
    CACHE PATH
    "Path to the Universal Kinematics SDK directory"
)

if(NOT EXISTS "${UNIVERSAL_KINEMATICS_SDK_DIR}/UniversalKinematics.h" OR
   NOT EXISTS "${UNIVERSAL_KINEMATICS_SDK_DIR}/UniversalKinematicsAPI.h")
    message(FATAL_ERROR
        "Universal Kinematics SDK not found at: ${UNIVERSAL_KINEMATICS_SDK_DIR}\n"
        "Point UNIVERSAL_KINEMATICS_SDK_DIR to the sdk folder of the compiled UniversalKinematics project."
    )
endif()

set(headers
    src/PCH.h
    src/Hooks.h
    src/UKClient.h
    src/IncessusClient.h
    src/MotionSourceClient.h
    src/MotusClient.h
    src/FeetOfSkyrimBridge.h
    src/StrideWarpModule.h
    src/SlopeWarpModule.h
    src/TerrainSDK.h
    src/IncessusSDK/IncessusAPI.h
    src/IncessusSDK/AnimalLocomotionRevolutionAPI.h
    src/MotusContinuusSDK/MotusContinuusAPI.h
    src/AnimalMotionEngine.h
    src/AngularCurveModule.h
    src/SpeedGovernorModule.h
)

set(sources
    src/main.cpp
    src/Hooks.cpp
    src/UKClient.cpp
    src/IncessusClient.cpp
    src/MotionSourceClient.cpp
    src/MotusClient.cpp
    src/FeetOfSkyrimBridge.cpp
    src/StrideWarpModule.cpp
    src/SlopeWarpModule.cpp
    src/AnimalMotionEngine.cpp
    src/AngularCurveModule.cpp
    src/SpeedGovernorModule.cpp
)

find_package(CommonLibSSE CONFIG REQUIRED)

# VERSION/AUTHOR are intentionally not forwarded here because the user's
# CommonLibSSE-NG 3.6.0 has the known parser bug on that argument path.
add_commonlibsse_plugin(${PROJECT_NAME}
    NAME "FormaMotus"
    USE_ADDRESS_LIBRARY
    SOURCES ${headers} ${sources}
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
# Changing this definition changes every compile command, forcing Ninja to rebuild the whole target once.
target_compile_definitions(${PROJECT_NAME} PRIVATE FORMA_MOTUS_BUILD_REV=513)
target_precompile_headers(${PROJECT_NAME} PRIVATE src/PCH.h)
target_include_directories(${PROJECT_NAME} PRIVATE
    src
    "${UNIVERSAL_KINEMATICS_SDK_DIR}"
)

# No UniversalKinematics.cpp/.h implementation is present in this project and
# no import library is required: UKClient resolves the provider's C ABI from
# UniversalKinematics.dll at runtime.

if(DEFINED ENV{SKYRIM_SKSE64_PLUGIN_DIR})
    add_custom_command(
        TARGET ${PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:${PROJECT_NAME}>"
            "$ENV{SKYRIM_SKSE64_PLUGIN_DIR}"
        VERBATIM
    )
endif()


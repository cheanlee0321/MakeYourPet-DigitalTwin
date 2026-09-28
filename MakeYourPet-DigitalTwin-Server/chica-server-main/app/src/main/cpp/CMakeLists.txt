cmake_minimum_required(VERSION 3.22.1)
project(chica_gait)

add_library(chica_gait SHARED
    apk_model.cpp
    pulse_conversion.cpp
    chica_gait_jni.cpp
)

target_compile_features(chica_gait PRIVATE cxx_std_17)
target_include_directories(chica_gait PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

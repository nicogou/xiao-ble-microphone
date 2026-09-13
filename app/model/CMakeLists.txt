cmake_minimum_required(VERSION 3.20.0)

if(CONFIG_APP_KWS)
    RECURSIVE_FIND_FILE_APPEND(MODEL_SOURCE "tflite-model" "*.cpp")
    zephyr_include_directories(.)
    zephyr_library_sources(${MODEL_SOURCE})
endif()


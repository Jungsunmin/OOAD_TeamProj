cmake_minimum_required(VERSION 3.20)
project(rvc_sw_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_TESTS "Enable unit tests" ON)

# Core library
add_library(rvc
  src/rvc_controller.cpp
)

# Allow includes like: #include "rvc_controller.h"
target_include_directories(rvc PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# App executable
add_executable(rvc_app
  src/main.cpp
)
target_link_libraries(rvc_app PRIVATE rvc)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
endif()

if(ENABLE_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
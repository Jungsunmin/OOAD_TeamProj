include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)

# For Windows/MSVC compatibility (harmless on macOS)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(googletest)

add_executable(rvc_tests
  UTest_ButtonInterface.cpp
  UTest_Controller.cpp
  UTest_DriveManager.cpp
  UTest_PathPlanner.cpp
  UTest_Timer.cpp
  UTest_CleanerManager.cpp
)

target_include_directories(rvc_tests
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(rvc_tests
  PRIVATE
    rvc
    GTest::gmock_main
)

include(GoogleTest)
gtest_discover_tests(rvc_tests)
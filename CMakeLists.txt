cmake_minimum_required(VERSION 3.24)
project(pclcloud_example)

set(CMAKE_CXX_STANDARD 17)

# Set Qt5 environment variables
set(Qt5_DIR "/opt/homebrew/Cellar/qt@5/5.15.8_2/lib/cmake/Qt5")
set(CMAKE_PREFIX_PATH "/opt/homebrew/opt/qt@5")
set(CMAKE_MODULE_PATH "/opt/homebrew/opt/qt@5/lib/cmake")

# Add the executable
add_executable(pclcloud_example main.cpp point_cloud_generator.cpp point_cloud_visualizer.cpp)

# Find the required PCL package
find_package(PCL 1.11 REQUIRED)

# Link the PCL libraries and include the necessary directories
target_link_libraries(pclcloud_example ${PCL_LIBRARIES})
target_include_directories(pclcloud_example PUBLIC ${PCL_INCLUDE_DIRS})

# Remove unsupported optimization flags for Clang
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    string(REPLACE "-ffloat-store" "" PCL_COMPILE_FLAGS "${PCL_COMPILE_FLAGS}")
    set(PCL_COMPILE_FLAGS "${PCL_COMPILE_FLAGS}")
endif()

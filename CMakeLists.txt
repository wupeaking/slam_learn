cmake_minimum_required(VERSION 2.6)
project(gtsamexamples CXX C)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++11 -DBOOST_MATH_DISABLE_FLOAT128 -O3")

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(
    include

    # /usr/local/include/eigen3/
)
message("gtsam include: " ${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam) # TODO: automatic search libs
message("gtsam: " ${GTSAM_LIBRARIES})

add_executable(gtsam_demo gtsam_demo.cc)

# link_libraries("/lib/x86_64-linux-gnu/libtbb.so.12")
target_link_directories(gtsam_demo PUBLIC "/usr/local/lib")

# target_include_directories(gtsam_demo PUBLIC "/usr/local/include/eigen3/")
target_link_libraries(gtsam_demo ${GTSAM_LIBRARIES} tbb metis-gtsam)
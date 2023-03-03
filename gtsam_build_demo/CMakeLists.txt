# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(
    include
)
message("gtsam include: " ${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam) # TODO: automatic search libs
message("gtsam: " ${GTSAM_LIBRARIES})

add_library(gtsam_dynamic SHARED gtsam_build.cc)

# link_libraries("/lib/x86_64-linux-gnu/libtbb.so.12")
target_link_directories(gtsam_dynamic PUBLIC "/usr/local/lib")

# target_include_directories(gtsam_demo PUBLIC "/usr/local/include/eigen3/")
target_link_libraries(gtsam_dynamic ${GTSAM_LIBRARIES} tbb metis-gtsam)
install(TARGETS gtsam_dynamic DESTINATION lib)
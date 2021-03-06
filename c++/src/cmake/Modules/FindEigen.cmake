find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
    message(STATUS "Eigen_INCLUDE_DIRS: ${Eigen3_INCLUDE_DIR}")
    message(STATUS "Eigen_LIBRARIES: ${Eigen3_LIBRARIES}")
    message(STATUS "Eigen_VERSION: ${Eigen3_VERSION}")
endif()

if (NOT Eigen3_FOUND)
    message(FATAL_ERROR "Could not find Eigen.")
endif()

include_directories(
        ${Eigen3_INCLUDE_DIR}
)
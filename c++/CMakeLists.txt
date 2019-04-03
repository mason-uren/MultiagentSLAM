cmake_minimum_required(VERSION 3.2)
project(MultiAgentSLAM VERSION 1.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)

# Modules
include("src/cmake/Modules/FindBoost.cmake")
include("src/cmake/Modules/FindEigen.cmake")
include("src/cmake/Modules/FindMKL.cmake")

# Find external libraries/headers
add_subdirectory(src/lib/nholmann_json)
add_subdirectory(src/lib/shared_structs)
add_subdirectory(src/lib/interfaces)
add_subdirectory(src/lib/templates)
add_subdirectory(src/lib/tools)

set(DIR_MACRO ${CMAKE_CURRENT_LIST_DIR}/src/src/)
set(
        PROJECT_SOURCES
        ${DIR_MACRO}Agent/Detection/Detection.cpp
        ${DIR_MACRO}Agent/Moments/Moments.cpp
        ${DIR_MACRO}Agent/Rover/Rover.cpp
        ${DIR_MACRO}Agent/Rover/RoverFactory.cpp
        ${DIR_MACRO}Slam/ActiveRovers/ActiveRovers.cpp
        ${DIR_MACRO}Slam/SEIF/Seif.cpp
        ${DIR_MACRO}Slam/SlamAdapter/SlamAdapter.cpp
        ${DIR_MACRO}Utilities/ConfigParser/ConfigParser.cpp
        ${DIR_MACRO}Utilities/Equations/Equations.cpp
        ${DIR_MACRO}Utilities/BinaryTree/RedBlackTree.cpp
)

# EXECUTABLE
add_executable(
        ${PROJECT_NAME} src/src/ros_adapter.cpp ${PROJECT_SOURCES}
)

target_include_directories(${PROJECT_NAME}
        PRIVATE src/lib/nholmann_json/include
        PRIVATE src/lib/shared_structs/include
        PRIVATE src/lib/interfaces/include
        PRIVATE src/lib/templates/include/Filters
        PRIAVTE src/lib/templates/include/Matrix
        PRIVATE src/lib/templates/include/MatrixManipulator
        PRIVATE src/lib/tools/include
)

target_link_libraries(${PROJECT_NAME}
        PUBLIC
            Boost::boost
            Eigen3::Eigen
)

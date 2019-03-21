cmake_minimum_required(VERSION 3.2)
project(MultiAgentSLAM VERSION 0.1 LANGUAGES CXX)

# Find external libraries/headers
add_subdirectory(src/lib/nholmann_json)
add_subdirectory(src/lib/shared_structs)
add_subdirectory(src/lib/interfaces)
add_subdirectory(src/lib/templates)
add_subdirectory(src/lib/tools)

set(CMAKE_CXX_STANDARD 14)

# BOOST
find_package(Boost REQUIRED)
if (Boost_FOUND)
    message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
    message(STATUS "Boost_VERSION: ${Boost_VERSION}")
endif ()
if(NOT Boost_FOUND)
    message(FATAL_ERROR "Could not find boost!")
endif()

include_directories(
        ${Boost_INCLUDE_DIR}
        ${catkin_INCLUDE_DIR}
)

set(DIR_MACRO src/src/)
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
        MultiAgentSLAM src/src/ros_adapter.cpp ${PROJECT_SOURCES}
)

target_include_directories(MultiAgentSLAM
        PRIVATE src/lib/nholmann_json/include
        PRIVATE src/lib/shared_structs/include
        PRIVATE src/lib/interfaces/include
        PRIVATE src/lib/templates/include/Filters
        PRIAVTE src/lib/templates/include/Matrix
        PRIVATE src/lib/templates/include/MatrixManipulator
        PRIVATE src/lib/tools/include
)

target_link_libraries(
        MultiAgentSLAM LINK_PUBLIC ${Boost_libraries}
)




cmake_minimum_required(VERSION 3.2)
project(MultiAgentSLAM VERSION 1.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)

# Make sure that custom modules are found
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake/modules/)

message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
###################################################
# Declare module dependencies
###################################################
include("${CMAKE_CURRENT_LIST_DIR}/cmake/modules/Boost.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/cmake/modules/FindEigen.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/cmake/modules/FindMKL.cmake")

###################################################
# Find external libraries/headers
###################################################
#add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/include/interfaces)
#add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/include/nholmann_json)
#add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/include/shared_structs)
#add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/include/templates)

find_package(Interfaces REQUIRED)
find_package(JSON REQUIRED)
find_package(SharedStructs REQUIRED)
find_package(Templates REQUIRED)


###################################################
# Create library target and set properties
###################################################
# Set library target name
set(LIBRARY_TARGET_NAME distributed_slam)
set(LIBRARY_EXPORT_NAME MADS)
set(
        PROJECT_SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/Agent/Detection/Detection.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Agent/Moments/Moments.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Agent/Rover/Rover.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Agent/Rover/RoverFactory.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Slam/ActiveRovers/ActiveRovers.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Slam/FeatureSet/FeatureSet.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Slam/SEIF/Seif.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Slam/SlamAdapter/SlamAdapter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Utilities/ConfigParser/ConfigParser.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Utilities/Equations/Equations.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Utilities/BinaryTree/RedBlackTree.cpp
)

add_library(
        ${LIBRARY_TARGET_NAME} src/Adapter.cpp
)

# Add an alias so that library can be used inside the build tree (ei. when testing)
add_library(
        ${LIBRARY_EXPORT_NAME}::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME}
)

# Set target properties
target_include_directories(${LIBRARY_TARGET_NAME}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(${LIBRARY_TARGET_NAME}
        PRIVATE
            cxx_auto_type
)
target_compile_options(${LIBRARY_TARGET_NAME}
        PRIVATE
            $<$<CXX_COMPILER_ID:GNU>:-Wall>
)

if(UNIX AND NOT APPLE)
    target_link_libraries(${LIBRARY_TARGET_NAME}
            PUBLIC
                Boost::boost
                Eigen3::Eigen
                rt
            PRIVATE
                Interfaces::Interfaces
#                JSON::JSON
#                SharedStructs::SharedStructs
#                Templates::Templates
    )
elseif(APPLE)
    target_link_libraries(${LIBRARY_TARGET_NAME}
            PUBLIC
                Boost::boost
                Eigen3::Eigen
            PRIVATE
                SharedStructs::SharedStructs
                Interfaces::Interfaces
                JSON::JSON
                Templates::Templates
        )
endif()

###################################################
# Installation instructions
###################################################
include(GNUInstallDirs)

set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${LIBRARY_EXPORT_NAME})

# Install library <lib-name> and register the target in the export <lib_name>-targets
install(
        TARGETS
            ${LIBRARY_TARGET_NAME}
        EXPORT
            ${LIBRARY_TARGET_NAME}-targets
        LIBRARY
            DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE
            DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Required so exported target has name `MADS` (Multi-Agent Distributed SLAM) and not <lib-name>
set_target_properties(${LIBRARY_TARGET_NAME}
        PROPERTIES
            EXPORT_NAME ${LIBRARY_EXPORT_NAME}
)

install(
        DIRECTORY
            include/
        DESTINATION
            ${CMAKE_INSTALL_INCLUDEDIR}
)

# Export the targets to a script
install(
        EXPORT
            ${LIBRARY_TARGET_NAME}-targets
        FILE
            ${LIBRARY_EXPORT_NAME}Targets.cmake
        NAMESPACE
            ${LIBRARY_EXPORT_NAME}::
        DESTINATION
            ${INSTALL_CONFIGDIR}
)

# Create ConfigVersion.shared_structs file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_EXPORT_NAME}ConfigVersion.cmake
        VERSION
            ${PROJECT_VERSION}
        COMPATIBILITY
            AnyNewerVersion
)

configure_package_config_file(
        ${CMAKE_CURRENT_LIST_DIR}/cmake/scripts/${LIBRARY_EXPORT_NAME}Config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_EXPORT_NAME}Config.cmake
        INSTALL_DESTINATION
            ${INSTALL_CONFIGDIR}
)

# Install the config, config-version, and the custom find modules
install(
        FILES
            ${CMAKE_CURRENT_LIST_DIR}/cmake/modules/Boost.cmake
            ${CMAKE_CURRENT_LIST_DIR}/cmake/modules/FindEigen.cmake
            ${CMAKE_CURRENT_LIST_DIR}/cmake/modules/FindMKL.cmake
            ${CMAKE_CURRENT_LIST_DIR}/cmake/modules/FindInterfaces.cmake
            ${CMAKE_CURRENT_LIST_DIR}/cmake/modules/FindSharedStructs.cmake
            ${CMAKE_CURRENT_LIST_DIR}/cmake/modules/FindJSON.cmake
            ${CMAKE_CURRENT_LIST_DIR}/cmake/modules/FindTemplates.cmake
            ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_EXPORT_NAME}Config.cmake
            ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_EXPORT_NAME}ConfigVersion.cmake
        DESTINATION
            ${INSTALL_CONFIGDIR}
)

###################################################
# Export from the build tree
###################################################
# BOOST
configure_file(
        ${CMAKE_CURRENT_LIST_DIR}/cmake/modules/Boost.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/Boost.cmake
        COPYONLY
)
# EIGEN
configure_file(
        ${CMAKE_CURRENT_LIST_DIR}/cmake/modules/FindEigen.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/Eigen.cmake
        COPYONLY
)
# MKL
configure_file(
        ${CMAKE_CURRENT_LIST_DIR}/cmake/modules/FindMKL.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/FindMKL.cmake
        COPYONLY
)
# INTERFACES
configure_file(
        ${CMAKE_CURRENT_LIST_DIR}/cmake/modules/FindInterfaces.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/FindInterfaces.cmake
        COPYONLY
)
# SHARED_STRUCTS
configure_file(
        ${CMAKE_CURRENT_LIST_DIR}/cmake/modules/FindSharedStructs.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/FindSharedStructs.cmake
        COPYONLY
)
# JSON
configure_file(
        ${CMAKE_CURRENT_LIST_DIR}/cmake/modules/FindJSON.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/FindJSON.cmake
        COPYONLY
)
# TEMPLATES
configure_file(
        ${CMAKE_CURRENT_LIST_DIR}/cmake/modules/FindTemplates.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/FindTemplates.cmake
        COPYONLY
)

export(
        EXPORT
            ${LIBRARY_TARGET_NAME}-targets
        FILE
            ${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_EXPORT_NAME}Targets.cmake
        NAMESPACE
            ${LIBRARY_EXPORT_NAME}::
)

# Register package in user's package registry
export(
        PACKAGE
            ${LIBRARY_EXPORT_NAME}
)
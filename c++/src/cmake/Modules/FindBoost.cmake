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
)
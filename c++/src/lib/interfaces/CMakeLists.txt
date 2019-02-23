project(interfaces VERSION 0.1)

# Header only library, therefore INTERFACE
add_library(interfaces INTERFACE)

# Interface targets only have interface properties
target_include_directories(interfaces
        INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include
        )

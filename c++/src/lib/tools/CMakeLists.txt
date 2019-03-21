project(tools VERSION 0.1)

# Header only library, therefore INTERFACE
add_library(tools INTERFACE)

# Interface targets only have interface properties
target_include_directories(tools
        INTERFACE include
        )
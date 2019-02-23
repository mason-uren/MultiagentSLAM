project(shared_structs VERSION 0.1)

# Header only library, therefore INTERFACE
add_library(shared_structs INTERFACE)

# Interface targets only have interface properties
target_include_directories(shared_structs
        INTERFACE include
)
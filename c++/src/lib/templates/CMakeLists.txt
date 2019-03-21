project(templates VERSION 0.1)

# Header only library, therefore INTERFACE
add_library(templates INTERFACE)

# Interface targets only have interface properties
target_include_directories(templates
        INTERFACE include
)
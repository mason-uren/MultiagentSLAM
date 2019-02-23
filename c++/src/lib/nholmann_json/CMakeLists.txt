project(nholmann_json VERSION 0.1)

# Header only library, therefore INTERFACE
add_library(nholmann_json INTERFACE)

# Interface targets only have interface properties
target_include_directories(nholmann_json
        INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

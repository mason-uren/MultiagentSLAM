project(interfaces VERSION 0.1)

# Header only library, therefore INTERFACE
add_library(interfaces INTERFACE)

# Interface targets only have interface properties
target_include_directories(interfaces
    INTERFACE
        $<INSTALL_INTERFACE::include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )

target_compile_features(interfaces
    INTERFACE
        cxx_std_11
)

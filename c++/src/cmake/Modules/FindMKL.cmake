set(mklroot "/opt/intel/mkl")

include_directories(${mklroot}/include)

find_library(mkl_lib ${mkl_lib}
        NAMES lib intel64
        HINTS "${mklroot}/lib/intel64" )

if(mkl_lib_FOUND)
    message(STATUS "Intel_MKL_LIBRARIES: ${MKL_LIB}")
endif()

set(mkl_lib
        "${mklroot}/lib/libmkl_intel_ilp64.a
        ${mklroot}/lib/libmkl_sequential.a
        ${mklroot}/lib/libmkl_core.a -lpthread -lm -ldl")

message(STATUS "Intel_MKL_ROOT: ${mklroot}")

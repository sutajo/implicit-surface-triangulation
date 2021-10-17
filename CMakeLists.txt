cmake_minimum_required(VERSION 3.1.0)

project(curvature_dependent_triangulation)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

set(GEOM_DIR libgeom) # https://github.com/salvipeter/libgeom
set(NANOFLANN_DIR nanoflann)
set(AUTODIFF_DIR autodiff)

include_directories(${GEOM_DIR} nano ${NANOFLANN_DIR}/include ${AUTODIFF_DIR})
link_directories(${GEOM_DIR}/build)

add_library(curvature_dependent_triangulation STATIC
    cdt.hh
    cdt.cc
)

set_property(TARGET curvature_dependent_triangulation PROPERTY CXX_STANDARD 17)

target_link_libraries(curvature_dependent_triangulation PRIVATE geom)

add_executable(curvature_dependent_triangulation_test
    test.cc
)

set_property(TARGET curvature_dependent_triangulation_test PROPERTY CXX_STANDARD 17)

target_link_libraries(curvature_dependent_triangulation_test PRIVATE curvature_dependent_triangulation)

set_target_properties(
    curvature_dependent_triangulation_test
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
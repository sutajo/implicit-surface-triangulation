cmake_minimum_required(VERSION 3.1.0)

project(curvature_dependent_triangulation)

add_subdirectory(spdlog)

get_target_property(SPDLOG_INCLUDES spdlog INCLUDE_DIRECTORIES)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

add_subdirectory(OpenMesh-8.1)
add_subdirectory(libgeom) # https://github.com/salvipeter/libgeom
set(NANOFLANN_DIR nanoflann)
set(AUTODIFF_DIR autodiff)

include_directories(include libgeom nano ${NANOFLANN_DIR}/include ${AUTODIFF_DIR} ${SPDLOG_INCLUDES})

add_library(curvature_dependent_triangulation STATIC
    cdt.hh
    cdt.cc
    src/util.cc
)

target_compile_options(curvature_dependent_triangulation PRIVATE -DSPDLOG_COMPILED_LIB)

set_property(TARGET curvature_dependent_triangulation PROPERTY CXX_STANDARD 17)

target_link_libraries(curvature_dependent_triangulation PRIVATE geom spdlog OpenMeshCore)

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

enable_testing()
find_package(GTest REQUIRED)
add_executable(
    unit_tests
    test/util.cc
)
set_property(TARGET unit_tests PROPERTY CXX_STANDARD 17)
target_link_libraries(unit_tests gtest gtest_main pthread curvature_dependent_triangulation)
add_test(unit_tests unit_tests)
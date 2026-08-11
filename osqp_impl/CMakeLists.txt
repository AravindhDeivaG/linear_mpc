# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find OSQP headers and library
find_path(OSQP_INCLUDE_DIR NAMES osqp/osqp.h PATHS /usr/local/include)
find_library(OSQP_LIBRARY NAMES osqp osqpstatic PATHS /usr/local/lib)

if(NOT OSQP_INCLUDE_DIR)
    message(FATAL_ERROR "OSQP headers not found!")
endif()

if(NOT OSQP_LIBRARY)
    message(FATAL_ERROR "OSQP library not found!")
endif()

# ---------------------------------------------------------------------------
# OSQP Wrapper Library Target
# ---------------------------------------------------------------------------
add_library(osqp_impl STATIC
    src/osqp_solver.cpp
)

# Expose include/ and OSQP include paths publicly
target_include_directories(osqp_impl PUBLIC
    include
    ${OSQP_INCLUDE_DIR}
)

# Link against Eigen and OSQP
target_link_libraries(osqp_impl PUBLIC
    Eigen3::Eigen
    ${OSQP_LIBRARY}
)

# ---------------------------------------------------------------------------
# Test Application
# ---------------------------------------------------------------------------

# Create an executable to test the solver wrapper functionality
add_executable(solver_demo bin/test_solver.cpp)
target_link_libraries(solver_demo PRIVATE osqp_impl)

# ---------------------------------------------------------------------------
# MPC Library & Demo Build configuration
# ---------------------------------------------------------------------------

# Create the shared library target for MPC logic
add_library(mpc SHARED
    src/mpc_controller.cpp
)

# Expose include/ publicly
target_include_directories(mpc PUBLIC
    include
)

# Link MPC logic against the OSQP solver wrapper
target_link_libraries(mpc PUBLIC
    osqp_impl
)

# Create the test executable
add_executable(mpc_demo bin/test_mpc.cpp)

# Link against the wrapper libraries
target_link_libraries(mpc_demo PRIVATE
    mpc
    imgui_wrapper
)

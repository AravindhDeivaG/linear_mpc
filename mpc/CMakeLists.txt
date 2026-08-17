# ---------------------------------------------------------------------------
# MPC Library & Demo Build configuration
# ---------------------------------------------------------------------------

# Create the shared library target for MPC logic
add_library(mpc SHARED
    src/mpc_controller.cpp
    src/dense_formulation.cpp
    src/sparse_formulation.cpp
)

# Expose include/ publicly
target_include_directories(mpc PUBLIC
    include
)

# Link MPC logic against the OSQP solver wrapper
target_link_libraries(mpc PUBLIC
    osqp_impl
)

# Create the test/validation executable
add_executable(mpc_demo bin/test_mpc.cpp)
target_link_libraries(mpc_demo PRIVATE
    mpc
    imgui_wrapper
)

add_executable(test_diff bin/test_diff.cpp)
target_link_libraries(test_diff PRIVATE
    mpc
    imgui_wrapper
)

add_executable(test_recorded_state bin/test_recorded_state.cpp)
target_link_libraries(test_recorded_state PRIVATE
    mpc
    imgui_wrapper
)

# Create the single-solver interactive visualization executable
add_executable(mpc_viz bin/mpc_viz.cpp)
target_link_libraries(mpc_viz PRIVATE
    mpc
    imgui_wrapper
)

# Create the dual-solver side-by-side debug visualization executable
add_executable(debug_mpc_viz bin/debug_mpc_viz.cpp)
target_link_libraries(debug_mpc_viz PRIVATE
    mpc
    imgui_wrapper
)

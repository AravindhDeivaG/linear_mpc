#include "imgui_wrapper.h"
#include <imgui.h> // Include imgui to access IsMouseDown directly
#include <cmath>
#include "mpc_controller.h"
#include <chrono>
#include <thread>

int main() {
    // Create an 800x600 window
    ImGuiWrapper wrapper(800, 600, "MPC Tracking Demo");
    wrapper.setVSync(false);

    // Circle positions (initialized in the center)
    float targetX = 400.0f;
    float targetY = 300.0f;
    float currentX = 200.0f;
    float currentY = 300.0f;

    // Time step
    const double dt = 0.02;

    // State and Reference
    Eigen::VectorXd x(4);
    x << currentX, currentY, 0, 0;

    // Reference state
    Eigen::VectorXd x_ref(4);
    x_ref << targetX, targetY, 0, 0;

    // Control inputs
    Eigen::VectorXd u(2);
    u << 0, 0;

    // Predicted states trajectory
    Eigen::VectorXd X_pred;

    // Create MPC controller
    MpcController mpc(30, dt);
    mpc.setCurrentState(x);
    mpc.setReferenceState(x_ref);

    const float targetRadius = 25.0f;
    const float currentRadius = 15.0f;

    bool isDragging = false;
    bool runMpc = false;

    while (!wrapper.shouldClose()) {
        auto start = std::chrono::high_resolution_clock::now();

        wrapper.beginFrame();

        // 1. Draw the Toggle Button at coordinates (10, 10)
        wrapper.drawToggleButton(10.0f, 10.0f, "Run MPC Loop", runMpc);

        // 2. Handle Mouse Dragging for the Target Circle (Direct State Check)
        float mouseX, mouseY;
        wrapper.getMousePos(mouseX, mouseY);

        if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            if (!isDragging) {
                // Check if we clicked within the target circle boundary
                float dx = mouseX - targetX;
                float dy = mouseY - targetY;
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist <= targetRadius) {
                    isDragging = true;
                }
            }
        } else {
            isDragging = false;
        }

        if (isDragging) {
            targetX = mouseX;
            targetY = mouseY;

            // Set x reference
            x_ref(0) = targetX;
            x_ref(1) = targetY;
        }

        // 3. Control law loop (left empty for you to implement MPC)
        if (runMpc) {
            // TODO: Populate with your MPC control law
            mpc.setCurrentState(x);
            mpc.setReferenceState(x_ref);
            mpc.doControl();
            mpc.getOptimalControl(u);
            x(0) = x(0) + x(2)*dt + u(0)*dt*dt/2;
            x(1) = x(1) + x(3)*dt + u(1)*dt*dt/2;
            x(2) = x(2) + u(0)*dt;
            x(3) = x(3) + u(1)*dt;

            // Get predicted states
            mpc.getPredictedStates(X_pred);
        }

        // 4. Render circles
        // Target: light larger circle (semi-transparent pinkish-red)
        currentX = x(0);
        currentY = x(1);
        wrapper.drawSolidCircle(targetX, targetY, targetRadius, 255, 130, 130, 150);

        // Current state: dark medium-sized circle (opaque navy blue)
        wrapper.drawSolidCircle(currentX, currentY, currentRadius, 20, 50, 100, 255);

        // Draw predicted trajectory as bright cyan small hollow circles
        if (runMpc && X_pred.size() > 0) {
            int horizon = mpc.getHorizon();
            for (int i = 0; i < horizon; ++i) {
                float px = X_pred(4 * i);
                float py = X_pred(4 * i + 1);
                wrapper.drawHollowCircle(px, py, 4.0f, 0, 255, 255, 255);
            }
        }

        wrapper.endFrame();

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        if (duration.count() < 20000) {
            std::this_thread::sleep_for(std::chrono::microseconds(20000 - duration.count()));
        }

        // auto end_frame = std::chrono::high_resolution_clock::now();
        // auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_frame - start);
        // std::cout << "Time taken by function: " << total_duration.count() << " microseconds" << std::endl;

    }

    return 0;
}

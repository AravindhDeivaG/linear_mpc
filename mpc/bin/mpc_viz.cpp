#include "imgui_wrapper.h"
#include <imgui.h>
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

    // Dimensions & Time step
    const int horizon = 15;
    const int nx = 4;
    const int nu = 2;
    const double dt = 0.02;

    // State and Reference
    Eigen::VectorXd x(nx);
    x << currentX, currentY, 0, 0;

    Eigen::VectorXd x_ref(nx);
    x_ref << targetX, targetY, 0, 0;

    Eigen::VectorXd u(nu);
    u << 0, 0;

    Eigen::VectorXd X_pred;

    // Define 2D double-integrator system dynamics and constraints
    Eigen::MatrixXd A(nx, nx);
    A.setIdentity();
    A(0,2) = dt;
    A(1,3) = dt;

    Eigen::MatrixXd B(nx, nu);
    B.setZero();
    B(0,0) = 0.5*dt*dt;
    B(1,1) = 0.5*dt*dt;
    B(2,0) = dt;
    B(3,1) = dt;

    Eigen::VectorXd x_min(nx), x_max(nx);
    x_min << -10000, -10000, -200, -200;
    x_max <<  10000,  10000,  200,  200;

    Eigen::VectorXd u_min(nu), u_max(nu);
    u_min << -500, -500;
    u_max <<  500,  500;

    Eigen::MatrixXd Q(nx, nx);
    Q.setZero();
    Q.block(0,0,2,2) = Eigen::MatrixXd::Identity(2,2)*10.0;
    Q.block(2,2,2,2) = Eigen::MatrixXd::Identity(2,2)*0.1;

    Eigen::MatrixXd R(nu, nu);
    R = Eigen::MatrixXd::Identity(nu, nu) * 1e-3;

    // Create and setup MPC controller with pre-allocated dimensions
    MpcController mpc(horizon, nx, nu, FormulationType::SPARSE);
    mpc.setSystemMatrices(A, B);
    mpc.setStateLimits(x_min, x_max);
    mpc.setInputLimits(u_min, u_max);
    mpc.setCostMatrices(Q, R);
    mpc.setup();

    mpc.setCurrentState(x);
    mpc.setReferenceState(x_ref);

    const float targetRadius = 25.0f;
    const float currentRadius = 15.0f;

    bool isDragging = false;
    bool runMpc = false;

    while (!wrapper.shouldClose()) {
        auto start = std::chrono::high_resolution_clock::now();

        wrapper.beginFrame();

        // 1. Draw Toggle Button
        wrapper.drawToggleButton(10.0f, 10.0f, "Run MPC Loop", runMpc);

        // 2. Mouse Dragging target
        float mouseX, mouseY;
        wrapper.getMousePos(mouseX, mouseY);

        if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            if (!isDragging) {
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
            x_ref(0) = targetX;
            x_ref(1) = targetY;
        }

        // 3. Control law loop
        if (runMpc) {
            mpc.setCurrentState(x);
            mpc.setReferenceState(x_ref);
            mpc.doControl();
            mpc.getOptimalControl(u);
            x(0) = x(0) + x(2)*dt + u(0)*dt*dt/2;
            x(1) = x(1) + x(3)*dt + u(1)*dt*dt/2;
            x(2) = x(2) + u(0)*dt;
            x(3) = x(3) + u(1)*dt;

            mpc.getPredictedStates(X_pred);
        }

        // 4. Render circles
        currentX = x(0);
        currentY = x(1);
        wrapper.drawSolidCircle(targetX, targetY, targetRadius, 255, 130, 130, 150);
        wrapper.drawSolidCircle(currentX, currentY, currentRadius, 20, 50, 100, 255);

        // Draw predicted trajectory
        if (runMpc && X_pred.size() > 0) {
            int h = mpc.getHorizon();
            for (int i = 0; i < h; ++i) {
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
    }

    return 0;
}

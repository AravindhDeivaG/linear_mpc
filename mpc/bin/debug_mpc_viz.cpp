#include "imgui_wrapper.h"
#include <imgui.h>
#include <cmath>
#include "mpc_controller.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <iomanip>

int main() {
    // Create an 800x600 window
    ImGuiWrapper wrapper(800, 600, "MPC Dense vs Sparse Trajectory Debugger");
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

    Eigen::VectorXd u_dense(nu);
    u_dense.setZero();
    Eigen::VectorXd u_sparse(nu);
    u_sparse.setZero();

    Eigen::VectorXd X_dense_pred;
    Eigen::VectorXd X_sparse_pred;

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

    // Setup DENSE MPC controller
    MpcController dense_mpc(horizon, nx, nu, FormulationType::DENSE);
    dense_mpc.setSystemMatrices(A, B);
    dense_mpc.setStateLimits(x_min, x_max);
    dense_mpc.setInputLimits(u_min, u_max);
    dense_mpc.setCostMatrices(Q, R);
    dense_mpc.setup();

    // Setup SPARSE MPC controller
    MpcController sparse_mpc(horizon, nx, nu, FormulationType::SPARSE);
    sparse_mpc.setSystemMatrices(A, B);
    sparse_mpc.setStateLimits(x_min, x_max);
    sparse_mpc.setInputLimits(u_min, u_max);
    sparse_mpc.setCostMatrices(Q, R);
    sparse_mpc.setup();

    const float targetRadius = 25.0f;
    const float currentRadius = 15.0f;

    bool isDragging = false;
    bool runMpc = false;
    double diff_u = 0.0;
    std::string recordStatusMsg = "";

    while (!wrapper.shouldClose()) {
        auto start = std::chrono::high_resolution_clock::now();

        wrapper.beginFrame();

        // 1. Controls Window
        ImGui::Begin("MPC Diagnostics");
        ImGui::Checkbox("Run MPC Loop", &runMpc);
        ImGui::SameLine();
        
        if (ImGui::Button("Record State Snapshot")) {
            std::ofstream ofs("recorded_state.txt");
            if (ofs.is_open()) {
                ofs << std::fixed << std::setprecision(6);
                ofs << x(0) << " " << x(1) << " " << x(2) << " " << x(3) << "\n";
                ofs << x_ref(0) << " " << x_ref(1) << " " << x_ref(2) << " " << x_ref(3) << "\n";
                ofs << u_dense(0) << " " << u_dense(1) << "\n";
                ofs << u_sparse(0) << " " << u_sparse(1) << "\n";
                ofs.close();

                std::cout << "\n==========================================" << std::endl;
                std::cout << "[STATE RECORDED TO recorded_state.txt]" << std::endl;
                std::cout << "x:     [" << x.transpose() << "]" << std::endl;
                std::cout << "x_ref: [" << x_ref.transpose() << "]" << std::endl;
                std::cout << "u_dense:  [" << u_dense.transpose() << "]" << std::endl;
                std::cout << "u_sparse: [" << u_sparse.transpose() << "]" << std::endl;
                std::cout << "diff:  " << diff_u << std::endl;
                std::cout << "==========================================\n" << std::endl;

                recordStatusMsg = "Recorded to recorded_state.txt!";
            } else {
                recordStatusMsg = "Failed to open recorded_state.txt!";
            }
        }

        if (!recordStatusMsg.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "%s", recordStatusMsg.c_str());
        }

        ImGui::Separator();
        ImGui::Text("Orange Circles: Dense Trajectory");
        ImGui::Text("Cyan Circles:   Sparse Trajectory");
        ImGui::Separator();

        ImGui::Text("u_dense:  (%.3f, %.3f)", u_dense(0), u_dense(1));
        ImGui::Text("u_sparse: (%.3f, %.3f)", u_sparse(0), u_sparse(1));
        ImGui::Text("Control Diff: %.6f", diff_u);

        if (diff_u > 0.01) {
            ImGui::TextColored(ImVec4(1.0f, 0.2f, 0.2f, 1.0f), "STATUS: DIVERGING!");
        } else {
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "STATUS: MATCHING");
        }
        ImGui::End();

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

        // 3. MPC Execution
        if (runMpc) {
            // Solve Dense
            dense_mpc.setCurrentState(x);
            dense_mpc.setReferenceState(x_ref);
            dense_mpc.doControl();
            dense_mpc.getOptimalControl(u_dense);
            dense_mpc.getPredictedStates(X_dense_pred);

            // Solve Sparse
            sparse_mpc.setCurrentState(x);
            sparse_mpc.setReferenceState(x_ref);
            sparse_mpc.doControl();
            sparse_mpc.getOptimalControl(u_sparse);
            sparse_mpc.getPredictedStates(X_sparse_pred);

            diff_u = (u_dense - u_sparse).norm();

            // Apply ONLY Sparse control update to physical state
            x(0) += x(2)*dt + 0.5*u_sparse(0)*dt*dt;
            x(1) += x(3)*dt + 0.5*u_sparse(1)*dt*dt;
            x(2) += u_sparse(0)*dt;
            x(3) += u_sparse(1)*dt;
        }

        // 4. Render shapes
        currentX = x(0);
        currentY = x(1);

        // Target (semi-transparent pink circle)
        wrapper.drawSolidCircle(targetX, targetY, targetRadius, 255, 130, 130, 150);

        // Current robot position (navy blue circle)
        wrapper.drawSolidCircle(currentX, currentY, currentRadius, 20, 50, 100, 255);

        // Draw DENSE predicted trajectory (hollow ORANGE circles)
        if (runMpc && X_dense_pred.size() > 0) {
            for (int i = 0; i < horizon; ++i) {
                float px = X_dense_pred(4 * i);
                float py = X_dense_pred(4 * i + 1);
                wrapper.drawHollowCircle(px, py, 6.0f, 255, 165, 0, 255); // Orange
            }
        }

        // Draw SPARSE predicted trajectory (hollow CYAN circles)
        if (runMpc && X_sparse_pred.size() > 0) {
            for (int i = 0; i < horizon; ++i) {
                float px = X_sparse_pred(4 * i);
                float py = X_sparse_pred(4 * i + 1);
                wrapper.drawHollowCircle(px, py, 4.0f, 0, 255, 255, 255); // Cyan
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

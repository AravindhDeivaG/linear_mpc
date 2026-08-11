#include "imgui_wrapper.h"
#include <imgui.h> // Include imgui to access IsMouseDown directly
#include <cmath>

int main() {
    // Create an 800x600 window
    ImGuiWrapper wrapper(800, 600, "MPC Tracking Demo");

    // Circle positions (initialized in the center)
    float targetX = 400.0f;
    float targetY = 300.0f;
    float currentX = 200.0f;
    float currentY = 300.0f;

    const float targetRadius = 25.0f;
    const float currentRadius = 15.0f;

    bool isDragging = false;
    bool runMpc = false;

    while (!wrapper.shouldClose()) {
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
        }

        // 3. Control law loop (left empty for you to implement MPC)
        if (runMpc) {
            // TODO: Populate with your MPC control law
        }

        // 4. Render circles
        // Target: light larger circle (semi-transparent pinkish-red)
        wrapper.drawSolidCircle(targetX, targetY, targetRadius, 255, 130, 130, 150);

        // Current state: dark medium-sized circle (opaque navy blue)
        wrapper.drawSolidCircle(currentX, currentY, currentRadius, 20, 50, 100, 255);

        wrapper.endFrame();
    }

    return 0;
}

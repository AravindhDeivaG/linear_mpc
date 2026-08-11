#include "imgui_wrapper.h"
#include <iostream>

int main() {
    try {
        // Create the wrapper window (800x600 size)
        ImGuiWrapper wrapper(800, 600, "ImGui Wrapper Test Application");

        float clickX = 0.0f;
        float clickY = 0.0f;
        bool isDown = false;

        // Main frame loop
        while (!wrapper.shouldClose()) {
            wrapper.beginFrame();

            // Check if left mouse button was clicked
            float tempX, tempY;
            if (wrapper.getMouseClick(tempX, tempY)) {
                clickX = tempX;
                clickY = tempY;
                isDown = true;
                std::cout << "Mouse Click Down detected at (" << clickX << ", " << clickY << ")\n";
            }

            // Check if left mouse button was released
            if (wrapper.getMouseRelease()) {
                isDown = false;
                std::cout << "Mouse Release detected\n";
            }

            // Draw shapes if mouse button is down
            if (isDown) {
                // Draw a smaller solid circle at the click position (e.g., radius 15, Blue color)
                wrapper.drawSolidCircle(clickX, clickY, 15.0f, 0, 0, 255, 255);

                // Draw a larger hollow circle centered at the same position (e.g., radius 30, Red color)
                wrapper.drawHollowCircle(clickX, clickY, 30.0f, 255, 0, 0, 255);
            }

            wrapper.endFrame();
        }
    } catch (const std::exception& e) {
        std::cerr << "Fatal Exception: " << e.what() << "\n";
        return -1;
    }

    return 0;
}

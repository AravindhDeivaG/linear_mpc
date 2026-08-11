#ifndef IMGUI_WRAPPER_H
#define IMGUI_WRAPPER_H

#include <string>

// Forward declaration of GLFWwindow to avoid exposing GLFW headers in the public wrapper header
struct GLFWwindow;

class ImGuiWrapper {
public:
    // Constructor: creates the window and initializes GLFW, OpenGL, and Dear ImGui
    ImGuiWrapper(int width, int height, const std::string& title);

    // Destructor: destroys the window and cleans up resources properly
    ~ImGuiWrapper();

    // Check if the window should close
    bool shouldClose() const;

    // Call at the start of each frame
    void beginFrame();

    // Call at the end of each frame to render and swap buffers
    void endFrame();

    // Checks if a mouse click was detected on the window in the current frame.
    // Returns true if a click occurred, setting x and y to the click position.
    bool getMouseClick(float& x, float& y);

    // Checks if the mouse button was released on the window in the current frame.
    // Returns true if a release occurred.
    bool getMouseRelease();

    // Draw a solid circle at (cx, cy) with radius r and RGB(A) color (0-255)
    void drawSolidCircle(float cx, float cy, float r, int red, int green, int blue, int alpha = 255);

    // Draw a hollow circle at (cx, cy) with radius r and RGB(A) color (0-255)
    void drawHollowCircle(float cx, float cy, float r, int red, int green, int blue, int alpha = 255);

    // Draw a toggle button/checkbox at (x, y) with text label. Returns current toggled state.
    bool drawToggleButton(float x, float y, const std::string& label, bool& state);

    // Retrieves the current mouse position coordinates
    void getMousePos(float& x, float& y);

    // Enables/disables vertical sync (VSync)
    void setVSync(bool enabled);

private:
    GLFWwindow* m_window;
    int m_width;
    int m_height;
    std::string m_title;
};

#endif // IMGUI_WRAPPER_H

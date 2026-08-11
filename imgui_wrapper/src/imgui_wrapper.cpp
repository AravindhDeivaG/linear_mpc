#include "imgui_wrapper.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <iostream>
#include <stdexcept>

// Constructor
ImGuiWrapper::ImGuiWrapper(int width, int height, const std::string& title)
    : m_width(width), m_height(height), m_title(title), m_window(nullptr) {
    
    // Set GLFW error callback
    glfwSetErrorCallback([](int error, const char* description) {
        std::cerr << "GLFW Error " << error << ": " << description << "\n";
    });

    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    // GL 3.0 + generally GLSL 130
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    // Create window
    m_window = glfwCreateWindow(m_width, m_height, m_title.c_str(), nullptr, nullptr);
    if (!m_window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }

    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(m_window, true);
    ImGui_ImplOpenGL3_Init("#version 130");
}

// Destructor
ImGuiWrapper::~ImGuiWrapper() {
    if (m_window) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();

        glfwDestroyWindow(m_window);
        glfwTerminate();
    }
}

bool ImGuiWrapper::shouldClose() const {
    return glfwWindowShouldClose(m_window);
}

void ImGuiWrapper::beginFrame() {
    glfwPollEvents();
    
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void ImGuiWrapper::endFrame() {
    // Rendering
    ImGui::Render();
    
    int display_w, display_h;
    glfwGetFramebufferSize(m_window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    
    // Clear screen to a dark gray color
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(m_window);
}

bool ImGuiWrapper::getMouseClick(float& x, float& y) {
    // ImGui::IsMouseClicked(0) returns true if left mouse button was clicked in the current frame
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        ImVec2 mousePos = ImGui::GetMousePos();
        x = mousePos.x;
        y = mousePos.y;
        return true;
    }
    return false;
}

bool ImGuiWrapper::getMouseRelease() {
    // ImGui::IsMouseReleased(0) returns true if left mouse button was released in the current frame
    return ImGui::IsMouseReleased(ImGuiMouseButton_Left);
}

void ImGuiWrapper::drawSolidCircle(float cx, float cy, float r, int red, int green, int blue, int alpha) {
    ImU32 color = IM_COL32(red, green, blue, alpha);
    // Draw on the background draw list (behind ImGui windows, covering the whole viewport)
    ImGui::GetBackgroundDrawList()->AddCircleFilled(ImVec2(cx, cy), r, color, 64);
}

void ImGuiWrapper::drawHollowCircle(float cx, float cy, float r, int red, int green, int blue, int alpha) {
    ImU32 color = IM_COL32(red, green, blue, alpha);
    // Draw on the background draw list
    ImGui::GetBackgroundDrawList()->AddCircle(ImVec2(cx, cy), r, color, 64, 2.0f); // 2.0f thickness
}

bool ImGuiWrapper::drawToggleButton(float x, float y, const std::string& label, bool& state) {
    ImGui::SetNextWindowPos(ImVec2(x, y), ImGuiCond_Always);
    ImGui::Begin("Toggle Control Panel", nullptr, 
                 ImGuiWindowFlags_NoTitleBar | 
                 ImGuiWindowFlags_NoResize | 
                 ImGuiWindowFlags_NoMove | 
                 ImGuiWindowFlags_NoScrollbar | 
                 ImGuiWindowFlags_NoSavedSettings | 
                 ImGuiWindowFlags_AlwaysAutoResize | 
                 ImGuiWindowFlags_NoBackground);
    ImGui::Checkbox(label.c_str(), &state);
    ImGui::End();
    return state;
}

void ImGuiWrapper::getMousePos(float& x, float& y) {
    ImVec2 mousePos = ImGui::GetMousePos();
    x = mousePos.x;
    y = mousePos.y;
}


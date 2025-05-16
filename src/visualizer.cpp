// Main loop
#include <SDL3/SDL.h>
#include <implot.h>
#include <imgui_impl_sdl3.h>

void SetupVisualizer()
{
    ImPlot::CreateContext();
}

void ShowVisualizer()
{
    ImPlot::ShowDemoWindow();
}
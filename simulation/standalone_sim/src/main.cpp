#include <Stonefish/core/GraphicalSimulationApp.h>
#include "HydrusSimManager.h"
#include <filesystem>
#include <string>

int main(int argc, char **argv)
{
    // Using default settings
    sf::RenderSettings s;
    s.verticalSync = true; // Enable vertical sync
    s.windowH = 1024;
    s.windowW = 1280; // Set window size
    sf::HelperSettings h;

    // Get the directory where the executable's source is located
    std::filesystem::path sourcePath = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::string simulationPath = sourcePath.string() + "/";

    HydrusSimManager manager(500.0);
    sf::GraphicalSimulationApp app("Simple simulator", simulationPath, s, h, &manager);
    app.Run();

    return 0;
}
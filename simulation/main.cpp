#include <Stonefish/core/GraphicalSimulationApp.h>
#include "HydrusSimManager.h"

int main(int argc, char **argv)
{
    // Using default settings
    sf::RenderSettings s;
    sf::HelperSettings h;

    HydrusSimManager manager(500.0);
    sf::GraphicalSimulationApp app("Simple simulator", "path_to_data", s, h, &manager);
    app.Run();

    return 0;
}
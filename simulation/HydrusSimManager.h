#include <Stonefish/core/SimulationManager.h>

class HydrusSimManager : public sf::SimulationManager
{
public:
    HydrusSimManager(sf::Scalar stepsPerSecond);
    void BuildScenario();
};
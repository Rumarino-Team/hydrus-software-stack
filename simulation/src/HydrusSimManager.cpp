#include "HydrusSimManager.h"
#include <Stonefish/entities/statics/Plane.h>
#include <Stonefish/entities/solids/Sphere.h>
#include <Stonefish/entities/solids/Polyhedron.h>
#include <utils/SystemUtil.hpp>
#include <core/NED.h>

HydrusSimManager::HydrusSimManager(sf::Scalar stepsPerSecond) : SimulationManager(stepsPerSecond)
{
}

void HydrusSimManager::BuildScenario()
{
    // Physical materialsw
    CreateMaterial("Aluminium", 2700.0, 0.8);
    CreateMaterial("Steel", 7810.0, 0.9);
    SetMaterialsInteraction("Aluminium", "Aluminium", 0.7, 0.5);
    SetMaterialsInteraction("Steel", "Steel", 0.4, 0.2);
    SetMaterialsInteraction("Aluminium", "Steel", 0.6, 0.4);

    // Graphical materials (looks)
    CreateLook("black", sf::Color::RGB(0.f, 0.f, 0.f), 0.1f, 0.1f);
    CreateLook("gray", sf::Color::Gray(0.5f), 0.3f, 0.2f);
    CreateLook("red", sf::Color::RGB(1.f, 0.f, 0.f), 0.1f, 0.f);

    // Create environment
    EnableOcean(0.0);
    getOcean()->setWaterType(0.2);
    getOcean()->EnableCurrents();
    getAtmosphere()->SetSunPosition(0.0, 60.0);
    getNED()->Init(0.0, 0.0, 0.0);

    sf::Plane *plane = new sf::Plane("Ground", 10000.0, "Steel", "gray");
    AddStaticEntity(plane, sf::Transform(sf::IQ(), sf::Vector3(0.0, 0.0, 5.0)));

    // Create object
    sf::BodyPhysicsSettings phy;
    phy.mode = sf::BodyPhysicsMode::SUBMERGED;
    phy.collisions = false;

    sf::Sphere *sph = new sf::Sphere("Sphere", phy, 0.1, sf::I4(), "Aluminium", "red");
    AddSolidEntity(sph, sf::Transform(sf::IQ(), sf::Vector3(0.0, 0.0, -1.0)));

    sf::Polyhedron *poly = new sf::Polyhedron("Hydrus", phy, sf::GetDataPath() + "models/hydrus_unibody.obj", sf::Scalar(0.01), sf::I4(), "Steel", "black");
    AddSolidEntity(poly, sf::Transform(sf::IQ(), sf::Vector3(0.0, -1.0, 0.0)));
}
#include "HydrusSimManager.h"
#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/core/GeneralRobot.h>
#include <Stonefish/entities/statics/Plane.h>
#include <Stonefish/entities/solids/Compound.h>
#include <Stonefish/entities/solids/Sphere.h>
#include <Stonefish/entities/solids/Polyhedron.h>
#include <utils/SystemUtil.hpp>
#include <core/NED.h>
#include <core/ScenarioParser.h>

HydrusSimManager::HydrusSimManager(sf::Scalar stepsPerSecond) : SimulationManager(stepsPerSecond)
{
}

void HydrusSimManager::BuildScenario()
{
    sf::ScenarioParser parser(this);
    bool success = parser.Parse(sf::GetDataPath() + "scenarios/hydrus_env.scn");
    if (!success)
        cCritical("Scenario parser: Parsing failed!");
}

// Keep the following commented out code for future reference or use
// // Physical materials
// CreateMaterial("Aluminium", 2700.0, 0.8);
// CreateMaterial("Steel", 7810.0, 0.9);
// SetMaterialsInteraction("Aluminium", "Aluminium", 0.7, 0.5);
// SetMaterialsInteraction("Steel", "Steel", 0.4, 0.2);
// SetMaterialsInteraction("Aluminium", "Steel", 0.6, 0.4);

// // Graphical materials (looks)
// CreateLook("green", sf::Color::RGB(0.f, 1.f, 0.f), 0.1f, 0.1f);
// CreateLook("gray", sf::Color::Gray(0.5f), 0.3f, 0.2f);
// CreateLook("red", sf::Color::RGB(1.f, 0.f, 0.f), 0.1f, 0.f);

// // Create environment
// EnableOcean(0.0);
// getOcean()->setWaterType(0.2);
// getOcean()->EnableCurrents();
// getAtmosphere()->SetSunPosition(0.0, 60.0);
// getNED()->Init(0.0, 0.0, 0.0);

// sf::Plane *plane = new sf::Plane("Ground", 10000.0, "Steel", "gray");
// AddStaticEntity(plane, sf::Transform(sf::IQ(), sf::Vector3(0.0, 0.0, 5.0)));

// // Create AUV
// sf::BodyPhysicsSettings phy;
// phy.mode = sf::BodyPhysicsMode::SUBMERGED;
// phy.buoyancy = true;
// phy.collisions = true;

// sf::Polyhedron *unibody = new sf::Polyhedron("hydrus_unibody", phy, sf::GetDataPath() + "models/hydrus_unibody_centered.obj", sf::Scalar(1), sf::Transform(sf::Quaternion(0, 0, -M_PI_2), sf::Vector3(0.0, 0.0, 0.0)), "Steel", "green");
// sf::Compound *vehicle = new sf::Compound("Hydrus", phy, unibody, sf::Transform(sf::Quaternion(0, 0, 0), sf::Vector3(0.0, 0.0, 0.0)));

// // Create thrusters
// std::array<std::string, 8> thrusterNames = {
//     "thruster_front_left",
//     "thruster_front_right",
//     "thruster_back_left",
//     "thruster_back_right",
//     "thruster_depth_1",
//     "thruster_depth_2",
//     "thruster_depth_3",
//     "thruster_depth_4"};
// std::array<sf::Thruster *, 8> thrusters;
// std::shared_ptr<sf::Polyhedron> propeller = std::make_shared<sf::Polyhedron>("Propeller", phy, sf::GetDataPath() + "models/propeller.obj", sf::Scalar(0.75), sf::I4(), "Dummy", "propeller");
// for (size_t i = 0; i < thrusterNames.size(); ++i)
// {
//     std::shared_ptr<sf::MechanicalPI> rotorDynamics;
//     rotorDynamics = std::make_shared<sf::MechanicalPI>(1.0, 10.0, 5.0, 5.0);
//     std::shared_ptr<sf::FDThrust> thrustModel;
//     thrustModel = std::make_shared<sf::FDThrust>(0.18, 0.48, 0.48, 0.05, true, getOcean()->getLiquid().density);
//     thrusters[i] = new sf::Thruster(thrusterNames[i], propeller, rotorDynamics, thrustModel, 0.18, true, 105.0, false, true);
// }

// sf::Robot *auv = new sf::GeneralRobot("HydrusAUV", false);
// auv->DefineLinks(vehicle);
// auv->BuildKinematicStructure();

// // Corner thrusters
// auv->AddLinkActuator(thrusters[0], "Hydrus", sf::Transform(sf::Quaternion(-M_PI_4 + M_PI_4 * -0.5, 0, 0), sf::Vector3(0.198, 0.407, 0.0)));
// auv->AddLinkActuator(thrusters[1], "Hydrus", sf::Transform(sf::Quaternion(M_PI_4 + M_PI_4 * 0.5, 0, 0), sf::Vector3(0.198, -0.408, 0.0)));
// auv->AddLinkActuator(thrusters[2], "Hydrus", sf::Transform(sf::Quaternion(-M_PI_2 + M_PI_4 * -0.5, 0, 0), sf::Vector3(-0.198, 0.407, 0.0)));
// auv->AddLinkActuator(thrusters[3], "Hydrus", sf::Transform(sf::Quaternion(M_PI_2 + M_PI_4 * 0.5, 0, 0), sf::Vector3(-0.198, -0.408, 0.0)));
// // Depth thrusters
// auv->AddLinkActuator(thrusters[4], "Hydrus", sf::Transform(sf::Quaternion(0, -M_PI_2, 0), sf::Vector3(0.211, 0.169, 0)));
// auv->AddLinkActuator(thrusters[5], "Hydrus", sf::Transform(sf::Quaternion(0, -M_PI_2, 0), sf::Vector3(0.211, -0.169, 0.0)));
// auv->AddLinkActuator(thrusters[6], "Hydrus", sf::Transform(sf::Quaternion(0, -M_PI_2, 0), sf::Vector3(-0.211, 0.169, 0.0)));
// auv->AddLinkActuator(thrusters[7], "Hydrus", sf::Transform(sf::Quaternion(0, -M_PI_2, 0), sf::Vector3(-0.211, -0.169, 0.0)));

// AddRobot(auv, sf::Transform(sf::Quaternion(0, 0, 0), sf::Vector3(0.0, 0.0, 0.0)));
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Box.hh>

namespace gazebo
{
  class UpwardForceWorldPlugin : public WorldPlugin
  {
  public:
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
    {
      this->world = _world;

      physics::ModelPtr waterPlane = world->ModelByName("water_plane");
      if (waterPlane)
      {
        ignition::math::Pose3d waterPose = waterPlane->WorldPose();
        this->heightThreshold = waterPose.Pos().Z();
      }
      else
      {
        this->heightThreshold = 2.0;
      }

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&UpwardForceWorldPlugin::OnUpdate, this));
      
      gzdbg << "[UpwardForceWorldPlugin] Loaded with height_threshold = " 
            << this->heightThreshold << "\n";
    }

    void OnUpdate()
    {
      // Loop through all models.
      for (auto model : this->world->Models())
      {
        if (model->IsStatic())
          continue;

        // Compute buoyancy for each link.
        for (auto link : model->GetLinks())
        {
          // Get the link's bounding box.
          ignition::math::AxisAlignedBox bbox = link->BoundingBox();
          double bottom = bbox.Min().Z();
          double top = bbox.Max().Z();
          double height = top - bottom;
          
          // Calculate submerged volume fraction.
          double volumeFraction = 0.0;
          if (top < this->heightThreshold)
            volumeFraction = 1.0; // fully submerged
          else if (bottom < this->heightThreshold)
            volumeFraction = (this->heightThreshold - bottom) / height;
          else
            volumeFraction = 0.0;
          
          // Estimate link's volume.
          double linkVolume = bbox.XLength() * bbox.YLength() * height;
          double submergedVolume = volumeFraction * linkVolume;

          // Compute buoyancy force.
          double fluidDensity = 1000.0; // kg/m^3
          double g = 9.81;              // m/s^2
          double buoyancyForceMagnitude = submergedVolume * fluidDensity * g;
          
          // Retrieve link's mass and compute its weight.
          double mass = (link->GetInertial()) ? link->GetInertial()->Mass() : 0.0;
          double weightForce = mass * g;
          
          // Net force (buoyancy minus weight).
          double netForceMagnitude = buoyancyForceMagnitude - weightForce;
          
          // Add damping force based on the link's vertical velocity.
          // (This helps stabilize the object.)
          ignition::math::Vector3d linearVel = link->WorldLinearVel();
          // Use only the vertical (Z) component.
          double verticalVel = linearVel.Z();
          double dampingCoefficient = 300.0; // Adjust this value as needed.
          double dampingForce = -dampingCoefficient * verticalVel;
          
          // Total vertical force.
          double totalVerticalForce = netForceMagnitude + dampingForce;
          
          gzdbg << "Model: " << model->GetName() 
                << ", Link: " << link->GetName()
                << ", Submerged Volume: " << submergedVolume
                << ", Buoyancy: " << buoyancyForceMagnitude 
                << ", Weight: " << weightForce 
                << ", Net: " << netForceMagnitude
                << ", Damping: " << dampingForce
                << ", Total: " << totalVerticalForce << "\n";
          
          // Apply the total vertical force.
          link->AddForce(ignition::math::Vector3d(0, 0, totalVerticalForce/700));
        }
      }
    }

  private:
    physics::WorldPtr world;
    event::ConnectionPtr updateConnection;
    double heightThreshold;
  };

  GZ_REGISTER_WORLD_PLUGIN(UpwardForceWorldPlugin)
}

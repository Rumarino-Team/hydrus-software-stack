// UpwardForceWorldPlugin.cpp

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  class UpwardForceWorldPlugin : public WorldPlugin
  {
  public:
    /// \brief Called when the plugin is loaded.
    /// \param[in] _world A pointer to the world.
    /// \param[in] _sdf SDF parameters for the plugin.
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
    {
      this->world = _world;

      physics::ModelPtr waterPlane = world->ModelByName("water_plane");

       if (waterPlane)
      {
        // Get the world pose of the water plane
        ignition::math::Pose3d waterPose = waterPlane->WorldPose();
        this->heightThreshold = waterPose.Pos().Z();
      } else {
        this->heightThreshold = 2.0;
      }

      // TODO: Take into account volume of model for more realistic buoyancy
      this->forceMagnitude = 10.0;

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&UpwardForceWorldPlugin::OnUpdate, this));
      
      gzdbg << "[UpwardForceWorldPlugin] Loaded with height_threshold = " 
            << this->heightThreshold << " and force = " 
            << this->forceMagnitude << "\n";
    }

    void OnUpdate()
    {
      for (auto model : this->world->Models())
      {
        if (model->IsStatic())
          continue;

        ignition::math::Pose3d pose = model->WorldPose();

        if (pose.Pos().Z() < this->heightThreshold)
        {
          for (auto link : model->GetLinks())
          {
            // Apply force in the +Z direction.
            link->AddForce(ignition::math::Vector3d(0, 0, this->forceMagnitude));
          }
        }
      }
    }

  private:
    /// \brief Pointer to the world.
    physics::WorldPtr world;

    /// \brief Connection to the world update event.
    event::ConnectionPtr updateConnection;

    /// \brief Height threshold (in meters) below which force is applied.
    double heightThreshold;

    /// \brief Magnitude of the upward force (in Newtons).
    double forceMagnitude;
  };

  GZ_REGISTER_WORLD_PLUGIN(UpwardForceWorldPlugin)
}

#ifndef PLUGIN_HH_
#define PLUGIN_HH_

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/System.hh"
#include "ignition/gazebo/Link.hh"

namespace ignition::gazebo {

// Global to track if we have reset the sim already. Used to change plugin
// behavior.
static std::atomic<bool> kDidReset {false};

// Plugin which does not implement the Reset API.
class TestPlugin : public System,
                   public ISystemConfigure,
                   public ISystemPreUpdate {
 public:
  void Configure(const Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 EntityComponentManager& ecm, EventManager& eventMgr) override;

  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override;

  bool grasping_ = false;
  Entity object_model_entity_;
  Entity joint_entity_;
  Link link_;
  Entity link_collision_entity_;
  Entity colliding_object_link_entity_;
  Entity detachable_joint_entity_;
  double command_position_ = 0;
  bool error_logged_ = false;
};
}  // namespace ignition::gazebo

#endif

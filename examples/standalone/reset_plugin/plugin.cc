#include "plugin.hh"

#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/components/ContactSensorData.hh"
#include "ignition/gazebo/components/DetachableJoint.hh"
#include "ignition/gazebo/components/JointPositionReset.hh"

using namespace ignition;
using namespace ignition::gazebo;
namespace components = ignition::gazebo::components;

void TestPlugin::Configure(const Entity& entity,
               const std::shared_ptr<const sdf::Element>& sdf,
               EntityComponentManager& ecm, EventManager& eventMgr) {
  auto model = Model(entity);
  link_ = Link(model.Links(ecm)[0]);
  joint_entity_ = model.Joints(ecm)[0];

  link_collision_entity_ = link_.Collisions(ecm)[0];

  // Add ContactSensorData component to link collision so that we can grasp
  // the object when the arm is in contact with it.
  ecm.CreateComponent(link_collision_entity_,
                      components::ContactSensorData());

  std::unordered_set<Entity> objects = entitiesFromScopedName("object", ecm);
  object_model_entity_ = *objects.begin();
}

void TestPlugin::PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) {
  auto pose = worldPose(object_model_entity_, ecm);
  // igndbg << "Object pose: " << pose.X() << " " << pose.Y() << " "
  //        << pose.Z() << "\n";

  if (pose.Y() < -0.01 && !error_logged_) {
    error_logged_ = true;
    ignerr << "Object moved unexpectedly to left of table! \n";
  }

  // The plugin behavior below is only for the first time the simulation runs,
  // so if we did reset already, just ignore.
  if (kDidReset) return;

  // Try grasping object with arm after 1.1 seconds
  if (!grasping_ && info.iterations > 1100) {
    std::optional<::ignition::msgs::Contacts> contact_data_optional =
        ecm.ComponentData<components::ContactSensorData>(
            link_collision_entity_);

    if (contact_data_optional &&
        contact_data_optional.value().contact_size() > 0) {
      auto collision1 = contact_data_optional.value().contact(0).collision1();
      if (collision1.id() != link_collision_entity_) {
        colliding_object_link_entity_ = ecm.ParentEntity(collision1.id());
      }
      auto collision2 = contact_data_optional.value().contact(0).collision2();
      if (collision2.id() != link_collision_entity_) {
        colliding_object_link_entity_ = ecm.ParentEntity(collision2.id());
      }

      // Add a DetachableJoint between arm link and object to emulate a
      // suction gripper.
      detachable_joint_entity_ = ecm.CreateEntity();
      components::DetachableJointInfo info(
          {.parentLink = link_.Entity(),
           .childLink = colliding_object_link_entity_});
      ecm.CreateComponent(detachable_joint_entity_,
                          components::DetachableJoint(info));
      grasping_ = true;
    }
  }

  // At 1 second, set command position to 0.15, which corresponds to the arm
  // lining up with the object.
  // At 3 seconds, set command position to 0.3, which corresponds to the arm
  // hanging out to the right of the table.
  if (info.iterations == 1000) {
    command_position_ = 0.15;
  } else if (info.iterations == 3000) {
    command_position_ = 0.3;
  }

  // Set joint position directly to commanded position by setting the
  // commanded position to the JointPositionReset component.
  auto& component =
      *(ecm.CreateComponent(joint_entity_, components::JointPositionReset()));
  component.Data().resize(1);
  component.Data()[0] = command_position_;
}

IGNITION_ADD_PLUGIN(ignition::gazebo::TestPlugin,
                    ::ignition::gazebo::System,
                    ignition::gazebo::TestPlugin::ISystemConfigure,
                    ignition::gazebo::TestPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::TestPlugin, "TestPlugin")

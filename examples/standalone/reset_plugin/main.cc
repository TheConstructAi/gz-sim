#include <iostream>

#include "ignition/common/Console.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/components/ContactSensorData.hh"
#include "ignition/gazebo/components/DetachableJoint.hh"
#include "ignition/gazebo/components/JointPositionReset.hh"
#include "ignition/transport/Node.hh"

#include "plugin.hh"

using namespace ignition;
using namespace ignition::gazebo;
namespace components = ignition::gazebo::components;

int main(int argc, char** argv) {
  ::ignition::common::Console::SetVerbosity(4);

  if (argc < 2) {
    ignerr << "You did not pass a path to a SDF file.\n";
    return -1;
  }

  std::string sdf_path = argv[argc - 1];
  ignition::gazebo::ServerConfig server_config;

  if (!server_config.SetSdfFile(sdf_path)) {
    std::cerr << "Failed to set SDF file [" << sdf_path << "]\n";
    return -1;
  }

  ignition::gazebo::Server server(server_config);
  server.Run(true, 5000, false);
  ignmsg << "Server ran for " << server.IterationCount().value_or(0)
            << " iterations \n";

  // First reset
  ::ignition::transport::Node node;
  ignition::msgs::WorldControl reset_msg;
  reset_msg.mutable_reset()->set_all(true);
  ignition::msgs::Boolean resp;
  bool success;
  if (!node.Request("world/default/control", reset_msg, 3000, resp, success) ||
      !success) {
    ignerr << "Failed to reset \n";
    return -1;
  } else {
    igndbg << "First reset" << std::endl;
  }
  kDidReset = true;

  server.Run(true, 5000, false);
  ignmsg << "Server ran for " << server.IterationCount().value_or(0)
         << " iterations \n";

  // Second reset
  if (!node.Request("world/default/control", reset_msg, 3000, resp, success) ||
      !success) {
    ignerr << "Failed to reset \n";
    return -1;
  } else {
    igndbg << "Second reset" << std::endl;
  }


  server.Run(true, 1000, false);

  return 0;
}

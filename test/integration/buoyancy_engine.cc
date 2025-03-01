/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <gtest/gtest.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/msgs/double.pb.h>
#include <ignition/transport/Node.hh>
#include <ignition/utilities/ExtraTestMacros.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Model.hh"

#include "ignition/gazebo/test_config.hh"
#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace gazebo;

class BuoyancyEngineTest : public InternalFixture<::testing::Test>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    InternalFixture::SetUp();
    this->pub = this->node.Advertise<ignition::msgs::Double>(
      "/model/buoyant_box/buoyancy_engine/");
  }

  /// \brief Node for communication
  public: ignition::transport::Node node;

  /// \brief Publishes commands
  public: ignition::transport::Node::Publisher pub;
};

/////////////////////////////////////////////////
TEST_F(BuoyancyEngineTest, IGN_UTILS_TEST_DISABLED_ON_WIN32(TestDownward))
{
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "buoyancy_engine.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  using namespace std::chrono_literals;
  server.SetUpdatePeriod(1ns);

  std::size_t iterations = 10000;

  test::Relay testSystem;
  std::vector<ignition::math::Pose3d> poses;

  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &/*_info*/,
                             const gazebo::EntityComponentManager &_ecm)
  {
    // Check pose
    Entity buoyantBox = _ecm.EntityByComponents(
      components::Model(), components::Name("buoyant_box"));
    EXPECT_NE(kNullEntity, buoyantBox);

    auto submarineBuoyantPose = _ecm.Component<components::Pose>(buoyantBox);
    EXPECT_NE(submarineBuoyantPose, nullptr);
    if (submarineBuoyantPose == nullptr)
    {
      ignerr << "Unable to get pose" <<std::endl;
      return;
    }
    poses.push_back(submarineBuoyantPose->Data());
  });

  server.AddSystem(testSystem.systemPtr);
  ignition::msgs::Double volume;
  volume.set_data(0);
  this->pub.Publish(volume);
  server.Run(true, iterations, false);

  EXPECT_LT(poses.rbegin()->Pos().Z(), poses.begin()->Pos().Z());
  EXPECT_NEAR(poses.rbegin()->Pos().X(), poses.begin()->Pos().X(), 1e-3);
  EXPECT_NEAR(poses.rbegin()->Pos().Y(), poses.begin()->Pos().Y(), 1e-3);
}

/////////////////////////////////////////////////
TEST_F(BuoyancyEngineTest, IGN_UTILS_TEST_DISABLED_ON_WIN32(TestUpward))
{
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "buoyancy_engine.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  using namespace std::chrono_literals;
  server.SetUpdatePeriod(1ns);

  std::size_t iterations = 10000;

  test::Relay testSystem;
  std::vector<ignition::math::Pose3d> poses;

  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &/*_info*/,
                             const gazebo::EntityComponentManager &_ecm)
  {
    // Check pose
    Entity buoyantBox = _ecm.EntityByComponents(
      components::Model(), components::Name("buoyant_box"));
    EXPECT_NE(kNullEntity, buoyantBox);

    auto submarineBuoyantPose = _ecm.Component<components::Pose>(buoyantBox);
    EXPECT_NE(submarineBuoyantPose, nullptr);
    if (submarineBuoyantPose == nullptr)
    {
      ignerr << "Unable to get pose" <<std::endl;
      return;
    }
    poses.push_back(submarineBuoyantPose->Data());
  });

  server.AddSystem(testSystem.systemPtr);
  ignition::msgs::Double volume;
  volume.set_data(10);
  this->pub.Publish(volume);
  server.Run(true, iterations, false);

  EXPECT_GT(poses.rbegin()->Pos().Z(), poses.begin()->Pos().Z());
  EXPECT_NEAR(poses.rbegin()->Pos().X(), poses.begin()->Pos().X(), 1e-3);
  EXPECT_NEAR(poses.rbegin()->Pos().Y(), poses.begin()->Pos().Y(), 1e-3);
}

/////////////////////////////////////////////////
TEST_F(BuoyancyEngineTest, IGN_UTILS_TEST_DISABLED_ON_WIN32(TestUpwardSurface))
{
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "buoyancy_engine.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  using namespace std::chrono_literals;
  server.SetUpdatePeriod(1ns);

  std::size_t iterations = 10000;

  test::Relay testSystem;
  std::vector<ignition::math::Pose3d> poses;

  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &/*_info*/,
                             const gazebo::EntityComponentManager &_ecm)
  {
    // Check pose
    Entity buoyantBox = _ecm.EntityByComponents(
      components::Model(), components::Name("buoyant_box_w_surface"));
    EXPECT_NE(kNullEntity, buoyantBox);

    auto submarineBuoyantPose = _ecm.Component<components::Pose>(buoyantBox);
    EXPECT_NE(submarineBuoyantPose, nullptr);
    if (submarineBuoyantPose == nullptr)
    {
      ignerr << "Unable to get pose" <<std::endl;
      return;
    }
    poses.push_back(submarineBuoyantPose->Data());
    double tol = 0.1;
    EXPECT_LT(submarineBuoyantPose->Data().Z(), 0 + tol);
  });

  server.AddSystem(testSystem.systemPtr);
  ignition::msgs::Double volume;
  volume.set_data(10);
  this->pub.Publish(volume);
  server.Run(true, iterations, false);

  EXPECT_GT(poses.rbegin()->Pos().Z(), poses.begin()->Pos().Z());
  EXPECT_NEAR(poses.rbegin()->Pos().X(), poses.begin()->Pos().X(), 1e-3);
  EXPECT_NEAR(poses.rbegin()->Pos().Y(), poses.begin()->Pos().Y(), 1e-3);
}


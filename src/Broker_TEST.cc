/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gtest/gtest.h"
#include "msgs/datagram.pb.h"
#include "swarm/Broker.hh"
#include "swarm/RobotPlugin.hh"

using namespace swarm;

//////////////////////////////////////////////////
/// \brief A sample broker client class.
class Client : public RobotPlugin
{
  /// \brief Constructor.
  /// \param[in] _id The client ID.
  public: Client(const uint32_t &_id)
    : id(_id)
  {
  }

  // Documentation inherited.
  void OnMsgReceived(const msgs::Datagram &/*_msg*/) const
  {
  }

  void OnNeighborsReceived(const std::vector<uint32_t> &/*_msg*/)
  {
  }

  /// \brief A client ID.
  public: uint32_t id;
};

//////////////////////////////////////////////////
/// \brief Test the basic API of the broker class.
TEST(brokerTest, Broker)
{
  Broker *broker = Broker::Instance();

  // Client #1.
  Broker *broker1 = Broker::Instance();
  Client client1(10);
  EXPECT_TRUE(broker1->Register(client1.id, &client1));

  // Try to register an existing client.
  EXPECT_FALSE(broker1->Register(client1.id, &client1));

  // Client #2.
  Broker *broker2 = Broker::Instance();
  Client client2(11);
  EXPECT_TRUE(broker2->Register(client2.id, &client2));

  EXPECT_EQ(broker->Clients().size(), 2u);

  // Bind.
  int port = 5000;
  uint32_t endPoint1 = client1.id * swarm::kMaxPort + port;
  EXPECT_TRUE(broker1->Bind(client1.id, &client1, endPoint1));
  EXPECT_FALSE(broker1->Bind(client1.id, &client1, endPoint1));
  uint32_t endPoint2 = client2.id * swarm::kMaxPort + port;
  EXPECT_TRUE(broker2->Bind(client2.id, &client2, endPoint2));
  EXPECT_FALSE(broker2->Bind(client2.id, &client2, endPoint2));

  const auto &endpoints = broker->EndPoints();

  ASSERT_TRUE(endpoints.find(endPoint1) != endpoints.end());
  EXPECT_EQ(endpoints.at(endPoint1).size(), 1u);
  ASSERT_TRUE(endpoints.find(endPoint2) != endpoints.end());
  EXPECT_EQ(endpoints.at(endPoint2).size(), 1u);

  // Push.
  msgs::Datagram msg;
  msg.set_src_address(client1.id);
  msg.set_dst_address(client2.id);
  msg.set_dst_port(port);
  msg.set_data("some data");
  broker1->Push(msg);
  broker2->Push(msg);
  EXPECT_EQ(broker->Messages().size(), 2u);

  // Unregister the clients.
  EXPECT_TRUE(broker1->Unregister(client1.id));
  EXPECT_TRUE(broker2->Unregister(client2.id));

  // Try to unregister clients that are not registered anymore.
  EXPECT_FALSE(broker1->Unregister(client1.id));
  EXPECT_FALSE(broker2->Unregister(client2.id));
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

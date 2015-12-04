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
#include <gtest/gtest.h>

#include "comms_plugin.hh"
#include "test/test_config.h"

using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(CommsPlugin)

//////////////////////////////////////////////////
CommsPlugin::CommsPlugin()
: RobotPlugin()
{
}

//////////////////////////////////////////////////
void CommsPlugin::Load(sdf::ElementPtr _sdf)
{
  this->testCase = _sdf->Get<int>("test_case");

  // Bind on my local address and default port.
  EXPECT_TRUE(this->Bind(&CommsPlugin::OnDataReceived, this, this->Host()));

  // Bind on the multicast group and default port.
  EXPECT_TRUE(this->Bind(&CommsPlugin::OnDataReceived, this, this->kMulticast));
}

//////////////////////////////////////////////////
void CommsPlugin::Update(const gazebo::common::UpdateInfo & /*_info*/)
{
  uint32_t dstAddress;

  if (this->iterations < 100)
  {
    if (this->Host() == Robot1)
      dstAddress = Robot2;
    else
      dstAddress = Robot1;

    if ((this->iterations == 0) &&
        ((this->testCase >= 0) && (this->testCase <= 11)))
    {
      // Try to send a message bigger than the MTU.
      std::string data(this->kMtu + 1, 'x');
      EXPECT_FALSE(this->SendTo(data, dstAddress));

      // Try to send a message less than the MTU.
      std::string data2(this->kMtu - 1, 'x');
      EXPECT_TRUE(this->SendTo(data2, dstAddress));
      this->numUnicastSent++;

      // Try to send a message equal to the MTU.
      std::string data3(this->kMtu, 'x');
      EXPECT_TRUE(this->SendTo(data3, dstAddress));
      this->numUnicastSent++;
    }

    // Send a unicast message.
    EXPECT_TRUE(this->SendTo("Unicast data", dstAddress));
    this->numUnicastSent++;

    // Send a broadcast message.
    dstAddress = this->kBroadcast;
    EXPECT_TRUE(this->SendTo("Broadcast data", dstAddress));
    this->numBroadcastSent++;

    // Send a multicast message.
    dstAddress = this->kMulticast;
    EXPECT_TRUE(this->SendTo("Multicast data", dstAddress));
    this->numMulticastSent++;
  }

  // Check for results only in one robot.
  if ((this->iterations == 100) && (this->Host() == Robot2))
  {
    int expectedNumMsgs = 0;

    switch (this->testCase)
    {
      // No drops or outages.
      case 0:
      case 4:
      {
        expectedNumMsgs = this->numUnicastSent + this->numBroadcastSent +
          this->numMulticastSent;
        break;
      }
      // All packages drop.
      case 1:
      case 2:
      case 3:
      case 5:
      case 6:
      case 11:
      {
        // We shouldn't see any messages.
        expectedNumMsgs = 0;
        break;
      }
      // 50% packages to remote destinations drop.
      case 7:
      {
        // The ideal number should be 151:
        // 50% of the 302 messages sent from the other robot.
        // Using 13458 as seed, we received 150.
        expectedNumMsgs = 150;
        break;
      }
      // Temporary outage.
      case 8:
      {
        // The length of the outage is going to be 10 iterations.
        // We should miss 30 messages.  Using 13456 as seed, we get one outage.
        expectedNumMsgs = this->numUnicastSent + this->numBroadcastSent +
          this->numMulticastSent - 30;
        break;
      }
      // Permanent outage.
      case 9:
      {
        // Robot with address 192.168.2.1 enters into an outage at iteration 26.
        // Two extra messages are sent on iteration 0, otherwise each iteration
        // sends three messages (uni, multi, broad).
        expectedNumMsgs = 74;
        break;
      }
      // Temporary outage + drops.
      case 10:
      {
        // Robot with address 192.168.2.1 enters into an outage at iteration 12
        // with a duration of 20 iterations.
        // This is a total of 20 * 3 = 60 missing messages.
        // 13 packages were dropped targeted to 192.168.2.2 .
        // From the ideal case in which we should receive 302 messages,
        // we missed 60 + 13 = 73.
        // The expected number of messages is: 302 - 73 = 229.
        expectedNumMsgs = this->numUnicastSent + this->numBroadcastSent +
          this->numMulticastSent - 73;
        break;
      }
      // Low max data rate to drop half of the messages due to a busy channel.
      case 12:
      {
        // We tweaked the maximum data rate to allow only 1 message per cycle.
        // There are 6 messages sent per cycle. Each iteration, we'll
        // receive the message with prob=3/6. The expectation would be to
        // receive 50 messages. Using 13458 as seed, we receive 52 messages.
        expectedNumMsgs = 52;
        break;
      }
      default:
      {
        gzerr << "CommsPlugin::Update() Test [" << this->testCase << "] "
              << "not expected." << std::endl;
        FAIL();
      }
    };

    EXPECT_EQ(this->numMsgsRecv, expectedNumMsgs);
  }

  this->iterations++;
}

//////////////////////////////////////////////////
void CommsPlugin::OnDataReceived(const uint32_t /*_srcAddress*/,
    const uint32_t /*_dstAddress*/, const uint32_t /*_dstPort*/,
    const std::string &/*_data*/)
{
  this->numMsgsRecv++;
}

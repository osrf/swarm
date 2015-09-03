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
  std::string dstAddress;

  if (this->iterations < 100)
  {
    if (this->Host() == "192.168.2.1")
      dstAddress = "192.168.2.2";
    else
      dstAddress = "192.168.2.1";

    if (this->iterations == 0)
    {
      // Try to send a message bigger than the MTU.
      std::string data(this->kMtu + 1, 'x');
      EXPECT_FALSE(this->SendTo(data, dstAddress));
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
  if ((this->iterations == 100) && (this->Host() == "192.168.2.2"))
  {
    int expectedNumMsgs = 0;

    switch (this->testCase)
    {
      // No drops or outages.
      case 0:
      case 4:
      {
        expectedNumMsgs = this->numUnicastSent + 2 * this->numBroadcastSent +
          2 * this->numMulticastSent;
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
        // We should only see your own broadcast/multicast messages.
        expectedNumMsgs = this->numBroadcastSent + this->numMulticastSent;
        break;
      }
      // 50% packages to remote destinations drop.
      case 7:
      {
        // The ideal number should be 350:
        // 100 of your own broadcast messages.
        // 100 of your own multicast messages.
        // 50% of the 300 messages sent from the other robot.
        // Using 13458 as seed, the expected number of messages is 347.
        expectedNumMsgs = 347;
        break;
      }
      // Temporary outage.
      case 8:
      {
        // The expectation is to have one outage. The length of the outage is
        // going to be 10 iterations. We should miss 30 messages.
        // Using 13458 as seed, we get one outage.
        expectedNumMsgs = 470;
        break;
      }
      // Permanent outage.
      case 9:
      {
        // The expectation is to have one outage
        // after 0.5 secs. We shouldn't receive any messages after that.
        // Using seed 13458, we got the outage after 0.34 secs.
        // The robot with address 192.168.2.1 didn't have any outage.
        // This means we executed 33 iterations before the outage.
        // In the first iteration nobody received a message.
        // A vehicle always receives its own multicast/broadcast messages.
        // The expected number of messages is: 200 + 32 * 3 = 296.
        expectedNumMsgs = 296;
        break;
      }
      // Temporary outage + drops.
      case 10:
      {
        // The expectation is to have one outage after 0.5 secs with
        // a duration of 0.2 secs and 15 drops.
        // Using seed 13458, we got an outage during the interval 0.34-0.54 sec.
        // The robot with address 192.168.2.1 didn't have any outage.
        // This means we were under outage during 20 iterations.
        // During the first iteration nobody receives messages.
        // This is a total of 20 * 3 = 60 missing messages.
        // 14 packages were dropped targeted to 192.168.2.2 .
        // From the ideal case in which we should receive 500 messages,
        // we missed 60 + 14 = 74.
        // The expected number of messages is: 500 - 74 = 426.
        expectedNumMsgs = 426;
        break;
      }
      default:
      {
        gzerr << "Test [" << this->testCase << "] not expected." << std::endl;
        FAIL();
      }
    };

    EXPECT_EQ(this->numMsgsRecv, expectedNumMsgs);
  }

  this->iterations++;
}

//////////////////////////////////////////////////
void CommsPlugin::OnDataReceived(const std::string &/*_srcAddress*/,
    const std::string &/*_data*/)
{
  this->numMsgsRecv++;
}

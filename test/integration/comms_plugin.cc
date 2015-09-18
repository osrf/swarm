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
        // The ideal number should be 351:
        // 100 of your own broadcast messages.
        // 100 of your own multicast messages.
        // 50% of the 302 messages sent from the other robot.
        // Using 13458 as seed, the expected number of messages is 354.
        expectedNumMsgs = 354;
        break;
      }
      // Temporary outage.
      case 8:
      {
        // The expectation is to have two outages. The length of the outage is
        // going to be 10 iterations. We should miss 60 messages.
        // Using 13458 as seed, we get one outage.
        expectedNumMsgs = (this->numUnicastSent + 2 *
            this->numBroadcastSent + 2 * this->numMulticastSent) - 60;
        break;
      }
      // Permanent outage.
      case 9:
      {
        // The expectation is one outage to occur on 192.168.2.1 at
        // iteration 9 and one outage on 192.168.2.2 on iteration 39.
        // A vehicle always receives its own multicast/broadcast messages.
        // The expected number of messages is:
        // broadcast + multicast + ((9 * 3) + 2)
        // Two extra messages are sent on iteration 0, otherwise each iteration
        // sends three message (uni, multi, broad).
        expectedNumMsgs =
          ((this->numBroadcastSent + this->numMulticastSent)) + 29;
        break;
      }
      // Temporary outage + drops.
      case 10:
      {
        // The expectation is to have one outage on iteration 9 with
        // a duration of 20 iterations for robot 192.168.2.1 using seed
        // 13458. This is a total of 20 * 3 = 60 missing messages.
        // 13 packages were dropped targeted to 192.168.2.2 .
        // From the ideal case in which we should receive 500 messages,
        // we missed 60 + 13 = 73.
        // The expected number of messages is: 502 - 73 = 429.
        int total = this->numUnicastSent + 2 * this->numBroadcastSent +
          2 * this->numMulticastSent;
        expectedNumMsgs = total - 73;
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
void CommsPlugin::OnDataReceived(const std::string &/*_srcAddress*/,
    const std::string &/*_data*/)
{
  this->numMsgsRecv++;
}

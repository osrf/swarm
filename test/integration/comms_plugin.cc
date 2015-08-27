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
  ASSERT_TRUE(_sdf->HasElement("comms_model"));

  auto const &commsModelElem = _sdf->GetElement("comms_model");

  if (commsModelElem->HasElement("neighbor_distance_min"))
  {
    this->neighborDistanceMin =
      commsModelElem->Get<double>("neighbor_distance_min");
  }
  if (commsModelElem->HasElement("neighbor_distance_max"))
  {
    this->neighborDistanceMax =
      commsModelElem->Get<double>("neighbor_distance_max");
  }
  if (commsModelElem->HasElement("neighbor_distance_penalty_tree"))
  {
    this->neighborDistancePenaltyTree =
      commsModelElem->Get<double>("neighbor_distance_penalty_tree");
  }
  if (commsModelElem->HasElement("comms_distance_min"))
  {
    this->commsDistanceMin =
      commsModelElem->Get<double>("comms_distance_min");
  }
  if (commsModelElem->HasElement("comms_distance_max"))
  {
    this->commsDistanceMax =
      commsModelElem->Get<double>("comms_distance_max");
  }
  if (commsModelElem->HasElement("comms_distance_penalty_tree"))
  {
    this->commsDistancePenaltyTree =
      commsModelElem->Get<double>("comms_distance_penalty_tree");
  }
  if (commsModelElem->HasElement("comms_drop_probability_min"))
  {
    this->commsDropProbabilityMin =
      commsModelElem->Get<double>("comms_drop_probability_min");
  }
  if (commsModelElem->HasElement("comms_drop_probability_max"))
  {
    this->commsDropProbabilityMax =
      commsModelElem->Get<double>("comms_drop_probability_max");
  }
  if (commsModelElem->HasElement("comms_outage_probability"))
  {
    this->commsOutageProbability =
      commsModelElem->Get<double>("comms_outage_probability");
  }
  if (commsModelElem->HasElement("comms_outage_duration_min"))
  {
    this->commsOutageDurationMin =
      commsModelElem->Get<double>("comms_outage_duration_min");
  }
  if (commsModelElem->HasElement("comms_outage_duration_max"))
  {
    this->commsOutageDurationMax =
      commsModelElem->Get<double>("comms_outage_duration_max");
  }

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

    // All packages drop.
    if (((ignition::math::equal(this->commsDropProbabilityMin, 1.0))       &&
         (ignition::math::equal(this->commsDropProbabilityMax, 1.0)))      ||
        (ignition::math::equal(this->commsOutageProbability,   1.0))       ||
        (this->neighborDistanceMax < 1.0)                                  ||
        (this->commsDistanceMax    < 1.0)                                  ||
        (ignition::math::equal(this->neighborDistancePenaltyTree, 201.0)))
    {
      // We should only see your own broadcast/multicast messages.
      expectedNumMsgs = this->numBroadcastSent + this->numMulticastSent;
    }
    // No drops or outages.
    else if ((ignition::math::equal(this->commsDropProbabilityMin, 0.0)) &&
             (ignition::math::equal(this->commsDropProbabilityMax, 0.0)) &&
             (ignition::math::equal(this->commsOutageProbability,  0.0)))
    {
      expectedNumMsgs = this->numUnicastSent + 2 * this->numBroadcastSent +
        2 * this->numMulticastSent;
    }
    // 50% packages drop to remote destinations drop.
    else if ((ignition::math::equal(this->commsDropProbabilityMin, 0.5)) &&
             (ignition::math::equal(this->commsDropProbabilityMax, 0.5)))
    {
      // The ideal number should be 350:
      // 100 of your own broadcast messages.
      // 100 of your own multicast messages.
      // 50% of the 300 messages sent from the other robot.
      // Using 13458 as seed, the expected number of messages is 350, yes!.
      expectedNumMsgs = 350;
    }
    // Outage.
    else if ((ignition::math::equal(this->commsOutageProbability, 0.5)) &&
             (ignition::math::equal(this->commsOutageDurationMin, 0.1)) &&
             (ignition::math::equal(this->commsOutageDurationMax, 0.1)))
    {
      // Temporary outage.
      // The expectation is to have one outage. The length of the outage is
      // going to be 10 iterations. We should miss 30 messages.
      // Using 13220 as seed, we get one outage.
      expectedNumMsgs = 470;
    }
    else if ((ignition::math::equal(this->commsOutageProbability,  0.5)) &&
             (ignition::math::equal(this->commsOutageDurationMin, -1.0)) &&
             (ignition::math::equal(this->commsOutageDurationMax, -1.0)))
    {
      // Permanent outage. The expectation is to have one outage
      // after 0.5 secs. We shouldn't receive any messages after that.
      // Using seed 13111, we got the outage after 0.3 secs.
      // The robot with address 192.168.2.1 didn't have any outage.
      // This means we executed 29 iterations before the outage.
      // In the first iteration nobody received a message.
      // A vehicle always receives its own multicast/broadcast messages.
      // The expected number of messages is: 200 + 28 * 3 = 284
      expectedNumMsgs = 284;
    }
    else if ((ignition::math::equal(this->commsOutageProbability,  0.5))  &&
             (ignition::math::equal(this->commsOutageDurationMin,  0.2))  &&
             (ignition::math::equal(this->commsOutageDurationMax,  0.2))  &&
             (ignition::math::equal(this->commsDropProbabilityMin, 0.05)) &&
             (ignition::math::equal(this->commsDropProbabilityMax, 0.05)))
    {
      // Temporary outage + drops. The expectation is to have one outage
      // after 0.5 secs with a duration of 0.2 secs.
      // Using seed 13111, we got a first outage during the interval 0.3-0.5 sec
      // and a second outage starting on 0.92 (the test ends at 1.0).
      // The robot with address 192.168.2.1 didn't have any outage.
      // This means we were under outage during 20 + 9 iterations. Also, during
      // the first iteration nobody receives messages. This is a total of
      // 30 * 3 = 90 missing messages.
      // Also 8 packages were dropped targeted to 192.168.2.2
      // From the ideal case in which we should receive 500 messages, we missed
      // 90 + 8 = 98.
      // The expected number of messages is: 500 - 98 = 402.

      expectedNumMsgs = 402;
    }
    else
    {
      gzerr << "This parameter configuration is not covered in the test"
            << std::endl;
      FAIL();
    }
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
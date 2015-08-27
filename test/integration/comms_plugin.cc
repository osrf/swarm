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
    // All packages drop.
    if (((ignition::math::equal(this->commsDropProbabilityMin, 1.0))  &&
         (ignition::math::equal(this->commsDropProbabilityMax, 1.0))) ||
        (ignition::math::equal(this->commsOutageProbability,   1.0))  ||
        (this->neighborDistanceMax < 1.0)                             ||
        (this->commsDistanceMax    < 1.0)                             ||
        (ignition::math::equal(this->neighborDistancePenaltyTree, 201.0)))
    {
      // We should only see your own broadcast/multicast messages.
      auto expectedNumMsgs = this->numBroadcastSent + this->numMulticastSent;
      EXPECT_EQ(this->numMsgsRecv, expectedNumMsgs);
    }
    // No drops or outages.
    else if ((ignition::math::equal(this->commsDropProbabilityMin, 0.0)) &&
             (ignition::math::equal(this->commsDropProbabilityMax, 0.0)) &&
             (ignition::math::equal(this->commsOutageProbability,  0.0)))
    {
      auto expectedNumMsgs = this->numUnicastSent + 2 * this->numBroadcastSent +
        2 * this->numMulticastSent;
      EXPECT_EQ(this->numMsgsRecv, expectedNumMsgs);
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
      auto expectedNumMsgs = 350;
      EXPECT_EQ(this->numMsgsRecv, expectedNumMsgs);
    }
    else
    {
      gzerr << "This parameter configuration is not covered in the test"
            << std::endl;
      FAIL();
    }
  }

  this->iterations++;
}

//////////////////////////////////////////////////
void CommsPlugin::OnDataReceived(const std::string &/*_srcAddress*/,
    const std::string &/*_data*/)
{
  this->numMsgsRecv++;
}

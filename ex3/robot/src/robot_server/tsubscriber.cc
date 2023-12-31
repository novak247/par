/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <iostream>
#include <string>
#include <ignition/transport.hh>

#include "robot_messages/robot_messages.pb.h"

//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void cb(const robot_messages::OdometryMsg &_msg)
{
  std::cout << "Msg: " << _msg.x() << ", " << _msg.y() << ", " << _msg.heading() << std::endl;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ignition::transport::Node node;
  std::string topic = "/turtlebot/odometry";

  // Subscribe to a topic by registering a callback.
  if (!node.Subscribe(topic, cb))
  {
    std::cerr << "Error subscribing to topic [" << topic << "]" << std::endl;
    return -1;
  }

  // Zzzzzz.
  ignition::transport::waitForShutdown();

  return 0;
}

// Copyright 2023 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Jason Hillier (robotics@hillier.us)

#include <numeric>

#include "ethercat_generic_plugins/generic_ec_stateproxy_slave.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ethercat_generic_plugins
{

  EcStateProxySlave::EcStateProxySlave()
      : GenericEcSlave() {}
  EcStateProxySlave::~EcStateProxySlave() {}

  void EcStateProxySlave::set_state_is_operational(bool value)
  {
    //subUnit->set_state_is_operational(value);
  }

  void EcStateProxySlave::processData(size_t entry_idx, uint8_t *domain_address)
  {
    return GenericEcSlave::processData(entry_idx, domain_address);
  }

  bool EcStateProxySlave::setupSlave(
      std::unordered_map<std::string, std::string> slave_parameters,
      std::vector<double> *state_interface,
      std::vector<double> *command_interface)
  {
    return GenericEcSlave::setupSlave(slave_parameters, state_interface, command_interface);
  }

} // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::EcStateProxySlave, ethercat_interface::EcSlave)

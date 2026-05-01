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

#include "ethercat_generic_plugins/generic_ec_multiplex_slave.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ethercat_generic_plugins
{

  EcMultiplexSlave::EcMultiplexSlave()
      : GenericEcSlave()
  {

    std::cout << "Creating Multiplex Slave" << std::endl;
  }
  EcMultiplexSlave::~EcMultiplexSlave() {}

  bool EcMultiplexSlave::initialized() { return initialized_; }

  void EcMultiplexSlave::set_state_is_operational(bool value)
  {
    subUnits_[0]->set_state_is_operational(value);
  }

  int EcMultiplexSlave::assign_activate_dc_sync()
  {
    if (subUnits_[0] != nullptr)
    {
      return subUnits_[0]->assign_activate_dc_sync();
    }
    return 0;
  }

  const ec_sync_info_t *EcMultiplexSlave::syncs()
  {
    return subUnits_[0]->syncs();
  }
  size_t EcMultiplexSlave::syncSize()
  {
    return subUnits_[0]->syncSize();
  }
  const ec_pdo_entry_info_t *EcMultiplexSlave::channels()
  {
    return subUnits_[0]->channels();
  }
  void EcMultiplexSlave::domains(DomainMap &domains) const
  {
    subUnits_[0]->domains(domains);
  }

  void EcMultiplexSlave::processData(size_t entry_idx, uint8_t *domain_address)
  {
    // hack copy internal config data !!!
    subUnits_[0].alias_ = alias_;
    subUnits_[0].position_ = position_;
    subUnits_[0].vendor_id_ = vendor_id_;
    subUnits_[0].product_id_ = product_id_;
    subUnits_[0]->sdo_config = sdo_config;
    // !!!

    subUnits_[0]->processData(entry_idx, domain_address);
    initialized_ = subUnits_[0]->initialized();
  }

  bool EcMultiplexSlave::setupSlave(
      std::unordered_map<std::string, std::string> slave_parameters,
      std::vector<double> *state_interface,
      std::vector<double> *command_interface)
  {
    std::cout << "Setting up Multiplex Slave" << std::endl;

    if (subUnits_[0] == nullptr)
    {
      subUnits_[0] = new EcCiA402Drive();
      return subUnits_[0]->setupSlave(slave_parameters, state_interface, command_interface);
    }
    return true;
  }

} // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::EcMultiplexSlave, ethercat_interface::EcSlave)

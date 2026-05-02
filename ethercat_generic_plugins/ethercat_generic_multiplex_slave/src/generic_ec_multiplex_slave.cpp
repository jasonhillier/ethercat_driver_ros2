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
      : GenericEcSlave() {}
  EcMultiplexSlave::~EcMultiplexSlave() {}

  bool EcMultiplexSlave::initialized() { return initialized_; }

  void EcMultiplexSlave::set_state_is_operational(bool value)
  {
    subUnit->set_state_is_operational(value);
  }

  int EcMultiplexSlave::assign_activate_dc_sync()
  {
    if (subUnit != nullptr)
    {
      return subUnit->assign_activate_dc_sync();
    }
    return 0;
  }
  /*
  const ec_sync_info_t *EcMultiplexSlave::syncs()
  {
    return subUnit->syncs();
  }
  size_t EcMultiplexSlave::syncSize()
  {
    return subUnit->syncSize();
  }
  const ec_pdo_entry_info_t *EcMultiplexSlave::channels()
  {
    return subUnit->channels();
  }
  */
  void EcMultiplexSlave::domains(DomainMap &domains) const
  {
    subUnit->domains(domains);
  }

  void EcMultiplexSlave::processData(size_t entry_idx, uint8_t *domain_address)
  {
    subUnit->processData(entry_idx, domain_address);
    initialized_ = subUnit->initialized();
  }

  bool EcMultiplexSlave::setupSlave(
      std::unordered_map<std::string, std::string> slave_parameters,
      std::vector<double> *state_interface,
      std::vector<double> *command_interface)
  {
    std::cout << "Setting up Multiplex Slave" << std::endl;

    if (subUnit == nullptr)
    {
      subUnit = new EcCiA402Drive();
      auto r = subUnit->setupSlave(slave_parameters, state_interface, command_interface);
      // hack copy internal config data !!!
      alias_ = subUnit->alias_;
      position_ = subUnit->position_;
      vendor_id_ = subUnit->vendor_id_;
      product_id_ = subUnit->product_id_;
      // merge all SDOs together
      /*
      for (const auto &sdo : subUnit->sdo_config)
      {
        sdo_master_config.push_back(sdo);
      }
      */

      sdo_config = subUnit->sdo_config;
      // !!!

      // merge all PDOs together
      for (const auto &rpdo : subUnit->rpdos_)
      {
        master_rpdos_.push_back(rpdo);
      }
      for (const auto &tpdo : subUnit->tpdos_)
      {
        master_tpdos_.push_back(tpdo);
      }
      rpdos_ = master_rpdos_;
      tpdos_ = master_tpdos_;
      return r;
    }
    return true;
  }

} // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::EcMultiplexSlave, ethercat_interface::EcSlave)

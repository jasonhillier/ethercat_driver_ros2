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

#ifndef ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_MULTIPLEX_SLAVE_HPP_
#define ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_MULTIPLEX_SLAVE_HPP_

#include <vector>
#include <string>
#include <unordered_map>
#include <limits>

#include "yaml-cpp/yaml.h"
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_interface/ec_pdo_single_interface_channel_manager.hpp"
#include "ethercat_generic_plugins/generic_ec_slave.hpp"
#include "ethercat_generic_plugins/generic_ec_cia402_drive.hpp"

namespace ethercat_generic_plugins
{

  class EcMultiplexSlave : public GenericEcSlave
  {
  public:
    EcMultiplexSlave();
    virtual ~EcMultiplexSlave();

    virtual const ec_sync_info_t *syncs();
    virtual size_t syncSize();
    virtual const ec_pdo_entry_info_t *channels();
    virtual void domains(DomainMap &domains) const;

    /** Returns true if drive has reached "operation enabled" state.
     *  The transition through the state machine is handled automatically. */
    bool initialized();

    virtual void processData(size_t entry_idx, uint8_t *domain_address);

    virtual bool setupSlave(
        std::unordered_map<std::string, std::string> slave_parameters,
        std::vector<double> *state_interface,
        std::vector<double> *command_interface);

  protected:
    EcCiA402Drive *subUnits_[2] = {nullptr, nullptr};
    bool initialized_ = false;
  };
} // namespace ethercat_generic_plugins

#endif // ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_MULTIPLEX_SLAVE_HPP_

/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group,
 Warsaw University of Technology
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the Warsaw University of Technology nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "subsystem_deployer/subsystem_deployer.h"
#include "subsystem_common/abstract_port_converter.h"
#include <subsystem_msgs/GetSubsystemInfo.h>

#include <iostream>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <tinyxml.h>

#include "Eigen/Dense"
#include "ros/ros.h"
#include <rtt/rtt-config.h>
#include <rtt/os/main.h>
#include <rtt/ConnPolicy.hpp>
#include <rtt/Logger.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <ocl/TaskBrowser.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/base/ExecutableInterface.hpp>
#include <rtt/plugin/PluginLoader.hpp>
#include <rtt_roscomm/rostopic.h>

using namespace RTT;
using namespace std;

class SubsystemDeployerRosService : public SubsystemDeployerRosServiceBase {
  std::string master_component_name_;
 public:
  SubsystemDeployerRosService(const SubsystemDeployer& d,
                              const std::string & master_component_name)
      : d_(d),
        ss_GetSubsystemInfo_(
            n_.advertiseService(d.getSubsystemName() + "/getSubsystemInfo",
                                &SubsystemDeployerRosService::getSubsystemInfo,
                                this)),
        master_component_name_(master_component_name) {
  }

  std::string quote(std::string const& name) {
    return "\"" + name + "\"";
  }

  void addConnection(const subsystem_msgs::ConnectionInfo& ci) {
    bool found = false;
    for (int l = 0; l < connections_.size(); ++l) {
      if (ci.component_from == connections_[l].component_from
          && ci.port_from == connections_[l].port_from
          && ci.component_to == connections_[l].component_to
          && ci.port_to == connections_[l].port_to) {
        found = true;
        break;
      }
    }
    if (!found) {
      connections_.push_back(ci);
    }
  }

  void scanService(Service::shared_ptr sv) {
    std::vector < std::string > comp_ports;
    comp_ports.clear();
    // Get all component ports
    comp_ports = sv->getPortNames();
    // Loop over all ports
    for (unsigned int j = 0; j < comp_ports.size(); j++) {
      Logger::log() << Logger::Debug << "Port: " << comp_ports[j] << Logger::endl;
      std::list < internal::ConnectionManager::ChannelDescriptor > chns = sv
          ->getPort(comp_ports[j])->getManager()->getConnections();
      std::list<internal::ConnectionManager::ChannelDescriptor>::iterator k;
      if (chns.empty()) {
        subsystem_msgs::ConnectionInfo ci;
        // Display unconnected ports as well!
//        m_dot << quote(comp_ports[j]) << "[shape=point];\n";
        base::InputPortInterface* ipi =
            dynamic_cast<base::InputPortInterface*>(sv->getPort(comp_ports[j]));
        base::OutputPortInterface* opi =
            dynamic_cast<base::OutputPortInterface*>(sv->getPort(comp_ports[j]));
        if (ipi != 0) {
          ci.component_to = ipi->getInterface()->getOwner()->getName();
          ci.port_to = comp_ports[j];
        } else {
          ci.component_from = opi->getInterface()->getOwner()->getName();
          ci.port_from = comp_ports[j];
        }
        ci.unconnected = true;
        addConnection(ci);
      }
      for (k = chns.begin(); k != chns.end(); k++) {
        base::ChannelElementBase::shared_ptr bs = k->get<1>();
        ConnPolicy cp = k->get<2>();
        Logger::log() << Logger::Debug << "Connection id: " << cp.name_id << Logger::endl;
        std::string comp_in, port_in;
        if (bs->getInputEndPoint()->getPort() != 0) {
          if (bs->getInputEndPoint()->getPort()->getInterface() != 0) {
            comp_in = bs->getInputEndPoint()->getPort()->getInterface()
                ->getOwner()->getName();
          } else {
            comp_in = "free input ports";
          }
          port_in = bs->getInputEndPoint()->getPort()->getName();
        }
        Logger::log() << Logger::Debug << "Connection starts at port: " << port_in << Logger::endl;
        Logger::log() << Logger::Debug << "Connection starts at component: " << comp_in << Logger::endl;
        std::string comp_out, port_out;
        if (bs->getOutputEndPoint()->getPort() != 0) {
          if (bs->getOutputEndPoint()->getPort()->getInterface() != 0) {
            comp_out = bs->getOutputEndPoint()->getPort()->getInterface()
                ->getOwner()->getName();
          } else {
            comp_out = "free output ports";
          }
          port_out = bs->getOutputEndPoint()->getPort()->getName();
        }

        subsystem_msgs::ConnectionInfo ci;
        ci.component_from = comp_in;
        ci.port_from = port_in;
        ci.component_to = comp_out;
        ci.port_to = port_out;
        ci.unconnected = false;
        addConnection(ci);
      }
    }
    // Recurse:
    Service::ProviderNames providers = sv->getProviderNames();
    for (Service::ProviderNames::iterator it = providers.begin();
        it != providers.end(); ++it) {
      scanService(sv->provides(*it));
    }
  }

  bool getAllConnections() {
    connections_.clear();
    std::vector<RTT::TaskContext*> components = d_.getAllComponents();
    for (unsigned int i = 0; i < components.size(); i++) {
      TaskContext* tc = components[i];
      scanService(tc->provides());
    }
  }

  bool getSubsystemInfo(subsystem_msgs::GetSubsystemInfo::Request &req,
                        subsystem_msgs::GetSubsystemInfo::Response &res) {

    res.is_initialized = false;

    if (!d_.isInitialized()) {
        return true;
    }

    for (int i = 0; i < d_.getLowerInputBuffers().size(); ++i) {
      res.lower_inputs.push_back(
          d_.getChannelName(d_.getLowerInputBuffers()[i].interface_alias_));
      res.alias_lower_inputs.push_back(
          d_.getLowerInputBuffers()[i].interface_alias_);
    }

    for (int i = 0; i < d_.getUpperInputBuffers().size(); ++i) {
      res.upper_inputs.push_back(
          d_.getChannelName(d_.getUpperInputBuffers()[i].interface_alias_));
      res.alias_upper_inputs.push_back(
          d_.getUpperInputBuffers()[i].interface_alias_);
    }

    for (int i = 0; i < d_.getLowerOutputBuffers().size(); ++i) {
      res.lower_outputs.push_back(
          d_.getChannelName(d_.getLowerOutputBuffers()[i].interface_alias_));
      res.alias_lower_outputs.push_back(
          d_.getLowerOutputBuffers()[i].interface_alias_);
    }

    for (int i = 0; i < d_.getUpperOutputBuffers().size(); ++i) {
      res.upper_outputs.push_back(
          d_.getChannelName(d_.getUpperOutputBuffers()[i].interface_alias_));
      res.alias_upper_outputs.push_back(
          d_.getUpperOutputBuffers()[i].interface_alias_);
    }

    std::vector<RTT::TaskContext*> components = d_.getAllComponents();

    for (int i = 0; i < components.size(); ++i) {
      RTT::TaskContext* tc = components[i];
      subsystem_msgs::ComponentInfo cinf;
      cinf.name = tc->getName();
      cinf.is_converter = d_.isConverter(cinf.name);
      cinf.latex = d_.getComponentNameLatex(tc->getName());
      std::vector<RTT::base::PortInterface*> ports = tc->ports()->getPorts();
      for (int ip = 0; ip < ports.size(); ++ip) {
        subsystem_msgs::PortInfo pinf;

        pinf.name = ports[ip]->getName();

        pinf.is_connected = ports[ip]->connected();

        RTT::base::InputPortInterface* ipi;
        RTT::base::OutputPortInterface* opi;
        if (ipi = dynamic_cast<RTT::base::InputPortInterface*>(ports[ip])) {
          pinf.is_input = true;
        } else if (opi =
            dynamic_cast<RTT::base::OutputPortInterface*>(ports[ip])) {
          pinf.is_input = false;
        }

        const RTT::types::TypeInfo* ti = ports[ip]->getTypeInfo();
        std::vector < std::string > type_names = ti->getTypeNames();
        for (int itn = 0; itn < type_names.size(); ++itn) {
          pinf.type_names.push_back(type_names[itn]);
        }

        cinf.ports.push_back(pinf);
      }
      res.components.push_back(cinf);
    }

    RTT::TaskContext* master_component = NULL;
    for (int i = 0; i < components.size(); ++i) {
      if (components[i]->getName() == master_component_name_) {
        master_component = components[i];
        break;
      }
    }
    if (!master_component) {
      return true;
    }

    boost::shared_ptr<subsystem_common::MasterServiceRequester> master_service =
        master_component->getProvider < subsystem_common::MasterServiceRequester
            > ("master");

    if (!master_service) {
      return true;
    }

    getAllConnections();
    for (int i = 0; i < connections_.size(); ++i) {
      connections_[i].name = d_.getConnectionName(
          connections_[i].component_from + "." + connections_[i].port_from,
          connections_[i].component_to + "." + connections_[i].port_to);
      connections_[i].latex = d_.getConnectionNameLatex(
          connections_[i].component_from + "." + connections_[i].port_from,
          connections_[i].component_to + "." + connections_[i].port_to);
      res.component_connections.push_back(connections_[i]);
//            Logger::log() << Logger::Info << "conn " << i << ": "
//                << connections_[i].component_from << "." << connections_[i].port_from << " -> "
//                << connections_[i].component_to << "." << connections_[i].port_to << ", name: " << connections_[i].name
//                << Logger::endl;
    }

    const std::vector<const subsystem_common::BehaviorBase*>& behaviors =
        master_service->getBehaviors();
    for (int i = 0; i < behaviors.size(); ++i) {
      const std::vector<std::string>& r = behaviors[i]->getRunningComponents();

      subsystem_msgs::BehaviorInfo bi;
      bi.name = behaviors[i]->getName();
      for (int j = 0; j < r.size(); ++j) {
        bi.running_components.push_back(r[j]);
      }
      bi.terminal_condition = behaviors[i]->getTerminalCondition();
      bi.error_condition = behaviors[i]->getErrorCondition();
      res.behaviors.push_back(bi);

//            Logger::log() << Logger::Info << "behavior " << i << ": "
//                << bi.name
//                << Logger::endl;
    }

    // state machine information

    const std::vector<const subsystem_common::StateBase*>& states =
        master_service->getStates();
    for (int i = 0; i < states.size(); ++i) {
      subsystem_msgs::StateInfo si;
      si.name = states[i]->getName();

      const std::vector<int>& behavior_indices =
          states[i]->getBehaviorIndices();
      for (int j = 0; j < behavior_indices.size(); ++j) {
        si.behavior_names.push_back(
            master_service->getBehaviors()[behavior_indices[j]]->getName());
      }

      const std::vector<std::pair<int, std::string>>& next_states = states[i]
          ->getNextStates();

      for (int j = 0; j < next_states.size(); ++j) {
        subsystem_msgs::NextStateInfo nsi;
        nsi.name = master_service->getStates()[next_states[j].first]->getName();
        nsi.init_cond = next_states[j].second;
        si.next_states.push_back(nsi);
      }

      res.state_machine.push_back(si);
    }

    // end of state machine information

    res.is_initialized = d_.isInitialized();

    return true;
  }

 private:
  const SubsystemDeployer& d_;
  ros::NodeHandle n_;
  ros::ServiceServer ss_GetSubsystemInfo_;
  std::vector<subsystem_msgs::ConnectionInfo> connections_;
};

SubsystemDeployer::SubsystemDeployer(const std::string& name)
    : name_(name),
      is_initialized_(false),
      stream_component_(NULL) {
}

const std::string& SubsystemDeployer::getConnectionName(
    const std::string& from, const std::string& to) const {
  static const std::string empty = std::string();
  for (std::list<Connection>::const_iterator it = connections_.begin();
      it != connections_.end(); ++it) {
    std::string to_conv = it->to + "_conv_in";
    if (it->from == from && (it->to == to || to_conv == to)) {
      return it->name;
    }
  }
  return empty;
}

const std::string& SubsystemDeployer::getConnectionNameLatex(
    const std::string& from, const std::string& to) const {
  static const std::string empty = std::string();
  for (std::list<Connection>::const_iterator it = connections_.begin();
      it != connections_.end(); ++it) {
    std::string to_conv = it->to + "_conv_in";
    if (it->from == from && (it->to == to || to_conv == to)) {
      return it->latex;
    }
  }
  return empty;
}

const std::string& SubsystemDeployer::getComponentNameLatex(
    const std::string& name) const {
  static const std::string empty = std::string();
  auto it = component_names_latex_.find(name);
  if (it == component_names_latex_.end()) {
    return empty;
  }
  return it->second;
}

bool SubsystemDeployer::isInitialized() const {
  return is_initialized_;
}

const std::string& SubsystemDeployer::getChannelName(
    const std::string& alias) const {
  static const std::string empty = std::string();
  std::map<std::string, std::string>::const_iterator it = io_buffers_.find(
      alias);
  if (it == io_buffers_.end()) {
    return empty;
  }
  return it->second;
}

const std::vector<subsystem_common::InputBufferInfo>& SubsystemDeployer::getLowerInputBuffers() const {
  return lowerInputBuffers_;
}

const std::vector<subsystem_common::InputBufferInfo>& SubsystemDeployer::getUpperInputBuffers() const {
  return upperInputBuffers_;
}

const std::vector<subsystem_common::OutputBufferInfo>& SubsystemDeployer::getLowerOutputBuffers() const {
  return lowerOutputBuffers_;
}

const std::vector<subsystem_common::OutputBufferInfo>& SubsystemDeployer::getUpperOutputBuffers() const {
  return upperOutputBuffers_;
}

const std::string& SubsystemDeployer::getSubsystemName() const {
  return subsystem_name_;
}

const std::string& SubsystemDeployer::getMasterPackageName() const {
  return master_package_name_;
}

bool SubsystemDeployer::import(const std::string& name) {
  if (!ros_import_.ready()) {
    Logger::log() << Logger::Error << "ros.import operation is not ready"
        << Logger::endl;
    return false;
  }

  if (!ros_import_(name)) {
    Logger::log() << Logger::Error << "could not import: " << name
        << Logger::endl;
    return false;
  }

  return true;
}

static void printInputBufferInfo(
    const subsystem_common::InputBufferInfo& info) {
  Logger::log() << ", interface_alias: \'" << info.interface_alias_ << "\'"
      << Logger::endl;
}

static void printOutputBufferInfo(
    const subsystem_common::OutputBufferInfo& info) {
  Logger::log() << ", interface_alias: \'" << info.interface_alias_ << "\'"
      << Logger::endl;
}

static bool loadROSParam(RTT::TaskContext* tc) {
  tc->loadService("rosparam");

  RTT::Service::shared_ptr tc_rosparam = tc->provides("rosparam");

  RTT::OperationCaller < bool() > tc_rosparam_getAll =
      tc_rosparam->getOperation("getAll");
  if (!tc_rosparam_getAll.ready()) {
    Logger::log() << Logger::Error
        << "could not get ROS parameter getAll operation for " << tc->getName()
        << Logger::endl;
    return false;
  }
  if (!tc_rosparam_getAll()) {
    Logger::log() << Logger::Warning << "could not read ROS parameters for "
        << tc->getName() << Logger::endl;
//        return false;     // TODO: this IS an error
  }

  return true;
}

template<class T>
static bool setComponentProperty(RTT::TaskContext *tc,
                                 const std::string& prop_name, const T& value) {
  RTT::Property<T>* property = dynamic_cast<RTT::Property<T>*>(tc->properties()
      ->getProperty(prop_name));
  if (!property) {
    Logger::log() << Logger::Error << "component " << tc->getName()
        << " does not have property " << prop_name << Logger::endl;
    return false;
  }
  property->set(value);

  return true;
}

bool SubsystemDeployer::setChannelsNames() {
/*  for (std::map<std::string, std::string>::const_iterator it =
      io_buffers_.begin(); it != io_buffers_.end(); ++it) {
    if (!setComponentProperty<std::string>(master_component_,
                                           "channel_name_" + it->first,
                                           it->second)) {
      Logger::log() << Logger::Error << "Could not set component \'"
          << master_component_->getName() << "\'property \'channel_name_"
          << it->first << "\'" << Logger::endl;
//TODO: set names for input buffers only
      //return false;
    }
  }
*/
  for (int i = 0; i < lowerInputBuffers_.size(); ++i) {
    const std::string& alias = lowerInputBuffers_[i].interface_alias_;
    const std::string& ch_name = getChannelName(alias);
    if (!setComponentProperty<std::string>(master_component_,
                                           std::string("channel_name_") + alias,
                                           ch_name)) {
      Logger::log() << Logger::Error << "Could not set channel name for i/o buffer \'"
          << alias << "\', \'" << ch_name << "\'" << Logger::endl;
      return false;
    }
  }
  for (int i = 0; i < upperInputBuffers_.size(); ++i) {
    const std::string& alias = upperInputBuffers_[i].interface_alias_;
    const std::string& ch_name = getChannelName(alias);
    if (!setComponentProperty<std::string>(master_component_,
                                           std::string("channel_name_") + alias,
                                           ch_name)) {
      Logger::log() << Logger::Error << "Could not set channel name for i/o buffer \'"
          << alias << "\', \'" << ch_name << "\'" << Logger::endl;
      return false;
    }
  }




  RTT::TaskContext* comp_y = dc_->getPeer("Y");
  for (int i = 0; i < lowerOutputBuffers_.size(); ++i) {
    const std::string& alias = lowerOutputBuffers_[i].interface_alias_;
    const std::string& ch_name = getChannelName(alias);
    if (!setComponentProperty<std::string>(comp_y,
                                           std::string("channel_name_") + alias,
                                           ch_name)) {
      Logger::log() << Logger::Error << "Could not set channel name for i/o buffer \'"
          << alias << "\', \'" << ch_name << "\'" << Logger::endl;
      return false;
    }
  }
  for (int i = 0; i < upperOutputBuffers_.size(); ++i) {
    const std::string& alias = upperOutputBuffers_[i].interface_alias_;
    const std::string& ch_name = getChannelName(alias);
    if (!setComponentProperty<std::string>(comp_y,
                                           std::string("channel_name_") + alias,
                                           ch_name)) {
      Logger::log() << Logger::Error << "Could not set channel name for i/o buffer \'"
          << alias << "\', \'" << ch_name << "\'" << Logger::endl;
      return false;
    }
  }

  return true;
}

bool SubsystemDeployer::deployBufferSplitComponent(
    const subsystem_common::BufferInfo& buf_info) {
  std::string suffix = "Split";
  std::string type = buf_info.interface_type_ + suffix;

  std::string name = buf_info.interface_alias_ + suffix;

  if (!dc_->loadComponent(name, type)) {
    Logger::log() << Logger::Error << "Unable to load component " << type
        << Logger::endl;
    return false;
  }
  RTT::TaskContext* comp = dc_->getPeer(name);
  if (!setTriggerOnStart(comp, false)) {
    return false;
  }

  buffer_split_components_.push_back(comp);

  return true;
}

bool SubsystemDeployer::deployBufferConcateComponent(
    const subsystem_common::BufferInfo& buf_info) {
  std::string suffix = "Concate";
  std::string type = buf_info.interface_type_ + suffix;

  std::string name = buf_info.interface_alias_ + suffix;

  if (!dc_->loadComponent(name, type)) {
    Logger::log() << Logger::Error << "Unable to load component " << type
        << Logger::endl;
    return false;
  }
  RTT::TaskContext* comp = dc_->getPeer(name);
  if (!setTriggerOnStart(comp, false)) {
    return false;
  }

  buffer_concate_components_.push_back(comp);

  return true;
}

bool SubsystemDeployer::isConverter(const std::string& name) const {
  for (int i = 0; i < converter_components_.size(); ++i) {
    if (converter_components_[i]->getName() == name) {
      return true;
    }
  }
  return false;
}

bool SubsystemDeployer::connectPorts(const std::string& from,
                                     const std::string& to,
                                     const ConnPolicy& cp) {
    static int counter = 0;

    RTT::base::PortInterface *pa = strToPort(from);
    RTT::base::PortInterface *pb = strToPort(to);
    if (!pa) {
      Logger::log() << Logger::Error << "no such port: '" << from << "'" << Logger::endl;
      return false;
    }
    if (!pb) {
      Logger::log() << Logger::Error << "no such port: '" << to << "'" << Logger::endl;
      return false;
    }

    std::string conv_type = subsystem_common::PortConverterFactory::Instance()
        ->getPortConverter(pa, pb);

    if (conv_type.empty()) {
        if (dc_->connect(from, to, cp)) {
            return true;
        } else {
            Logger::log() << Logger::Error << "could not find converted for ports: '" << from
                << "' and '" << to << "'" << Logger::endl;
            return false;
        }
    } else {
        //Logger::log() << Logger::Info << "Could not create direct connection: between '" << from << "' and '" << to << "', trying to create data converter..." << Logger::endl;

        std::ostringstream strs;
        strs << counter;
        counter++;

        std::string comp_name = std::string("conv") + strs.str();
        if (!dc_->loadComponent(comp_name, conv_type)) {
          Logger::log() << Logger::Error << "Unable to load component " << conv_type
              << Logger::endl;
          return false;
        }

        if (!dc_->connect(from, comp_name + ".data_INPORT", cp)) {
          Logger::log() << Logger::Error << "Unable to connect: " << from << " and "
              << comp_name << ".data_INPORT" << Logger::endl;
          return false;
        }
        if (!dc_->connect(comp_name + ".data_OUTPORT", to, cp)) {
          Logger::log() << Logger::Error << "Unable to connect: " << comp_name
              << ".data_OUTPORT" << " and " << to << Logger::endl;
          return false;
        }

        // set name of both new connections the same as the old one
        connections_.push_back(
            Connection(from, comp_name + ".data_INPORT",
                       getConnectionName(from, to),
                       getConnectionNameLatex(from, to), cp));
        connections_.push_back(
            Connection(comp_name + ".data_OUTPORT", to, getConnectionName(from, to),
                       getConnectionNameLatex(from, to), cp));

        converter_components_.push_back(dc_->getPeer(comp_name));
        Logger::log() << Logger::Info << "Created data converter between '" << from << "' and '" << to << "', so the above error is harmless." << Logger::endl;
        return true;
    }
    return false;
}

bool SubsystemDeployer::createInputBuffers(
    const std::vector<subsystem_common::InputBufferInfo>& buffers) {
  for (int i = 0; i < buffers.size(); ++i) {
    const subsystem_common::InputBufferInfo& buf_info = buffers[i];

    if (!deployBufferSplitComponent(buf_info)) {
      return false;
    }

    const std::string alias = buf_info.interface_alias_;

    // connect Split-Rx ports
    if (!connectPorts(master_component_name_ + "." + alias + "_OUTPORT",
                      alias + "Split.msg_INPORT",
                      ConnPolicy(ConnPolicy::DATA, ConnPolicy::UNSYNC))) {
      Logger::log() << Logger::Error << "could not connect ports Split-Rx: " << alias
          << Logger::endl;
      return false;
    }

  }
  return true;
}

bool SubsystemDeployer::createOutputBuffers(
    const std::vector<subsystem_common::OutputBufferInfo>& buffers) {
  RTT::TaskContext* comp = dc_->getPeer("Y");

  if (!comp) {
    std::string type = getMasterPackageName() + "_types::OutputBuffers";
    if (!dc_->loadComponent("Y", type)) {
      Logger::log() << Logger::Error << "Unable to load component " << type
          << Logger::endl;
      return false;
    }
    comp = dc_->getPeer("Y");
    if (!setTriggerOnStart(comp, false)) {      // TODO: verify
      return false;
    }
  }

  for (int i = 0; i < buffers.size(); ++i) {
    const subsystem_common::OutputBufferInfo& buf_info = buffers[i];

    if (!deployBufferConcateComponent(buf_info)) {
      return false;
    }

    const std::string alias = buf_info.interface_alias_;

    if (!connectPorts(alias + "Concate.msg_OUTPORT",
                      std::string("Y.") + alias + "_INPORT",
                      ConnPolicy(ConnPolicy::DATA, ConnPolicy::UNSYNC))) {
      Logger::log() << Logger::Error << "could not connect ports Concate-Y: " << alias
          << Logger::endl;
      return false;
    }
  }
  return true;
}

bool SubsystemDeployer::setTriggerOnStart(RTT::TaskContext* tc, bool trigger) {
  RTT::base::AttributeBase* base = tc->attributes()->getAttribute(
      "TriggerOnStart");
  if (!base) {
    Logger::log() << Logger::Error << "component " << tc->getName()
        << " does not have attribute " << "TriggerOnStart" << Logger::endl;
    return false;
  }
  RTT::Attribute<bool>* triggerOnStart =
      static_cast<RTT::Attribute<bool>*>(base);
  if (!triggerOnStart) {
    Logger::log() << Logger::Error << "component " << tc->getName()
        << " does not have attribute " << "TriggerOnStart" << " of type bool"
        << Logger::endl;
    return false;
  }
  triggerOnStart->set(trigger);
  return true;
}

bool SubsystemDeployer::initializeSubsystem(
    const std::string& master_package_name,
    const std::string& subsystem_subname, int cpu_num) {

  cpu_num_ = cpu_num;

  master_package_name_ = master_package_name;
  subsystem_name_ = subsystem_subname;

  Logger::In in("SubsystemDeployer::init " + getSubsystemName());

  master_component_name_ = "master_component";  //std::string("master_") + master_package_name;
  ros_service.reset(
      new SubsystemDeployerRosService(*this, master_component_name_));

  dc_.reset(new OCL::DeploymentComponent(name_));

  /*
   RTT::Activity* dc_activity = dynamic_cast<RTT::Activity* >(dc_->getActivity());
   if (!dc_activity) {
   Logger::log() << Logger::Warning << "Could not set scheduler for Deployment Component to ORO_SCHED_RT. Could not get Activity." << Logger::endl;
   }
   else {
   //        if (!master_activity->setScheduler(ORO_SCHED_RT)) {
   //            Logger::log() << Logger::Warning << "Could not set scheduler for Master Component to ORO_SCHED_RT." << Logger::endl;
   //        }
   if (!dc_activity->setPriority(0)) {
   Logger::log() << Logger::Warning << "Could not set priority for Deployment Component to 0." << Logger::endl;
   }
   }
   */

  dc_->import("rtt_ros");

  RTT::Service::shared_ptr ros = RTT::internal::GlobalService::Instance()
      ->getService("ros");
  if (!ros) {
    Logger::log() << Logger::Error
        << "rtt_ros: ros service could not be loaded (NULL pointer)"
        << Logger::endl;
    return false;
  }

  ros_import_ = ros->getOperation("import");

  if (!import("rtt_roscomm") || !import("rtt_rosparam")
      || !import("rtt_rosclock") || !import("rtt_rospack")
      || !import("rtt_actionlib") || !import("subsystem_common")
      || !import("conman") || !import("rtt_diagnostic_msgs")
      || !import("eigen_typekit")) {
    Logger::log() << Logger::Error << "could not load some core dependencies"
        << Logger::endl;
    return false;
  }

  Logger::log() << Logger::Info << "loaded core dependencies" << Logger::endl;

  use_sim_time_ = false;
  ros::param::get("/use_sim_time", use_sim_time_);
  if (use_sim_time_) {
    Logger::log() << Logger::Info << "Simulation time is enabled."
        << Logger::endl;
  } else {
    Logger::log() << Logger::Info << "Simulation time is disabled."
        << Logger::endl;
  }

  //
  // load conman scheme
  //
  if (!dc_->loadComponent("scheme", "conman::Scheme")) {
    Logger::log() << Logger::Error << "could not load conman::Scheme"
        << Logger::endl;
    return false;
  }

  scheme_ = dc_->getPeer("scheme");

  if (!setTriggerOnStart(scheme_, false)) {
    return false;
  }

  if (!scheme_) {
    Logger::log() << Logger::Error << "scheme in NULL" << Logger::endl;
    return false;
  }
  Logger::log() << Logger::Info << "configuring component 'scheme'"
      << Logger::endl;
  scheme_->configure();

  //
  // load master component
  //
  if (!dc_->loadComponent(master_component_name_, "MasterComponent")) {
    Logger::log() << Logger::Error << "could not load MasterComponent"
        << Logger::endl;
    return false;
  }

  master_component_ = dc_->getPeer(master_component_name_);

  if (!master_component_) {
    Logger::log() << Logger::Error << "master_component in NULL"
        << Logger::endl;
    return false;
  }

  setComponentProperty<bool>(master_component_, "use_sim_time", use_sim_time_);

  //
  // load subsystem-specific master service
  //
  if (!import(master_package_name)) {
    Logger::log() << Logger::Error << "Master package could not be loaded!"
        << Logger::endl;
    return false;
  }

  if (!master_component_->loadService(master_package_name + "_master")) {
    Logger::log() << Logger::Error << "Could not load service \'"
        << master_package_name << "_master\'" << Logger::endl;
    return false;
  }

  RTT::OperationCaller<bool(RTT::TaskContext*)> master_component_addConmanScheme =
      master_component_->getOperation("addConmanScheme");
  if (!master_component_addConmanScheme.ready()) {
    Logger::log() << Logger::Error
        << "Could not get addConmanScheme operation of Master Component"
        << Logger::endl;
    return false;
  }

  if (!master_component_addConmanScheme(scheme_)) {
    Logger::log() << Logger::Error
        << "Could not add Conman Scheme to Master Component" << Logger::endl;
    return false;
  }

  RTT::OperationCaller<void(const std::string&)> master_component_setThreadName =
      master_component_->getOperation("setThreadName");
  if (!master_component_setThreadName.ready()) {
    Logger::log() << Logger::Error
        << "Could not get setThreadName operation of Master Component"
        << Logger::endl;
    return false;
  }

  master_component_setThreadName(master_package_name);

  //
  // manage ports and ipc buffers
  //
  std::vector < std::string > master_port_names = master_component_->ports()
      ->getPortNames();
  for (int i = 0; i < master_port_names.size(); ++i) {
    Logger::log() << Logger::Info << "master_component port[" << i << "]: "
        << master_port_names[i] << Logger::endl;
  }

  master_service_ = master_component_->getProvider
      < subsystem_common::MasterServiceRequester > ("master");
  if (!master_service_) {
    Logger::log() << Logger::Error
        << "Unable to load subsystem_common::MasterService from master_component"
        << Logger::endl;
    return false;
  }

  // inputs
  master_service_->getLowerInputBuffers(lowerInputBuffers_);
  master_service_->getUpperInputBuffers(upperInputBuffers_);

  Logger::log() << Logger::Info << "lowerInputBuffers:" << Logger::endl;
  for (int i = 0; i < lowerInputBuffers_.size(); ++i) {
    printInputBufferInfo(lowerInputBuffers_[i]);
  }

  Logger::log() << Logger::Info << "upperInputBuffers:" << Logger::endl;
  for (int i = 0; i < upperInputBuffers_.size(); ++i) {
    printInputBufferInfo(upperInputBuffers_[i]);
  }

  if (!createInputBuffers(lowerInputBuffers_)) {
    Logger::log() << Logger::Error << "Could not create lower input buffers"
        << Logger::endl;
    return false;
  }

  if (!createInputBuffers(upperInputBuffers_)) {
    Logger::log() << Logger::Error << "Could not create upper input buffers"
        << Logger::endl;
    return false;
  }

  // outputs
  master_service_->getLowerOutputBuffers(lowerOutputBuffers_);
  master_service_->getUpperOutputBuffers(upperOutputBuffers_);

  Logger::log() << Logger::Info << "lowerOutputBuffers:" << Logger::endl;
  for (int i = 0; i < lowerOutputBuffers_.size(); ++i) {
    printOutputBufferInfo(lowerOutputBuffers_[i]);
  }

  Logger::log() << Logger::Info << "upperOutputBuffers:" << Logger::endl;
  for (int i = 0; i < upperOutputBuffers_.size(); ++i) {
    printOutputBufferInfo(upperOutputBuffers_[i]);
  }

  if (!createOutputBuffers(lowerOutputBuffers_)) {
    Logger::log() << Logger::Error << "Could not create lower output buffers"
        << Logger::endl;
    return false;
  }

  if (!createOutputBuffers(upperOutputBuffers_)) {
    Logger::log() << Logger::Error << "Could not create upper output buffers"
        << Logger::endl;
    return false;
  }

  //
  // diagnostics ROS interface
  //
  dc_->loadComponent("diag", "DiagnosticComponent");
  diag_component_ = dc_->getPeer("diag");
  if (!diag_component_->setPeriod(0.5)) {
    Logger::log() << Logger::Error << "could not change period of component \'"
        << diag_component_->getName() << Logger::endl;
    return false;
  }

  RTT::base::PortInterface* diag_port_out_ = diag_component_->ports()->getPort(
      "diag_OUTPORT");
  if (diag_port_out_) {
    if (!diag_port_out_->createStream(
        rtt_roscomm::topic(std::string("/") + getSubsystemName() + "/diag"))) {
      Logger::log() << Logger::Error << "could not create ROS stream for port \'"
          << diag_component_->getName() << "." << diag_port_out_->getName()
          << "\'" << Logger::endl;
      return false;
    }
  } else {
    Logger::log() << Logger::Error << "component \'" << diag_component_->getName()
        << "\' does not have port \'diag_OUTPORT\'" << Logger::endl;
    return false;
  }

  Logger::log() << Logger::Info << "OK" << Logger::endl;

  Logger::log() << Logger::Info << "Master Component ports:" << Logger::endl;
  std::vector < std::string > port_names = master_component_->ports()
      ->getPortNames();
  for (int i = 0; i < port_names.size(); ++i) {
    Logger::log() << Logger::Info << "  " << port_names[i] << Logger::endl;
  }

  return true;
}

std::vector<RTT::TaskContext*> SubsystemDeployer::getAllComponents() const {
  std::vector<RTT::TaskContext*> result;

  std::vector < std::string > peer_names = dc_->getPeerList();
  // TODO: sort it according to execution order

  RTT::OperationCaller<bool(std::vector<std::string> &order)> scheme_getExecutionOrder = scheme_
      ->getOperation("getExecutionOrder");
  if (!scheme_getExecutionOrder.ready()) {
    Logger::log() << Logger::Error
        << "Could not get getExecutionOrder operation of Conman scheme" << Logger::endl;
    return result;
  }

  std::vector<std::string> exec_order;
  if (!scheme_getExecutionOrder(exec_order)) {
    Logger::log() << Logger::Error
        << "Could not get execution order from Conman scheme" << Logger::endl;
    return result;
  }

//  Logger::log() << Logger::Info << "execution order of Conman scheme:" << Logger::endl;
  result.push_back(master_component_);
  for (int i = 0; i < exec_order.size(); ++i) {
//    Logger::log() << Logger::Info << "    " << dc_->getPeer(exec_order[i])->getName() << Logger::endl;
    result.push_back(dc_->getPeer(exec_order[i]));
  }

  return result;
}

std::vector<RTT::TaskContext*> SubsystemDeployer::getCoreComponents() const {
  std::vector<RTT::TaskContext*> result;
  result.push_back(master_component_);
  result.push_back(scheme_);
  result.push_back(diag_component_);
  if (stream_component_) {
    result.push_back(stream_component_);
  }
  result.insert(result.end(), buffer_split_components_.begin(),
                buffer_split_components_.end());
  result.insert(result.end(), buffer_concate_components_.begin(),
                buffer_concate_components_.end());
  return result;
}

std::vector<RTT::TaskContext*> SubsystemDeployer::getNonCoreComponents() const {
  std::vector<RTT::TaskContext*> result;
  std::vector<RTT::TaskContext*> core = getCoreComponents();

  std::vector < std::string > peer_names = dc_->getPeerList();

  for (int i = 0; i < peer_names.size(); ++i) {
    const std::string& name = peer_names[i];
    bool is_core = false;
    for (int j = 0; j < core.size(); ++j) {
      if (core[j]->getName() == name) {
        is_core = true;
        break;
      }
    }
    if (!is_core) {
      TaskContext* tc = dc_->getPeer(name);
      result.push_back(tc);
    }
  }
  return result;
}

RTT::base::PortInterface* SubsystemDeployer::strToPort(
    const std::string &path) const {
  size_t first_dot = path.find(".");
  if (first_dot == std::string::npos) {
    return NULL;
  }

  TaskContext* tc = dc_->getPeer(path.substr(0, first_dot));
  if (!tc) {
    return NULL;
  }
  return tc->ports()->getPort(path.substr(first_dot + 1, std::string::npos));
}

bool SubsystemDeployer::connectionExists(const std::string& from,
                                         const std::string& to) const {
  RTT::base::PortInterface* pi_f = strToPort(from);
  RTT::base::PortInterface* pi_t = strToPort(from);
  if (!pi_f || !pi_t) {
    return false;
  }

  Logger::log() << Logger::Info << "port \'" << from << "\' is connected to:"
      << Logger::endl;

  std::list < internal::ConnectionManager::ChannelDescriptor > chns = pi_f
      ->getManager()->getConnections();
//      std::list<internal::ConnectionManager::ChannelDescriptor>::iterator k;
  for (auto k = chns.begin(); k != chns.end(); k++) {
    base::ChannelElementBase::shared_ptr bs = k->get<1>();
    /*        std::string comp_in, port_in;
     if(bs->getInputEndPoint()->getPort() != 0) {
     if (bs->getInputEndPoint()->getPort()->getInterface() != 0 ) {
     comp_in = bs->getInputEndPoint()->getPort()->getInterface()->getOwner()->getName();
     }
     else{
     comp_in = "free input ports";
     }
     port_in = bs->getInputEndPoint()->getPort()->getName();
     }
     Logger::log() << Logger::Debug << "Connection starts at port: " << port_in << Logger::endl;
     Logger::log() << Logger::Debug << "Connection starts at component: " << comp_in << Logger::endl;
     std::string comp_out, port_out;
     */
    if (bs->getOutputEndPoint()->getPort() != 0) {
      Logger::log() << Logger::Info << "    \'"
          << bs->getOutputEndPoint()->getPort()->getName() << "\'"
          << Logger::endl;

      if (bs->getOutputEndPoint()->getPort()->getInterface() != 0) {
        std::string comp_out =
            bs->getOutputEndPoint()->getPort()->getInterface()->getOwner()
                ->getName();
        std::string port_out = bs->getOutputEndPoint()->getPort()->getName();
        if (comp_out + "." + port_out == to) {
          return true;
        }
//                if (bs->getOutputEndPoint()->getPort() == pi_t) {
//                    return true;
//                }
      }
//            else{
//            comp_out = "free output ports";
//            }
//          port_out = bs->getOutputEndPoint()->getPort()->getName();
    }
  }
  return false;
}

bool SubsystemDeployer::isInputPort(const std::string &path) const {
  return dynamic_cast<RTT::base::InputPortInterface*>(strToPort(path)) != NULL;
}

bool SubsystemDeployer::isOutputPort(const std::string &path) const {
  return dynamic_cast<RTT::base::OutputPortInterface*>(strToPort(path)) != NULL;
}

bool SubsystemDeployer::isSubsystemBuffer(const std::string& port_name) const {
  size_t first_dot = port_name.find(".");
  if (first_dot == std::string::npos) {
    return false;
  }
  std::string comp_name = port_name.substr(0, first_dot);

  if (comp_name == "Y") {
    return true;
  }

  return false;
}

bool SubsystemDeployer::isSubsystemOutput(const std::string& port_name) const {
  size_t first_dot = port_name.find(".");
  if (first_dot == std::string::npos) {
    return false;
  }

  std::string comp_name = port_name.substr(0, first_dot);

  for (int i = 0; i < buffer_concate_components_.size(); ++i) {
    if (buffer_concate_components_[i]->getName() == comp_name) {
      return true;
    }
  }

  return false;
}

bool SubsystemDeployer::configure(int rt_prio) {
  Logger::In in("SubsystemDeployer::configure " + getSubsystemName());

  if (!setChannelsNames()) {
    Logger::log() << Logger::Error << "Could not set names for some i/o buffers"
        << Logger::endl;
    return false;
  }

  // disable Trigger On Start for all components
  std::vector<RTT::TaskContext*> all_components = getAllComponents();
  for (int i = 0; i < all_components.size(); ++i) {
    setTriggerOnStart(all_components[i], false);
  }

  const std::vector<RTT::TaskContext*> core_components = getCoreComponents();
  const std::vector<RTT::TaskContext*> non_core_components =
      getNonCoreComponents();

  Logger::log() << Logger::Info
      << "[before master_component configure] scheme_->getActivity(): "
      << (scheme_->getActivity()) << Logger::endl;

  // configure unconfigured core peers
  // master component can be configured after all peers are added to scheme
  for (int i = 0; i < core_components.size(); ++i) {
    if (core_components[i] == master_component_) {
      continue;
    }
    if (core_components[i] == diag_component_) {
      continue;
    }
    if (!core_components[i]->isConfigured()) {
      Logger::log() << Logger::Info << "configuring component '"
          << core_components[i]->getName() << "'" << Logger::endl;
      if (!core_components[i]->configure()) {
        Logger::log() << Logger::Error << "Unable to configure component "
            << core_components[i]->getName() << Logger::endl;
        return false;
      }
    }
  }

  std::list<Connection> connections_to_join = connections_;

  for (std::list<Connection>::iterator it = connections_to_join.begin();
      it != connections_to_join.end(); it++) {
    if (isSubsystemOutput(it->to)) {
      Logger::log() << Logger::Info << "Subsystem output: " << it->from << "->" << it->to
          << Logger::endl;
    }
  }

  // try connecting ports before components configuration
  for (std::list<Connection>::iterator it = connections_to_join.begin();
      it != connections_to_join.end();) {
    if (connectionExists(it->from, it->to)) {
      ++it;
      continue;
    }
    if (isSubsystemBuffer(it->from)) {
      Logger::log() << Logger::Error << "Could not connect ports \'" << it->from
          << "\' and \'" << it->to << "\'. Port \'" << it->from
          << "\' is subsystem i/o buffer." << Logger::endl;
      return false;
    }

    if (isSubsystemBuffer(it->to)) {
      Logger::log() << Logger::Error << "Could not connect ports \'" << it->from
          << "\' and \'" << it->to << "\'. Port \'" << it->to
          << "\' is subsystem i/o buffer." << Logger::endl;
      return false;
    }

    if (!isInputPort(it->to) || !isOutputPort(it->from)) {
      ++it;
      continue;
    }

    if (connectPorts(it->from, it->to, it->cp)) {
      connections_to_join.erase(it++);
    } else {
      ++it;
    }
  }

  // load orocos services
  for (int i = 0; i < non_core_components.size(); ++i) {
    RTT::TaskContext *tc = non_core_components[i];
    std::map<std::string, std::vector<std::string> >::iterator it =
        component_services_.find(tc->getName());
    if (it != component_services_.end()) {
      for (int j = 0; j < it->second.size(); ++j) {
        if (!tc->loadService(it->second[j])) {
          Logger::log() << Logger::Error << "Could not load service \'" << it->second[j]
              << "\' for component \'" << tc->getName() << "\'"
              << Logger::endl;
          return false;
        }
      }
    }
  }

  // configure other unconfigured peers
  for (int i = 0; i < non_core_components.size(); ++i) {
    if (non_core_components[i] == master_component_) {
      continue;
    }
    if (non_core_components[i] == diag_component_) {
      continue;
    }
    loadROSParam(non_core_components[i]);
    if (!non_core_components[i]->isConfigured()) {
      Logger::log() << Logger::Info << "loading ROS parameters for \'"
          << non_core_components[i]->getName() << "\'" << Logger::endl;
      Logger::log() << Logger::Info << "configuring component '"
          << non_core_components[i]->getName() << "'" << Logger::endl;
      if (!non_core_components[i]->configure()) {
        Logger::log() << Logger::Error << "Unable to configure component "
            << non_core_components[i]->getName() << Logger::endl;
        return false;
      }
    } else {
      Logger::log() << Logger::Info << "component \'" << non_core_components[i]->getName()
          << "\' is already configured" << Logger::endl;
    }
  }

  // connect remaining ports
  for (std::list<Connection>::iterator it = connections_to_join.begin();
      it != connections_to_join.end(); ++it) {
    if (connectionExists(it->from, it->to)) {
      continue;
    }
    if (!isInputPort(it->to)) {
      Logger::log() << Logger::Error << "port \'" << it->to << "\' is not an input port"
          << Logger::endl;
      return false;
    }

    if (!isOutputPort(it->from)) {
      Logger::log() << Logger::Error << "port \'" << it->from
          << "\' is not an output port" << Logger::endl;
      return false;
    }

    if (!connectPorts(it->from, it->to, it->cp)) {
      Logger::log() << Logger::Error << "Unable to connect \'" << it->from << "\' and \'"
          << it->to << "\'" << Logger::endl;
      return false;
    }
  }

  RTT::OperationCaller < bool(std::shared_ptr<RTT::base::InputPortInterface>)
      > stream_addInputPort;

  std::vector < std::pair<std::string, std::string> > stream_connections;
  std::vector < std::pair<std::string, std::string> > streams;

  int stream_count = 0;

  // connect ROS topics
  for (std::list<std::pair<std::string, std::string> >::iterator it =
                    ros_streams_.begin(); it != ros_streams_.end(); ++it) {
        RTT::base::PortInterface *pi = strToPort(it->first);
        if (NULL == pi) {
          Logger::log() << Logger::Error << "Streamed port \'" << it->first
              << "\' does not exist." << Logger::endl;
          return false;
        }

        // Create streaming component if it does not exist
        if (!stream_component_) {
            const std::string comp_name = "streaming_component";
            const std::string comp_type = "controller_common::BypassComponent";
            if (!dc_->loadComponent(comp_name, comp_type)) {
            Logger::log() << Logger::Error << "Unable to load component '" << comp_name
                << "' of type '" << comp_type << "'" << Logger::endl;
            return false;
            }

            stream_component_ = dc_->getPeer(comp_name);
            if (NULL == stream_component_) {
            Logger::log() << Logger::Error << "Unable to get component '" << comp_name
                << "' of type '" << comp_type << "'" << Logger::endl;
            return false;
            }
            Logger::log() << Logger::Info << "Created streaming component '" << comp_name
              << "' of type '" << comp_type << "'" << Logger::endl;
            stream_addInputPort = stream_component_->getOperation("addInputPort");
            if (!stream_addInputPort.ready()) {
            Logger::log() << Logger::Error
                << "Could not get 'addInputPort' operation for streaming component '"
                << comp_name << "'" << Logger::endl;
            return false;
            }
        }

        if (NULL == dynamic_cast<RTT::base::OutputPortInterface*>(pi)) {
            // Create input stream
            Logger::log() << Logger::Info << "Creating input ROS stream \'" << it->first
                          << "\'" << Logger::endl;

            std::shared_ptr < RTT::base::InputPortInterface > p_port = std::shared_ptr
                < RTT::base::InputPortInterface
                > (dynamic_cast<RTT::base::InputPortInterface*>(pi->clone()));

            stringstream ss;
            ss << stream_count;
            ++stream_count;

            p_port->setName(ss.str());
            if (!stream_addInputPort(p_port)) {
              Logger::log() << Logger::Error << "Could not add port '" << p_port->getName()
                  << "' to streaming component" << Logger::endl;
              return false;
            }

            // Connect to Orocos
            stream_connections.push_back(
                std::make_pair(
                    stream_component_->getName() + "." + p_port->getName() + "_OUTPORT", it->first));
            // Connect to ROS
            streams.push_back(
                std::make_pair(
                    stream_component_->getName() + "." + p_port->getName() + "_INPORT", it->second));
        }
        else {
            // Create output stream
            Logger::log() << Logger::Info << "Creating output ROS stream \'" << it->first
                          << "\'" << Logger::endl;
            std::shared_ptr < RTT::base::InputPortInterface > p_port = std::shared_ptr
                < RTT::base::InputPortInterface
                > (dynamic_cast<RTT::base::InputPortInterface*>(pi->antiClone()));

            stringstream ss;
            ss << stream_count;
            ++stream_count;

            p_port->setName(ss.str());
            if (!stream_addInputPort(p_port)) {
              Logger::log() << Logger::Error << "Could not add port '" << p_port->getName()
                  << "' to streaming component" << Logger::endl;
              return false;
            }

            // Connect to Orocos
            stream_connections.push_back(
                std::make_pair(
                    it->first,
                    stream_component_->getName() + "." + p_port->getName()
                        + "_INPORT"));
            // Connect to ROS
            streams.push_back(
                std::make_pair(
                    stream_component_->getName() + "." + p_port->getName() + "_OUTPORT",
                    it->second));
        }
  }

  if (stream_component_) {
    if (!stream_component_->setPeriod(0.01)) {
      Logger::log() << Logger::Error << "could not change period of component \'"
          << stream_component_->getName() << Logger::endl;
      return false;
    }
    stream_component_->configure();
    for (int i = 0; i < stream_connections.size(); ++i) {
      if (!connectPorts(stream_connections[i].first,
                        stream_connections[i].second, ConnPolicy())) {
        Logger::log() << Logger::Error << "could not connect ports for streaming \'"
            << stream_connections[i].first << "', '"
            << stream_connections[i].second << "'" << Logger::endl;
        return false;
      }
    }

    for (int i = 0; i < streams.size(); ++i) {
      if (!dc_->stream(streams[i].first,
                       rtt_roscomm::topic(streams[i].second))) {
        Logger::log() << Logger::Error << "Unable to connect \'" << streams[i].first
            << "\' and \'" << streams[i].second << "\'" << Logger::endl;
        return false;
      } else {
        Logger::log() << Logger::Info << "created ROS stream: \'" << streams[i].first
            << "\' -> \'" << streams[i].second << "\'" << Logger::endl;
      }
    }

    // connect ports and create streams
    stream_component_->start();
  }

  // initialize conman scheme
  RTT::OperationCaller<bool(const std::string&)> scheme_addBlock = scheme_
      ->getOperation("addBlock");
  if (!scheme_addBlock.ready()) {
    Logger::log() << Logger::Error
        << "Could not get addBlock operation of Conman scheme" << Logger::endl;
    return false;
  }
  std::vector<RTT::TaskContext*> conman_peers = getNonCoreComponents();
  conman_peers.insert(conman_peers.end(), buffer_split_components_.begin(),
                      buffer_split_components_.end());
  conman_peers.insert(conman_peers.end(), buffer_concate_components_.begin(),
                      buffer_concate_components_.end());
  conman_peers.insert(conman_peers.end(), converter_components_.begin(),
                      converter_components_.end());
//    conman_peers.push_back(dc_->getPeer("Y"));
  std::vector<bool> conman_peers_running(conman_peers.size(), false);
  for (int i = 0; i < conman_peers.size(); ++i) {
    if (conman_peers[i]->isRunning()) {
      conman_peers[i]->stop();
      conman_peers_running[i] = true;
    }
    scheme_->addPeer(conman_peers[i]);
    if (!scheme_addBlock(conman_peers[i]->getName())) {
      Logger::log() << Logger::Warning
          << "Could not add block to Conman scheme: "
          << conman_peers[i]->getName() << Logger::endl;
      return true;
    } else {
      Logger::log() << Logger::Info << "added block to conman scheme: "
          << conman_peers[i]->getName() << Logger::endl;
    }

    // Disable passing port events to peers' master (i.e. scheme and master_component).
    // The reason is that, the callback functions are defined in context of scheme peers.
    std::map<std::string, std::string>::const_iterator it =
        components_ros_action_.find(conman_peers[i]->getName());
    if (it != components_ros_action_.end()) {
//      conman_peers[i]->engine()->setMaster(0);
    }
  }

  all_components = getAllComponents();
  // add all peers to diagnostics component
  for (int i = 0; i < all_components.size(); ++i) {
    if (all_components[i]->getName() != diag_component_->getName()) {
      diag_component_->addPeer(all_components[i]);
    }
  }

  Logger::log() << Logger::Info << "configuring component '"
      << diag_component_->getName() << "'" << Logger::endl;
  if (!diag_component_->configure()) {
    Logger::log() << Logger::Error << "Unable to configure component "
        << diag_component_->getName() << Logger::endl;
    return false;
  }

  //
  // latch connections
  //
  RTT::OperationCaller<bool(const std::string&, const std::string&, const bool)> scheme_latchConnections =
      scheme_->getOperation("latchConnections");
  if (!scheme_latchConnections.ready()) {
    Logger::log() << Logger::Error
        << "Could not get latchConnections operation of Conman scheme"
        << Logger::endl;
    return false;
  }

  for (int i = 0; i < latched_connections_.size(); ++i) {
    if (!scheme_latchConnections(latched_connections_[i].first,
                                 latched_connections_[i].second, true)) {
      Logger::log() << Logger::Error << "Could not latch connections from \'"
          << latched_connections_[i].first << "\' and \'"
          << latched_connections_[i].second << "\'" << Logger::endl;
      return false;
    }
  }

  Logger::log() << Logger::Info << "configuring component '"
      << master_component_->getName() << "'" << Logger::endl;
  master_component_->configure();

  // connect ROS actions
  for (int i = 0; i < conman_peers.size(); ++i) {
    std::map<std::string, std::string>::const_iterator it =
        components_ros_action_.find(conman_peers[i]->getName());
    if (it != components_ros_action_.end()) {
      if (!conman_peers[i]->loadService("actionlib")) {
        Logger::log() << Logger::Error
            << "Could not load service \'actionlib\' for component \'"
            << it->first << "\', action \'" << it->second << "\'"
            << Logger::endl;
        return false;
      }

      RTT::Service::shared_ptr actionlib_service = conman_peers[i]->provides(
          "actionlib");
      if (!actionlib_service) {
        Logger::log() << Logger::Error
            << "Could not get service \'actionlib\' of component \'"
            << it->first << "\', action \'" << it->second << "\'"
            << Logger::endl;
        return false;
      }
      RTT::OperationCaller<bool(const std::string&)> connect_action =
          actionlib_service->getOperation("connect");
      if (!connect_action.ready()) {
        Logger::log() << Logger::Error
            << "Could not get operation \'connect\' of action_service of component \'"
            << it->first << "\', action \'" << it->second << "\'"
            << Logger::endl;
        return false;
      }

      if (!connect_action(it->second)) {
        Logger::log() << Logger::Error << "Could not connect action \'" << it->second
            << "\' to action server" << Logger::endl;
        return false;
      }
    }
  }

  //
  // remove unused ports from msg concate/split components
  //
  for (int i = 0; i < core_components.size(); ++i) {
    RTT::TaskContext* tc = core_components[i];

    RTT::OperationInterfacePart *removeUnconnectedPortsOp;
    if (tc->provides()->hasOperation("removeUnconnectedPorts")
        && (removeUnconnectedPortsOp = tc->provides()->getOperation(
            "removeUnconnectedPorts")) != NULL) {
      RTT::OperationCaller < bool() > removeUnconnectedPorts =
          RTT::OperationCaller < bool()
              > (removeUnconnectedPortsOp, tc->engine());
      if (removeUnconnectedPorts.ready()) {
        size_t before = tc->ports()->getPorts().size();
        removeUnconnectedPorts();
        size_t after = tc->ports()->getPorts().size();
        Logger::log() << Logger::Info << "Removed unconnected ports of " << tc->getName()
            << ": reduced from " << before << " to " << after << Logger::endl;
      } else {
        Logger::log() << Logger::Warning << "Could not removeUnconnectedPorts() of "
            << tc->getName() << Logger::endl;
      }
    }
  }

  if (rt_prio > 0 && !use_sim_time_) {
    RTT::Activity* master_activity =
        dynamic_cast<RTT::Activity*>(master_component_->getActivity());
    if (!master_activity) {
      Logger::log() << Logger::Warning
          << "Could not set scheduler for Master Component to ORO_SCHED_RT. Could not get Activity."
          << Logger::endl;
    } else {
      if (!master_activity->setScheduler(ORO_SCHED_RT)) {
        Logger::log() << Logger::Warning
            << "Could not set scheduler for Master Component to ORO_SCHED_RT."
            << Logger::endl;
      }
      if (!master_activity->setPriority(rt_prio)) {
        Logger::log() << Logger::Warning
            << "Could not set priority for Master Component (scheduler: ORO_SCHED_RT)."
            << Logger::endl;
      }
      if (!master_activity->setCpuAffinity(1 << cpu_num_)) {
        Logger::log() << Logger::Warning << "Could not set CPU affinity mask."
            << Logger::endl;
      } else {
        Logger::log() << Logger::Info
            << "Set CPU affinity mask of master_activity to " << (1 << cpu_num_)
            << Logger::endl;
      }
    }
  }

  // start conman scheme first
  if (!scheme_->start()) {
    Logger::log() << Logger::Error << "Unable to start component: " << scheme_->getName()
        << Logger::endl;
    return false;
  }

  if (scheme_->getTaskState() != RTT::TaskContext::Running) {
    Logger::log() << Logger::Error << "Component is not in the running state: "
        << scheme_->getName() << Logger::endl;
    return false;
  }

  // start other core peers
  for (int i = 0; i < core_components.size(); ++i) {
    if (!core_components[i]->isRunning()) {
      if (!core_components[i]->start()) {
        Logger::log() << Logger::Error << "Unable to start component "
            << core_components[i]->getName() << Logger::endl;
        return false;
      }
    }
  }

  // start non-core components that should always run
  for (int i = 0; i < conman_peers.size(); ++i) {
    std::set<std::string>::const_iterator it = components_initially_running_
        .find(conman_peers[i]->getName());
    if (it != components_initially_running_.end() || conman_peers_running[i]) {
      conman_peers[i]->start();
    }
  }

  for (int i = 0; i < converter_components_.size(); ++i) {
    converter_components_[i]->start();
  }

  if (!master_component_->trigger()) {
    Logger::log() << Logger::Error << "Unable to trigger component "
        << master_component_->getName() << Logger::endl;
    return false;
  }

  /*
  std::vector<RTT::TaskContext*> all_comp = getAllComponents();
  for (int i = 0; i < all_comp.size(); ++i) {
    RTT::Activity* act =
        dynamic_cast<RTT::Activity*>(all_comp[i]->getActivity());
    if (act) {
      Logger::log() << Logger::Info << "component \'" << all_comp[i]->getName()
          << "\', activity: \'" << act->getName() << "\', sched: "
          << act->getScheduler() << ", prio: " << act->getPriority()
          << ", CPU affinity: " << act->getCpuAffinity() << Logger::endl;
    } else {
      Logger::log() << Logger::Info << "component \'" << all_comp[i]->getName()
          << "\', activity: NULL" << Logger::endl;
    }
  }
  */
  //Logger::log() << Logger::Info << "OK" << Logger::endl;

  is_initialized_ = true;

  return true;
}

bool SubsystemDeployer::runXmls(const std::vector<std::string>& xmlFiles) {
//    using namespace tinyxml2;
  for (std::vector<std::string>::const_iterator iter = xmlFiles.begin();
      iter != xmlFiles.end(); ++iter) {
    Logger::In in(
        std::string("SubsystemDeployer::runXmls " + getSubsystemName())
            + (*iter));
    TiXmlDocument doc;
    doc.LoadFile(iter->c_str());
    TiXmlElement *root = doc.RootElement();
    if (!root) {
      Logger::log() << Logger::Error << "no root element" << Logger::endl;
      return false;
    }
    if (strcmp(root->Value(), "subsystem_configuration") != 0) {
      Logger::log() << Logger::Error << "wrong root element: \'"
          << std::string(root->Value())
          << "\', should be: \'subsystem_configuration\'" << Logger::endl;
      return false;
    }

    //
    // <import>
    //
    const TiXmlElement *import_elem = root->FirstChildElement("import");
    while (import_elem) {
      const char* import_package_elem = import_elem->Attribute("package");
      if (!import_package_elem) {
        Logger::log() << Logger::Error << "wrong value of \'import\' element"
            << Logger::endl;
        return false;
      }
      if (!import(import_package_elem)) {
        Logger::log() << Logger::Error << "could not import \'"
            << std::string(import_package_elem) << "\'" << Logger::endl;
        return false;
      }
      import_elem = import_elem->NextSiblingElement("import");
    }

    //
    // <component>
    //
    const TiXmlElement *component_elem = root->FirstChildElement("component");
    while (component_elem) {
      const char *type_attr = component_elem->Attribute("type");
      const char *name_attr = component_elem->Attribute("name");
      const char *latex_attr = component_elem->Attribute("latex");
      const char *running_attr = component_elem->Attribute("running");
      const char *ros_action_attr = component_elem->Attribute("ros_action");

      if (!name_attr) {
        Logger::log() << Logger::Error << "Attribute \'name\' of <component> is not set"
            << Logger::endl;
        return false;
      }
      // the component can be added earlier, by e.g. gazebo
      if (!dc_->hasPeer(name_attr)) {
        if (!type_attr) {
          Logger::log() << Logger::Error << "Attribute \'type\' of <component> \'"
              << std::string(name_attr) << "\' is not set" << Logger::endl;
          return false;
        }
        if (!dc_->loadComponent(name_attr, type_attr)) {
          Logger::log() << Logger::Error << "Unable to load component \'"
              << std::string(name_attr) << "\' of type \'"
              << std::string(type_attr) << "\'" << Logger::endl;
          return false;
        }
      }

      if (running_attr
          && (strcmp(running_attr, "true") == 0
              || strcmp(running_attr, "True") == 0
              || strcmp(running_attr, "TRUE") == 0)) {
        components_initially_running_.insert(std::string(name_attr));
      }

      if (ros_action_attr) {
        components_ros_action_.insert(
            std::make_pair<std::string, std::string>(
                std::string(name_attr), std::string(ros_action_attr)));
      }

      const TiXmlElement *service_elem = root->FirstChildElement("service");
      while (service_elem) {
        const char* service_name_elem = service_elem->Attribute("package");

        if (!service_name_elem) {
          Logger::log() << Logger::Error << "wrong service definition for component\'"
              << std::string(name_attr) << "\'" << Logger::endl;
          return false;
        }

        std::map<std::string, std::vector<std::string> >::iterator it =
            component_services_.find(name_attr);
        if (it == component_services_.end()) {
          it = component_services_.insert(
              std::make_pair<std::string, std::vector<std::string> >(
                  std::string(name_attr), std::vector<std::string>())).first;
        }
        it->second.push_back(std::string(service_name_elem));
        service_elem = service_elem->NextSiblingElement("service");
      }

      if (latex_attr) {
        component_names_latex_.insert(
            std::make_pair<std::string, std::string>(std::string(name_attr),
                                                     std::string(latex_attr)));
      }

      component_elem = component_elem->NextSiblingElement("component");
    }

    //
    // <io_buffer>
    //
    const TiXmlElement *io_buffer_elem = root->FirstChildElement("io_buffer");
    while (io_buffer_elem) {
      const char *alias_attr = io_buffer_elem->Attribute("alias");
      const char *name_attr = io_buffer_elem->Attribute("name");

      if (alias_attr && name_attr) {
        io_buffers_.insert(
            std::make_pair<std::string, std::string>(std::string(alias_attr),
                                                     std::string(name_attr)));
      } else {
        Logger::log() << Logger::Error
            << "wrong <io_buffer> definition: missing \'alias\' or \'name\' attribute"
            << Logger::endl;
        return false;
      }

      io_buffer_elem = io_buffer_elem->NextSiblingElement("io_buffer");
    }

    //
    // <connection>
    //
    const TiXmlElement *connection_elem = root->FirstChildElement("connection");
    while (connection_elem) {
      const char *from_attr = connection_elem->Attribute("from");
      const char *to_attr = connection_elem->Attribute("to");
      const char *name_attr = connection_elem->Attribute("name");
      const char *latex_attr = connection_elem->Attribute("latex");

      ConnPolicy cp = ConnPolicy();
      cp.lock_policy = ConnPolicy::UNSYNC;
      const TiXmlElement *conn_policy_elem = connection_elem->FirstChildElement(
          "conn_policy");
      if (conn_policy_elem) {
        const char *conn_policy_type_attr = conn_policy_elem->Attribute("type");
        const char *conn_policy_size_attr = conn_policy_elem->Attribute("size");
        if (!conn_policy_type_attr) {
          Logger::log() << Logger::Error
              << "'type' attribute must be defined for connection policy."
              << Logger::endl;
          return false;
        }

        int buffer_size = 0;
        if (conn_policy_size_attr) {
          if (sscanf(conn_policy_size_attr, "%d", &buffer_size) != 1) {
            Logger::log() << Logger::Error
                << "could not parse 'size' attribute connection policy."
                << Logger::endl;
            return false;
          }
          if (buffer_size <= 0) {
            Logger::log() << Logger::Error
                << "wrong 'size' attribute in connection policy: "
                << buffer_size << Logger::endl;
            return false;
          }
        }

        if (strcmp(conn_policy_type_attr, "data") == 0) {
          if (conn_policy_size_attr) {
            Logger::log() << Logger::Error
                << "'size' attribute should not be defined for 'data' connection policy."
                << Logger::endl;
            return false;
          }
          cp.type = ConnPolicy::DATA;
          cp.size = buffer_size;
          // do nothing, this is the default connection policy
        } else if (strcmp(conn_policy_type_attr, "buffer") == 0) {
          if (!conn_policy_size_attr) {
            Logger::log() << Logger::Error
                << "'size' attribute must be defined for 'buffer' connection policy."
                << Logger::endl;
            return false;
          }
          cp.lock_policy = ConnPolicy::LOCK_FREE;
          cp.type = ConnPolicy::BUFFER;
          cp.size = buffer_size;
        } else if (strcmp(conn_policy_type_attr, "circular_buffer") == 0) {
          if (!conn_policy_size_attr) {
            Logger::log() << Logger::Error
                << "'size' attribute must be defined for 'circular_buffer' connection policy."
                << Logger::endl;
            return false;
          }
          cp.lock_policy = ConnPolicy::LOCK_FREE;
          cp.type = ConnPolicy::CIRCULAR_BUFFER;
          cp.size = buffer_size;
          Logger::log() << Logger::Info << "creating circular buffer of size "
              << buffer_size << "" << Logger::endl;
        } else {
          Logger::log() << Logger::Error << "'type' attribute has wrong value: "
              << std::string(conn_policy_type_attr) << Logger::endl;
          return false;
        }
      }

      if (from_attr && to_attr) {
        std::string from = from_attr;
        std::string to = to_attr;
        std::string name;
        std::string latex;
        if (name_attr) {
          name = name_attr;
          Logger::log() << Logger::Info << "connection " << from << "->" << to
              << "  has name: " << name << Logger::endl;
        } else {
          Logger::log() << Logger::Warning << "connection " << from << "->" << to
              << "  has no name" << Logger::endl;
        }
        if (latex_attr) {
          latex = latex_attr;
          //Logger::log() << Logger::Info << "connection " << from << "->" << to << "  has name: " << name << Logger::endl;
        }
        connections_.push_back(Connection(from, to, name, latex, cp));
      } else {
        Logger::log() << Logger::Error
            << "wrong connection definition: missing \'from\' or \'to\' attribute"
            << Logger::endl;
        return false;
      }

      connection_elem = connection_elem->NextSiblingElement("connection");
    }

    //
    // <ros_stream>
    //
    const TiXmlElement *ros_stream_elem = root->FirstChildElement("ros_stream");
    while (ros_stream_elem) {
      const char *port_attr = ros_stream_elem->Attribute("port");
      const char *topic_attr = ros_stream_elem->Attribute("topic");

      if (port_attr && topic_attr) {
        ros_streams_.push_back(
            std::make_pair<std::string, std::string>(std::string(port_attr),
                                                     std::string(topic_attr)));
      } else {
        Logger::log() << Logger::Error
            << "wrong ros_stream definition: missing \'port\' or \'topic\' attribute"
            << Logger::endl;
        return false;
      }
      ros_stream_elem = ros_stream_elem->NextSiblingElement("ros_stream");
    }

    //
    // <latched_connections>
    //
    const TiXmlElement *latched_connections_elem = root->FirstChildElement(
        "latched_connections");
    if (latched_connections_elem) {
      const TiXmlElement *components_elem = latched_connections_elem
          ->FirstChildElement("components");
      while (components_elem) {
        const char *from_attr = components_elem->Attribute("from");
        const char *to_attr = components_elem->Attribute("to");
        if (from_attr && to_attr) {
          latched_connections_.push_back(
              std::pair<std::string, std::string>(from_attr, to_attr));
        } else {
          Logger::log() << Logger::Error
              << "wrong latched_connections definition: missing \'from\' or \'to\' attribute"
              << Logger::endl;
          return false;
        }
        components_elem = components_elem->NextSiblingElement("components");
      }
    }
  }
  return true;
}

bool SubsystemDeployer::runScripts(
    const std::vector<std::string>& scriptFiles) {
  /******************** WARNING ***********************
   *   NO log(...) statements before __os_init() !!!!!
   ***************************************************/
//  bool deploymentOnlyChecked = false;

  /* Only start the scripts after the Orb was created. Processing of
   scripts stops after the first failed script, and -1 is returned.
   Whether a script failed or all scripts succeeded, in non-daemon
   and non-checking mode the TaskBrowser will be run to allow
   inspection if the input is a tty.
   */
  bool result = true;
  for (std::vector<std::string>::const_iterator iter = scriptFiles.begin();
      iter != scriptFiles.end() && result; ++iter) {
    if (!(*iter).empty()) {
      if ((*iter).rfind(".xml", std::string::npos) == (*iter).length() - 4
          || (*iter).rfind(".cpf", std::string::npos) == (*iter).length() - 4) {
        /*                        if ( deploymentOnlyChecked ) {
         if (!dc_->loadComponents( (*iter) )) {
         result = false;
         Logger::log() << Logger::Error << "Failed to load file: '"<< (*iter) <<"'." << Logger::endl;
         } else if (!dc_.configureComponents()) {
         result = false;
         Logger::log() << Logger::Error << "Failed to configure file: '"<< (*iter) <<"'." << Logger::endl;
         }
         // else leave result=true and continue
         } else {*/
        result = dc_->kickStart((*iter));
//                        }
        continue;
      }

      if ((*iter).rfind(".ops", std::string::npos) == (*iter).length() - 4
          || (*iter).rfind(".osd", std::string::npos) == (*iter).length() - 4
          || (*iter).rfind(".lua", std::string::npos) == (*iter).length() - 4) {
        result = dc_->runScript((*iter)) && result;
        continue;
      }
      Logger::log() << Logger::Error << "Unknown extension of file: '" << (*iter)
          << "'. Must be xml, cpf for XML files or, ops, osd or lua for script files."
          << Logger::endl;
    }
  }
  return result;
}

bool SubsystemDeployer::runTaskBrowser() {

//            if ( !deploymentOnlyChecked ) {
  if (isatty (fileno(stdin))) {OCL::TaskBrowser tb( dc_.get() );
  tb.loop();
} else {
  dc_->waitForInterrupt();
}

dc_->shutdownDeployment();
//            }
}

void SubsystemDeployer::waitForInterrupt() {
    dc_->waitForInterrupt();
    dc_->shutdownDeployment();
}

boost::shared_ptr<OCL::DeploymentComponent>& SubsystemDeployer::getDc() {
  return dc_;
}


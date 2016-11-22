/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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

#ifndef MASTER_COMPONENT_H__
#define MASTER_COMPONENT_H__

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include <math.h>

#include <vector>
#include <string>

#include "common_behavior/abstract_behavior.h"
#include "common_behavior/abstract_state.h"

using namespace RTT;

template <
    template <template <typename Type> class RTTport> class InterfaceLoStatus,
    class ContainerHiCommand >
class MasterComponent: public RTT::TaskContext {
public:
    explicit MasterComponent(const std::string &name);

    bool configureHook();

    bool startHook();

    void stopHook();

    void updateHook();

    std::string getDiag();

private:
    typedef InterfaceLoStatus<RTT::InputPort > InterfaceLoStatusInput;
    typedef typename InterfaceLoStatusInput::Container ContainerLoStatus;

    // parameters
    std::vector<std::string > state_names_;
    std::string initial_state_name_;

    std::vector<std::string > behavior_names_;

    typename InterfaceLoStatusInput::Container status_in_;
    InterfaceLoStatusInput status_ports_in_;

    ContainerHiCommand cmd_in_;
    RTT::InputPort<ContainerHiCommand > port_command_in_;

    RTT::OutputPort<uint32_t> port_status_test_out_;

    std::vector<std::shared_ptr<BehaviorBase<ContainerLoStatus, ContainerHiCommand> > > behaviors_;

    std::vector<std::shared_ptr<StateBase<ContainerLoStatus, ContainerHiCommand> > > states_;
    std::shared_ptr<StateBase<ContainerLoStatus, ContainerHiCommand> > current_state_;

    // pointer to conman scheme TaskContext
    TaskContext *scheme_;

    // conman scheme operations
    RTT::OperationCaller<bool(const std::string &)> hasBlock_;
    RTT::OperationCaller<bool(int, const std::vector<std::string>&, const std::vector<std::string>&)> addGraphConfiguration_;
    RTT::OperationCaller<bool(int)> switchToConfiguration_;

    std::vector<TaskContext* > scheme_peers_;
    std::vector<std::vector<bool> > is_running_;

    bool state_switch_;
    uint32_t packet_counter_;

    int diag_current_state_id_;
    bool diag_cmd_in_received_;
};

template <
    template <template <typename Type> class RTTport> class InterfaceLoStatus,
    class ContainerHiCommand >
MasterComponent<InterfaceLoStatus, ContainerHiCommand >::MasterComponent(const std::string &name) :
    TaskContext(name, PreOperational),
    status_ports_in_(*this),
    diag_current_state_id_(0),
    diag_cmd_in_received_(false)
{
    this->ports()->addPort("command_INPORT", port_command_in_);
    this->ports()->addPort("status_test_OUTPORT", port_status_test_out_);

    this->addOperation("getDiag", &MasterComponent::getDiag, this, RTT::ClientThread);

    addProperty("state_names", state_names_);
    addProperty("initial_state_name", initial_state_name_);
}

template <
    template <template <typename Type> class RTTport> class InterfaceLoStatus,
    class ContainerHiCommand >
std::string MasterComponent<InterfaceLoStatus, ContainerHiCommand >::getDiag() {
// this method may not be RT-safe
    int state_id = diag_current_state_id_;
    if (state_id < 0 || state_id >= states_.size()) {
        return "";
    }
    
    return "state: " + states_[state_id]->getStateName() + ", behavior: " + states_[state_id]->getBehaviorName() + ", " + (diag_cmd_in_received_?"<receiving commands>":"<no commands>");
}

template <
    template <template <typename Type> class RTTport> class InterfaceLoStatus,
    class ContainerHiCommand >
bool MasterComponent<InterfaceLoStatus, ContainerHiCommand >::configureHook() {
    Logger::In in("MasterComponent::configureHook");

    for (auto it = StateFactory<ContainerLoStatus, ContainerHiCommand >::Instance()->getStates().begin();
        it != StateFactory<ContainerLoStatus, ContainerHiCommand >::Instance()->getStates().end(); ++it)
    {
        Logger::log() << Logger::Info << "state: " << it->first << Logger::endl;
    }

    // retrieve states list
    for (int i = 0; i < state_names_.size(); ++i) {
        auto b_ptr = StateFactory<ContainerLoStatus, ContainerHiCommand >::Instance()->Create( state_names_[i] );
        if (b_ptr) {
            states_.push_back(b_ptr);
        }
        else {
            Logger::log() << Logger::Error << "unknown state: " << state_names_[i] << Logger::endl;
            return false;
        }
    }

    // retrieve behavior names
    for (int i = 0; i < states_.size(); ++i) {
        const std::string& behavior_name = states_[i]->getBehaviorName();
        bool add = true;
        for (int j = 0; j < behavior_names_.size(); ++j) {
            if (behavior_names_[j] == behavior_name) {
                add = false;
                break;
            }
        }
        if (add) {
            behavior_names_.push_back(behavior_name);
        }
    }

    // retrieve behaviors list
    for (int i = 0; i < behavior_names_.size(); ++i) {
        auto b_ptr = BehaviorFactory<ContainerLoStatus, ContainerHiCommand >::Instance()->Create( behavior_names_[i] );
        if (b_ptr) {
            behaviors_.push_back(b_ptr);
        }
        else {
            Logger::log() << Logger::Error << "unknown behavior: " << behavior_names_[i] << Logger::endl;
            return false;
        }
    }

    // select initial state
    for (int i = 0; i < states_.size(); ++i) {
        if (states_[i]->getStateName() == initial_state_name_) {
            current_state_ = states_[i];
            diag_current_state_id_ = i;
        }
    }

    if (!current_state_) {
        Logger::log() << Logger::Error << "unknown initial state: " << initial_state_name_ << Logger::endl;
        return false;
    }

    // get names of all components that are needed for all behaviors
    std::set<std::string > switchable_components;

    for (int i = 0; i < behaviors_.size(); ++i) {
        const std::vector<std::string >& comp_vec = behaviors_[i]->getRunningComponents();
        for (int j = 0; j < comp_vec.size(); ++j) {
            switchable_components.insert( comp_vec[j] );
        }
    }






    TaskContext::PeerList l = this->getPeerList();
    if (l.size() != 1) {
        Logger::log() << Logger::Error << "wrong number of peers: " << l.size() << ", should be 1" << Logger::endl;
        return false;
    }

    TaskContext::PeerList::const_iterator it = l.begin();
    scheme_ = this->getPeer( (*it) );

    RTT::OperationInterfacePart *hasBlockOp = scheme_->getOperation("hasBlock");
    if (hasBlockOp == NULL) {
        Logger::log() << Logger::Error << "the peer " << (*it) << " has no matching operation hasBlock" << Logger::endl;
        return false;
    }

    hasBlock_ =  RTT::OperationCaller<bool(const std::string &)>(
        hasBlockOp, scheme_->engine());


    for (std::set<std::string >::const_iterator it = switchable_components.begin(); it != switchable_components.end(); ++it) {
        if (!hasBlock_( *it )) {
            Logger::log() << Logger::Error << "could not find a component \'" << (*it) << "\' in the scheme blocks list" << Logger::endl;
            return false;
        }
    }

    RTT::OperationInterfacePart *addGraphConfigurationOp = scheme_->getOperation("addGraphConfiguration");
    if (addGraphConfigurationOp == NULL) {
        Logger::log() << Logger::Error << "the peer " << (*it) << " has no matching operation addGraphConfiguration" << Logger::endl;
        return false;
    }

    addGraphConfiguration_ = RTT::OperationCaller<bool(int, const std::vector<std::string>&, const std::vector<std::string>&)>(
        addGraphConfigurationOp, scheme_->engine());

    RTT::OperationInterfacePart *switchToConfigurationOp = scheme_->getOperation("switchToConfiguration");
    if (switchToConfigurationOp == NULL) {
        Logger::log() << Logger::Error << "the peer " << (*it) << " has no matching operation switchToConfiguration" << Logger::endl;
        return false;
    }

    switchToConfiguration_ = RTT::OperationCaller<bool(int)>(
        switchToConfigurationOp, scheme_->engine());

    // add graph configuration for each behavior
    for (int i = 0; i < behaviors_.size(); ++i) {
        std::vector<std::string > vec_stopped;
        const std::vector<std::string >& vec_running = behaviors_[i]->getRunningComponents();
        for (std::set<std::string >::const_iterator ic = switchable_components.begin(); ic != switchable_components.end(); ++ic) {
            bool is_running = false;
            for (int ir = 0; ir < vec_running.size(); ++ir) {
                if ( (*ic) == vec_running[ir] ) {
                    is_running = true;
                    break;
                }
            }
            if (!is_running) {
                vec_stopped.push_back( *ic );
            }
        }
        addGraphConfiguration_(i, vec_stopped, vec_running);
    }

    // retrieve the vector of peers of conman scheme
    TaskContext::PeerList scheme_peers_names = scheme_->getPeerList();
    for (int pi = 0; pi < scheme_peers_names.size(); ++pi) {
        scheme_peers_.push_back( scheme_->getPeer(scheme_peers_names[pi]) );
    }

    return true;
}

template <
    template <template <typename Type> class RTTport> class InterfaceLoStatus,
    class ContainerHiCommand >
bool MasterComponent<InterfaceLoStatus, ContainerHiCommand >::startHook() {
    state_switch_ = true;
    packet_counter_ = 1;
    return true;
}

template <
    template <template <typename Type> class RTTport> class InterfaceLoStatus,
    class ContainerHiCommand >
void MasterComponent<InterfaceLoStatus, ContainerHiCommand >::stopHook() {
}

template <
    template <template <typename Type> class RTTport> class InterfaceLoStatus,
    class ContainerHiCommand >
void MasterComponent<InterfaceLoStatus, ContainerHiCommand >::updateHook() {
    Logger::In in("MasterComponent::updateHook");

    int id_faulty_module;
    int id_faulty_submodule;

    //
    // read HW status (from previous iteration)
    //
    status_ports_in_.readPorts();
    status_ports_in_.convertToROS(status_in_);

    //
    // read commands
    //
    bool cmd_in_received = (port_command_in_.read(cmd_in_) == NewData);
    if (!cmd_in_received) {
        // set msg to default value (all fields are 0, false, etc.)
        cmd_in_ = ContainerHiCommand();
    }
    diag_cmd_in_received_ = cmd_in_received;

    // get current behavior
    std::shared_ptr<BehaviorBase<ContainerLoStatus, ContainerHiCommand> > current_behavior;
    for (int i = 0; i < behaviors_.size(); ++i) {
        if (current_state_->getBehaviorName() == behaviors_[i]->getName()) {
            current_behavior = behaviors_[i];
            break;
        }
    }

    //
    // check error condition
    //
    bool pred_err = false;
    pred_err = current_behavior->checkErrorCondition(status_in_, cmd_in_, scheme_peers_);

    if (pred_err) {
        int next_state_index = -1;
        for (int i = 0; i < states_.size(); ++i) {
            if ( states_[i]->checkInitialCondition(status_in_, cmd_in_, scheme_peers_, current_state_->getStateName(), true) ) {
                if (next_state_index == -1) {
                    next_state_index = i;
                }
                else {
                    Logger::In in("MasterComponent::updateHook");
                    Logger::log() << Logger::Error << "two or more states have the same initial condition (err): current_state="
                        << current_state_->getStateName() << Logger::endl;
                    error();
                }
            }
        }
        if (next_state_index == -1) {
            Logger::In in("MasterComponent::updateHook");
            Logger::log() << Logger::Error << "cannot switch to new state (initial condition, err): current_state="
                << current_state_->getStateName() << Logger::endl;
        }
        else {
            Logger::log() << Logger::Error << "state_switch from "
               << current_state_->getStateName() << Logger::endl;

            current_state_ = states_[next_state_index];
            diag_current_state_id_ = next_state_index;

            Logger::log() << Logger::Error << "state_switch to "
               << current_state_->getStateName() << Logger::endl;

            state_switch_ = true;
        }
    }
    else {
        //
        // check stop condition
        //
        bool pred_stop = false;
        pred_stop = current_behavior->checkStopCondition(status_in_, cmd_in_, scheme_peers_);

        if (pred_stop) {
            int next_state_index = -1;
            for (int i = 0; i < states_.size(); ++i) {
                if ( states_[i]->checkInitialCondition(status_in_, cmd_in_, scheme_peers_, current_state_->getStateName(), false) ) {
                    if (next_state_index == -1) {
                        next_state_index = i;
                    }
                    else {
                        Logger::In in("MasterComponent::updateHook");
                        Logger::log() << Logger::Error << "two or more states have the same initial condition (stop): current_state="
                            << current_state_->getStateName() << Logger::endl;
                        error();
                    }
                }
            }
            if (next_state_index == -1) {
                Logger::In in("MasterComponent::updateHook");
                Logger::log() << Logger::Error << "cannot switch to new state (initial condition, stop): current_state="
                    << current_state_->getStateName() << Logger::endl;
            }
            else {
                Logger::log() << Logger::Error << "state_switch from "
                   << current_state_->getStateName() << Logger::endl;

                current_state_ = states_[next_state_index];
                diag_current_state_id_ = next_state_index;

                Logger::log() << Logger::Error << "state_switch to "
                   << current_state_->getStateName() << Logger::endl;

                state_switch_ = true;
            }
        }
    }

    //
    // if the state has changed, reorganize the graph
    //
    if (state_switch_) {
        const std::string& behavior_name = current_state_->getBehaviorName();

        for (int i = 0; i < behaviors_.size(); ++i) {
            if (behaviors_[i]->getName() == behavior_name) {
                switchToConfiguration_(i);
                break;
            }
        }
        state_switch_ = false;
    }

    //
    // write test field
    //
    ++packet_counter_;
    port_status_test_out_.write(packet_counter_);
}

#endif  // MASTER_COMPONENT_H__


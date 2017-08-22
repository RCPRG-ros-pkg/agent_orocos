/*
 Copyright (c) 2016, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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


#ifndef SUBSYSTEM_COMMON_MASTER_SERVICE_H_
#define SUBSYSTEM_COMMON_MASTER_SERVICE_H_

#include "subsystem_common/input_data.h"
#include "subsystem_common/buffer_info.h"
#include "subsystem_common/abstract_predicate_list.h"

#include <rtt/RTT.hpp>
#include <rtt/Service.hpp>
#include <rtt/Logger.hpp>
#include <rtt/plugin/PluginLoader.hpp>

namespace subsystem_common {

class MasterService: public RTT::Service {
 public:
  explicit MasterService(RTT::TaskContext* owner) :
      RTT::Service("master", owner)
  {
    this->addOperation("configureBuffers", &MasterService::configureBuffers, this, RTT::ClientThread);
    this->addOperation("cleanupBuffers", &MasterService::cleanupBuffers, this, RTT::ClientThread);

    this->addOperation("getBuffers", &MasterService::getBuffers, this, RTT::ClientThread);
    this->addOperation("writePorts", &MasterService::writePorts, this, RTT::ClientThread);
    this->addOperation("allocateBuffersData", &MasterService::allocateBuffersData, this, RTT::ClientThread);

    this->addOperation("getLowerInputBuffers", &MasterService::getLowerInputBuffers, this, RTT::ClientThread);
    this->addOperation("getUpperInputBuffers", &MasterService::getUpperInputBuffers, this, RTT::ClientThread);
    this->addOperation("getLowerOutputBuffers", &MasterService::getLowerOutputBuffers, this, RTT::ClientThread);
    this->addOperation("getUpperOutputBuffers", &MasterService::getUpperOutputBuffers, this, RTT::ClientThread);

    this->addOperation("getBehaviors", &MasterService::getBehaviors, this, RTT::ClientThread);
    this->addOperation("getStateName", &MasterService::getStateName, this, RTT::ClientThread);
    this->addOperation("getStatesCount", &MasterService::getStatesCount, this, RTT::ClientThread);
    this->addOperation("getInitialState", &MasterService::getInitialState, this, RTT::ClientThread);

    this->addOperation("allocatePredicateList", &MasterService::allocatePredicateList, this, RTT::ClientThread);
    this->addOperation("calculatePredicates", &MasterService::calculatePredicates, this, RTT::ClientThread);
    this->addOperation("getPredicatesStr", &MasterService::getPredicatesStr, this, RTT::ClientThread);

    this->addOperation("iterationEnd", &MasterService::iterationEnd, this, RTT::ClientThread);

    this->addOperation("bufferGroupRead", &MasterService::bufferGroupRead, this, RTT::ClientThread);

    this->addOperation("checkErrorCondition", &MasterService::checkErrorCondition, this, RTT::ClientThread);
    this->addOperation("checkStopCondition", &MasterService::checkStopCondition, this, RTT::ClientThread);
    this->addOperation("getNextState", &MasterService::getNextState, this, RTT::ClientThread);
    this->addOperation("getStateBufferGroup", &MasterService::getStateBufferGroup, this, RTT::ClientThread);

    this->addOperation("getRunningComponentsInState", &MasterService::getRunningComponentsInState, this, RTT::ClientThread);
  }

  virtual bool configureBuffers() = 0;
  virtual void cleanupBuffers() = 0;

  // OROCOS ports operations
  virtual void getBuffers(InputDataPtr& in_data) = 0;
  virtual void writePorts(InputDataPtr& in_data) = 0;
  virtual InputDataPtr allocateBuffersData() const = 0;

  // subsystem buffers
  virtual void getLowerInputBuffers(std::vector<InputBufferInfo >&) const = 0;
  virtual void getUpperInputBuffers(std::vector<InputBufferInfo >&) const = 0;
  virtual void getLowerOutputBuffers(std::vector<OutputBufferInfo >&) const = 0;
  virtual void getUpperOutputBuffers(std::vector<OutputBufferInfo >&) const = 0;

  // FSM parameters
  virtual std::vector<std::string > getBehaviors() const = 0;
  virtual const std::string& getStateName(int) const = 0;
  virtual int getStatesCount() const = 0;
  virtual std::string getInitialState() const = 0;

  virtual PredicateListPtr allocatePredicateList() = 0;
  virtual void calculatePredicates(const InputDataConstPtr&, const std::vector<const RTT::TaskContext*>&, PredicateListPtr&) const = 0;

  // this method may not be RT-safe
  virtual std::string getPredicatesStr(const PredicateListConstPtr&) const = 0;

  virtual void iterationEnd() = 0;

  virtual bool bufferGroupRead(size_t id, double timeout) = 0;

  virtual bool checkErrorCondition(int, const PredicateListConstPtr&) const = 0;
  virtual bool checkStopCondition(int, const PredicateListConstPtr&) const = 0;
  virtual int getNextState(int, const PredicateListConstPtr&) const = 0;
  virtual const BufferGroup& getStateBufferGroup(int) const = 0;

  virtual const std::vector<std::string >& getRunningComponentsInState(int) const = 0;
};

}   // namespace subsystem_common

#endif  // SUBSYSTEM_COMMON_MASTER_SERVICE_H_


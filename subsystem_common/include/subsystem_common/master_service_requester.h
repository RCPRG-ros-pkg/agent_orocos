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

#ifndef SUBSYSTEM_COMMON_MASTER_SERVICE_REQUESTER_H_
#define SUBSYSTEM_COMMON_MASTER_SERVICE_REQUESTER_H_

#include "subsystem_common/input_data.h"
#include "subsystem_common/buffer_info.h"
#include "subsystem_common/abstract_predicate_list.h"
#include "subsystem_common/abstract_state.h"
#include "subsystem_common/abstract_behavior.h"

#include <string>

#include "rtt/RTT.hpp"

namespace subsystem_common {

class MasterServiceRequester : public RTT::ServiceRequester {
 public:
  explicit MasterServiceRequester(RTT::TaskContext *owner)
    : RTT::ServiceRequester("master", owner)
    , configureBuffers("configureBuffers")
    , cleanupBuffers("cleanupBuffers")
    , getBuffers("getBuffers")
    , writePorts("writePorts")
    , allocateBuffersData("allocateBuffersData")
    , getLowerInputBuffers("getLowerInputBuffers")
    , getUpperInputBuffers("getUpperInputBuffers")
    , getLowerOutputBuffers("getLowerOutputBuffers")
    , getUpperOutputBuffers("getUpperOutputBuffers")
    , getBehaviors("getBehaviors")
    , getStates("getStates")
    , getInitialStateIndex("getInitialStateIndex")
    , allocatePredicateList("allocatePredicateList")
    , calculatePredicates("calculatePredicates")
    , getPredicatesStr("getPredicatesStr")
    , iterationBegin("iterationBegin")
    , iterationEnd("iterationEnd")
    , bufferGroupRead("bufferGroupRead")
    , getStateBufferGroup("getStateBufferGroup")
  {
    this->addOperationCaller(configureBuffers);
    this->addOperationCaller(cleanupBuffers);

    this->addOperationCaller(getBuffers);
    this->addOperationCaller(writePorts);
    this->addOperationCaller(allocateBuffersData);

    this->addOperationCaller(getLowerInputBuffers);
    this->addOperationCaller(getUpperInputBuffers);
    this->addOperationCaller(getLowerOutputBuffers);
    this->addOperationCaller(getUpperOutputBuffers);

    this->addOperationCaller(getBehaviors);
    this->addOperationCaller(getStates);
    this->addOperationCaller(getInitialStateIndex);

    this->addOperationCaller(allocatePredicateList);
    this->addOperationCaller(calculatePredicates);
    this->addOperationCaller(getPredicatesStr);

    this->addOperationCaller(iterationBegin);
    this->addOperationCaller(iterationEnd);

    this->addOperationCaller(bufferGroupRead);

    this->addOperationCaller(getStateBufferGroup);
  }

  RTT::OperationCaller<bool()> configureBuffers;
  RTT::OperationCaller<void()> cleanupBuffers;

  // OROCOS ports operations
  RTT::OperationCaller<void(InputDataPtr&)> getBuffers;
  RTT::OperationCaller<void (InputDataPtr&)> writePorts;
  RTT::OperationCaller<InputDataPtr()> allocateBuffersData;

  // subsystem buffers
  RTT::OperationCaller<void(std::vector<InputBufferInfo >&)> getLowerInputBuffers;
  RTT::OperationCaller<void(std::vector<InputBufferInfo >&)> getUpperInputBuffers;
  RTT::OperationCaller<void(std::vector<OutputBufferInfo >&)> getLowerOutputBuffers;
  RTT::OperationCaller<void(std::vector<OutputBufferInfo >&)> getUpperOutputBuffers;

  // FSM parameters
  RTT::OperationCaller<const std::vector<const BehaviorBase* >&() > getBehaviors;
  RTT::OperationCaller<const std::vector<const StateBase* >&() > getStates;
  RTT::OperationCaller<int()> getInitialStateIndex;

  RTT::OperationCaller<PredicateListPtr() > allocatePredicateList;
  RTT::OperationCaller<void(const InputDataConstPtr&, const std::vector<const RTT::TaskContext*>&, PredicateListPtr&) > calculatePredicates;
  RTT::OperationCaller<std::string(const PredicateListConstPtr&) > getPredicatesStr;

  RTT::OperationCaller<void()> iterationBegin;
  RTT::OperationCaller<void()> iterationEnd;

  RTT::OperationCaller<bool(size_t, double, bool)> bufferGroupRead;

  RTT::OperationCaller<const BufferGroup&(int) > getStateBufferGroup;
};
}   // namespace subsystem_common

#endif  // SUBSYSTEM_COMMON_MASTER_SERVICE_REQUESTER_H_


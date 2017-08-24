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

#ifndef SUBSYSTEM_COMMON_ABSTRACT_STATE_H_
#define SUBSYSTEM_COMMON_ABSTRACT_STATE_H_

#include <subsystem_common/abstract_predicate_list.h>

namespace subsystem_common {

class StateBase {
public:
    StateBase(  const std::string& name,
                const std::vector<int >& behavior_indices,
                const std::vector<std::pair<int, std::string > >& next_states)
        : name_(name)
        , behavior_indices_(behavior_indices)
        , next_states_(next_states) {
    }

    const std::string& getName() const {
        return name_;
    }

    const std::vector<int >& getBehaviorIndices() const {
        return behavior_indices_;
    }

    const std::vector<std::pair<int, std::string > >& getNextStates() const {
        return next_states_;
    }

    virtual int calculateNextState(const subsystem_common::PredicateListConstPtr& pred_list) const = 0;

private:
    std::string name_;
    std::vector<int > behavior_indices_;
    std::vector<std::pair<int, std::string > > next_states_;
};

}   // namespace subsystem_common

#endif  // SUBSYSTEM_COMMON_ABSTRACT_STATE_H_


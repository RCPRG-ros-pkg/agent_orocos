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

#ifndef SUBSYSTEM_COMMON_ABSTRACT_PREDICATE_LIST_H__
#define SUBSYSTEM_COMMON_ABSTRACT_PREDICATE_LIST_H__

#include <boost/shared_ptr.hpp>

namespace subsystem_common {

class PredicateList {
public:
    bool IN_ERROR;
    bool CURRENT_BEHAVIOR_OK;

    virtual PredicateList& operator=(const PredicateList& arg) = 0;

    virtual int getPredicatesCount() const = 0;
    virtual bool getPredicateValue(int idx) const = 0;
    virtual const std::string& getPredicateName(int idx) const = 0;
};

typedef boost::shared_ptr<PredicateList > PredicateListPtr;
typedef boost::shared_ptr<const PredicateList > PredicateListConstPtr;

};  // namespace subsystem_common

#endif  // SUBSYSTEM_COMMON_ABSTRACT_PREDICATE_LIST_H__


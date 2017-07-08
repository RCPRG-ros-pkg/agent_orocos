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

#ifndef __COMMON_BEHAVIOR_ABSTRACT_STATE_H__
#define __COMMON_BEHAVIOR_ABSTRACT_STATE_H__

#include "common_behavior/input_data.h"

#include <map>
#include <vector>
#include <string>

#include "rtt/RTT.hpp"

namespace common_behavior {

class StateBase {
public:

    virtual const std::string& getNextState(const PredicateListConstPtr& pred_list) const = 0;

    const std::string& getStateName() const {
        return state_name_;
    }

    const std::string& getShortStateName() const {
        return short_state_name_;
    }

    const std::vector<std::string>& getBehaviorNames() const {
        return behavior_names_;
    }

protected:
    StateBase(const std::string& state_name, const std::string& short_state_name, const std::vector<std::string >& behavior_names)
        : state_name_(state_name)
        , short_state_name_(short_state_name)
        , behavior_names_(behavior_names)
    { }

private:
    std::string state_name_;
    std::string short_state_name_;
    std::vector<std::string > behavior_names_;
};

class StateFactory
{
private:
    map<string, function<StateBase*(void)> > factoryFunctionRegistry;

    StateFactory() {}

public:
    shared_ptr<StateBase > Create(string name)
    {
        StateBase* instance = nullptr;

        // find name in the registry and call factory method.
        auto it = factoryFunctionRegistry.find(name);
        if(it != factoryFunctionRegistry.end())
            instance = it->second();

        // wrap instance in a shared ptr and return
        if(instance != nullptr)
            return std::shared_ptr<StateBase >(instance);
        else
            return nullptr;
    }

    void RegisterFactoryFunction(string name,
    function<StateBase*(void)> classFactoryFunction)
    {
        // register the class factory function
        factoryFunctionRegistry[name] = classFactoryFunction;
    }

    const map<string, function<StateBase*(void)> >& getStates() const {
        return factoryFunctionRegistry;
    }

    static StateFactory* Instance()
    {
        static StateFactory factory;
        return &factory;
    }
};

template<class T>
class StateRegistrar {
public:
    StateRegistrar()
    {
        std::string name;
        {
            T sample;
            name = sample.getStateName();
        }
        // register the class factory function 
        StateFactory::Instance()->RegisterFactoryFunction(name,
                [](void) -> StateBase * { return new T();});
    }
};

#define LITERAL_registrar_state_(X) registrar_state_##X
#define EXPAND_registrar_state_(X) LITERAL_registrar_state_(X)

#define REGISTER_STATE( STATE_CLASS ) static common_behavior::StateRegistrar<STATE_CLASS > EXPAND_registrar_state_(__LINE__)

};  // namespace common_behavior

#endif  // __COMMON_BEHAVIOR_ABSTRACT_STATE_H__


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

#ifndef __COMMON_INTERFACES_INTERFACE_PORTS_H__
#define __COMMON_INTERFACES_INTERFACE_PORTS_H__

#include <cstring>
#include <vector>
#include <string>

#include <boost/shared_ptr.hpp>

#include <rtt/RTT.hpp>

#include <common_interfaces/interface_ports_fwd.h>

using namespace RTT;

namespace interface_ports {

template <typename T >
class InputPort : public InputPortInterface<T > {
public:
    explicit InputPort(RTT::TaskContext *tc, const std::string &port_name)
        : port_(new RTT::InputPort<T >(port_name + "_INPORT"))
        , tc_(tc)
    {
        tc->ports()->addPort(*port_);
    }

    virtual bool read(T &data) {
        return (port_->read(data, false) == RTT::NewData);
    }

    bool removeUnconnectedPorts() {
        if (!port_->connected()) {
            tc_->ports()->removePort( port_->getName() );
            port_.reset();
            return true;
        }
        return false;
    }

protected:

    boost::shared_ptr<RTT::InputPort<T > > port_;
    RTT::TaskContext* tc_;
};

template <typename T >
class OutputPort : public OutputPortInterface<T > {
public:
    explicit OutputPort(RTT::TaskContext *tc, const std::string &port_name)
        : port_(new RTT::OutputPort<T >(port_name + "_OUTPORT", false) )
        , tc_(tc)
    {
        tc->ports()->addPort(*port_);
    }

    virtual bool write(const T &data) {
        port_->write(data);
        return true;
    }

    bool removeUnconnectedPorts() {
        if (!port_->connected()) {
            tc_->ports()->removePort( port_->getName() );
            port_.reset();
            return true;
        }
        return false;
    }

protected:

    boost::shared_ptr<RTT::OutputPort<T > > port_;
    RTT::TaskContext* tc_;
};

template <typename T >
class InputPortInterfaceFactory
{
private:
    std::map<string, InputPortInterface<T >* (*)(RTT::TaskContext *tc, const std::string& prefix) > factoryFunctionRegistry;

    InputPortInterfaceFactory() {}

public:
    boost::shared_ptr<InputPortInterface<T > > Create(string name, RTT::TaskContext *tc, const std::string& prefix)
    {
        InputPortInterface<T > * instance = NULL;

        // find name in the registry and call factory method.
        typename std::map<string, InputPortInterface<T >* (*)(RTT::TaskContext *tc, const std::string& prefix) >::const_iterator it = factoryFunctionRegistry.find(name);
        if(it != factoryFunctionRegistry.end())
            instance = it->second(tc, prefix);

        // wrap instance in a shared ptr and return
        if(instance != NULL)
            return boost::shared_ptr<InputPortInterface<T > >(instance);
        else
            return boost::shared_ptr<InputPortInterface<T > >();
    }

    void RegisterFactoryFunction(string name, InputPortInterface<T >* (*classFactoryFunction)(RTT::TaskContext *tc, const std::string& prefix) )
    {
        // register the class factory function
        factoryFunctionRegistry[name] = classFactoryFunction;
    }

    static InputPortInterfaceFactory* Instance()
    {
        static InputPortInterfaceFactory factory;
        return &factory;
    }
};

template<class T>
InputPortInterface<typename T::Container_ >* InputPortInterfaceFactoryFunction(RTT::TaskContext *tc, const std::string& prefix) {
    return new T(tc, prefix);
}

template<class T>
class InputPortInterfaceRegistrar {
public:
    InputPortInterfaceRegistrar(const std::string& name)
    {
        Logger::log() << Logger::Debug << "registering InputPortInterface: " << name << Logger::endl;
        // register the class factory function 
        InputPortInterfaceFactory<typename T::Container_ >::Instance()->RegisterFactoryFunction(name, InputPortInterfaceFactoryFunction<T >);
    }
};

template <typename T >
class OutputPortInterfaceFactory
{
private:
    std::map<string, OutputPortInterface<T >* (*)(RTT::TaskContext *tc, const std::string& prefix) > factoryFunctionRegistry;

    OutputPortInterfaceFactory() {}

public:
    boost::shared_ptr<OutputPortInterface<T > > Create(string name, RTT::TaskContext *tc, const std::string& prefix)
    {
        OutputPortInterface<T > * instance = NULL;

        // find name in the registry and call factory method.
        typename std::map<string, OutputPortInterface<T >* (*)(RTT::TaskContext *tc, const std::string& prefix) >::const_iterator it = factoryFunctionRegistry.find(name);
        if(it != factoryFunctionRegistry.end())
            instance = it->second(tc, prefix);

        // wrap instance in a shared ptr and return
        if(instance != NULL)
            return boost::shared_ptr<OutputPortInterface<T > >(instance);
        else
            return boost::shared_ptr<OutputPortInterface<T > >();
    }

    void RegisterFactoryFunction(string name, OutputPortInterface<T >* (*classFactoryFunction)(RTT::TaskContext *tc, const std::string& prefix) )
    {
        // register the class factory function
        factoryFunctionRegistry[name] = classFactoryFunction;
    }

    static OutputPortInterfaceFactory* Instance()
    {
        static OutputPortInterfaceFactory factory;
        return &factory;
    }
};

template<class T>
OutputPortInterface<typename T::Container_ >* OutputPortInterfaceFactoryFunction(RTT::TaskContext *tc, const std::string& prefix) {
    return new T(tc, prefix);
}

template<class T>
class OutputPortInterfaceRegistrar {
public:
    OutputPortInterfaceRegistrar(const std::string& name)
    {
        Logger::log() << Logger::Debug << "registering OutputPortInterface: " << name << Logger::endl;
        // register the class factory function 
        OutputPortInterfaceFactory<typename T::Container_ >::Instance()->RegisterFactoryFunction(name, OutputPortInterfaceFactoryFunction<T >);
    }
};

};  // namespace interface_ports

#define LITERAL_input_registrar_(X) input_registrar_##X
#define EXPAND_input_registrar_(X) LITERAL_input_registrar_(X)

#define LITERAL_output_registrar_(X) output_registrar_##X
#define EXPAND_output_registrar_(X) LITERAL_output_registrar_(X)

#define REGISTER_InputPortInterface( CLASS, NAME ) static interface_ports::InputPortInterfaceRegistrar<CLASS > EXPAND_input_registrar_(__LINE__)(NAME);

#define REGISTER_OutputPortInterface( CLASS, NAME ) static interface_ports::OutputPortInterfaceRegistrar<CLASS > EXPAND_output_registrar_(__LINE__)(NAME);

#endif  // __COMMON_INTERFACES_INTERFACE_PORTS_H__


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

#ifndef SUBSYSTEM_COMMON_ABSTRACT_PORT_CONVERTER_H__
#define SUBSYSTEM_COMMON_ABSTRACT_PORT_CONVERTER_H__

#include <map>
#include <vector>
#include <string>

#include "rtt/Component.hpp"
#include "rtt/RTT.hpp"

using namespace RTT;

namespace subsystem_common {

class ConverterBase {
public:
    virtual ~ConverterBase() {}
};

template <typename Tfrom, typename Tto >
class Converter : public ConverterBase {
public:

    typedef Tfrom TypeFrom;
    typedef Tto TypeTo;

    virtual ~Converter() {}

    static bool isCompatible(RTT::base::PortInterface *from, RTT::base::PortInterface *to) {
        if (!from || !to) {
            return false;
        }

        RTT::OutputPort<Tfrom > *port_a = dynamic_cast<RTT::OutputPort<Tfrom >* >(from);
        RTT::InputPort<Tto > *port_b = dynamic_cast<RTT::InputPort<Tto >* >(to);

        if (!port_a || !port_b) {
            return false;
        }
        return true;
    }

    virtual void convert(const Tfrom&, Tto&) const = 0;
};

template <typename C >
class ConverterComponent : public RTT::TaskContext {
public:
    explicit ConverterComponent(const std::string &name)
        : RTT::TaskContext(name)
        , port_in_("data_INPORT")
        , port_out_("data_OUTPORT", false)
        , converter_(new C())
    {
        ports()->addLocalPort(port_in_);
        ports()->addLocalPort(port_out_);
        this->addOperation("isDataConverter", &ConverterComponent::isDataConverter, this, RTT::ClientThread);
    }

    bool isDataConverter() const {
        return true;
    }

    void updateHook() {
        if (port_in_.read(in_, false) == RTT::NewData) {
            converter_->convert(in_, out_);
            port_out_.write(out_);
        }
    }

private:
    RTT::InputPort<typename C::TypeFrom > port_in_;
    typename C::TypeFrom in_;

    RTT::OutputPort<typename C::TypeTo > port_out_;
    typename C::TypeTo out_;

    std::shared_ptr<C > converter_;
};

class ConverterPortsBase {
public:
    virtual ~ConverterPortsBase() {}

    virtual void convert() = 0;
};

class PortConverterFactory
{
private:
    typedef bool (*isCompatibleFunction)(RTT::base::PortInterface *, RTT::base::PortInterface *);

    std::vector<std::string > factoryFunctionRegistryNames_;
    std::vector<isCompatibleFunction > factoryFunctionRegistryIsCompatible_;
    std::vector<function<ConverterBase*(void)> > factoryFunctionRegistryCreate_;

    PortConverterFactory() {}

public:
    void RegisterFactoryFunction(isCompatibleFunction fun, function<ConverterBase*(void)> classFactoryFunction, const std::string& name) {
        factoryFunctionRegistryNames_.push_back(name);
        factoryFunctionRegistryIsCompatible_.push_back(fun);
        factoryFunctionRegistryCreate_.push_back(classFactoryFunction);
    }

    std::string getPortConverter(RTT::base::PortInterface *from, RTT::base::PortInterface *to) const {
        for (int i = 0; i < factoryFunctionRegistryIsCompatible_.size(); ++i) {
            if (factoryFunctionRegistryIsCompatible_[i](from, to)) {
                return factoryFunctionRegistryNames_[i];
            }
        }

        return std::string();
    }

    ConverterBase* createConverter(RTT::base::PortInterface *from, RTT::base::PortInterface *to) const {
        for (int i = 0; i < factoryFunctionRegistryIsCompatible_.size(); ++i) {
            if (factoryFunctionRegistryIsCompatible_[i](from, to)) {
                return factoryFunctionRegistryCreate_[i]();
            }
        }
        Logger::log() << Logger::Error << "PortConverterFactory::createConverter: ERROR: converter not found" << Logger::endl;
        return NULL;
    }

    static PortConverterFactory* Instance()
    {
        static PortConverterFactory factory;
        return &factory;
    }
};

template<class T>
class PortConverterRegistrar {
public:
    PortConverterRegistrar(const std::string &name)
    {
        Logger::log() << Logger::Info << "PortConverterRegistrar: " << name << Logger::endl;

        // register the class factory function 
        PortConverterFactory::Instance()->RegisterFactoryFunction(&T::isCompatible, [](void) -> ConverterBase * { return new T();}, name);
    }
};

template <typename TYPE_FROM, typename TYPE_TO >
class ConverterPorts : public ConverterPortsBase {
public:

    explicit ConverterPorts(const std::string &name)
        : port_from_(name + "_INPORT")
        , port_to_(name + "_OUTPORT")
    {
    }

    bool initialize() {
        RTT::OutputPort<TYPE_FROM > port_from;
        RTT::InputPort<TYPE_TO > port_to;
        converter_ = std::dynamic_pointer_cast<Converter<TYPE_FROM, TYPE_TO > >( std::shared_ptr<ConverterBase >(subsystem_common::PortConverterFactory::Instance()->createConverter(&port_from, &port_to)));
        return converter_ != NULL;
    }

    void convert() {
        if (port_from_.read(data_in_) == RTT::NewData) {
            converter_->convert(data_in_, data_out_);
            port_to_.write(data_out_);
        }
    }

    RTT::InputPort<TYPE_FROM > port_from_;
    RTT::OutputPort<TYPE_TO > port_to_;

private:
    TYPE_FROM data_in_;
    TYPE_TO data_out_;

    std::shared_ptr<Converter<TYPE_FROM, TYPE_TO > > converter_;
};

class MultiConverterComponent: public RTT::TaskContext {
public:
    explicit MultiConverterComponent(const std::string &name)
        : TaskContext(name, PreOperational)
        , initialized_(true)
    {
    }

    template <typename TYPE_FROM, typename TYPE_TO >
    bool addConverter(const std::string &name) {
        RTT::Logger::In in(std::string("MultiConverterComponent(") + getName() + ")::addConverter");

        std::shared_ptr<ConverterPorts<TYPE_FROM, TYPE_TO> > pconv(new ConverterPorts<TYPE_FROM, TYPE_TO>(name));
        if (!pconv->initialize()) {
            Logger::log() << RTT::Logger::Error << "could not load ConverterPorts: '" << name << "'" << Logger::endl;
            initialized_ = false;
            return false;
        }
        this->ports()->addPort(pconv->port_from_);
        this->ports()->addPort(pconv->port_to_);
        converters_.push_back( std::dynamic_pointer_cast<ConverterPortsBase >(pconv) );

        return true;
    }

private:

    bool configureHook() {
        return initialized_;
    }

    void updateHook() {
        for (int i = 0; i < converters_.size(); ++i) {
            converters_[i]->convert();
        }
    }

    std::vector<std::shared_ptr<ConverterPortsBase > > converters_;
    bool initialized_;
};

// component
#define LITERAL_registrar_port_converter_(X) registrar_port_converter_##X
#define EXPAND_registrar_port_converter_(X) LITERAL_registrar_port_converter_(X)

#define LITERAL_port_converter_component_name__(X) Component##X
#define EXPAND_port_converter_component_name_(X) LITERAL_port_converter_component_name__(X)

#define REGISTER_PORT_CONVERTER( CONVERTER_CLASS ) static subsystem_common::PortConverterRegistrar<CONVERTER_CLASS > EXPAND_registrar_port_converter_(__LINE__)("Component"#CONVERTER_CLASS);\
 typedef subsystem_common::ConverterComponent<CONVERTER_CLASS > EXPAND_port_converter_component_name_(CONVERTER_CLASS);\
 ORO_LIST_COMPONENT_TYPE(EXPAND_port_converter_component_name_(CONVERTER_CLASS));

};  // namespace subsystem_common

#endif  // SUBSYSTEM_COMMON_ABSTRACT_PORT_CONVERTER_H__


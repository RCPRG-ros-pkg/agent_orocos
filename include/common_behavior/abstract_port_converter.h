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

#ifndef COMMON_BEHAVIOR_ABSTRACT_PORT_CONVERTER_H__
#define COMMON_BEHAVIOR_ABSTRACT_PORT_CONVERTER_H__

#include <map>
#include <vector>
#include <string>

#include "rtt/Component.hpp"
#include "rtt/RTT.hpp"

namespace common_behavior {

template <typename Tfrom, typename Tto >
class Converter {
public:

    typedef Tfrom TypeFrom;
    typedef Tto TypeTo;

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

    void stopHook() {
        port_out_.clear();
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

class PortConverterFactory
{
private:
    typedef bool (*isCompatibleFunction)(RTT::base::PortInterface *, RTT::base::PortInterface *);

    std::vector<std::pair<isCompatibleFunction, std::string > > factoryFunctionRegistry_;

    PortConverterFactory() {}

public:
    void RegisterFactoryFunction(isCompatibleFunction fun, const std::string& name) {
        factoryFunctionRegistry_.push_back( std::pair<isCompatibleFunction, std::string >(fun, name) );
    }

    std::string getPortConverter(RTT::base::PortInterface *from, RTT::base::PortInterface *to) const {
        for (int i = 0; i < factoryFunctionRegistry_.size(); ++i) {
            if (factoryFunctionRegistry_[i].first(from, to)) {
                return factoryFunctionRegistry_[i].second;
            }
        }
        return std::string();
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
        std::cout << "PortConverterRegistrar: " << name << std::endl;

        // register the class factory function 
        PortConverterFactory::Instance()->RegisterFactoryFunction(&T::isCompatible, name);
    }
};

#define LITERAL_registrar_port_converter_(X) registrar_port_converter_##X
#define EXPAND_registrar_port_converter_(X) LITERAL_registrar_port_converter_(X)

#define LITERAL_port_converter_component_name__(X) Component##X
#define EXPAND_port_converter_component_name_(X) LITERAL_port_converter_component_name__(X)
 
#define REGISTER_PORT_CONVERTER( CONVERTER_CLASS ) static common_behavior::PortConverterRegistrar<CONVERTER_CLASS > EXPAND_registrar_port_converter_(__LINE__)("Component"#CONVERTER_CLASS);\
 typedef common_behavior::ConverterComponent<CONVERTER_CLASS > EXPAND_port_converter_component_name_(CONVERTER_CLASS);\
 ORO_LIST_COMPONENT_TYPE(EXPAND_port_converter_component_name_(CONVERTER_CLASS));

};  // namespace common_behavior

#endif  // COMMON_BEHAVIOR_ABSTRACT_PORT_CONVERTER_H__


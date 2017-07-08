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

#ifndef COMMON_INTERFACES_ABSTRACT_BUFFER_CONVERTER_H__
#define COMMON_INTERFACES_ABSTRACT_BUFFER_CONVERTER_H__

#include <stdint.h>
#include <functional>
#include <map>
#include <vector>
#include <string>
#include <boost/function.hpp>

namespace common_interfaces {

using namespace std;

template <typename Tmsg >
class BufferConverter {
public:
    typedef Tmsg TypeMsg;

    virtual int getDataSize() const = 0;
    virtual void convertToMsg(const uint8_t*, Tmsg&) const = 0;
    virtual void convertFromMsg(const Tmsg&, uint8_t*) const = 0;
};

template <typename Tmsg >
class BufferConverterFactory
{
private:
    std::map<std::string, std::function<BufferConverter<Tmsg >*(void)> > factoryFunctionRegistry;

    BufferConverterFactory() {}

public:
    shared_ptr<BufferConverter<Tmsg > > Create(string name)
    {
        BufferConverter<Tmsg >* instance = nullptr;

        // find name in the registry and call factory method.
        auto it = factoryFunctionRegistry.find(name);
        if(it != factoryFunctionRegistry.end())
            instance = it->second();

        // wrap instance in a shared ptr and return
        if(instance != nullptr)
            return std::shared_ptr<BufferConverter<Tmsg > >(instance);
        else
            return nullptr;
    }

    void RegisterFactoryFunction(string name,
        function<BufferConverter<Tmsg >*(void)> classFactoryFunction)
    {
        // register the class factory function
        factoryFunctionRegistry[name] = classFactoryFunction;
    }

//    const map<string, function<BufferConverter*(void)> >& getStates() const {
//        return factoryFunctionRegistry;
//    }

    static BufferConverterFactory<Tmsg >* Instance()
    {
        static BufferConverterFactory<Tmsg > factory;
        return &factory;
    }
};

template<class T>
class BufferConverterRegistrar {
public:
    BufferConverterRegistrar(const std::string& name)
    {
        // register the class factory function 
        BufferConverterFactory<typename T::TypeMsg >::Instance()->RegisterFactoryFunction(name,
                [](void) -> BufferConverter<typename T::TypeMsg > * { return new T();});
    }
};

#define LITERAL_registrar_buffer_converter_(X) registrar_buffer_converter_##X
#define EXPAND_registrar_buffer_converter_(X) LITERAL_registrar_buffer_converter_(X)

#define REGISTER_BUFFER_CONVERTER( BUFFER_CONVERTER_CLASS ) static common_interfaces::BufferConverterRegistrar<BUFFER_CONVERTER_CLASS > EXPAND_registrar_buffer_converter_(__LINE__)(#BUFFER_CONVERTER_CLASS)

//#define REGISTER_BUFFER_CONVERTER( BUFFER_CONVERTER_CLASS ) static common_interfaces::BufferConverterRegistrar<BUFFER_CONVERTER_CLASS > registrar_buffer_converter

};  // namespace common_interfaces

#endif  // COMMON_INTERFACES_ABSTRACT_BUFFER_CONVERTER_H__


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

#ifndef SUBSYSTEM_COMMON_BUFFER_INFO_H_
#define SUBSYSTEM_COMMON_BUFFER_INFO_H_

namespace subsystem_common {

class BufferInfo {
public:
    BufferInfo( const std::string& interface_type,
                const std::string& interface_alias)
        : interface_alias_(interface_alias)
        , interface_type_(interface_type) {
    }

    // the prefix used to generate interface classes with macro
    // ORO_LIST_INTERFACE_COMPONENTS
    std::string interface_alias_;
    std::string interface_type_;
};

class InputBufferInfo : public BufferInfo {
public:
    InputBufferInfo(    const std::string& interface_type,
                        const std::string& interface_alias)

        : BufferInfo(interface_type, interface_alias)
    {
    }
};

class OutputBufferInfo : public BufferInfo {
public:
    OutputBufferInfo(   const std::string& interface_type,
                        const std::string& interface_alias)
        : BufferInfo(interface_type, interface_alias) {
    }
};

}   // namespace subsystem_common

#endif  // SUBSYSTEM_COMMON_BUFFER_INFO_H_


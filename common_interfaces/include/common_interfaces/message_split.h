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

#ifndef MESSAGE_SPLIT_H__
#define MESSAGE_SPLIT_H__

#include <vector>
#include <string>

#include "rtt/RTT.hpp"
#include "rtt/Logger.hpp"

using namespace RTT;

template <template <template <typename Type> class RTTport> class Interface>
class MessageSplit: public RTT::TaskContext {
public:
    typedef Interface<RTT::OutputPort > InterfaceOutport;

    explicit MessageSplit(const std::string& name) :
        TaskContext(name, PreOperational),
        out_(*this),
        port_msg_in_("msg_INPORT")
    {
        this->ports()->addPort(port_msg_in_);
        this->addOperation("getDiag", &MessageSplit::getDiag, this, RTT::ClientThread);
    }

    std::string getDiag() {
    // this method may not be RT-safe
        if (diag_buf_valid_) {
            std::stringstream ss;
            ros::message_operations::Printer<Container >::stream(ss, "", diag_buf_);
            return ss.str();
        }
        return "<no data>";
    }

    bool configureHook() {
        return true;
    }

    bool startHook() {
        return true;
    }

    void updateHook() {
        if (port_msg_in_.read(msg_) == RTT::NewData) {
            out_.convertFromROS(msg_);
            out_.writePorts();
            diag_buf_ = msg_;
            diag_buf_valid_ = true;
        }
    }

private:

    InterfaceOutport out_;

    typename InterfaceOutport::Container_ msg_;
    RTT::InputPort<typename InterfaceOutport::Container_ > port_msg_in_;

    Container diag_buf_;
    bool diag_buf_valid_;
};


#endif  // MESSAGE_SPLIT_H__


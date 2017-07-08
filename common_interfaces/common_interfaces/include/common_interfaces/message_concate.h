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

#ifndef COMMON_INTERFACES_MESSAGE_CONCATE_H__
#define COMMON_INTERFACES_MESSAGE_CONCATE_H__

#include <vector>
#include <string>

#include "rtt/RTT.hpp"
#include "rtt/Logger.hpp"

using namespace RTT;

template <class Interface >
class MessageConcate: public RTT::TaskContext {
public:

    explicit MessageConcate(const std::string& name) :
        TaskContext(name),
        in_(this),
        port_msg_out_("msg_OUTPORT")
    {
        this->ports()->addPort(port_msg_out_);
        this->addOperation("getDiag", &MessageConcate::getDiag, this, RTT::ClientThread);
        this->addOperation("removeUnconnectedPorts", &MessageConcate::removeUnconnectedPorts, this, RTT::ClientThread);

        Logger::In in(std::string("MessageConcate::MessageConcate (") + name + ")");

        RTT::DataFlowInterface::Ports p = this->ports()->getPorts();
        log(RTT::Info) << "Ports:" << Logger::endl;
        for (int i = 0; i < p.size(); ++i) {
            log(RTT::Info) << "    " << p[i]->getName() << Logger::endl;
        }
    }

    std::string getDiag() {
        std::stringstream ss;
        ss << "time: " << last_ports_time_;

        if (diag_buf_valid_) {
            ss << ", <data ok>";
        }
        else {
            ss << ", <no obligatory data>";;
        }
        return ss.str();
    }

    bool startHook() {
        return true;
    }

    void updateHook() {
        RTT::os::TimeService::Seconds time1 = RTT::nsecs_to_Seconds(RTT::os::TimeService::Instance()->getNSecs());
        bool read_successful = in_.read(msg_);
        RTT::os::TimeService::Seconds time2 = RTT::nsecs_to_Seconds(RTT::os::TimeService::Instance()->getNSecs());
        last_ports_time_ = time2 - time1;
        if (read_successful) {
            port_msg_out_.write(msg_);
        }
        diag_buf_valid_ = read_successful;
    }

    bool removeUnconnectedPorts() {
        in_.removeUnconnectedPorts();
    }

private:

    Interface in_;

    typename Interface::Container_ msg_;
    RTT::OutputPort<typename Interface::Container_ > port_msg_out_;

    bool diag_buf_valid_;

    RTT::Seconds last_ports_time_;
};


#endif  // COMMON_INTERFACES_MESSAGE_CONCATE_H__


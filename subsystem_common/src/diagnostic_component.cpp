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

#include <vector>
#include <string>
#include <cstring>
#include <sstream>
#include <math.h>
#include <sys/time.h>

#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <rtt/base/PortInterface.hpp>
#include "rtt/RTT.hpp"
#include "rtt/os/TimeService.hpp"

#include <diagnostic_msgs/DiagnosticArray.h>

using namespace RTT;

class DiagnosticComponent: public RTT::TaskContext {
public:
    explicit DiagnosticComponent(const std::string &name);

    bool configureHook();

    bool startHook();

    void stopHook();

    void updateHook();

private:

    class Diag {
    public:
        std::string component_name_;
        RTT::OperationCaller<std::string()> getDiag_;
    };

    diagnostic_msgs::DiagnosticArray diag_out_;
    RTT::OutputPort<diagnostic_msgs::DiagnosticArray > port_diag_out_;

    std::vector<TaskContext* > peers_;
    std::vector<Diag > diag_vec_;

//    std::vector<double >
};

static std::string getTaskStatusChar(RTT::TaskContext* t)
{
    if (t->inFatalError())
        return "F";
    if (t->inRunTimeError())
        return "E";
    if (t->inException())
        return "X";
    if (t->isRunning() )
        return "R"; // Running
    if (t->isConfigured() )
        return "S"; // Stopped
    return "U";     // Unconfigured/Preoperational
}

DiagnosticComponent::DiagnosticComponent(const std::string &name) :
    TaskContext(name, PreOperational),
    port_diag_out_("diag_OUTPORT", true)
{
    this->ports()->addPort(port_diag_out_);
}

bool DiagnosticComponent::configureHook() {
    Logger::In in("DiagnosticComponent::configureHook");

    TaskContext::PeerList l = this->getPeerList();

    diag_out_.status.resize(2);

    diag_out_.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;
    diag_out_.status[0].name = "components";
    diag_out_.status[0].hardware_id = "0";
    diag_out_.status[0].message = "";
    diag_out_.status[0].values.resize(l.size());

    for (int i = 0; i < l.size(); ++i) {
        TaskContext *tc = this->getPeer( l[i] );
        if (tc == NULL) {
            Logger::log() << Logger::Error << "could not find peer "
                          << l[i] << Logger::endl;
            return false;
        }
        peers_.push_back(tc);
        diag_out_.status[0].values[i].key = l[i];          // component name
        diag_out_.status[0].values[i].value = "UNKNOWN";   // component state
    }

    for (int i = 0; i < l.size(); ++i) {
        TaskContext *tc = this->getPeer( l[i] );

        RTT::OperationInterfacePart *getDiagOp;
        if (tc->provides()->hasOperation("getDiag") && (getDiagOp = tc->provides()->getOperation("getDiag")) != NULL) {
            Diag diag;
            diag.component_name_ = tc->getName();
            diag.getDiag_ = RTT::OperationCaller<std::string()>(getDiagOp, tc->engine());
            diag_vec_.push_back(diag);
            Logger::log() << Logger::Info << "Added diagnostics for component "
                << tc->getName()
                << Logger::endl;
        }
        else {
            Logger::log() << Logger::Warning << "component "
                << tc->getName() << " does not provide diagnostic information"
                << Logger::endl;
        }
    }

    diag_out_.status[1].level = diagnostic_msgs::DiagnosticStatus::OK;
    diag_out_.status[1].name = "diagnostics";
    diag_out_.status[1].hardware_id = "1";
    diag_out_.status[1].message = "";
    diag_out_.status[1].values.resize(diag_vec_.size());

    return true;
}

bool DiagnosticComponent::startHook() {
    diag_out_.header.seq = 0;
    return true;
}

void DiagnosticComponent::stopHook() {
}

void DiagnosticComponent::updateHook() {
    bool changed = true;//false;

    // states of components
    for (int i = 0; i < peers_.size(); ++i) {
        std::string value = getTaskStatusChar( peers_[i] );
        if (value != diag_out_.status[0].values[i].value) {
            diag_out_.status[0].values[i].value = value;  // component state
            changed = true;
        }
    }

    for (int i = 0; i < diag_vec_.size(); ++i) {
        diag_out_.status[1].values[i].key = diag_vec_[i].component_name_;
        diag_out_.status[1].values[i].value = diag_vec_[i].getDiag_();
    }

    if (changed) {
        diag_out_.header.seq++;
        diag_out_.header.stamp = ros::Time::now();
        port_diag_out_.write(diag_out_);
    }
}

ORO_LIST_COMPONENT_TYPE(DiagnosticComponent)


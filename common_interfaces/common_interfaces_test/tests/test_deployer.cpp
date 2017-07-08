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

#include "test_deployer.h"
#include <rtt/Logger.hpp>

namespace message_tests {

using namespace RTT;

  TestDeployer::TestDeployer(const std::string& name)
      : dc_(new OCL::DeploymentComponent(name)) {

    char* comp_path = getenv("RTT_COMPONENT_PATH");
//    Logger::log() << Logger::Error << "RTT_COMPONENT_PATH=" << std::string(comp_path) << Logger::endl;
    
    RTT::Property<std::string >* prop = dynamic_cast<RTT::Property<std::string >* >(dc_->getProperty("RTT_COMPONENT_PATH"));
    prop->set(comp_path);

    dc_->configure();

    dc_->import("rtt_ros");

    RTT::Service::shared_ptr ros = RTT::internal::GlobalService::Instance()->getService("ros");
    if (!ros) {
      Logger::log() << Logger::Error << "rtt_ros: ros service could not be loaded (NULL pointer)" << Logger::endl;
    }

    ros_import_ = ros->getOperation("import");
  }

  bool TestDeployer::import(const std::string& name) {
    if (!ros_import_.ready()) {
      Logger::log() << Logger::Error << "ros.import operation is not ready" << Logger::endl;
      return false;
    }

    if (!ros_import_(name)) {
      Logger::log() << Logger::Error << "could not import: " << name << Logger::endl;
      return false;
    }

    return true;
  }

  boost::shared_ptr<OCL::DeploymentComponent > TestDeployer::getDc() {
    return dc_;
  }

  TestDeployer& TestDeployer::Instance() {
    static TestDeployer d("test_deployer");
    return d;
  }

}   // namespace message_tests


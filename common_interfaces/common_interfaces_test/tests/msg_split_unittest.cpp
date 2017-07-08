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

#include <limits.h>
#include "common_interfaces_test_msgs/Container.h"
#include <rtt/extras/SlaveActivity.hpp>

#include "gtest/gtest.h"

#include "test_deployer.h"

namespace message_split_tests {

using namespace common_interfaces_test_msgs;

class TestComponentOut : public RTT::TaskContext {

public:

    explicit TestComponentOut(const std::string& name) :
        TaskContext(name, PreOperational),
        cont_()
    {
        this->ports()->addPort("cont1_sub1_field1", port1);
        this->ports()->addPort("cont1_sub1_field2", port2);
        this->ports()->addPort("cont1_sub2_field1", port3);
        this->ports()->addPort("cont1_sub2_field2", port4);
        this->ports()->addPort("cont1_field1", port5);
        this->ports()->addPort("cont1_field2", port6);
        this->ports()->addPort("cont2_sub1_field1", port7);
        this->ports()->addPort("cont2_sub1_field2", port8);
        this->ports()->addPort("cont2_sub2_field1", port9);
        this->ports()->addPort("cont2_sub2_field2", port10);
        this->ports()->addPort("cont2_field1", port11);
        this->ports()->addPort("cont2_field2", port12);
        this->ports()->addPort("cont3", port13);
        this->ports()->addPort("cont4", port14);
        this->ports()->addPort("field1", port15);
        this->ports()->addPort("field2", port16);
    }

    bool configureHook() {
        return true;
    }

    bool startHook() {
        return true;
    }

    void updateHook() {
        status_[port1.getName()] = port1.read(cont_.cont1.sub1.field1);
        status_[port2.getName()] = port2.read(cont_.cont1.sub1.field2);
        status_[port3.getName()] = port3.read(cont_.cont1.sub2.field1);
        status_[port4.getName()] = port4.read(cont_.cont1.sub2.field2);
        status_[port5.getName()] = port5.read(cont_.cont1.field1);
        status_[port6.getName()] = port6.read(cont_.cont1.field2);
        status_[port7.getName()] = port7.read(cont_.cont2.sub1.field1);
        status_[port8.getName()] = port8.read(cont_.cont2.sub1.field2);
        status_[port9.getName()] = port9.read(cont_.cont2.sub2.field1);
        status_[port10.getName()] = port10.read(cont_.cont2.sub2.field2);
        status_[port11.getName()] = port11.read(cont_.cont2.field1);
        status_[port12.getName()] = port12.read(cont_.cont2.field2);
        status_[port13.getName()] = port13.read(cont_.cont3);
        status_[port14.getName()] = port14.read(cont_.cont4);
        status_[port15.getName()] = port15.read(cont_.field1);
        status_[port16.getName()] = port16.read(cont_.field2);
    }

    bool connectPorts(RTT::TaskContext* other, const std::vector<std::string >& not_connected_ports = std::vector<std::string >()) {
        RTT::DataFlowInterface::Ports ports = this->ports()->getPorts();
        for (int i = 0; i < ports.size(); ++i) {
            bool ignore = false;
            for (int j = 0; j < not_connected_ports.size(); ++j) {
                if (ports[i]->getName() == not_connected_ports[j] || ports[i]->getName() + "_OUTPORT" == not_connected_ports[j]) {
                    ignore = true;
                }
            }
            if (ignore) {
                continue;
            }
            RTT::base::PortInterface* pi = other->ports()->getPort(ports[i]->getName() + "_OUTPORT");
            if (!pi || !ports[i]->connectTo( pi )) {
                return false;
            }
        }
        return true;
    }

    Container getData() const {
        return cont_;
    }

    RTT::FlowStatus getStatus(const std::string& port_name) {
        return status_[port_name];
    }

    const std::map<std::string, RTT::FlowStatus >& getStatus() const {
        return status_;
    }

private:
    Container cont_;

    RTT::InputPort<uint32_t > port1;
    RTT::InputPort<uint32_t > port2;
    RTT::InputPort<uint32_t > port3;
    RTT::InputPort<uint32_t > port4;
    RTT::InputPort<double > port5;
    RTT::InputPort<double > port6;
    RTT::InputPort<uint32_t > port7;
    RTT::InputPort<uint32_t > port8;
    RTT::InputPort<uint32_t > port9;
    RTT::InputPort<uint32_t > port10;
    RTT::InputPort<double > port11;
    RTT::InputPort<double > port12;
    RTT::InputPort<SubContainer > port13;
    RTT::InputPort<SubContainer > port14;
    RTT::InputPort<uint32_t > port15;
    RTT::InputPort<uint32_t > port16;

    std::map<std::string, RTT::FlowStatus > status_;
};

class TestComponentIn : public RTT::TaskContext {

public:

    explicit TestComponentIn(const std::string& name) :
        TaskContext(name, PreOperational)
    {
        this->ports()->addPort("msg", port);
    }

    bool configureHook() {
        return true;
    }

    bool startHook() {
        return true;
    }

    void updateHook() {
        port.write(cont_);
    }

    bool connectPorts(RTT::TaskContext* other) {
        if (!port.connectTo( other->ports()->getPort("msg_INPORT") )) {
            return false;
        }
        return true;
    }

    void setData(const Container& cont) {
        cont_ = cont;
    }


private:
    Container cont_;

    RTT::OutputPort<Container > port;
};

void initContainerData(Container& cont_in) {
  cont_in.cont1.sub1.field1 = 1;
  cont_in.cont1.sub1.field1_valid = true;
  cont_in.cont1.sub1.field2 = 2;
  cont_in.cont1.sub1_valid = true;
  cont_in.cont1.sub2.field1 = 3;
  cont_in.cont1.sub2.field1_valid = true;
  cont_in.cont1.sub2.field2 = 4;
  cont_in.cont1.field1 = 5.0;
  cont_in.cont1.field1_valid = true;
  cont_in.cont1.field2 = 6.0;
  cont_in.cont1_valid = true;
  cont_in.cont2.sub1.field1 = 7;
  cont_in.cont2.sub1.field1_valid = true;
  cont_in.cont2.sub1.field2 = 8;
  cont_in.cont2.sub1_valid = true;
  cont_in.cont2.sub2.field1 = 9;
  cont_in.cont2.sub2.field1_valid = true;
  cont_in.cont2.sub2.field2 = 10;
  cont_in.cont2.field1 = 11.0;
  cont_in.cont2.field1_valid = true;
  cont_in.cont2.field2 = 12.0;
  cont_in.cont3.sub1.field1 = 13;
  cont_in.cont3.sub1.field1_valid = true;
  cont_in.cont3.sub1.field2 = 14;
  cont_in.cont3.sub1_valid = true;
  cont_in.cont3.sub2.field1_valid = true;
  cont_in.cont3.sub2.field1 = 15;
  cont_in.cont3.sub2.field2 = 16;
  cont_in.cont3.field1 = 17.0;
  cont_in.cont3.field1_valid = true;
  cont_in.cont3.field2 = 18.0;
  cont_in.cont3_valid = true;
  cont_in.cont4.sub1.field1 = 19;
  cont_in.cont4.sub1.field1_valid = true;
  cont_in.cont4.sub1.field2 = 21;
  cont_in.cont4.sub1_valid = true;
  cont_in.cont4.sub2.field1 = 22;
  cont_in.cont4.sub2.field1_valid = true;
  cont_in.cont4.sub2.field2 = 23;
  cont_in.cont4.field1 = 24.0;
  cont_in.cont4.field1_valid = true;
  cont_in.cont4.field2 = 25.0;
  cont_in.field1 = 26;
  cont_in.field1_valid = true;
  cont_in.field2 = 27;
}


// Tests for class MessageSplit.

// Tests MessageSplit class for data valid on all input ports.
TEST(MessageSplitTest, AllValid) {

  message_tests::TestDeployer& d = message_tests::TestDeployer::Instance();

  EXPECT_TRUE( d.getDc()->import("rtt_common_interfaces_test_subsystem_ports") );
  EXPECT_TRUE(d.getDc()->loadComponent("split", "common_interfaces_test_msgs_ContainerSplit"));
  RTT::TaskContext *split = d.getDc()->getPeer("split");
  EXPECT_TRUE(split != NULL);

  // the component under test
  split->setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( split->configure() );
  EXPECT_TRUE( split->start() );
  EXPECT_EQ( split->ports()->getPortNames().size(), 24);

  // component that writes data on input ports of the component under test
  TestComponentIn test_in("test_in");
  test_in.setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( test_in.connectPorts(split) );
  EXPECT_TRUE( test_in.configure() );
  EXPECT_TRUE( test_in.start() );

  // component that reads data from output port of the component under test
  TestComponentOut test_out("test_out");
  test_out.setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( test_out.connectPorts(split) );
  EXPECT_TRUE( test_out.configure() );
  EXPECT_TRUE( test_out.start() );

  // generate some data
  Container cont_in;
  initContainerData(cont_in);

  // load the data into test_in component
  test_in.setData( cont_in );

  // execute all components
  test_in.getActivity()->execute();
  split->getActivity()->execute();
  test_out.getActivity()->execute();

  // get the result data
  Container cont_out = test_out.getData();

  // check data flow status
  EXPECT_EQ(test_out.getStatus("cont1_sub1_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont1_sub1_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont1_sub2_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont1_sub2_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont1_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont1_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_sub1_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_sub1_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_sub2_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_sub2_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont3"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont4"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("field2"), RTT::NewData);


  // check the data
  EXPECT_EQ(cont_out.cont1.sub1.field1,         cont_in.cont1.sub1.field1);
  EXPECT_EQ(cont_out.cont1.sub1.field2,         cont_in.cont1.sub1.field2);
  EXPECT_EQ(cont_out.cont1.sub2.field1,         cont_in.cont1.sub2.field1);
  EXPECT_EQ(cont_out.cont1.sub2.field2,         cont_in.cont1.sub2.field2);
  EXPECT_EQ(cont_out.cont1.field1,              cont_in.cont1.field1);
  EXPECT_EQ(cont_out.cont1.field2,              cont_in.cont1.field2);

  EXPECT_EQ(cont_out.cont2.sub1.field1,         cont_in.cont2.sub1.field1);
  EXPECT_EQ(cont_out.cont2.sub1.field2,         cont_in.cont2.sub1.field2);
  EXPECT_EQ(cont_out.cont2.sub2.field1,         cont_in.cont2.sub2.field1);
  EXPECT_EQ(cont_out.cont2.sub2.field2,         cont_in.cont2.sub2.field2);
  EXPECT_EQ(cont_out.cont2.field1,              cont_in.cont2.field1);
  EXPECT_EQ(cont_out.cont2.field2,              cont_in.cont2.field2);

  EXPECT_EQ(cont_out.cont3.sub1.field1,         cont_in.cont3.sub1.field1);
  EXPECT_EQ(cont_out.cont3.sub1.field2,         cont_in.cont3.sub1.field2);
  EXPECT_EQ(cont_out.cont3.sub2.field1,         cont_in.cont3.sub2.field1);
  EXPECT_EQ(cont_out.cont3.sub2.field2,         cont_in.cont3.sub2.field2);
  EXPECT_EQ(cont_out.cont3.field1,              cont_in.cont3.field1);
  EXPECT_EQ(cont_out.cont3.field2,              cont_in.cont3.field2);

  EXPECT_EQ(cont_out.cont4.sub1.field1,         cont_in.cont4.sub1.field1);
  EXPECT_EQ(cont_out.cont4.sub1.field2,         cont_in.cont4.sub1.field2);
  EXPECT_EQ(cont_out.cont4.sub2.field1,         cont_in.cont4.sub2.field1);
  EXPECT_EQ(cont_out.cont4.sub2.field2,         cont_in.cont4.sub2.field2);
  EXPECT_EQ(cont_out.cont4.field1,              cont_in.cont4.field1);
  EXPECT_EQ(cont_out.cont4.field2,              cont_in.cont4.field2);

  EXPECT_EQ(cont_out.field1,                    cont_in.field1);

  EXPECT_EQ(cont_out.field2,                    cont_in.field2);

  // stop & cleanup
  test_in.stop();
  test_in.cleanup();

  test_out.stop();
  test_out.cleanup();

  split->stop();
  split->cleanup();

  d.getDc()->kickOutAll();
}

// Tests MessageSplit class for data valid on some input ports.
TEST(MessageSplitTest, AllValidExceptLowest) {

  message_tests::TestDeployer& d = message_tests::TestDeployer::Instance();

  EXPECT_TRUE( d.getDc()->import("rtt_common_interfaces_test_subsystem_ports") );
  EXPECT_TRUE(d.getDc()->loadComponent("split", "common_interfaces_test_msgs_ContainerSplit"));
  RTT::TaskContext *split = d.getDc()->getPeer("split");
  EXPECT_TRUE(split != NULL);

  // the component under test
  split->setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( split->configure() );
  EXPECT_TRUE( split->start() );
  EXPECT_EQ( split->ports()->getPortNames().size(), 24);

  // component that writes data on input ports of the component under test
  TestComponentIn test_in("test_in");
  test_in.setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( test_in.connectPorts(split) );
  EXPECT_TRUE( test_in.configure() );
  EXPECT_TRUE( test_in.start() );

  // component that reads data from output port of the component under test
  TestComponentOut test_out("test_out");
  test_out.setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( test_out.connectPorts(split) );
  EXPECT_TRUE( test_out.configure() );
  EXPECT_TRUE( test_out.start() );

  // generate some data
  Container cont_in;
  initContainerData(cont_in);
  cont_in.cont1.sub1.field1_valid = false;

  // load the data into test_in component
  test_in.setData( cont_in );

  // execute all components
  test_in.getActivity()->execute();
  split->getActivity()->execute();
  test_out.getActivity()->execute();

  // get the result data
  Container cont_out = test_out.getData();

  // check data flow status
  EXPECT_EQ(test_out.getStatus("cont1_sub1_field1"), RTT::NoData);
  EXPECT_EQ(test_out.getStatus("cont1_sub1_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont1_sub2_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont1_sub2_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont1_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont1_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_sub1_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_sub1_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_sub2_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_sub2_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont3"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont4"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("field2"), RTT::NewData);


  // check the data
  // TODO: due to bug https://github.com/orocos-toolchain/rtt/issues/177
  // value of data sample read on port with RTT::NoData status is undefined
  // TODO: uncomment these line when the bug is resolved
  //EXPECT_EQ(cont_out.cont1.sub1.field1,         0);
  EXPECT_EQ(cont_out.cont1.sub1.field2,         cont_in.cont1.sub1.field2);
  EXPECT_EQ(cont_out.cont1.sub2.field1,         cont_in.cont1.sub2.field1);
  EXPECT_EQ(cont_out.cont1.sub2.field2,         cont_in.cont1.sub2.field2);
  EXPECT_EQ(cont_out.cont1.field1,              cont_in.cont1.field1);
  EXPECT_EQ(cont_out.cont1.field2,              cont_in.cont1.field2);

  EXPECT_EQ(cont_out.cont2.sub1.field1,         cont_in.cont2.sub1.field1);
  EXPECT_EQ(cont_out.cont2.sub1.field2,         cont_in.cont2.sub1.field2);
  EXPECT_EQ(cont_out.cont2.sub2.field1,         cont_in.cont2.sub2.field1);
  EXPECT_EQ(cont_out.cont2.sub2.field2,         cont_in.cont2.sub2.field2);
  EXPECT_EQ(cont_out.cont2.field1,              cont_in.cont2.field1);
  EXPECT_EQ(cont_out.cont2.field2,              cont_in.cont2.field2);

  EXPECT_EQ(cont_out.cont3.sub1.field1,         cont_in.cont3.sub1.field1);
  EXPECT_EQ(cont_out.cont3.sub1.field2,         cont_in.cont3.sub1.field2);
  EXPECT_EQ(cont_out.cont3.sub2.field1,         cont_in.cont3.sub2.field1);
  EXPECT_EQ(cont_out.cont3.sub2.field2,         cont_in.cont3.sub2.field2);
  EXPECT_EQ(cont_out.cont3.field1,              cont_in.cont3.field1);
  EXPECT_EQ(cont_out.cont3.field2,              cont_in.cont3.field2);

  EXPECT_EQ(cont_out.cont4.sub1.field1,         cont_in.cont4.sub1.field1);
  EXPECT_EQ(cont_out.cont4.sub1.field2,         cont_in.cont4.sub1.field2);
  EXPECT_EQ(cont_out.cont4.sub2.field1,         cont_in.cont4.sub2.field1);
  EXPECT_EQ(cont_out.cont4.sub2.field2,         cont_in.cont4.sub2.field2);
  EXPECT_EQ(cont_out.cont4.field1,              cont_in.cont4.field1);
  EXPECT_EQ(cont_out.cont4.field2,              cont_in.cont4.field2);

  EXPECT_EQ(cont_out.field1,                    cont_in.field1);

  EXPECT_EQ(cont_out.field2,                    cont_in.field2);

  // stop & cleanup
  test_in.stop();
  test_in.cleanup();

  test_out.stop();
  test_out.cleanup();

  split->stop();
  split->cleanup();

  d.getDc()->kickOutAll();
}

// Tests MessageSplit class for data valid on some input ports.
TEST(MessageSplitTest, AllValidExceptMiddle) {

  message_tests::TestDeployer& d = message_tests::TestDeployer::Instance();

  EXPECT_TRUE( d.getDc()->import("rtt_common_interfaces_test_subsystem_ports") );
  EXPECT_TRUE(d.getDc()->loadComponent("split", "common_interfaces_test_msgs_ContainerSplit"));
  RTT::TaskContext *split = d.getDc()->getPeer("split");
  EXPECT_TRUE(split != NULL);

  // the component under test
  split->setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( split->configure() );
  EXPECT_TRUE( split->start() );
  EXPECT_EQ( split->ports()->getPortNames().size(), 24);

  // component that writes data on input ports of the component under test
  TestComponentIn test_in("test_in");
  test_in.setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( test_in.connectPorts(split) );
  EXPECT_TRUE( test_in.configure() );
  EXPECT_TRUE( test_in.start() );

  // component that reads data from output port of the component under test
  TestComponentOut test_out("test_out");
  test_out.setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( test_out.connectPorts(split) );
  EXPECT_TRUE( test_out.configure() );
  EXPECT_TRUE( test_out.start() );

  // generate some data
  Container cont_in;
  initContainerData(cont_in);
  cont_in.cont1.sub1_valid = false;

  // load the data into test_in component
  test_in.setData( cont_in );

  // execute all components
  test_in.getActivity()->execute();
  split->getActivity()->execute();
  test_out.getActivity()->execute();

  // get the result data
  Container cont_out = test_out.getData();

  // check data flow status
  EXPECT_EQ(test_out.getStatus("cont1_sub1_field1"), RTT::NoData);
  EXPECT_EQ(test_out.getStatus("cont1_sub1_field2"), RTT::NoData);
  EXPECT_EQ(test_out.getStatus("cont1_sub2_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont1_sub2_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont1_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont1_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_sub1_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_sub1_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_sub2_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_sub2_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont3"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont4"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("field2"), RTT::NewData);


  // check the data
  // TODO: due to bug https://github.com/orocos-toolchain/rtt/issues/177
  // value of data sample read on port with RTT::NoData status is undefined
  // TODO: uncomment these line when the bug is resolved
  //EXPECT_EQ(cont_out.cont1.sub1.field1,         0);
  //EXPECT_EQ(cont_out.cont1.sub1.field2,         0);
  EXPECT_EQ(cont_out.cont1.sub2.field1,         cont_in.cont1.sub2.field1);
  EXPECT_EQ(cont_out.cont1.sub2.field2,         cont_in.cont1.sub2.field2);
  EXPECT_EQ(cont_out.cont1.field1,              cont_in.cont1.field1);
  EXPECT_EQ(cont_out.cont1.field2,              cont_in.cont1.field2);

  EXPECT_EQ(cont_out.cont2.sub1.field1,         cont_in.cont2.sub1.field1);
  EXPECT_EQ(cont_out.cont2.sub1.field2,         cont_in.cont2.sub1.field2);
  EXPECT_EQ(cont_out.cont2.sub2.field1,         cont_in.cont2.sub2.field1);
  EXPECT_EQ(cont_out.cont2.sub2.field2,         cont_in.cont2.sub2.field2);
  EXPECT_EQ(cont_out.cont2.field1,              cont_in.cont2.field1);
  EXPECT_EQ(cont_out.cont2.field2,              cont_in.cont2.field2);

  EXPECT_EQ(cont_out.cont3.sub1.field1,         cont_in.cont3.sub1.field1);
  EXPECT_EQ(cont_out.cont3.sub1.field2,         cont_in.cont3.sub1.field2);
  EXPECT_EQ(cont_out.cont3.sub2.field1,         cont_in.cont3.sub2.field1);
  EXPECT_EQ(cont_out.cont3.sub2.field2,         cont_in.cont3.sub2.field2);
  EXPECT_EQ(cont_out.cont3.field1,              cont_in.cont3.field1);
  EXPECT_EQ(cont_out.cont3.field2,              cont_in.cont3.field2);

  EXPECT_EQ(cont_out.cont4.sub1.field1,         cont_in.cont4.sub1.field1);
  EXPECT_EQ(cont_out.cont4.sub1.field2,         cont_in.cont4.sub1.field2);
  EXPECT_EQ(cont_out.cont4.sub2.field1,         cont_in.cont4.sub2.field1);
  EXPECT_EQ(cont_out.cont4.sub2.field2,         cont_in.cont4.sub2.field2);
  EXPECT_EQ(cont_out.cont4.field1,              cont_in.cont4.field1);
  EXPECT_EQ(cont_out.cont4.field2,              cont_in.cont4.field2);

  EXPECT_EQ(cont_out.field1,                    cont_in.field1);

  EXPECT_EQ(cont_out.field2,                    cont_in.field2);

  // stop & cleanup
  test_in.stop();
  test_in.cleanup();

  test_out.stop();
  test_out.cleanup();

  split->stop();
  split->cleanup();

  d.getDc()->kickOutAll();
}

// Tests MessageSplit class for data valid on some input ports.
TEST(MessageSplitTest, AllValidExceptHigh) {

  message_tests::TestDeployer& d = message_tests::TestDeployer::Instance();

  EXPECT_TRUE( d.getDc()->import("rtt_common_interfaces_test_subsystem_ports") );
  EXPECT_TRUE(d.getDc()->loadComponent("split", "common_interfaces_test_msgs_ContainerSplit"));
  RTT::TaskContext *split = d.getDc()->getPeer("split");
  EXPECT_TRUE(split != NULL);

  // the component under test
  split->setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( split->configure() );
  EXPECT_TRUE( split->start() );
  EXPECT_EQ( split->ports()->getPortNames().size(), 24);

  // component that writes data on input ports of the component under test
  TestComponentIn test_in("test_in");
  test_in.setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( test_in.connectPorts(split) );
  EXPECT_TRUE( test_in.configure() );
  EXPECT_TRUE( test_in.start() );

  // component that reads data from output port of the component under test
  TestComponentOut test_out("test_out");
  test_out.setActivity( new RTT::extras::SlaveActivity() );
  EXPECT_TRUE( test_out.connectPorts(split) );
  EXPECT_TRUE( test_out.configure() );
  EXPECT_TRUE( test_out.start() );

  // generate some data
  Container cont_in;
  initContainerData(cont_in);
  cont_in.cont1_valid = false;

  // load the data into test_in component
  test_in.setData( cont_in );

  // execute all components
  test_in.getActivity()->execute();
  split->getActivity()->execute();
  test_out.getActivity()->execute();

  // get the result data
  Container cont_out = test_out.getData();

  // check data flow status
  EXPECT_EQ(test_out.getStatus("cont1_sub1_field1"), RTT::NoData);
  EXPECT_EQ(test_out.getStatus("cont1_sub1_field2"), RTT::NoData);
  EXPECT_EQ(test_out.getStatus("cont1_sub2_field1"), RTT::NoData);
  EXPECT_EQ(test_out.getStatus("cont1_sub2_field2"), RTT::NoData);
  EXPECT_EQ(test_out.getStatus("cont1_field1"), RTT::NoData);
  EXPECT_EQ(test_out.getStatus("cont1_field2"), RTT::NoData);
  EXPECT_EQ(test_out.getStatus("cont2_sub1_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_sub1_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_sub2_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_sub2_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont2_field2"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont3"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("cont4"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("field1"), RTT::NewData);
  EXPECT_EQ(test_out.getStatus("field2"), RTT::NewData);

  // check the data
  // TODO: due to bug https://github.com/orocos-toolchain/rtt/issues/177
  // value of data sample read on port with RTT::NoData status is undefined
  // TODO: uncomment these line when the bug is resolved
//  EXPECT_EQ(cont_out.cont1.sub1.field1,         0);
//  EXPECT_EQ(cont_out.cont1.sub1.field2,         0);
//  EXPECT_EQ(cont_out.cont1.sub2.field1,         0);
//  EXPECT_EQ(cont_out.cont1.sub2.field2,         0);
//  EXPECT_EQ(cont_out.cont1.field1,              0.0);
//  EXPECT_EQ(cont_out.cont1.field2,              0.0);

  EXPECT_EQ(cont_out.cont2.sub1.field1,         cont_in.cont2.sub1.field1);
  EXPECT_EQ(cont_out.cont2.sub1.field2,         cont_in.cont2.sub1.field2);
  EXPECT_EQ(cont_out.cont2.sub2.field1,         cont_in.cont2.sub2.field1);
  EXPECT_EQ(cont_out.cont2.sub2.field2,         cont_in.cont2.sub2.field2);
  EXPECT_EQ(cont_out.cont2.field1,              cont_in.cont2.field1);
  EXPECT_EQ(cont_out.cont2.field2,              cont_in.cont2.field2);

  EXPECT_EQ(cont_out.cont3.sub1.field1,         cont_in.cont3.sub1.field1);
  EXPECT_EQ(cont_out.cont3.sub1.field2,         cont_in.cont3.sub1.field2);
  EXPECT_EQ(cont_out.cont3.sub2.field1,         cont_in.cont3.sub2.field1);
  EXPECT_EQ(cont_out.cont3.sub2.field2,         cont_in.cont3.sub2.field2);
  EXPECT_EQ(cont_out.cont3.field1,              cont_in.cont3.field1);
  EXPECT_EQ(cont_out.cont3.field2,              cont_in.cont3.field2);

  EXPECT_EQ(cont_out.cont4.sub1.field1,         cont_in.cont4.sub1.field1);
  EXPECT_EQ(cont_out.cont4.sub1.field2,         cont_in.cont4.sub1.field2);
  EXPECT_EQ(cont_out.cont4.sub2.field1,         cont_in.cont4.sub2.field1);
  EXPECT_EQ(cont_out.cont4.sub2.field2,         cont_in.cont4.sub2.field2);
  EXPECT_EQ(cont_out.cont4.field1,              cont_in.cont4.field1);
  EXPECT_EQ(cont_out.cont4.field2,              cont_in.cont4.field2);

  EXPECT_EQ(cont_out.field1,                    cont_in.field1);

  EXPECT_EQ(cont_out.field2,                    cont_in.field2);

  // stop & cleanup
  test_in.stop();
  test_in.cleanup();

  test_out.stop();
  test_out.cleanup();

  split->stop();
  split->cleanup();

  d.getDc()->kickOutAll();
}

// TODO: write unit tests for container ports

};  // namespace message_split_tests


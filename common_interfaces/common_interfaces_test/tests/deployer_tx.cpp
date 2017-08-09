/*
 Copyright (c) 2016, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include "common_interfaces_test_msgs/Container.h"
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/os/main.h>

#include "test_deployer.h"
#include "container_utils.h"

using namespace common_interfaces_test_msgs;

template <typename T>
class TestComponentOut : public RTT::TaskContext {

public:

    explicit TestComponentOut(const std::string& name)
        : TaskContext(name, PreOperational)
        , port("msg_OUTPORT", false)
    {
        this->ports()->addPort(port);
    }

    bool configureHook() {
        return true;
    }

    bool startHook() {
        return true;
    }

    void updateHook() {
        port.write(msg_);
    }

    bool connectPort(RTT::TaskContext* other) {
        RTT::base::PortInterface* pi = other->ports()->getPort("msg_INPORT");
        if (!pi || !port.connectTo( pi )) {
            return false;
        }
        return true;
    }

    void setData(const T& msg) {
        msg_ = msg;
    }

private:
    T msg_;
    RTT::OutputPort<T > port;
};

pthread_mutex_t mutex_stop = PTHREAD_MUTEX_INITIALIZER;
bool stop = false;

bool isStopped() {
    pthread_mutex_lock(&mutex_stop);
    bool result = stop;
    pthread_mutex_unlock(&mutex_stop);
    return result;
}

void func_stop(union sigval) {
    pthread_mutex_lock(&mutex_stop);
    stop = true;
    pthread_mutex_unlock(&mutex_stop);
}

void printUsage() {
    std::cout << "usage: deployer_tx stop|term time_sec time_nsec status_pipe_name" << std::endl;
}

int main(int argc, char *argv[]) {
    if (argc != 5) {
        std::cout << "wrong number of args" << std::endl;
        printUsage();
        return -1;
    }

    bool term;

    if (strcmp(argv[1], "stop") == 0) {
        term = false;
    }
    else if (strcmp(argv[1], "term") == 0) {
        term = true;
    }
    else {
        std::cout << "wrong value of arg[1]" << std::endl;
        printUsage();
        return -2;
    }

    time_t sec = atol(argv[2]);
    long nsec = atol(argv[3]);

    if (sec == 0 && nsec == 0) {
        std::cout << "wrong value of arg[2] and arg[3]" << std::endl;
        printUsage();
        return -3;
    }

    const std::string status_pipe_name(argv[4]);
    int status_pipe_fd;
    status_pipe_fd = open(status_pipe_name.c_str() , O_WRONLY);
    if (status_pipe_fd < 0) {
        std::cout << "could not open pipe \'" << status_pipe_name << "\'" << std::endl;
        return -4;
    }
    DeployerTxStatus status = DeployerTxStatus();
    write(status_pipe_fd, &status , sizeof(status));

    if (__os_init(argc, argv) != 0) {
        std::cout << "could not initialize rtt" << std::endl;
        return -4;
    }

    message_tests::TestDeployer& d = message_tests::TestDeployer::Instance();
    if (!d.getDc()->import("rtt_common_interfaces_test_subsystem_buffers")) {
        return 1;
    }

    if (!d.getDc()->loadComponent("tx", "common_interfaces_test_msgs_ContainerTx")) {
        return 2;
    }
    RTT::TaskContext *tx = d.getDc()->getPeer("tx");
    if (tx == NULL) {
        return 3;
    }
    tx->setActivity( new RTT::extras::SlaveActivity() );
    RTT::Property<std::string >* tx_channel_name = dynamic_cast<RTT::Property<std::string >* >(tx->getProperty("channel_name"));
    if (tx_channel_name == NULL) {
        return 4;
    }
    tx_channel_name->set("channel");
    if (!tx->configure()) {
        return 5;
    }

    TestComponentOut<common_interfaces_test_msgs::Container > test_out("test_out");
    test_out.setActivity( new RTT::extras::SlaveActivity() );
    if (!test_out.connectPort(tx)) {
        return 6;
    }
    if (!test_out.configure()) {
        return 7;
    }

    if (!tx->start()) {
        return 8;
    }

    if (!test_out.start()) {
        return 9;
    }

    //
    // timer for stop event
    //
    struct sigevent sig_stop;
    sig_stop.sigev_signo = 0;                   // Notification signal
    sig_stop.sigev_value.sival_ptr = NULL;      // Data passed with notification
    if (term) {
        sig_stop.sigev_notify = SIGEV_SIGNAL;       // Notification method
        sig_stop.sigev_signo = SIGINT;
    }
    else {
        sig_stop.sigev_notify = SIGEV_THREAD;       // Notification method
        sig_stop.sigev_notify_function = &func_stop;
    }
    sig_stop.sigev_notify_attributes = NULL;    // Attributes for notification thread (SIGEV_THREAD)

    timer_t timer_stop;
    timer_create(CLOCK_REALTIME, &sig_stop, &timer_stop);

    struct itimerspec time_stop;
    time_stop.it_interval.tv_sec = 0;
    time_stop.it_interval.tv_nsec = 0;
    time_stop.it_value.tv_sec = sec;
    time_stop.it_value.tv_nsec = nsec;
    timer_settime(timer_stop, 0, &time_stop, NULL);

    status.initialized_ = true;
    write(status_pipe_fd, &status , sizeof(status));

    while (!isStopped()) {
//        std::cout << "loop tx" << std::endl;
        Container cont;
        randomContainerData(cont);
        test_out.setData(cont);
        test_out.getActivity()->execute();
        tx->getActivity()->execute();
    }

    Container cont = Container();
    test_out.setData(cont);
    test_out.getActivity()->execute();
    tx->getActivity()->execute();

    // stop & cleanup
    test_out.stop();
    test_out.cleanup();

    tx->stop();
    tx->cleanup();

    d.getDc()->kickOutComponent("tx");
    __os_exit();

    close(status_pipe_fd);

    return 0;
}


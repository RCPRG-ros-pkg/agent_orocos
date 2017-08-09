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
class TestComponentIn : public RTT::TaskContext {

public:

    explicit TestComponentIn(const std::string& name)
        : TaskContext(name, PreOperational)
        , port("msg_INPORT")
        , read_successful_(false)
    {
        this->ports()->addPort(port);
    }

    bool configureHook() {
        return true;
    }

    bool startHook() {
        return true;
    }

    void stopHook() {
//        std::cout << "TestComponentIn.stopHook: state: " << (int)getTaskState() << std::endl;
        if (getTaskState() == TaskContext::Exception || getTaskState() == TaskContext::RunTimeError) {
            recover();
//            std::cout << "TestComponentIn.stopHook: state after recover: " << (int)getTaskState() << std::endl;
        }
    }

    void updateHook() {
        read_successful_ = (port.read(msg_) == RTT::NewData);
    }

    bool connectPort(RTT::TaskContext* other) {
        if (!port.connectTo( other->ports()->getPort("msg_OUTPORT") )) {
            return false;
        }
        return true;
    }

    Container getData() {
        return msg_;
    }

    bool isReadSuccessful() const {
        return read_successful_;
    }

private:
    T msg_;

    bool read_successful_;

    RTT::InputPort<T > port;
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
    std::cout << "usage: deployer_rx stop|term time_sec time_nsec status_pipe_name" << std::endl;
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
    DeployerRxStatus status = DeployerRxStatus();
    write(status_pipe_fd, &status , sizeof(status));

    if (__os_init(argc, argv) != 0) {
        std::cout << "could not initialize rtt" << std::endl;
        return -4;
    }

    message_tests::TestDeployer& d = message_tests::TestDeployer::Instance();
    if (!d.getDc()->import("rtt_common_interfaces_test_subsystem_buffers")) {
        std::cout << "could not import rtt_common_interfaces_test_subsystem_buffers" << std::endl;
        return 1;
    }

    if (!d.getDc()->loadComponent("rx", "common_interfaces_test_msgs_ContainerRx")) {
        std::cout << "could not load component rx" << std::endl;
        return 2;
    }
    RTT::TaskContext *rx = d.getDc()->getPeer("rx");
    if (rx == NULL) {
        std::cout << "could not get component rx" << std::endl;
        return 3;
    }
    rx->setActivity( new RTT::extras::SlaveActivity() );
    RTT::Property<std::string >* rx_channel_name = dynamic_cast<RTT::Property<std::string >* >(rx->getProperty("channel_name"));
    if (rx_channel_name == NULL) {
        std::cout << "could not set channel name" << std::endl;
        return 4;
    }
    rx_channel_name->set("channel");

    RTT::Property<double >* rx_timeout_s = dynamic_cast<RTT::Property<double >* >(rx->getProperty("timeout_s"));
    if (rx_timeout_s == NULL) {
        std::cout << "could not set rx timeout" << std::endl;
        return 4;
    }
    rx_timeout_s->set(0.1);

    if (!rx->configure()) {
        std::cout << "could not configure rx" << std::endl;
        return 5;
    }

    TestComponentIn<common_interfaces_test_msgs::Container > test_in("test_in");
    test_in.setActivity( new RTT::extras::SlaveActivity() );
    if (!test_in.connectPort(rx)) {
        std::cout << "could not connect ports" << std::endl;
        return 6;
    }
    if (!test_in.configure()) {
        std::cout << "could not configure test component" << std::endl;
        return 7;
    }

    if (!rx->start()) {
        std::cout << "could not start rx component" << std::endl;
        return 8;
    }

    if (!test_in.start()) {
        std::cout << "could not start test component" << std::endl;
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

    bool any_read_successful = false;
    bool all_checksums_ok = true;
    bool all_reads_successful = true;

    status.initialized_ = true;
    write(status_pipe_fd, &status , sizeof(status));

    while (!isStopped()) {
        DeployerRxStatus status_prev = status;
//        std::cout << "loop rx" << std::endl;
        Container cont;
        randomContainerData(cont);
        rx->getActivity()->execute();
        test_in.getActivity()->execute();

        if (test_in.isReadSuccessful()) {
            status.any_read_successful_ = true;
            Container cont = test_in.getData();
            if (cont.field2 != calculateChecksum(cont)) {
                all_checksums_ok = false;
            }
            status.checksum_ok_ = all_checksums_ok;
        }
        else {
            all_reads_successful = false;
        }
        status.all_reads_successful_ = all_reads_successful;

        if (!status.isEqual(status_prev)) {
            write(status_pipe_fd, &status , sizeof(status));
        }
    }

    // stop & cleanup
    test_in.stop();
    test_in.cleanup();

    rx->stop();
    rx->cleanup();

    d.getDc()->kickOutComponent("rx");
    __os_exit();

    close(status_pipe_fd);

    return 0;
}


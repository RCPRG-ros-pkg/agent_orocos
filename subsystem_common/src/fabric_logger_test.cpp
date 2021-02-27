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

#include "subsystem_common/fabric_logger.h"
#include <iostream>
#include <atomic>
#include <thread>
#include <stdlib.h>

using subsystem_common::FabricLogger;
using subsystem_common::FabricLoggerInterfaceRtPtr;

void threadFun1(bool& stop, int sleep_time)
{
    FabricLoggerInterfaceRtPtr test_A = FabricLogger::createNewInterfaceRt("test_A", 1000);
    if (test_A) {
        std::cout << "created test_A" << std::endl;
    }
    else {
        test_A = FabricLogger::getInterfaceRt("test_A");
    }

    FabricLoggerInterfaceRtPtr test_B = FabricLogger::createNewInterfaceRt("test_B", 1000);
    if (test_B) {
        std::cout << "created test_B" << std::endl;
    }
    else {
        test_B = FabricLogger::getInterfaceRt("test_B");
    }

    int msg_idx = 0;
    while (!stop) {
        test_A << "the A message no. " << msg_idx << " with int: " << 10 << " and double: " << 10.1 << FabricLogger::End();

        test_B << "the B message no. " << msg_idx << FabricLogger::End();

        ++msg_idx;

        std::this_thread::sleep_for(std::chrono::microseconds(rand()%sleep_time));
    }
}
 
void threadFun2(bool& stop, int sleep_time)
{
    // Wait for initialization of logs
    while (true) {
        if (FabricLogger::getInterfaceRt("test_A") && FabricLogger::getInterfaceRt("test_B")) {
            break;
        }
    }
    while (!stop) {
        std::vector<std::string > vec;
        vec = FabricLogger::getAllLogs("test_A");
        for (int i = 0; i < vec.size(); ++i) {
            std::cout << vec[i] << std::endl;
        }
        vec = FabricLogger::getAllLogs("test_B");
        for (int i = 0; i < vec.size(); ++i) {
            std::cout << vec[i] << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(rand()%sleep_time));
    }
}
 
int main() {
    bool stop;

    int test_time_seconds = 60;

    // Test 0: small buffer, the same sleep time for both threads
    std::cout << "Test 0" << std::endl;
    stop = false;
    std::thread t1(threadFun1, std::ref(stop), 100);
    std::thread t2(threadFun2, std::ref(stop), 100);
    std::this_thread::sleep_for( std::chrono::seconds(test_time_seconds) );
    stop = true;
    t1.join();
    t2.join();

    // Test 1: small buffer, frequency of producer is 10 times higher than consumer
    std::cout << "Test 1" << std::endl;
    stop = false;
    std::thread t3(threadFun1, std::ref(stop), 100);
    std::thread t4(threadFun2, std::ref(stop), 10000);
    std::this_thread::sleep_for( std::chrono::seconds(test_time_seconds) );
    stop = true;
    t3.join();
    t4.join();

    // Test 2: small buffer, frequency of producer is 10 times lower than consumer
    std::cout << "Test 2" << std::endl;
    stop = false;
    std::thread t5(threadFun1, std::ref(stop), 1000);
    std::thread t6(threadFun2, std::ref(stop), 100);
    std::this_thread::sleep_for( std::chrono::seconds(test_time_seconds) );
    stop = true;
    t5.join();
    t6.join();

    std::cout << "Test END" << std::endl;

    return 0;
}

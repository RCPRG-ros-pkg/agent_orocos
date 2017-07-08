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

#ifndef COMMON_INTERFACES_TEST_CONTAINER_UTILS_H_
#define COMMON_INTERFACES_TEST_CONTAINER_UTILS_H_

#include "common_interfaces_test_msgs/Container.h"

class DeployerRxStatus {
public:
    bool initialized_;
    bool any_read_successful_;
    bool checksum_ok_;
    bool all_reads_successful_;

    bool isEqual(const DeployerRxStatus& other) const {
        return initialized_ == other.initialized_
            && any_read_successful_ == other.any_read_successful_
            && checksum_ok_ == other.checksum_ok_
            && all_reads_successful_ == other.all_reads_successful_;
    }

    DeployerRxStatus()
        : initialized_(false)
        , any_read_successful_(false)
        , checksum_ok_(false)
        , all_reads_successful_(false)
    {}
};

class DeployerTxStatus {
public:
    bool initialized_;

    bool isEqual(const DeployerRxStatus& other) const {
        return initialized_ == other.initialized_;
    }

    DeployerTxStatus()
        : initialized_(false)
    {}
};

uint32_t calculateChecksum(common_interfaces_test_msgs::Container& cont_in);
void randomContainerData(common_interfaces_test_msgs::Container& cont_in);

#endif  // COMMON_INTERFACES_TEST_CONTAINER_UTILS_H_


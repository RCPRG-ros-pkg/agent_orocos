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

#include <future>
#include <limits.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include "common_interfaces_test_msgs/Container.h"
#include <rtt/extras/SlaveActivity.hpp>

#include "gtest/gtest.h"

#include "test_deployer.h"
#include "container_utils.h"

#include "shm_comm/shm_channel.h"

namespace message_concate_tests {

using namespace common_interfaces_test_msgs;

void executeTxRxTest(bool remove_channel, bool terminate, int sec, long nsec) {

        if (remove_channel) {
            shm_remove_channel("channel");
        }

        const std::string fifo_rx_status("/tmp/common_interfaces_test_fifo_rx_status");
        const std::string fifo_tx_status("/tmp/common_interfaces_test_fifo_tx_status");
        mkfifo(fifo_rx_status.c_str(), 0666);
        mkfifo(fifo_tx_status.c_str(), 0666);

        const std::string stop_method(terminate?"term":"stop");
        const std::string time_str(std::to_string(sec) + " " + std::to_string(nsec));

        const std::string rx_deployer_cmd( std::string("rosrun common_interfaces_test deployer_rx ") + stop_method + " " + time_str + " " + fifo_rx_status );
        const std::string tx_deployer_cmd( std::string("rosrun common_interfaces_test deployer_tx ") + stop_method + " " + time_str + " " + fifo_tx_status );

        int rx_status_pipe_fd;
        rx_status_pipe_fd = open(fifo_rx_status.c_str() , O_RDONLY | O_NONBLOCK);
        if (rx_status_pipe_fd < 0) {
            std::cout << "could not open pipe \'" << fifo_rx_status << "\'" << std::endl;
        }
        EXPECT_TRUE(rx_status_pipe_fd > 0);

        int tx_status_pipe_fd;
        tx_status_pipe_fd = open(fifo_tx_status.c_str() , O_RDONLY | O_NONBLOCK);
        if (tx_status_pipe_fd < 0) {
            std::cout << "could not open pipe \'" << fifo_tx_status << "\'" << std::endl;
        }
        EXPECT_TRUE(tx_status_pipe_fd > 0);

        auto fut_rx = std::async(std::launch::async,
            [rx_deployer_cmd] { return system(rx_deployer_cmd.c_str()); });
        auto fut_tx = std::async(std::launch::async,
            [tx_deployer_cmd] { return system(tx_deployer_cmd.c_str()); });

        DeployerRxStatus rx_status = DeployerRxStatus();
        DeployerTxStatus tx_status = DeployerTxStatus();
        while (true) {
            int rx_nbytes = read(rx_status_pipe_fd, &rx_status , sizeof(rx_status));
            int tx_nbytes = read(tx_status_pipe_fd, &tx_status , sizeof(tx_status));
            if (fut_rx.wait_for(std::chrono::seconds(0)) == std::future_status::ready && fut_tx.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                break;
            }
        }

        EXPECT_TRUE(tx_status.initialized_);

        EXPECT_TRUE(rx_status.initialized_);
        EXPECT_TRUE(rx_status.any_read_successful_);
        EXPECT_TRUE(rx_status.checksum_ok_);

        close(rx_status_pipe_fd);
        close(tx_status_pipe_fd);
}

// Tests MessageRx/MessageTx classes
TEST(MessageRxTxTest, CreateTxRxTermRm) {
    for (int i = 0; i < 20; ++i) {
        executeTxRxTest(true, true, 0, 100000000);
    }
}

TEST(MessageRxTxTest, CreateTxRxTerm) {
    for (int i = 0; i < 20; ++i) {
        executeTxRxTest(false, true, 0, 100000000);
    }
}

// Tests MessageRx/MessageTx classes
TEST(MessageRxTxTest, CreateTxRxStopRm) {
    for (int i = 0; i < 20; ++i) {
        executeTxRxTest(true, false, 0, 100000000);
    }
}

TEST(MessageRxTxTest, CreateTxRxStop) {
    for (int i = 0; i < 20; ++i) {
        executeTxRxTest(false, false, 0, 100000000);
    }
}

void executeTxTest(bool terminate, int rx_sec, long rx_nsec, int tx_sec, long tx_nsec) {
    const std::string fifo_tx_status("/tmp/common_interfaces_test_fifo_tx_status");
    const std::string fifo_rx_status("/tmp/common_interfaces_test_fifo_rx_status");

    const std::string stop_method(terminate?"term":"stop");
    const std::string rx_time_str(std::to_string(rx_sec) + " " + std::to_string(rx_nsec));
    const std::string tx_time_str(std::to_string(tx_sec) + " " + std::to_string(tx_nsec));

    const std::string rx_deployer_cmd( std::string("rosrun common_interfaces_test deployer_rx ") + stop_method + " " + rx_time_str + " " + fifo_rx_status );
    const std::string tx_deployer_cmd( std::string("rosrun common_interfaces_test deployer_tx ") + stop_method + " " + tx_time_str + " " + fifo_tx_status );

    mkfifo(fifo_rx_status.c_str(), 0666);
    int rx_status_pipe_fd = open(fifo_rx_status.c_str() , O_RDONLY | O_NONBLOCK);
    if (rx_status_pipe_fd < 0) {
        std::cout << "could not open pipe \'" << fifo_rx_status << "\'" << std::endl;
    }
    EXPECT_TRUE(rx_status_pipe_fd > 0);

    auto fut_rx = std::async(std::launch::async,
        [rx_deployer_cmd] { return system(rx_deployer_cmd.c_str()); });

    DeployerRxStatus rx_status = DeployerRxStatus();

//    int i = 0;
    while (true) {
        mkfifo(fifo_tx_status.c_str(), 0666);
        int tx_status_pipe_fd = tx_status_pipe_fd = open(fifo_tx_status.c_str() , O_RDONLY | O_NONBLOCK);
        if (tx_status_pipe_fd < 0) {
            std::cout << "could not open pipe \'" << fifo_tx_status << "\'" << std::endl;
        }
        EXPECT_TRUE(tx_status_pipe_fd > 0);

        auto fut_tx = std::async(std::launch::async,
            [tx_deployer_cmd] { return system(tx_deployer_cmd.c_str()); });

        DeployerTxStatus tx_status = DeployerTxStatus();
        while (true) {
            int rx_nbytes = read(rx_status_pipe_fd, &rx_status , sizeof(rx_status));
            int tx_nbytes = read(tx_status_pipe_fd, &tx_status , sizeof(tx_status));
            if (fut_rx.wait_for(std::chrono::seconds(0)) == std::future_status::ready || fut_tx.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                break;
            }
        }

//        std::cout << "iteration " << i << ": "
//            << (tx_status.initialized_?"t":"f") << " "
//            << (rx_status.initialized_?"t":"f") << " "
//            << (rx_status.any_read_successful_?"t":"f") << " "
//            << (rx_status.checksum_ok_?"t":"f") << " "
//            << std::endl;
//        ++i;

        if (fut_rx.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            // rx was closed
            break;
        }
        else {
            EXPECT_TRUE(tx_status.initialized_);
            EXPECT_TRUE(rx_status.initialized_);
            EXPECT_TRUE(rx_status.any_read_successful_);
            EXPECT_TRUE(rx_status.checksum_ok_);
        }

        close(tx_status_pipe_fd);
    }
    close(rx_status_pipe_fd);
}

// Tests MessageRx/MessageTx classes
TEST(MessageRxTxTest, CreateTxTerm) {
    executeTxTest(true, 10, 0, 0, 100000000);
}

// Tests MessageRx/MessageTx classes
TEST(MessageRxTxTest, CreateTxStop) {
    executeTxTest(false, 10, 0, 0, 100000000);
}

void executeRxTest(bool terminate, int rx_sec, long rx_nsec, int tx_sec, long tx_nsec) {
    const std::string fifo_tx_status("/tmp/common_interfaces_test_fifo_tx_status");
    const std::string fifo_rx_status("/tmp/common_interfaces_test_fifo_rx_status");

    const std::string stop_method(terminate?"term":"stop");
    const std::string rx_time_str(std::to_string(rx_sec) + " " + std::to_string(rx_nsec));
    const std::string tx_time_str(std::to_string(tx_sec) + " " + std::to_string(tx_nsec));

    const std::string rx_deployer_cmd( std::string("rosrun common_interfaces_test deployer_rx ") + stop_method + " " + rx_time_str + " " + fifo_rx_status );
    const std::string tx_deployer_cmd( std::string("rosrun common_interfaces_test deployer_tx ") + stop_method + " " + tx_time_str + " " + fifo_tx_status );

    mkfifo(fifo_tx_status.c_str(), 0666);
    int tx_status_pipe_fd = open(fifo_tx_status.c_str() , O_RDONLY | O_NONBLOCK);
    if (tx_status_pipe_fd < 0) {
        std::cout << "could not open pipe \'" << fifo_tx_status << "\'" << std::endl;
    }
    EXPECT_TRUE(tx_status_pipe_fd > 0);

    auto fut_tx = std::async(std::launch::async,
        [tx_deployer_cmd] { return system(tx_deployer_cmd.c_str()); });

    DeployerTxStatus tx_status = DeployerTxStatus();

    while (true) {
        mkfifo(fifo_rx_status.c_str(), 0666);
        int rx_status_pipe_fd = rx_status_pipe_fd = open(fifo_rx_status.c_str() , O_RDONLY | O_NONBLOCK);
        if (rx_status_pipe_fd < 0) {
            std::cout << "could not open pipe \'" << fifo_rx_status << "\'" << std::endl;
        }
        EXPECT_TRUE(rx_status_pipe_fd > 0);

        auto fut_rx = std::async(std::launch::async,
            [rx_deployer_cmd] { return system(rx_deployer_cmd.c_str()); });

        DeployerRxStatus rx_status = DeployerRxStatus();
        while (true) {
            int rx_nbytes = read(rx_status_pipe_fd, &rx_status , sizeof(rx_status));
            int tx_nbytes = read(tx_status_pipe_fd, &tx_status , sizeof(tx_status));
            if (fut_rx.wait_for(std::chrono::seconds(0)) == std::future_status::ready || fut_tx.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                break;
            }
        }

        if (fut_tx.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            // tx was closed
            break;
        }
        else {
            EXPECT_TRUE(rx_status.initialized_);
            EXPECT_TRUE(rx_status.any_read_successful_);
            EXPECT_TRUE(rx_status.all_reads_successful_);
            EXPECT_TRUE(rx_status.checksum_ok_);
            EXPECT_TRUE(tx_status.initialized_);
        }

        close(rx_status_pipe_fd);
    }
    close(tx_status_pipe_fd);
}

// Tests MessageRx/MessageTx classes
TEST(MessageRxTxTest, CreateRxTerm) {
    executeRxTest(true, 0, 100000000, 10, 0);
}

// Tests MessageRx/MessageTx classes
TEST(MessageRxTxTest, CreateRxStop) {
    executeRxTest(false, 0, 100000000, 10, 0);
}

};  // namespace message_concate_tests


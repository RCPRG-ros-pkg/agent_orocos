/*
 Copyright (c) 2021, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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

Author: Dawid Seredynski
*/

#include "subsystem_common/fabric_logger.h"

#include <iostream>
#include <stdexcept>
#include <iostream>
#include <sstream>

namespace subsystem_common {

std::shared_ptr<FabricLogger > FabricLogger::m_instance = std::shared_ptr<FabricLogger >();
std::shared_ptr<FabricLogger > FabricLogger::m_null_instance = std::shared_ptr<FabricLogger >();
std::mutex FabricLogger::m_mutex;

//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// FabricLogger                                                                                 //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////

FabricLogger::FabricLogger()
{}

FabricLoggerInterfaceRtPtr FabricLogger::createNewInterfaceRt(const std::string& name, int buffer_size) {
    std::lock_guard<std::mutex> guard(m_mutex);

    // Singleton
    if (!m_instance) {
        m_instance.reset( new FabricLogger() );
    }

    for (int ii = 0; ii < m_instance->m_interfaces.size(); ++ii) {
        if (m_instance->m_interfaces[ii]->getName() == name) {
            std::cout << "ERROR at FabricLogger::createNewInterfaceRt:"
                        << " An interface with the same name is already registred: \""
                        << name << "\"" << std::endl;
            return FabricLoggerInterfaceRtPtr();
        }
    }
    int new_interface_idx = m_instance->m_interfaces.size();
    LoggerInterfaceDataPtr data_ptr( new LoggerInterfaceData(name, buffer_size) );
    m_instance->m_interfaces.push_back( data_ptr );

    return FabricLoggerInterfaceRtPtr( new FabricLoggerInterfaceRt( data_ptr ) );
}

FabricLoggerInterfaceRtPtr FabricLogger::getInterfaceRt(const std::string& name) {
    std::lock_guard<std::mutex> guard(m_mutex);

    if (!m_instance) {
        m_instance.reset( new FabricLogger() );
    }

    FabricLoggerInterfaceRtPtr result;

    for (int ii = 0; ii < m_instance->m_interfaces.size(); ++ii) {
        if (m_instance->m_interfaces[ii]->getName() == name) {
            return FabricLoggerInterfaceRtPtr( new FabricLoggerInterfaceRt( m_instance->m_interfaces[ii] ) );
        }
    }
    return FabricLoggerInterfaceRtPtr();
}

std::vector<std::string> FabricLogger::getAllLogs(const std::string& name) {
    for (int ii = 0; ii < m_instance->m_interfaces.size(); ++ii) {
        if (m_instance->m_interfaces[ii]->getName() == name) {
            return m_instance->m_interfaces[ii]->getAllLogs();
        }
    }
    return std::vector<std::string>();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// CircularBuffer                                                                               //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////

CircularBuffer::CircularBuffer(int max_buffer_size)
: m_max_buffer_size(max_buffer_size)
, m_buffer_write_pos(0)
, m_buffer_read_pos(0)
{
    if (m_max_buffer_size <= 0) {
        throw std::invalid_argument("buffer_size <= 0");
    }
    m_buffer = new unsigned char[m_max_buffer_size];
}

CircularBuffer::~CircularBuffer() {
    delete[] m_buffer;
}

void CircularBuffer::addItem(unsigned char item_type, unsigned char* buf, int size) {
    int total_size = 1 + sizeof(int) + size;

    if (total_size >= m_max_buffer_size-1) {
        throw std::invalid_argument("Too small buffer for adding an item");
    }

    // This is the only place where m_buffer_write_pos is changed and it is done by one thread
    int write_pos = m_buffer_write_pos.load( std::memory_order_seq_cst );

    // Make some space in the buffer, if needed
    while (true) {
        // Calculate buffer size
        int buffer_size;
        // m_buffer_read_pos may change at any time
        int read_pos = m_buffer_read_pos.load( std::memory_order_seq_cst );
        if (read_pos <= write_pos) {
            buffer_size = write_pos - read_pos;
        }
        else {
            buffer_size = write_pos + m_max_buffer_size - read_pos;
        }

        if (buffer_size + total_size >= m_max_buffer_size-1) {
            popItem(NULL, 0);
        }
        else {
            break;
        }
    }

    // Write item type
    m_buffer[write_pos] = item_type;
    write_pos = (write_pos+1) % m_max_buffer_size;

    // Write item size
    unsigned char* size_ptr = (unsigned char*)(&size);
    for (int i = 0; i < sizeof(int); ++i) {
        m_buffer[write_pos] = size_ptr[i];
        write_pos = (write_pos+1) % m_max_buffer_size;
    }

    // Write item data
    for (int i = 0; i < size; ++i) {
        m_buffer[write_pos] = buf[i];
        write_pos = (write_pos+1) % m_max_buffer_size;
    }
    m_buffer_write_pos.store( write_pos, std::memory_order_seq_cst );
}

bool CircularBuffer::popItem(unsigned char* buf, int max_size) {
    // Get current read pos - it may change at any time
    int write_pos = m_buffer_write_pos.load( std::memory_order_seq_cst );
    int read_pos = m_buffer_read_pos.load( std::memory_order_seq_cst );
    int read_pos_tmp = read_pos;

    if (read_pos == write_pos) {
        return false;
    }

    read_pos_tmp = (read_pos_tmp+1) % m_max_buffer_size;

    // Read item data size
    unsigned char data_size_vec[sizeof(int)];
    for (int i = 0; i < sizeof(int); ++i) {
        data_size_vec[i] = m_buffer[read_pos_tmp];
        read_pos_tmp = (read_pos_tmp+1) % m_max_buffer_size;
    }
    int data_size = *((int*)(&data_size_vec[0]));
    int total_size = 1 + sizeof(int) + data_size;

    // The total size of data is read, but we have to check again, if the item is valid
    int read_pos_again = m_buffer_read_pos.load( std::memory_order_seq_cst );
    if (read_pos_again != read_pos) {
        return false;
    }

    if (buf != NULL && total_size <= max_size) {
        read_pos_tmp = read_pos;
        // Copy the item
        for (int out_buf_pos = 0; out_buf_pos < total_size; ++out_buf_pos) {
            buf[out_buf_pos] = m_buffer[read_pos_tmp];
            read_pos_tmp = (read_pos_tmp+1) % m_max_buffer_size;
        }
    }

    // Update the new read pos, if it is still valid
    int new_read_pos = (read_pos + total_size) % m_max_buffer_size;

    int expected_pos = read_pos;
    if (m_buffer_read_pos.compare_exchange_strong(expected_pos, new_read_pos)) {
        if (total_size > max_size) {
            return false;
        }
        // The read pos did not change, content of buf is valid
        return true;
    }
    // else
    // The item was consumed, so content of buf is valid
    return false;
}

bool CircularBuffer::isEmpty() {
    int write_pos = m_buffer_write_pos.load( std::memory_order_seq_cst );
    int read_pos = m_buffer_read_pos.load( std::memory_order_seq_cst );
    return read_pos == write_pos;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// FabricLogger::LoggerInterfaceData                                                            //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////

FabricLogger::LoggerInterfaceData::LoggerInterfaceData(const std::string& name, int buffer_size)
: m_name( name )
, m_buf( buffer_size )
{}

FabricLogger::LoggerInterfaceData::~LoggerInterfaceData() {
}

const std::string FabricLogger::LoggerInterfaceData::getName() const {
    return m_name;
}

void FabricLogger::LoggerInterfaceData::addItem(int i) {
    m_buf.addItem(ITEM_INT, (unsigned char*)&i, sizeof(int));
}

void FabricLogger::LoggerInterfaceData::addItem(unsigned int ui) {
    m_buf.addItem(ITEM_UINT, (unsigned char*)&ui, sizeof(unsigned int));
}

void FabricLogger::LoggerInterfaceData::addItem(double d) {
    m_buf.addItem(ITEM_DOUBLE, (unsigned char*)&d, sizeof(double));
}

void FabricLogger::FabricLogger::LoggerInterfaceData::addItem(const char* str) {
    int str_len = 0;
    for (; ; ++str_len) {
        if (str[str_len] == 0) {
            break;
        }
    }
    str_len++;  // Include terminating \0
    m_buf.addItem(ITEM_STR, (unsigned char*)&str[0], str_len);
}

void FabricLogger::FabricLogger::LoggerInterfaceData::addItem(const std::string& str) {
    addItem(str.c_str());
}

void FabricLogger::FabricLogger::LoggerInterfaceData::addItem(const FabricLogger::End& end) {
    m_buf.addItem(ITEM_END, NULL, 0);
}

// This method is not RT-safe
std::vector<std::string> FabricLogger::FabricLogger::LoggerInterfaceData::getAllLogs() {
    const int local_buf_size = 10000;
    unsigned char buf[local_buf_size];

    std::vector<std::string> result;
    std::stringstream ss;
    while (!m_buf.isEmpty()) {
        bool item_received = m_buf.popItem(&buf[0], local_buf_size);
        //std::cout << "FabricLogger::LoggerInterfaceData::getAllLogs() " << item_received << std::endl;
        if (item_received) {
            if (buf[0] == ITEM_END) {
                //std::cout << "end" << std::endl;
                result.push_back(ss.str());
                //std::cout << ss.str() << std::endl;
                ss = std::stringstream();
            }
            else if (buf[0] == ITEM_INT) {
                //std::cout << "int" << std::endl;
                ss << (*((int*)(&buf[1+sizeof(int)])));
                //std::cout << ss.str() << (*((int*)(&buf[2]))) << std::endl;
            }
            else if (buf[0] == ITEM_UINT) {
                //std::cout << "int" << std::endl;
                ss << (*((unsigned int*)(&buf[1+sizeof(unsigned int)])));
                //std::cout << ss.str() << (*((int*)(&buf[2]))) << std::endl;
            }
            else if (buf[0] == ITEM_DOUBLE) {
                //std::cout << "double " << (*((double*)(&buf[1+sizeof(int)]))) << std::endl;
                ss << (*((double*)(&buf[1+sizeof(int)])));
                //std::cout << ss.str() << (*((double*)(&buf[2]))) << std::endl;
            }
            else if (buf[0] == ITEM_STR) {
                //std::cout << "str" << std::endl;
                const char* c_str = (char*)(&buf[1+sizeof(int)]);
                ss << std::string(c_str);
                //std::cout << ss.str() << std::string(((char*)(&buf[2]))) << std::endl;
            }
            else {
                throw std::logic_error("Wrong item type");
            }
        }
        else {
            break;
        }
    }

    return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// FabricLoggerInterfaceRt                                                                      //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////
FabricLoggerInterfaceRt::FabricLoggerInterfaceRt(FabricLogger::LoggerInterfaceDataPtr data_ptr)
: m_data_ptr( data_ptr )
{}

void FabricLoggerInterfaceRt::addItem(int i) {
    m_data_ptr->addItem(i);
}

void FabricLoggerInterfaceRt::addItem(unsigned int ui) {
    m_data_ptr->addItem(ui);
}

void FabricLoggerInterfaceRt::addItem(double d) {
    m_data_ptr->addItem(d);
}

void FabricLoggerInterfaceRt::addItem(const char* str) {
    m_data_ptr->addItem(str);
}

void FabricLoggerInterfaceRt::addItem(const std::string& str) {
    m_data_ptr->addItem(str);
}

void FabricLoggerInterfaceRt::addItem(const FabricLogger::End& end) {
    m_data_ptr->addItem(end);
}

FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, int i) {
    log->addItem(i);
    return log ;
}

FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, unsigned int ui) {
    log->addItem(ui);
    return log ;
}

FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, double d) {
    log->addItem(d);
    return log ;
}

FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, const char* str) {
    log->addItem(str);
    return log ;
}
FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, const std::string& str) {
    log->addItem(str);
    return log ;
}

FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, const FabricLogger::End& end) {
    log->addItem(end);
    return log ;
}

}   // namespace subsystem_common

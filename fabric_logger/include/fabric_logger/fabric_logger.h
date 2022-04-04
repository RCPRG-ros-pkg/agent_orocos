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


#ifndef FABRIC_LOGGER_H_
#define FABRIC_LOGGER_H_

#include <memory>
#include <vector>
#include <atomic>
#include <mutex>

#include <ros/time.h>

namespace fabric_logger {

class CircularBuffer {
protected:
    const int m_max_buffer_size;
    unsigned char* m_buffer;
    std::atomic<int > m_buffer_write_pos;
    std::atomic<int > m_buffer_read_pos;
    // (m_buffer_write_pos == m_buffer_read_pos) <=> (the buffer is empty)
public:
    CircularBuffer(int max_buffer_size);
    virtual ~CircularBuffer();
    void addItem(unsigned char item_type, unsigned char* buf, int size);
    bool popItem(int& item_type, int& item_size, unsigned char* item_data, int max_size);
    bool isEmpty();
};

class FabricLoggerInterfaceRt;
typedef std::shared_ptr<FabricLoggerInterfaceRt > FabricLoggerInterfaceRtPtr;

class FabricLogger;
typedef std::shared_ptr<FabricLogger > FabricLoggerPtr;

class FabricLogger {
public:
    class End {
    protected:
        const ros::Time m_timestamp;
        const ros::WallTime m_walltime;
    public:
        End();
        const ros::Time& getTimestamp() const;
        const ros::WallTime& getWallTime() const;
    };
protected:
    class LoggerInterfaceData {
    protected:
        enum {ITEM_END=1, ITEM_PADDING=2, ITEM_INT=3, ITEM_UINT=4, ITEM_LUINT=5, ITEM_STR=6,
                ITEM_DOUBLE=7};
        const std::string m_name;
        CircularBuffer m_buf;
    public:
        LoggerInterfaceData(const std::string& name, int buffer_size);
        virtual ~LoggerInterfaceData();
        const std::string getName() const;
        void addItem(int i);
        void addItem(unsigned int i);
        void addItem(long unsigned int i);
        void addItem(double i);
        void addItem(const char* str);
        void addItem(const std::string& str);
        void addItem(const End& end);

        std::vector<std::string> getAllLogs();
    };
    typedef std::shared_ptr<LoggerInterfaceData > LoggerInterfaceDataPtr;

    static FabricLoggerPtr m_instance;
    static FabricLoggerPtr m_null_instance;
    FabricLogger();
    std::vector<LoggerInterfaceDataPtr > m_interfaces;
    static std::mutex m_mutex;
public:
    static std::shared_ptr<FabricLoggerInterfaceRt > createNewInterfaceRt(const std::string& name, int buffer_size);
    static std::shared_ptr<FabricLoggerInterfaceRt > getInterfaceRt(const std::string& name);
    static std::vector<std::string> getInterfaceNames();
    friend class FabricLoggerInterfaceRt;

    static std::vector<std::string> getAllLogs(const std::string& name);
};
 
FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, int i);
FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, unsigned int i);
FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, long unsigned int i);
FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, double i);
FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, const char* str);
FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, const std::string& str);
FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, const FabricLogger::End& end);

class FabricLoggerInterfaceRt {
protected:
    FabricLoggerInterfaceRt(FabricLogger::LoggerInterfaceDataPtr data_ptr);
    const FabricLogger::LoggerInterfaceDataPtr m_data_ptr;

    void addItem(int i);
    void addItem(unsigned int i);
    void addItem(long unsigned int i);
    void addItem(double d);
    void addItem(const char* str);
    void addItem(const std::string& str);
    void addItem(const FabricLogger::End& end);
public:
    friend FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, int i);
    friend FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, unsigned int i);
    friend FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, long unsigned int i);
    friend FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, double i);
    friend FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, const char* str);
    friend FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, const std::string& str);
    friend FabricLoggerInterfaceRtPtr& operator << (FabricLoggerInterfaceRtPtr& log, const FabricLogger::End& end);

    friend class FabricLogger;

};

}   // namespace fabric_logger

#endif  // FABRIC_LOGGER_H_

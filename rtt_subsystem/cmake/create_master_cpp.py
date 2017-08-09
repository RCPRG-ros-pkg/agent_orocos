#!/usr/bin/python
import sys
import copy

import roslib

import gencpp
import genmsg

from  roslib import packages,msgs
import os

from cStringIO import StringIO

import argparse

import parse_subsystem_xml

def logicExprToCpp(expr, predicates):
    cpp = expr

    predicates_copy = copy.copy(predicates)
    predicates_copy.append("IN_ERROR")
    predicates_copy.append("TRUE")
    predicates_copy.append("FALSE")
    predicates_copy.append("CURRENT_BEHAVIOR_OK")

#    for st in states:
#        predicates_copy.append("PREV_STATE_" + st.name)
    predicates_sorted = sorted(predicates_copy, key=lambda pred: -len(pred))

    pred_id_map = {}
    id_pred_map = {}
    count = 0
    for pred in predicates_sorted:
        pred_id = "{" + str(count) + "}"
        pred_id_map[pred] = pred_id
        id_pred_map[pred_id] = pred
        count = count + 1

    # find all predicates
    for pred in predicates_sorted:
        cpp = cpp.replace(pred, pred_id_map[pred])

    # find all 'not'
    idx = 0
    while True:
        idx = cpp.find("not", idx)
        if idx == -1:
            break

        if idx + 3 >= len(cpp):
            raise Exception('invalid expression', 'invalid logic expression syntax, character ' + idx + ", expr: '" + cpp + "'")

        if (idx == 0 or cpp[idx-1].isspace() or cpp[idx-1] == '(') and cpp[idx+3].isspace():
            cpp = cpp[:idx] + '!' + cpp[idx+3:]
            idx = idx + 1
        else:
            idx = idx + 3

    # find all 'and'
    idx = 0
    while True:
        idx = cpp.find("and", idx)
        if idx == -1:
            break

        if idx + 3 >= len(cpp) or idx == 0:
            raise Exception('invalid expression', 'invalid logic expression syntax, character ' + idx + ", expr: '" + cpp + "'")

        if cpp[idx-1].isspace() and cpp[idx+3].isspace():
            cpp = cpp[:idx] + '&&' + cpp[idx+3:]
            idx = idx + 2
        else:
            idx = idx + 3

    # find all 'or'
    idx = 0
    while True:
        idx = cpp.find("or", idx)
        if idx == -1:
            break

        if idx + 2 >= len(cpp) or idx == 0:
            raise Exception('invalid expression', 'invalid logic expression syntax, character ' + idx + ", expr: '" + cpp + "'")

        if cpp[idx-1].isspace() and cpp[idx+2].isspace():
            cpp = cpp[:idx] + '||' + cpp[idx+2:]
            idx = idx + 2
        else:
            idx = idx + 2

    # find all predicates
    for pred in predicates_sorted:
        if pred == 'TRUE':
            cpp = cpp.replace(pred_id_map[pred], "true")
        elif pred == 'FALSE':
            cpp = cpp.replace(pred_id_map[pred], "false")
        else:
            cpp = cpp.replace(pred_id_map[pred], "p->" + pred)

    return cpp

def eprint(s):
    sys.stderr.write(s + '\n')
    sys.stderr.flush()

def generate_boost_serialization(package, port_def, output_cpp):
    """
    Generate a boost::serialization header

    @param msg_path: The path to the .msg file
    @type msg_path: str
    """
    mc = genmsg.msg_loader.MsgContext()

    with open(port_def, 'r') as f:
        read_data = f.read()

    sd = parse_subsystem_xml.parseSubsystemXml(read_data)

    s = StringIO()
    s.write("// autogenerated by rtt_subsystem/create_master_h.py\n")
    s.write("// do not modify this file\n\n")
    s.write("#include <rtt/plugin/ServicePlugin.hpp>\n")
    s.write("#include <rtt/RTT.hpp>\n")
    s.write("#include <rtt/Logger.hpp>\n")
    s.write("#include <ros/param.h>\n")
    s.write("#include <rtt_rosclock/rtt_rosclock.h>\n")


    s.write("#include \"common_behavior/master_service.h\"\n")
    s.write("#include \"" + package + "/master.h\"\n")

    for p_in in sd.buffers_in:
        s.write("#include \"" + p_in.type_pkg + "/" + p_in.type_name + ".h\"\n")

    for p in sd.buffers_in:
        if p.converter:
            s.write("#include <common_interfaces/abstract_buffer_converter.h>\n")
            break

    s.write("#include <shm_comm/shm_channel.h>\n")
    s.write("#include <vector>\n")
    s.write("#include <string>\n")

    s.write("using namespace RTT;\n\n")

    s.write("\nnamespace " + package + "_types {\n\n")

    #
    # _Master
    #
    s.write("class " + package + "_Master : public common_behavior::MasterService {\n")
    s.write("public:\n")

#
# constructor
#
    s.write("    explicit " + package + "_Master(RTT::TaskContext* owner)\n")
    s.write("        : common_behavior::MasterService(owner)\n")
    s.write("        , owner_(owner)\n")
    for p_in in sd.buffers_in:
        s.write("        , " + p_in.alias + "_new_data_(false)\n")
    s.write("    {\n")

    for buf_in in sd.buffers_in:
        s.write("        owner->addProperty(\"channel_name_" + buf_in.alias + "\", channel_name_" + buf_in.alias + "_);\n")


    for p_in in sd.buffers_in:
# TODO: verify this
#        if sd.trigger_methods.onNewData(p_in.alias):
#            s.write("        owner_->addEventPort(\"" + p_in.alias + "_INPORT\", port_" + p_in.alias + "_in_);\n")
#        else:
#            s.write("        owner_->addPort(\"" + p_in.alias + "_INPORT\", port_" + p_in.alias + "_in_);\n")
        s.write("        owner_->addPort(\"" + p_in.alias + "_OUTPORT\", port_" + p_in.alias + "_out_);\n")
#        if sd.trigger_methods.onNoData(p_in.alias):
#            s.write("        owner_->addEventPort(\"" + p_in.alias + "_no_data_INPORT_\", port_" + p_in.alias + "_no_data_trigger_in__);\n")
#        s.write("\n")

    s.write("        bool use_sim_time = false;\n")
    s.write("        ros::param::get(\"/use_sim_time\", use_sim_time);\n")
    s.write("        if (use_sim_time) {\n")
#        s.write("        bool use_sim_time = false;\n")
#        s.write("        ros::param::get(\"/use_sim_time\", use_sim_time);\n")
#        s.write("        if (use_sim_time) {\n")
    if sd.trigger_gazebo:
        s.write("            if (!boost::dynamic_pointer_cast<RTT::internal::GlobalService >(RTT::internal::GlobalService::Instance())->require(\"gazebo_rtt_service\")) {\n")
        s.write("                RTT::Logger::log() << RTT::Logger::Error << \"could not load service 'gazebo_rtt_service'\" << RTT::Logger::endl;\n")
        s.write("            }\n")
        s.write("            else {\n")
        s.write("                RTT::Service::shared_ptr gazebo_rtt_service = RTT::internal::GlobalService::Instance()->getService(\"gazebo_rtt_service\");\n")
        s.write("                RTT::OperationInterfacePart *singleStepOp = gazebo_rtt_service->getOperation(\"singleStep\");\n")
        s.write("                if (singleStepOp == NULL) {\n")
        s.write("                    RTT::Logger::log() << RTT::Logger::Error << \"the service \" << gazebo_rtt_service->getName() << \" has no matching operation singleStep\" << RTT::Logger::endl;\n")
        s.write("                }\n")
        s.write("                else {\n")
        s.write("                    singleStep_ =  RTT::OperationCaller<void()>(singleStepOp);\n")
        s.write("                }\n")
        s.write("            }\n")
    s.write("            rtt_rosclock::disable_sim();\n")

    if sd.use_ros_sim_clock:
        s.write("            rtt_rosclock::use_ros_clock_topic();\n")

#        s.write("            RTT::Service::shared_ptr rosclock = RTT::internal::GlobalService::Instance()->getService(\"ros\")->getService(\"clock\");\n")
#
#        s.write("            RTT::Service::shared_ptr rosclock = RTT::internal::GlobalService::Instance()->getService(\"ros\")->getService(\"clock\");\n")
#        s.write("            if (!rosclock) {\n")
#        s.write("                RTT::Logger::log() << RTT::Logger::Error << \"could not get 'ros.clock' service\" << RTT::Logger::endl;\n")
#        s.write("            }\n")
#        s.write("            else {\n")
#        s.write("                RTT::OperationCaller<void()> useROSClockTopic = rosclock->getOperation(\"useROSClockTopic\");\n")
#        s.write("                if (!useROSClockTopic.ready()) {\n")
#        s.write("                    RTT::Logger::log() << RTT::Logger::Error << \"could not get 'useROSClockTopic' operation of 'ros.clock'\" << RTT::Logger::endl;\n")
#        s.write("                }\n")
#        s.write("                else {\n")
#        s.write("                    useROSClockTopic();\n")
#        s.write("                }\n")
#        s.write("            }\n")
    else:
        s.write("            rtt_rosclock::use_manual_clock();\n")

    s.write("            rtt_rosclock::enable_sim();\n")

    s.write("        }\n")

    for pred in sd.predicates:
        s.write("        pred_" + pred + " = PredicateFactory::Instance()->getPredicate(\"" + package + "_types::" + pred + "\");\n")

    for st in sd.states:
        state_id = sd.getStateId(st.name)
        s.write("        state_buffer_group_[" + str(state_id) + "].min_period = " + str(st.buffer_group_min_period) + ";\n")
        s.write("        state_buffer_group_[" + str(state_id) + "].first_timeout = " + str(st.buffer_group_first_timeout) + ";\n")
        s.write("        state_buffer_group_[" + str(state_id) + "].next_timeout = " + str(st.buffer_group_next_timeout) + ";\n")
        s.write("        state_buffer_group_[" + str(state_id) + "].first_timeout_sim = " + str(st.buffer_group_first_timeout_sim) + ";\n")
        s.write("        state_buffer_group_[" + str(state_id) + "].id = " + str(sd.getBufferGroupId(st.buffer_group_name)) + ";\n")
        #st.buffer_group_name ?

        running_components = set()
        for pb in st.behaviors:
            for b in sd.behaviors:
                if pb == b.name:
                    for rc in b.running_components:
                        running_components.add(rc)
                    break

        for rc in running_components:
            s.write("        state_running_components_[" + str(state_id) + "].push_back(\"" + rc + "\");\n")

    s.write("    }\n\n")


#
# destructor
#

    s.write("    virtual ~" + package + "_Master() {\n")
    s.write("    }\n\n")

#
# configureBuffers()
#
    s.write("    bool configureBuffers() {\n")
    s.write("        Logger::In in(\"" + package + "::configureBuffers\");\n")
    s.write("        int result = 0;\n")
    s.write("        void *pbuf = NULL;\n")
    for buf_in in sd.buffers_in:
        buf_name = buf_in.alias
        if buf_in.converter:
            s.write("        converter_" + buf_name + "_ = common_interfaces::BufferConverterFactory<" + buf_in.getTypeCpp() + " >::Instance()->Create(\"" + buf_in.converter + "\");\n")
            s.write("        if (!converter_" + buf_name + "_) {\n")
            s.write("            Logger::log() << Logger::Error << \"could not find buffer data converter '" + buf_in.converter + "'\" << Logger::endl;\n")
            s.write("            return false;\n")
            s.write("        }\n")
            s.write("        Logger::log() << Logger::Info << \"using data converter '" + buf_in.converter + "' for buffer '" + buf_name + "'\" << Logger::endl;\n")

        s.write("        if (channel_name_" + buf_name + "_.empty()) {\n")
        s.write("            Logger::log() << Logger::Error << \"parameter \'channel_name_" + buf_name + "\' is empty\" << Logger::endl;\n")
        s.write("            return false;\n")
        s.write("        }\n")
        s.write("        Logger::log() << Logger::Info << \"using channel name '\" << channel_name_" + buf_name + "_ << \"' for buffer '" + buf_name + "'\" << Logger::endl;\n")

        s.write("        bool create_channel_" + buf_name + " = false;\n")

        s.write("        Logger::log() << Logger::Info << \"trying to connect to channel\" << Logger::endl;\n")
        s.write("        result = shm_connect_reader(channel_name_" + buf_name + "_.c_str(), &re_" + buf_name + "_);\n")
        s.write("        if (result == SHM_INVAL) {\n")
        s.write("            Logger::log() << Logger::Error << \"shm_connect_reader('" + buf_name + "'): invalid parameters\" << Logger::endl;\n")
        s.write("            return false;\n")
        s.write("        }\n")
        s.write("        else if (result == SHM_FATAL) {\n")
        s.write("            Logger::log() << Logger::Error << \"shm_connect_reader('" + buf_name + "'): memory error\" << Logger::endl;\n")
        s.write("            return false;\n")
        s.write("        }\n")
        s.write("        else if (result == SHM_NO_CHANNEL) {\n")
        s.write("            Logger::log() << Logger::Warning << \"shm_connect_reader('" + buf_name + "'): could not open shm object, trying to initialize the channel...\" << Logger::endl;\n")
        s.write("            create_channel_" + buf_name + " = true;\n")
        s.write("        }\n")
        s.write("        else if (result == SHM_CHANNEL_INCONSISTENT) {\n")
        s.write("            Logger::log() << Logger::Warning << \"shm_connect_reader('" + buf_name + "'): shm channel is inconsistent, trying to initialize the channel...\" << Logger::endl;\n")
        s.write("            create_channel_" + buf_name + " = true;\n")
        s.write("        }\n")
        s.write("        else if (result == SHM_ERR_INIT) {\n")
        s.write("            Logger::log() << Logger::Error << \"shm_connect_reader('" + buf_name + "'): could not initialize channel\" << Logger::endl;\n")
        s.write("            return false;\n")
        s.write("        }\n")
        s.write("        else if (result == SHM_ERR_CREATE) {\n")
        s.write("            Logger::log() << Logger::Warning << \"shm_connect_reader('" + buf_name + "'): could not create reader\" << Logger::endl;\n")
        s.write("            create_channel_" + buf_name + " = true;\n")
        s.write("        }\n")

        s.write("        if (!create_channel_" + buf_name + ") {\n")
        s.write("            Logger::log() << Logger::Info << \"trying to read from channel\" << Logger::endl;\n")
        s.write("            void *pbuf = NULL;\n")
        s.write("            result = shm_reader_buffer_get(re_" + buf_name + "_, &pbuf);\n")
        s.write("            if (result < 0) {\n")
        s.write("                Logger::log() << Logger::Warning << \"shm_reader_buffer_get('" + buf_name + "'): error: \" << result << Logger::endl;\n")
        s.write("                create_channel_" + buf_name + " = true;\n")
        s.write("            }\n")
        s.write("        }\n")

        s.write("        if (create_channel_" + buf_name + ") {\n")
        s.write("            size_t data_size;\n")
        if buf_in.converter:
            s.write("            data_size = converter_" + buf_name + "_->getDataSize();\n")
        else:
            s.write("            data_size = sizeof(" + buf_in.getTypeCpp() + ");\n")
        s.write("            Logger::log() << Logger::Info << \"trying to create channel\" << Logger::endl;\n")
        s.write("            result = shm_create_channel(channel_name_" + buf_name + "_.c_str(), data_size, 1, true);\n")
        s.write("            if (result != 0) {\n")
        s.write("                Logger::log() << Logger::Error << \"create_shm_object('" + buf_name + "'): error: \" << result << \"   errno: \" << errno << Logger::endl;\n")
        s.write("                return false;\n")
        s.write("            }\n")

        s.write("            Logger::log() << Logger::Info << \"trying to connect to channel\" << Logger::endl;\n")
        s.write("            result = shm_connect_reader(channel_name_" + buf_name + "_.c_str(), &re_" + buf_name + "_);\n")
        s.write("            if (result != 0) {\n")
        s.write("                Logger::log() << Logger::Error << \"shm_connect_reader('" + buf_name + "'): error: \" << result << Logger::endl;\n")
        s.write("                return false;\n")
        s.write("            }\n")
        s.write("        }\n")

        s.write("        result = shm_reader_buffer_get(re_" + buf_name + "_, &pbuf);\n")
        s.write("        if (result < 0) {\n")
        s.write("            Logger::log() << Logger::Error << \"shm_reader_buffer_get('" + buf_name + "'): error: \" << result << Logger::endl;\n")
        s.write("            return false;\n")
        s.write("        }\n")
        s.write("        buf_prev_" + buf_name + "_ = pbuf;\n")

    s.write("        return true;\n")
    s.write("    }\n\n")

#
# cleanupBuffers()
#
    s.write("    void cleanupBuffers() {\n")
    for buf_in in sd.buffers_in:
        s.write("        shm_release_reader(re_" + buf_in.alias + "_);\n")
    s.write("    }\n\n")

    s.write("    void initBuffersData(common_behavior::InputDataPtr& in_data) const {\n")
    s.write("        boost::shared_ptr<InputData > in = boost::static_pointer_cast<InputData >(in_data);\n")
    for p_in in sd.buffers_in:
        s.write("        in->" + p_in.alias + " = " + p_in.getTypeCpp() + "();\n")
    s.write("    }\n\n")

#
# getBuffers()
#
    s.write("    void getBuffers(common_behavior::InputDataPtr& in_data) {\n")
    s.write("        boost::shared_ptr<InputData > in = boost::static_pointer_cast<InputData >(in_data);\n")
    for p_in in sd.buffers_in:
        s.write("        in->" + p_in.alias + " = " + p_in.alias + "_data_;\n")
        s.write("        in->" + p_in.alias + "_valid = " + p_in.alias + "_new_data_;\n")
        s.write("        " + p_in.alias + "_data_ = " + p_in.getTypeCpp() + "();\n")
        s.write("        " + p_in.alias + "_new_data_ = false;\n")
        s.write("\n")
    s.write("    }\n\n")

#
# writePorts()
#
    s.write("    void writePorts(common_behavior::InputDataPtr& in_data) {\n")
    s.write("        boost::shared_ptr<InputData> in = boost::static_pointer_cast<InputData >(in_data);\n")
    for p_in in sd.buffers_in:
        s.write("        if (in->" + p_in.alias + "_valid) {\n")
        s.write("            port_" + p_in.alias + "_out_.write(in->" + p_in.alias + ");\n")
        s.write("        }\n")
    s.write("    }\n\n")

#
# getDataSample()
#
    s.write("    common_behavior::InputDataPtr getDataSample() const {\n")
    s.write("        boost::shared_ptr<InputData > ptr(new InputData());\n")
    for p_in in sd.buffers_in:
        s.write("        ptr->" + p_in.alias + " = " + p_in.getTypeCpp() + "();\n")
    s.write("        return boost::static_pointer_cast<common_behavior::InputData >( ptr );\n")
    s.write("    }\n\n")

#
# getLowerInputBuffers()
#
    s.write("    void getLowerInputBuffers(std::vector<common_behavior::InputBufferInfo >& info) const {\n")
    s.write("        info = std::vector<common_behavior::InputBufferInfo >();\n")
    for p in sd.buffers_in:
        if p.side == 'bottom':
            s.write("        info.push_back(common_behavior::InputBufferInfo(\"" + p.getTypeStr() + "\", \"" + p.alias + "\"));\n")
    s.write("    }\n\n")

#
# getUpperInputBuffers()
#
    s.write("    void getUpperInputBuffers(std::vector<common_behavior::InputBufferInfo >& info) const {\n")
    s.write("        info = std::vector<common_behavior::InputBufferInfo >();\n")

    for p in sd.buffers_in:
        if p.side == 'top':
            s.write("        info.push_back(common_behavior::InputBufferInfo(\"" + p.getTypeStr() + "\", \"" + p.alias + "\"));\n")
    s.write("    }\n\n")

#
# getLowerOutputBuffers()
#
    s.write("    void getLowerOutputBuffers(std::vector<common_behavior::OutputBufferInfo >& info) const {\n")
    s.write("        info = std::vector<common_behavior::OutputBufferInfo >();\n")
    for p in sd.buffers_out:
        if p.side == 'bottom':
            s.write("        info.push_back(common_behavior::OutputBufferInfo(\"" + p.getTypeStr() + "\", \"" + p.alias + "\"));\n")
    s.write("    }\n\n")

#
# getUpperOutputBuffers()
#
    s.write("    void getUpperOutputBuffers(std::vector<common_behavior::OutputBufferInfo >& info) const {\n")
    s.write("        info = std::vector<common_behavior::OutputBufferInfo >();\n")
    for p in sd.buffers_out:
        if p.side == 'top':
            s.write("        info.push_back(common_behavior::OutputBufferInfo(\"" + p.getTypeStr() + "\", \"" + p.alias + "\"));\n")
    s.write("    }\n\n")

#
# getBehaviors()
#
    s.write("    std::vector<std::string > getBehaviors() const {\n")
    
    s.write("        return std::vector<std::string >({\n")
    for b in sd.behaviors:
        s.write("                   \"" + b.name + "\",\n")
    s.write("                   });\n")
    s.write("    }\n\n")

#
# getStateName()
#
    s.write("    const std::string& getStateName(int state_id) const {\n")
    s.write("        static const std::vector<std::string > states({\n")
    for st in sd.states:
        s.write("                   \"" + st.name + "\",\n")
    s.write("                   });\n")
    s.write("        return states[state_id];\n")
    s.write("    }\n\n")

#
# getStatesCount()
#
    s.write("    int getStatesCount() const {\n")
    s.write("        return " + str(len(sd.states)) + ";\n")
    s.write("    }\n\n")

#
# getInitialState()
#
    s.write("    std::string getInitialState() const {\n")
    s.write("        static const std::string initial_state(\"" + sd.getInitialStateName() + "\");\n")
    s.write("        return initial_state;\n")
    s.write("    }\n\n")

#
# allocatePredicateList()
#
    s.write("    common_behavior::PredicateListPtr allocatePredicateList() {\n")
    for pred in sd.predicates:
        s.write("        if (!pred_" + pred + ") {\n")
        s.write("            return NULL;\n")
        s.write("        }\n")
    s.write("        PredicateListPtr ptr(new PredicateList());\n")
    s.write("        return boost::static_pointer_cast<common_behavior::PredicateList >( ptr );\n")
    s.write("    }\n\n")

#
# calculatePredicates()
#
    s.write("    void calculatePredicates(const common_behavior::InputDataConstPtr& in_data, const std::vector<const RTT::TaskContext*>& components, common_behavior::PredicateListPtr& pred_list) const {\n")
    s.write("        PredicateListPtr p = boost::static_pointer_cast<PredicateList >( pred_list );\n")
    s.write("        InputDataConstPtr d = boost::static_pointer_cast<const InputData >( in_data );\n")
    for pred in sd.predicates:
        s.write("        p->" + pred + " = pred_" + pred + "(d, components);\n")
    s.write("        p->IN_ERROR = false;\n")
    s.write("        p->CURRENT_BEHAVIOR_OK = false;\n")
    s.write("    }\n\n")

#
# getPredicatesStr()
#
    s.write("    std::string getPredicatesStr(const common_behavior::PredicateListConstPtr& pred_list) const {\n")
    s.write("        PredicateListConstPtr p = boost::static_pointer_cast<const PredicateList >( pred_list );\n")
    s.write("        std::string r;\n")
    for pred in sd.predicates:
        s.write("        r = r + \"" + pred + ":\" + (p->" + pred + "?\"t\":\"f\") + \", \";\n")
    s.write("        r = r + \"IN_ERROR:\" + (p->IN_ERROR?\"t\":\"f\") + \", \";\n")
    s.write("        r = r + \"CURRENT_BEHAVIOR_OK:\" + (p->CURRENT_BEHAVIOR_OK?\"t\":\"f\") + \", \";\n")
    s.write("        return r;\n")
    s.write("    }\n\n")

#
# iterationEnd()
#
    s.write("    void iterationEnd() {\n")
    if sd.trigger_gazebo:
        s.write("        singleStep_();\n")
    else:
        s.write("        // do nothing\n")
    s.write("    }\n\n")


#
# bufferGroupRead()
#
    s.write("    bool bufferGroupRead(size_t id, double timeout) {\n")
    for buffer_group in sd.buffer_groups:
        s.write("        if (id == " + str(sd.getBufferGroupId(buffer_group.name)) + ") {\n")
        s.write("            void *pbuf = NULL;\n")
        s.write("            bool all_obligatory_reads_successful = true;\n")
        s.write("            double timeout_base_s, timeout_s;\n")
        s.write("            timeout_base_s = timeout;\n")
        s.write("            int read_status;\n")
        s.write("            timespec ts;\n")

        for buf_name in buffer_group.obligatory:
            buf_in = None
            for b in sd.buffers_in:
                if b.alias == buf_name:
                    buf_in = b
                    break
            if not buf_in:
                raise Exception('buffer not found', 'buffer specified in trigger methods <read_data>: \'' + buf_name + '\' could not be found in defined buffers')

            s.write("            timeout_s = timeout_base_s;// - (rtt_rosclock::rtt_wall_now() - time_last_s_).toSec();\n")

            s.write("            read_status = -1;\n")
            s.write("            if (timeout_s > 0) {\n")
            s.write("                clock_gettime(CLOCK_REALTIME, &ts);\n")
            s.write("                int timeout_sec = (int)timeout_s;\n")
            s.write("                int timeout_nsec = (int)((timeout_s - (double)timeout_sec) * 1000000000.0);\n")

            s.write("                ts.tv_sec += timeout_sec;\n")
            s.write("                ts.tv_nsec += timeout_nsec;\n")
            s.write("                if (ts.tv_nsec >= 1000000000) {\n")
            s.write("                    ts.tv_nsec -= 1000000000;\n")
            s.write("                    ++ts.tv_sec;\n")
            s.write("                }\n")

            s.write("                read_status = shm_reader_buffer_timedwait(re_" + buf_name + "_, &ts, &pbuf);\n")
            s.write("            }\n")
            s.write("            else {\n")
            s.write("                read_status = shm_reader_buffer_get(re_" + buf_name + "_, &pbuf);\n")
            s.write("            }\n")

            s.write("            if (read_status == SHM_TIMEOUT) {\n")
            s.write("                all_obligatory_reads_successful = false;\n")
#            s.write("                buf." + buf_name + "_valid = false;\n")
            s.write("            }\n")
            s.write("            else if (read_status == 0 && pbuf != buf_prev_" + buf_name + "_) {\n")
            s.write("                buf_prev_" + buf_name + "_ = pbuf;\n")
            if buf_in.converter:
                s.write("                converter_" + buf_name + "_->convertToMsg(reinterpret_cast<const uint8_t* >(pbuf), " + buf_name + "_data_);\n")
            else:
                s.write("                " + buf_name + "_data_ = *reinterpret_cast<" + buf_in.type_pkg + "::" + buf_in.type_name + "* >(pbuf);\n")
            s.write("                " + buf_name + "_new_data_ = true;\n")
            s.write("            }\n")
            s.write("            else if (read_status > 0) {\n")
            s.write("                all_obligatory_reads_successful = false;\n")
#            s.write("                buf." + buf_name + "_valid = false;\n")
            s.write("            }\n")
            s.write("            else {\n")
            s.write("                all_obligatory_reads_successful = false;\n")
#            s.write("                buf." + buf_name + "_valid = false;\n")
            s.write("                Logger::log() << Logger::Error << getName() << \" shm_reader_buffer_timedwait('" + buf_name + "') status: \" << read_status << Logger::endl;\n")
            s.write("                owner_->error();\n")
            s.write("                return false;\n")
            s.write("            }\n")


        for buf_name in buffer_group.optional:
            buf_in = None
            for b in sd.buffers_in:
                if b.alias == buf_name:
                    buf_in = b
                    break
            if not buf_in:
                raise Exception('buffer not found', 'buffer specified in trigger methods <read_data>: \'' + buf_name + '\' could not be found in defined buffers')

            s.write("            read_status = shm_reader_buffer_get(re_" + buf_name + "_, &pbuf);\n")

            s.write("            if (read_status == 0 && pbuf != buf_prev_" + buf_name + "_) {\n")
            s.write("                buf_prev_" + buf_name + "_ = pbuf;\n")
            if buf_in.converter:
                s.write("                converter_" + buf_name + "_->convertToMsg(reinterpret_cast<const uint8_t* >(pbuf), " + buf_name + "_data_);\n")
            else:
                s.write("                " + buf_name + "_data_ = *reinterpret_cast<" + buf_in.type_pkg + "::" + buf_in.type_name + "* >(pbuf);\n")
            s.write("                " + buf_name + "_new_data_ = true;\n")
            s.write("            }\n")
            s.write("            else if (read_status > 0) {\n")
#            s.write("                buf." + buf_name + "_new_data_ = false;\n")
            s.write("            }\n")
            s.write("            else {\n")
#            s.write("                buf." + buf_name + "_valid = false;\n")
            s.write("                Logger::log() << Logger::Error << getName() << \" shm_reader_buffer_timedwait('" + buf_name + "') status: \" << read_status << Logger::endl;\n")
            s.write("                owner_->error();\n")
            s.write("                return false;\n")
            s.write("            }\n")
        s.write("            return all_obligatory_reads_successful;\n")

#        s.write("            last_read_successful_ = all_obligatory_reads_successful;\n")
#        s.write("            port_msg_out_.write(buf);\n")
#        s.write("            timeout_s = " + str(rb.min_period) + " - (rtt_rosclock::rtt_wall_now() - time_last_s_).toSec();  // calculate sleep time\n")
#        s.write("            if (timeout_s > 0) {\n")
#        s.write("                timespec ts2;\n")
#        s.write("                ts2.tv_sec = 0;\n")
#        s.write("                ts2.tv_nsec = static_cast<int >(timeout_s*1000000000.0);\n")
#        s.write("                nanosleep( &ts2, NULL );\n")
#        s.write("            }\n")
#        s.write("            time_last_s_ = rtt_rosclock::rtt_wall_now();     // save last write time\n")
        s.write("        }\n")
    s.write("        owner_->error();\n")
    s.write("        return false;\n")
    s.write("    }\n\n")

#
# checkErrorCondition()
#
    s.write("    bool checkErrorCondition(int state_id, const common_behavior::PredicateListConstPtr& pred_list) const {\n")
    s.write("        PredicateListConstPtr p = boost::static_pointer_cast<const PredicateList >( pred_list );\n")
    s.write("        switch (state_id) {\n")
    for st in sd.states:
        state_id = sd.getStateId(st.name)
        predicate_string = ""
        separator = ""
        for pb in st.behaviors:
            for b in sd.behaviors:
                if pb == b.name:
                    predicate_string += separator + logicExprToCpp(b.err_cond, sd.predicates)
                    separator = " || "
                    break
        s.write("            case " + str(state_id) + ":\n")
        s.write("                return " + predicate_string + ";\n")
    s.write("        }\n")
    s.write("        Logger::log() << Logger::Error << getName() << \"wrong state id: \" << state_id << Logger::endl;\n")
    s.write("        owner_->error();\n")
    s.write("        return true;\n")
    s.write("    }\n\n")

#
# checkStopCondition()
#
    s.write("    bool checkStopCondition(int state_id, const common_behavior::PredicateListConstPtr& pred_list) const {\n")
    s.write("        PredicateListConstPtr p = boost::static_pointer_cast<const PredicateList >( pred_list );\n")
    s.write("        switch (state_id) {\n")
    for st in sd.states:
        state_id = sd.getStateId(st.name)
        predicate_string = ""
        separator = ""
        for pb in st.behaviors:
            for b in sd.behaviors:
                if pb == b.name:
                    predicate_string += separator + logicExprToCpp(b.stop_cond, sd.predicates)
                    separator = " || "
                    break
        s.write("            case " + str(state_id) + ":\n")
        s.write("                return " + predicate_string + ";\n")
    s.write("        }\n")
    s.write("        Logger::log() << Logger::Error << getName() << \"wrong state id: \" << state_id << Logger::endl;\n")
    s.write("        owner_->error();\n")
    s.write("        return true;\n")
    s.write("    }\n\n")

#
# getNextState()
#
    s.write("    int getNextState(int state_id, const common_behavior::PredicateListConstPtr& pred_list) const {\n")
    s.write("        PredicateListConstPtr p = boost::static_pointer_cast<const PredicateList >( pred_list );\n")
    s.write("        switch (state_id) {\n")
    for st in sd.states:
        state_id = sd.getStateId(st.name)
        s.write("            case " + str(state_id) + ":\n")
        s.write("              {\n")
        s.write("                int conditions_true_count = 0;\n")
        for i in range(len(st.next_states)):
            ns_cond = st.next_states[i][1]
            s.write("                bool next_state_pred_val_" + str(i) + " = (" + logicExprToCpp(ns_cond, sd.predicates) + ");\n")
            s.write("                conditions_true_count += (next_state_pred_val_" + str(i) + "?1:0);\n")

        s.write("                if (conditions_true_count > 1) {\n")
        s.write("                    return -1;\n")
        s.write("                }\n")

        for i in range(len(st.next_states)):
            s.write("                if (next_state_pred_val_" + str(i) + ") {\n")
            s.write("                    return " + str(sd.getStateId(st.next_states[i][0])) + ";\n")
            s.write("                }\n")

        s.write("                return -2;\n")
        s.write("              }\n")
    s.write("        }\n")
    s.write("        return -3;\n")
    s.write("    }\n\n")

#
# getStateBufferGroup()
#
    s.write("    const common_behavior::BufferGroup& getStateBufferGroup(int state_id) const {\n")
    s.write("        return state_buffer_group_[state_id];\n")
    s.write("    }\n\n")

#
# getRunningComponentsInState()
#
    s.write("    const std::vector<std::string >& getRunningComponentsInState(int state_id) const {\n")
    s.write("        return state_running_components_[state_id];\n")
    s.write("    }\n\n")

    s.write("protected:\n")
    for p in sd.buffers_in:
#        s.write("    RTT::InputPort<" + p.getTypeCpp() + " > port_" + p.alias + "_in_;\n")
        s.write("    RTT::OutputPort<" + p.getTypeCpp() + " > port_" + p.alias + "_out_;\n")
#        if sd.trigger_methods.onNoData(p.alias):
#            s.write("    RTT::InputPort<bool > port_" + p.alias + "_no_data_trigger_in__;\n")
        s.write("    bool " + p.alias + "_new_data_;\n")
        s.write("    " + p.getTypeCpp() + " " + p.alias + "_data_;\n")

    s.write("\n    RTT::TaskContext* owner_;\n")
    if sd.trigger_gazebo:
        s.write("    RTT::OperationCaller<void()> singleStep_;\n")

    for pred in sd.predicates:
        s.write("    predicateFunction pred_" + pred + ";\n")

    for buf_in in sd.buffers_in:
        buf_name = buf_in.alias
        s.write("    std::string channel_name_" + buf_name + "_;\n")
        s.write("    shm_reader_t* re_" + buf_name + "_;\n")
        s.write("    void *buf_prev_" + buf_name + "_;\n")

        if buf_in.converter:
            s.write("    std::shared_ptr<common_interfaces::BufferConverter<" + buf_in.getTypeCpp() + " > >converter_" + buf_name + "_;\n")

    s.write("    std::array<common_behavior::BufferGroup, " + str(len(sd.states)) + " > state_buffer_group_;\n")
    s.write("    std::array<std::vector<std::string >, " + str(len(sd.states)) + " > state_running_components_;\n")

    s.write("};\n\n")

    s.write("common_behavior::PredicateList& PredicateList::operator=(const common_behavior::PredicateList& arg) {\n")
    s.write("    const PredicateList* p = static_cast<const PredicateList* >(&arg);\n")
    for pred in sd.predicates:
        s.write("    " + pred + " = p->" + pred + ";\n")
    s.write("    IN_ERROR = p->IN_ERROR;\n")
    s.write("    CURRENT_BEHAVIOR_OK = p->CURRENT_BEHAVIOR_OK;\n")
    s.write("    return *static_cast<common_behavior::PredicateList* >(this);\n")
    s.write("};\n\n")



    s.write("};  // namespace " + package + "_types\n\n")

    s.write("ORO_SERVICE_NAMED_PLUGIN(" + package + "_types::" + package + "_Master, \"" + package + "_master\");\n")


    (output_dir,filename) = os.path.split(output_cpp)
    try:
        os.makedirs(output_dir)
    except OSError, e:
        pass

    f = open(output_cpp, 'w')
    print >> f, s.getvalue()

    s.close()


def create_boost_headers(argv, stdout, stderr):
    parser = argparse.ArgumentParser(description='Generate boost serialization header for ROS message.')
    parser.add_argument('pkg',metavar='PKG',type=str, nargs=1,help='The package name.')
    parser.add_argument('port_def',metavar='PORT_DEF',type=str, nargs=1,help='Port definition file.')
    parser.add_argument('output_cpp',metavar='OUTPUT_CPP',type=str, nargs=1,help='Output cpp file.')

    args = parser.parse_args()

    print args.pkg[0], args.port_def[0], args.output_cpp[0]

    generate_boost_serialization(args.pkg[0], args.port_def[0], args.output_cpp[0])

if __name__ == "__main__":
    try:
        create_boost_headers(sys.argv, sys.stdout, sys.stderr)
    except Exception, e:
        sys.stderr.write("Failed to generate boost headers: " + str(e))
        raise


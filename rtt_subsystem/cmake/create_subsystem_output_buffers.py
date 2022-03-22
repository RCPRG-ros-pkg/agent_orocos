#!/usr/bin/python
import sys

import roslib

import gencpp
import genmsg

from  roslib import packages,msgs
import os

from cStringIO import StringIO

import argparse

import parse_subsystem_xml

def getShmCodeInclude(shm_version):
    if shm_version == 1:
        result = "#include <shm_comm/shm_channel.h>\n"
    elif shm_version == 2:
        result = "#include <shm_comm/shm2_channel.h>\n"
    else:
        raise Exception('Wrong version of shm: "{}"'.format(shm_version))
    return result

def getShmCodeWriterConfigure(indent, shm_version, buf):
    if shm_version == 1:
        result = "        result = shm_connect_writer(shm_name_" + buf.alias + "_.c_str(), &wr_" + buf.alias + "_);\n"
        result += "        if (result == SHM_INVAL) {\n"
        result += "            Logger::log() << Logger::Error << \"shm_connect_writer(" + buf.alias + "): invalid parameters\" << Logger::endl;\n"
        result += "            return false;\n"
        result += "        }\n"
        result += "        else if (result == SHM_FATAL) {\n"
        result += "            Logger::log() << Logger::Error << \"shm_connect_writer(" + buf.alias + "): memory error\" << Logger::endl;\n"
        result += "            return false;\n"
        result += "        }\n"
        result += "        else if (result == SHM_NO_CHANNEL) {\n"
        result += "            Logger::log() << Logger::Warning << \"shm_connect_writer(" + buf.alias + "): could not open shm object, trying to initialize the channel...\" << Logger::endl;\n"
        result += "            create_channel = true;\n"
        result += "        }\n"
        result += "        else if (result == SHM_CHANNEL_INCONSISTENT) {\n"
        result += "            Logger::log() << Logger::Warning << \"shm_connect_writer(" + buf.alias + "): shm channel is inconsistent, trying to initialize the channel...\" << Logger::endl;\n"
        result += "            create_channel = true;\n"
        result += "        }\n"
        result += "        else if (result == SHM_ERR_INIT) {\n"
        result += "            Logger::log() << Logger::Error << \"shm_connect_writer(" + buf.alias + "): could not initialize channel\" << Logger::endl;\n"
        result += "            return false;\n"
        result += "        }\n"
        result += "        else if (result == SHM_ERR_CREATE) {\n"
        result += "            Logger::log() << Logger::Error << \"shm_connect_writer(" + buf.alias + "): could not create reader\" << Logger::endl;\n"
        result += "            return false;\n"
        result += "        }\n"

        result += "        if (create_channel) {\n"
        if buf.converter:
            result += "            result = shm_create_channel(shm_name_" + buf.alias + "_.c_str(), converter_" + buf.alias + "_->getDataSize(), 1, true);\n"
        else:
            result += "            result = shm_create_channel(shm_name_" + buf.alias + "_.c_str(), sizeof(" + buf.getTypeCpp() + "), 1, true);\n"
        result += "            if (result != 0) {\n"
        result += "                Logger::log() << Logger::Error << \"create_shm_object(" + buf.alias + "): error: \" << result << \"   errno: \" << errno << Logger::endl;\n"
        result += "                return false;\n"
        result += "            }\n"

        result += "            result = shm_connect_writer(shm_name_" + buf.alias + "_.c_str(), &wr_" + buf.alias + "_);\n"
        result += "            if (result != 0) {\n"
        result += "                Logger::log() << Logger::Error << \"shm_connect_writer(" + buf.alias + "): error: \" << result << Logger::endl;\n"
        result += "                return false;\n"
        result += "            }\n"
        result += "        }\n"
    elif shm_version == 2:
        if buf.converter:
            data_size_code = "converter_{}_->getDataSize()".format(buf.alias)
        else:
            data_size_code = "sizeof({})".format(buf.getTypeCpp())
        result = '{}wr_{}_.reset(new ShmWriter(shm_name_{}_.c_str(), {}, 1));\n'.format(indent, buf.alias, buf.alias, data_size_code)
        result += '{}wr_{}_->connect();\n'.format(indent, buf.alias)
    else:
        raise Exception('Wrong version of shm: "{}"'.format(shm_version))
    return result

def getShmCodeWriterCleanup(indent, shm_version, buf):
    if shm_version == 1:
        result = "{}shm_release_writer(wr_{}_);\n".format(indent, buf.alias)
    elif shm_version == 2:
        result = ''
    else:
        raise Exception('Wrong version of shm: "{}"'.format(shm_version))
    return result

def getShmCodeWriterGet(indent, shm_version, buf):
    if shm_version == 1:
        result = "{}get_result = shm_writer_buffer_get(wr_{}_, &buf_{}_);\n".format(indent, buf.alias, buf.alias)
    elif shm_version == 2:
        result = '{}buf_{}_ = wr_{}_->getBuffer();\n'.format(indent, buf.alias, buf.alias)
        result += '{}get_result = ((buf_{}_==NULL)?-1:0);\n'.format(indent, buf.alias)
    else:
        raise Exception('Wrong version of shm: "{}"'.format(shm_version))
    return result

def getShmCodeWriterWrite(indent, shm_version, buf):
    if shm_version == 1:
        result = "{}shm_writer_buffer_write(wr_{}_);\n".format(indent, buf.alias)
    elif shm_version == 2:
        result = "{}wr_{}_->write();\n".format(indent, buf.alias)
    else:
        raise Exception('Wrong version of shm: "{}"'.format(shm_version))
    return result

def getShmCodeWriterDeclaration(indent, shm_version, buf):
    if shm_version == 1:
        result = "{}shm_writer_t* wr_{}_;\n".format(indent, buf.alias)
    elif shm_version == 2:
        result = "{}std::shared_ptr<ShmWriter > wr_{}_;\n".format(indent, buf.alias)
    else:
        raise Exception('Wrong version of shm: "{}"'.format(shm_version))
    return result

def generate_boost_serialization(package, port_def, output_cpp):
    """
    Generate a boost::serialization header

    @param msg_path: The path to the .msg file
    @type msg_path: str
    """
    shm_version = 2

    mc = genmsg.msg_loader.MsgContext()

    with open(port_def, 'r') as f:
        read_data = f.read()

    sd = parse_subsystem_xml.parseSubsystemXml(read_data)

    s = StringIO()
    s.write("// autogenerated by rtt_subsystem/create_subsystem_output_buffers.py\n")
    s.write("// do not modify this file\n\n")

#    s.write("#include \"subsystem_common/input_data.h\"\n")
#    s.write("#include \"subsystem_common/abstract_behavior.h\"\n")
#    s.write("#include \"subsystem_common/abstract_predicate_list.h\"\n\n")

    for p_in in sd.buffers_out:
        s.write("#include \"" + p_in.type_pkg + "/" + p_in.type_name + ".h\"\n")

    for p in sd.buffers_out:
        if p.converter:
            s.write("#include <common_interfaces/abstract_buffer_converter.h>\n")
            break

    s.write( getShmCodeInclude(shm_version) )

    s.write("#include <vector>\n")
    s.write("#include <string>\n")
    s.write("#include <rtt/RTT.hpp>\n")
    s.write("#include <rtt/Logger.hpp>\n")
    s.write("#include <rtt/Component.hpp>\n")
    s.write("#include <rtt_rosclock/rtt_rosclock.h>\n\n")

    s.write("using namespace RTT;\n\n")

    s.write("\nnamespace " + package + "_types {\n\n")


    s.write("class OutputBuffers: public RTT::TaskContext {\n")
    s.write("public:\n")
    s.write("    explicit OutputBuffers(const std::string& name)\n")
    s.write("        : TaskContext(name, PreOperational)\n")
    for p in sd.buffers_out:
        s.write("        , buf_" + p.alias + "_(NULL)\n")
        s.write("        , port_" + p.alias + "_in_(\"" + p.alias + "_INPORT\")\n")

    s.write("    {\n")
    for p in sd.buffers_out:
        s.write("        this->ports()->addPort(port_" + p.alias + "_in_);\n")
        s.write("        addProperty(\"channel_name_" + p.alias + "\", channel_name_" + p.alias + "_);\n")

    s.write("        this->addOperation(\"getDiag\", &OutputBuffers::getDiag, this, RTT::ClientThread);\n")

    s.write("    }\n\n")

    s.write("    // this method in not RT-safe\n")
    s.write("    std::string getDiag() {\n")
    s.write("        std::stringstream ss;\n")
#TODO: diagnostics
    s.write("        return ss.str();\n")
    s.write("    }\n\n")

    s.write("    bool configureHook() {\n")
    s.write("        Logger::In in(\"" + package + "::OutputBuffers::configureHook\");\n")

    s.write("        bool create_channel;\n")
    s.write("        int result;\n")

    for p in sd.buffers_out:
        if p.converter:
            s.write("        converter_" + p.alias + "_ = common_interfaces::BufferConverterFactory<" + p.getTypeCpp() + " >::Instance()->Create(\"" + p.converter + "\");\n")
            s.write("        if (!converter_" + p.alias + "_) {\n")
            s.write("            Logger::log() << Logger::Error << \"could not find buffer data converter '" + p.converter + "'\" << Logger::endl;\n")
            s.write("            return false;\n")
            s.write("        }\n")

        s.write("        if (channel_name_" + p.alias + "_.empty()) {\n")
        s.write("            Logger::log() << Logger::Error << \"channel_name_" + p.alias + " is empty\" << Logger::endl;\n")
        s.write("            return false;\n")
        s.write("        }\n")

        s.write("        shm_name_" + p.alias + "_ = channel_name_" + p.alias + "_;\n")

        s.write("        create_channel = false;\n")

        s.write( getShmCodeWriterConfigure("        ", shm_version, p) )

    s.write("        return true;\n")
    s.write("    }\n\n")

    s.write("    void cleanupHook() {\n")
    for p in sd.buffers_out:
        s.write( getShmCodeWriterCleanup("        ", shm_version, p) )
    s.write("    }\n\n")

    s.write("    void stopHook() {\n")
    s.write("    }\n\n")

    s.write("    bool startHook() {\n")
    s.write("        int get_result;\n")
    for p in sd.buffers_out:
        #s.write("        void *pbuf_" + p.alias + " = NULL;\n")
        s.write( getShmCodeWriterGet("        ", shm_version, p) )
        s.write("        if (get_result < 0) {\n")
        s.write("            Logger::In in(\"" + package + "::OutputBuffers::startHook\");\n")
        s.write("            Logger::log() << Logger::Error << \"shm_writer_buffer_get(" + p.alias + ")\" << Logger::endl;\n")
        s.write("            return false;\n")
        s.write("        }\n")
        #s.write("        buf_" + p.alias + "_ = pbuf_" + p.alias + ";\n")

    s.write("        return true;\n")
    s.write("    }\n\n")

    s.write("    void updateHook() {\n")
    for p in sd.buffers_out:
        s.write("        if (port_" + p.alias + "_in_.read(cmd_out_" + p.alias + "_) == RTT::NewData) {\n")
#        s.write("            diag_buf_ = cmd_out_" + p.alias + "_;\n")
    #    s.write("            diag_buf_valid_ = true;\n")
        s.write("            if (buf_" + p.alias + "_ == NULL) {\n")
        s.write("                Logger::In in(\"" + package + "::OutputBuffers::updateHook\");\n")
        s.write("                Logger::log() << Logger::Error << \"writer get NULL buffer (" + p.alias + ")\" << Logger::endl;\n")
        s.write("                error();\n")
        s.write("            }\n")
        s.write("            else {\n")
        if p.converter:
            s.write("                converter_" + p.alias + "_->convertFromMsg(cmd_out_" + p.alias + "_, reinterpret_cast<uint8_t* >(buf_" + p.alias + "_));\n")
        else:
            s.write("                *reinterpret_cast<" + p.getTypeCpp() + "*>(buf_" + p.alias + "_) = cmd_out_" + p.alias + "_;\n")

        s.write( getShmCodeWriterWrite("                ", shm_version, p) )
        s.write("            }\n")
        #s.write("            void *pbuf = NULL;\n")
        s.write("            int get_result;\n")
        s.write( getShmCodeWriterGet("            ", shm_version, p) )
        #s.write("            shm_writer_buffer_get(wr_" + p.alias + "_, &pbuf);\n")
        #s.write("            buf_" + p.alias + "_ = pbuf;\n")
        s.write("        }\n")
    #    s.write("        else {\n")
    #    s.write("            diag_buf_valid_ = false;\n")
    #    s.write("        }\n")
    s.write("    }\n\n")

    s.write("private:\n")

    s.write("    // properties\n")
    for p in sd.buffers_out:
        s.write("    std::string channel_name_" + p.alias + "_;\n")

    for p in sd.buffers_out:
        s.write("    std::string shm_name_" + p.alias + "_;\n")
        s.write( getShmCodeWriterDeclaration("    ", shm_version, p) )
        s.write("    void *buf_" + p.alias + "_;\n")
        s.write("    " + p.getTypeCpp() + " cmd_out_" + p.alias + "_;\n")
        s.write("    RTT::InputPort<" + p.getTypeCpp() + " > port_" + p.alias + "_in_;\n")
        if p.converter:
            s.write("    std::shared_ptr<common_interfaces::BufferConverter<" + p.getTypeCpp() + " > >converter_" + p.alias + "_;\n")

    s.write("};\n\n")

    s.write("};  // namespace " + package + "_types\n\n")

    s.write("ORO_LIST_COMPONENT_TYPE(" + package + "_types::OutputBuffers)\n")

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
        #sys.exit(1)

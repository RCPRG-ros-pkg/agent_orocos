#!/usr/bin/python3
import sys

import roslib

import gencpp
import genmsg

from  roslib import packages,msgs
import os

from io import StringIO

import argparse

NAME='create_boost_header'

MSG_TYPE_TO_CPP = {'byte': 'int8_t',
                   'char': 'uint8_t',
                   'bool': 'uint8_t',
                   'uint8': 'uint8_t',
                   'int8': 'int8_t',
                   'uint16': 'uint16_t',
                   'int16': 'int16_t',
                   'uint32': 'uint32_t',
                   'int32': 'int32_t',
                   'uint64': 'uint64_t',
                    'int64': 'int64_t',
                   'float32': 'float',
                   'float64': 'double',
                   'string': 'std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > ',
                   'time': 'ros::Time',
                   'duration': 'ros::Duration'}

#used
def msg_type_to_cpp(type):
    """
    Converts a message type (e.g. uint32, std_msgs/String, etc.) into the C++ declaration
    for that type (e.g. uint32_t, std_msgs::String_<ContainerAllocator>)

    @param type: The message type
    @type type: str
    @return: The C++ declaration
    @rtype: str
    """
    (base_type, is_array, array_len) = genmsg.msgs.parse_type(type)
    cpp_type = None
    if (genmsg.msgs.is_builtin(base_type)):
        cpp_type = MSG_TYPE_TO_CPP[base_type]
    elif (len(base_type.split('/')) == 1):
        if (genmsg.msgs.is_header_type(base_type)):
            cpp_type = ' ::std_msgs::Header '
        else:
            cpp_type = '%s '%(base_type)
    else:
        pkg = base_type.split('/')[0]
        msg = base_type.split('/')[1]
        cpp_type = ' ::%s::%s '%(pkg, msg)

    if (is_array):
        if (array_len is None):
            raise
        else:
            return 'boost::array<%s, %s> '%(cpp_type, array_len)
    else:
        return cpp_type

def default_value(type):
    """
    Returns the value to initialize a message member with.  0 for integer types, 0.0 for floating point, false for bool,
    empty string for everything else

    @param type: The type
    @type type: str
    """
    if type in ['byte', 'int8', 'int16', 'int32', 'int64',
                'char', 'uint8', 'uint16', 'uint32', 'uint64']:
        return '0'
    elif type in ['float32', 'float64']:
        return '0.0'
    elif type == 'bool':
        return 'false'

    return ""

def write_boost_includes(s, spec, port_spec_dict):
    """
    Writes the message-specific includes

    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec to iterate over
    @type spec: roslib.msgs.MsgSpec
    @param serializer: The serializer type for which to include headers
    @type serializer: str
    """
    for field in spec.parsed_fields():
        if (not field.is_builtin):
            if field.name in port_spec_dict:
                port_spec = port_spec_dict[field.name]
                if port_spec[0] == 'container':
                    (pkg, name) = genmsg.names.package_resource_name(field.base_type)
                    pkg = (pkg or spec.package) # convert '' to this package
                    s.write('#include <%s/subsystem_buffers/%s.h>\n'%(pkg,  name))

    s.write('\n')

def parse_comment_to_subsystem_buffer_spec(comment):
    id_str = 'subsystem_buffer{'

    pos_beg = comment.find(id_str)
    if pos_beg < 0:
        return None

    pos_end = comment.find('}', pos_beg)
    if pos_end < 0:
        # subsystem_buffer declaration must be complete
        return None

    decl = comment[pos_beg + len(id_str) : pos_end]
    decl_list = decl.split(';')

    decl_dict = {}
    for item in decl_list:
        pos = item.find(':')
        if pos < 0:
            continue
        decl_dict[item[:pos].strip()] = item[pos+1:].strip()

    if not 'type' in decl_dict:
        return None
    type = decl_dict['type']

    validity_field_name = ''
    if 'validity' in decl_dict:
        validity_field_name = decl_dict['validity']

    return (type, validity_field_name)

def get_port_spec_dict(spec):
    port_spec_dict = {}
    # process msg declaration line by line, search for port_spec
    for line in spec.text.splitlines():
        comment_start = line.find('#')
        if comment_start <= 0:
            continue
        declaration = line[:comment_start-1]
        comment = line[comment_start:]
        success = True
        try:
            field_type, name = genmsg.msg_loader._load_field_line(declaration, spec.package)
        except:
            success = False
        if success:
            port_spec = parse_comment_to_subsystem_buffer_spec(comment)
            if port_spec != None:
                port_spec_dict[name] = port_spec

    return port_spec_dict

def write_boost_serialization(s, spec, cpp_name_prefix, file):
    """
    Writes the boost::serialize function for a message

    @param s: Stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    @param cpp_name_prefix: The C++ prefix to prepend to a message to refer to it (e.g. "std_msgs::")
    @type cpp_name_prefix: str
    """
    (cpp_msg_unqualified, cpp_msg_with_alloc, _) = gencpp.cpp_message_declarations(cpp_name_prefix, spec.short_name)

#    port_spec_dict = get_port_spec_dict(spec)

    s.write("/* Auto-generated by create_subsystem_ports_source.py for file %s */\n"%(file))
    s.write('#include <%s/subsystem_buffers/%s.h>\n'%(spec.package,spec.short_name))
    s.write('#include <common_interfaces/interface_ports.h>\n')

    s.write('namespace %s {\n\n'%(spec.package))

    for field in spec.parsed_fields():
        s.write('// name: %s   type: %s\n'%(field.name, msg_type_to_cpp(field.type)))

    fields_with_validity = []
    validity_fields = []
    # find all validity fields
    for field in spec.parsed_fields():
        for field_valid in spec.parsed_fields():
            if field.name + "_valid" == field_valid.name and field_valid.type == "bool":
                fields_with_validity.append(field.name)
                validity_fields.append(field.name + "_valid")

    #
    # input ports
    #
    s.write('%s_InputPorts::%s_InputPorts(RTT::TaskContext *tc, const std::string& prefix) {\n'%(spec.short_name, spec.short_name))

    s.write('  this_container__.reset( new interface_ports::InputPort<%s >(tc, prefix ) );\n'%(spec.short_name))

    # generate member list
    for field in spec.parsed_fields():
        if field.name in validity_fields:
            continue
        if field.is_builtin:
            s.write('  %s_.reset( new interface_ports::InputPort<%s >(tc, prefix + std::string(prefix.empty()?"":"_") + \"%s\") );\n'%(field.name, msg_type_to_cpp(field.type), field.name))
        else:
            s.write('  %s_ = interface_ports::InputPortInterfaceFactory<%s >::Instance()->Create("%s", tc, prefix + std::string(prefix.empty()?"":"_") + \"%s\");\n'%(field.name, msg_type_to_cpp(field.type)[:-1], field.type, field.name))
            s.write('  if (!%s_) {\n'%(field.name))
            s.write('    %s_.reset( new interface_ports::InputPort<%s >(tc, prefix + std::string(prefix.empty()?"":"_") + \"%s\") );\n'%(field.name, msg_type_to_cpp(field.type), field.name))
            s.write('  }\n')
#        if field.name in port_spec_dict:
#            port_spec = port_spec_dict[field.name]
#            if port_spec[0] == 'container':
#                if field.is_builtin:
#                    raise
#                s.write('  %s_ = interface_ports::InputPortInterfaceFactory<%s >::Instance()->Create("%s", tc, prefix + std::string(prefix.empty()?"":"_") + \"%s\");\n'%(field.name, msg_type_to_cpp(field.type)[:-1], field.type, field.name))
#            elif port_spec[0] == 'port':
#                s.write('  %s_.reset( new interface_ports::InputPort<%s >(tc, prefix + std::string(prefix.empty()?"":"_") + \"%s\") );\n'%(field.name, msg_type_to_cpp(field.type), field.name))
    s.write('}\n\n')

    s.write('bool %s_InputPorts::read(%s& ros) {\n'%(spec.short_name, spec.short_name))
    s.write('  if (!(this_container__ && this_container__->read(ros))) {\n')

    s.write('    bool result = true;\n')
    s.write('    //ros = %s();\n'%(spec.short_name))
    for field in spec.parsed_fields():
        if field.name in validity_fields:
            continue
        if field.name in fields_with_validity:
            left_side = 'ros.' + field.name + '_valid = '
        else:
            left_side = 'result &= '
        s.write('    ' + left_side + '%s_ && %s_->read(ros.%s);\n'%(field.name, field.name, field.name))

#        if field.name in port_spec_dict:
#            port_spec = port_spec_dict[field.name]
#            validity_field = port_spec[1]
#            if validity_field:
#                left_side = 'ros.' + validity_field + ' = '
#            else:
#                left_side = 'result &= '
#            s.write('    ' + left_side + '%s_ && %s_->read(ros.%s);\n'%(field.name, field.name, field.name))
    s.write('    if (!result) {\n')
    s.write('      ros = %s();\n'%(spec.short_name))
    s.write('    }\n')

    s.write('    return result;\n')
    s.write('  }\n')
    s.write('  return true;\n')
    s.write('}\n\n')

    s.write('bool %s_InputPorts::removeUnconnectedPorts() {\n'%(spec.short_name))
    s.write('  bool result = true;\n')
    s.write('  if (this_container__->removeUnconnectedPorts()) {\n')
    s.write('    this_container__.reset();\n')
    s.write('  }\n')
    s.write('  else {\n')
    s.write('    result = false;\n')
    s.write('  }\n')
    for field in spec.parsed_fields():
        if field.name in validity_fields:
            continue
        s.write('  if (%s_->removeUnconnectedPorts()) {\n'%(field.name))
        s.write('    %s_.reset();\n'%(field.name))
        s.write('  }\n')
        s.write('  else {\n')
        s.write('    result = false;\n')
        s.write('  }\n')
#        if field.name in port_spec_dict:
#            port_spec = port_spec_dict[field.name]
#            s.write('  if (%s_->removeUnconnectedPorts()) {\n'%(field.name))
#            s.write('    %s_.reset();\n'%(field.name))
#            s.write('  }\n')
#            s.write('  else {\n')
#            s.write('    result = false;\n')
#            s.write('  }\n')
    s.write('  return result;\n')
    s.write('}\n')

    s.write('REGISTER_InputPortInterface( %s_InputPorts, "%s/%s" );\n\n'%(spec.short_name, spec.package, spec.short_name))

    #
    # output ports
    #
    s.write('%s_OutputPorts::%s_OutputPorts(RTT::TaskContext *tc, const std::string& prefix) {\n'%(spec.short_name, spec.short_name))

    s.write('  this_container__.reset( new interface_ports::OutputPort<%s >(tc, prefix ) );\n'%(spec.short_name))
    # generate member list
    for field in spec.parsed_fields():
        if field.name in validity_fields:
            continue
        if field.is_builtin:
            s.write('  %s_.reset( new interface_ports::OutputPort<%s >(tc, prefix + std::string(prefix.empty()?"":"_") + \"%s\") );\n'%(field.name, msg_type_to_cpp(field.type), field.name))
        else:
            s.write('  %s_ = interface_ports::OutputPortInterfaceFactory<%s >::Instance()->Create("%s", tc, prefix + std::string(prefix.empty()?"":"_") + \"%s\");\n'%(field.name, msg_type_to_cpp(field.type)[:-1], field.type, field.name))
            s.write('  if (!%s_) {\n'%(field.name))
            s.write('    %s_.reset( new interface_ports::OutputPort<%s >(tc, prefix + std::string(prefix.empty()?"":"_") + \"%s\") );\n'%(field.name, msg_type_to_cpp(field.type), field.name))
            s.write('  }\n')

#        if field.name in port_spec_dict:
#            port_spec = port_spec_dict[field.name]
#            if port_spec[0] == 'container':
#                if field.is_builtin:
#                    raise Exception('built in type', "field \'" + field.name + "\' is specified as container, but it is built-in type" )
#                s.write('  %s_ = interface_ports::OutputPortInterfaceFactory<%s >::Instance()->Create("%s", tc, prefix + std::string(prefix.empty()?"":"_") + \"%s\");\n'%(field.name, msg_type_to_cpp(field.type)[:-1], field.type, field.name))
#            elif port_spec[0] == 'port':
#                s.write('  %s_.reset( new interface_ports::OutputPort<%s >(tc, prefix + std::string(prefix.empty()?"":"_") + \"%s\") );\n'%(field.name, msg_type_to_cpp(field.type), field.name))
    s.write('}\n\n')

    s.write('bool %s_OutputPorts::write(const %s& ros) {\n'%(spec.short_name, spec.short_name))

    s.write('  this_container__ && this_container__->write(ros);\n')

    for field in spec.parsed_fields():
        if field.name in validity_fields:
            continue
        if field.name in fields_with_validity:
            s.write('  if (ros.%s_valid) {\n  '%(field.name))
        s.write('    %s_ && %s_->write(ros.%s);\n'%(field.name, field.name, field.name))
        if field.name in fields_with_validity:
            s.write('  }\n')

#        if field.name in port_spec_dict:
#            port_spec = port_spec_dict[field.name]
#            validity_field = port_spec[1]
#            if validity_field:
#                s.write('  if (ros.%s) {\n  '%(validity_field))
#            s.write('    %s_ && %s_->write(ros.%s);\n'%(field.name, field.name, field.name))
#            if validity_field:
#                s.write('  }\n')

    s.write('  return true;\n')
    s.write('}\n')

    s.write('bool %s_OutputPorts::removeUnconnectedPorts() {\n'%(spec.short_name))
    s.write('  bool result = true;\n')
    s.write('  if (this_container__->removeUnconnectedPorts()) {\n')
    s.write('    this_container__.reset();\n')
    s.write('  }\n')
    s.write('  else {\n')
    s.write('    result = false;\n')
    s.write('  }\n')
    for field in spec.parsed_fields():
        if field.name in validity_fields:
            continue
        s.write('  if (%s_->removeUnconnectedPorts()) {\n'%(field.name))
        s.write('    %s_.reset();\n'%(field.name))
        s.write('  }\n')
        s.write('  else {\n')
        s.write('    result = false;\n')
        s.write('  }\n')
    s.write('  return result;\n')
    s.write('}\n')

    s.write('REGISTER_OutputPortInterface( %s_OutputPorts, "%s/%s" );\n\n'%(spec.short_name, spec.package, spec.short_name))

    s.write('} // namespace %s\n\n'%(spec.package))



def generate_boost_serialization(package, msg_path, msg_type, boost_header_path):
    """
    Generate a boost::serialization header

    @param msg_path: The path to the .msg file
    @type msg_path: str
    """
    mc = genmsg.msg_loader.MsgContext()

    spec = genmsg.msg_loader.load_msg_from_file(mc, msg_path, msg_type)
    cpp_prefix = '%s::'%(package)

    s = StringIO()
    write_boost_serialization(s, spec, cpp_prefix, msg_path)

    (output_dir,filename) = os.path.split(boost_header_path)
    try:
        os.makedirs(output_dir)
    except OSError:
        pass

    f = open(boost_header_path, 'w')
    f.write(s.getvalue() + '\n')

    s.close()


def create_boost_headers(argv, stdout, stderr):
    parser = argparse.ArgumentParser(description='Generate boost serialization header for ROS message.')
    parser.add_argument('pkg',metavar='PKG',type=str, nargs=1,help='The package name.')
    parser.add_argument('msg_type',metavar='MSG_TYPE',type=str, nargs=1,help='The message type.')
    parser.add_argument('msg_file_path',metavar='MSG_FILE_PATH',type=str, nargs=1,help='The full path to a .msg file.')
    parser.add_argument('boost_file_path',metavar='BOOST_HEADER_PATH',type=str, nargs=1,help='The full path to the generated boost header file.')

    args = parser.parse_args()

    generate_boost_serialization(args.pkg[0], args.msg_file_path[0], args.msg_type[0], args.boost_file_path[0])

if __name__ == "__main__":
    try:
        create_boost_headers(sys.argv, sys.stdout, sys.stderr)
    except Exception as e:
        sys.stderr.write("Failed to generate boost headers: " + str(e))
        raise
        #sys.exit(1)

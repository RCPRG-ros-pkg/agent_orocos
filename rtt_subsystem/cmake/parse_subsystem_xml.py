import xml.dom.minidom as minidom

def str_to_bool(s):
    if s.upper() == 'TRUE':
        return True
    if s.upper() == 'FALSE':
        return False
    raise ValueError("Wrong boolean value: " + s)

def str_to_side(s):
    result = s.lower()
    if result != "top" and result != "bottom":
        raise ValueError("Wrong value of \'side\' attribute: " + s)
    return result

class InputPort:
    def parse(self, xml):
        self.alias = xml.getAttribute("alias")
        type_s = xml.getAttribute("type").split("::")
        if len(type_s) != 2:
            raise Exception('in', 'wrong type attribute of <buffers> <in> tag: ' + type + ', should be \'package::typename\"')
        self.type_pkg = type_s[0]
        self.type_name = type_s[1]
        self.side = str_to_side(xml.getAttribute("side"))
        self.converter = xml.getAttribute("converter")

    def __init__(self, xml=None):
        if xml:
            self.parse(xml)

    def getTypeStr(self):
        return self.type_pkg + "_" + self.type_name

    def getTypeCpp(self):
        return self.type_pkg + "::" + self.type_name

class OutputPort:
    def parse(self, xml):
        self.alias = xml.getAttribute("alias")
        type_s = xml.getAttribute("type").split("::")
        if len(type_s) != 2:
            raise Exception('in', 'wrong type attribute of <buffers> <out> tag: ' + type + ', should be \'package::typename\"')
        self.type_pkg = type_s[0]
        self.type_name = type_s[1]
        self.side = str_to_side(xml.getAttribute("side"))
        self.converter = xml.getAttribute("converter")

    def __init__(self, xml=None):
        if xml:
            self.parse(xml)

    def getTypeStr(self):
        return self.type_pkg + "_" + self.type_name

    def getTypeCpp(self):
        return self.type_pkg + "::" + self.type_name

class SubsystemState:
    def parse(self, xml):
        self.name = xml.getAttribute("name")

        self.behaviors = []
        for b in xml.getElementsByTagName("behavior"):
            self.behaviors.append(b.getAttribute("name"))

        self.next_states = []
        for ns in xml.getElementsByTagName("next_state"):
            self.next_states.append( (ns.getAttribute("name"), ns.getAttribute("init_cond")) )

        buffer_group = xml.getElementsByTagName("buffer_group")
        if len(buffer_group) != 1:
            raise Exception('<state>', 'exactly one <buffer_group> must be specified')

        self.buffer_group_name = buffer_group[0].getAttribute("name")
        if not self.buffer_group_name:
            raise Exception('<state>', '<buffer_group> must have attribute \'name\'')

        if not buffer_group[0].getAttribute("min_period"):
            raise Exception('<state>', '<buffer_group> must have attribute \'min_period\'')
        self.buffer_group_min_period = float(buffer_group[0].getAttribute("min_period"))

        if not buffer_group[0].getAttribute("first_timeout"):
            raise Exception('<state>', '<buffer_group> must have attribute \'first_timeout\'')
        self.buffer_group_first_timeout = float(buffer_group[0].getAttribute("first_timeout"))

        if not buffer_group[0].getAttribute("next_timeout"):
            raise Exception('<state>', '<buffer_group> must have attribute \'next_timeout\'')
        self.buffer_group_next_timeout = float(buffer_group[0].getAttribute("next_timeout"))

        if not buffer_group[0].getAttribute("first_timeout_sim"):
            raise Exception('<state>', '<buffer_group> must have attribute \'first_timeout_sim\'')
        self.buffer_group_first_timeout_sim = float(buffer_group[0].getAttribute("first_timeout_sim"))

    def __init__(self, xml=None):
        if xml:
            self.parse(xml)

class SubsystemBehavior:
    def parse(self, xml):
        self.name = xml.getAttribute("name")
        self.stop_cond = xml.getAttribute("stop_cond")
        self.err_cond = xml.getAttribute("err_cond")

        self.running_components = []
        for rc in xml.getElementsByTagName("running_component"):
            self.running_components.append( rc.getAttribute("name") )

    def __init__(self, xml=None):
        if xml:
            self.parse(xml)

class BufferGroup:
    def __init__(self, xml):
        self.name = xml.getAttribute("name")
        obligatory = xml.getElementsByTagName('obligatory')
        optional = xml.getElementsByTagName('optional')

        if len(obligatory) == 0 and len(optional) == 0:
            raise Exception('<buffer_group>', 'at least one of <obligatory>, <optional> should be specified')

        self.obligatory = []
        for ob in obligatory:
            name = ob.getAttribute("name")
            if not name:
                raise Exception('<obligatory>', 'attribute \'name\' must be specified')
            self.obligatory.append(name)

        self.optional = []
        for op in optional:
            name = op.getAttribute("name")
            if not name:
                raise Exception('<optional>', 'attribute \'name\' must be specified')
            self.optional.append(name)

class SubsystemDefinition:
    def __init__(self):
        self.buffers_in = []
        self.buffers_out = []
        self.buffer_groups = []
        self.predicates = []
        self.states = []
        self.initial_state_name = None
        self.behaviors = []
        self.period = None
        self.use_ros_sim_clock = False
        self.trigger_gazebo = False
        self.use_sim_clock = False

    def getBufferGroupId(self, name):
        group_id = 0
        for gr in self.buffer_groups:
            if gr.name == name:
                return group_id
            group_id += 1
        return None

    def getStateId(self, state_name):
        state_id = 0
        for st in self.states:
            if st.name == state_name:
                return state_id
            state_id += 1
        return None

    def getBehaviorId(self, behavior_name):
        behavior_id = 0
        for b in self.behaviors:
            if b.name == behavior_name:
                return behavior_id
            behavior_id += 1
        return None

    def getInitialStateName(self):
        return self.initial_state_name

    def getBufferGroupId(self, name):
        index = 0
        for buffer_group in self.buffer_groups:
            if buffer_group.name == name:
                return index
            index = index + 1
        raise Exception('<getBufferGroupId>', 'buffer group \'' + buffer_group.name + '\' not found')

    def parseBuffers(self, xml):
        # <in>
        for p_in in xml.getElementsByTagName('in'):
            p = InputPort(p_in)
            self.buffers_in.append(p)

        # <out>
        for p_out in xml.getElementsByTagName('out'):
            p = OutputPort(p_out)
            self.buffers_out.append(p)

    def parseBufferGroups(self, xml):
        # <buffer_group>
        for buffer_group in xml.getElementsByTagName('buffer_group'):
            g = BufferGroup(buffer_group)
            self.buffer_groups.append(g)

    def parsePredicates(self, xml):
        for p in xml.getElementsByTagName("predicate"):
            self.predicates.append( p.getAttribute("name") )

    def parseStates(self, xml):

        self.initial_state_name = xml.getAttribute("initial")
        if not self.initial_state_name:
            raise Exception('states initial', 'attribute \'initial\' in <states> node is not set')

        for s in xml.getElementsByTagName("state"):
            state = SubsystemState(s)
            self.states.append( state )

    def parseBehaviors(self, xml):
        for b in xml.getElementsByTagName("behavior"):
            behavior = SubsystemBehavior(b)
            self.behaviors.append( behavior )

    def parse(self, xml):
        # <buffers>
        buffers = xml.getElementsByTagName("buffers")
        if len(buffers) != 1:
            raise Exception('buffers', 'wrong number of <buffers> tags, should be 1')
        self.parseBuffers(buffers[0])

        # <buffer_groups>
        buffer_groups = xml.getElementsByTagName("buffer_groups")
        if len(buffer_groups) != 1:
            raise Exception('buffer_groups', 'wrong number of <buffer_groups> tags, should be 1')
        self.parseBufferGroups(buffer_groups[0])

        # <predicates>
        predicates = xml.getElementsByTagName("predicates")
        if len(predicates) != 1:
            raise Exception('predicates', 'wrong number of <predicates> tags, should be 1')

        self.parsePredicates(predicates[0])

        # <behaviors>
        behaviors = xml.getElementsByTagName("behaviors")
        if len(behaviors) != 1:
            raise Exception('behaviors', 'wrong number of <behaviors> tags, should be 1')

        self.parseBehaviors(behaviors[0])

        # <states>
        states = xml.getElementsByTagName("states")
        if len(states) != 1:
            raise Exception('states', 'wrong number of <states> tags, should be 1')

        self.parseStates(states[0])

        # sanity checks
        found_initial_state = False
        for state in self.states:
            if state.name == self.initial_state_name:
                found_initial_state = True
            found = False
            for buffer_group in self.buffer_groups:
                if buffer_group.name == state.buffer_group_name:
                    found = True
                    break
            if not found:
                raise Exception('state, buffer_group', 'state \'' + state.name + '\' has wrong buffer group: \'' + state.buffer_group_name + '\'')
        if not found_initial_state:
            raise Exception('initial_state', 'initial_state \'' + self.initial_state_name + '\' not found')

        for buffer_group in self.buffer_groups:
            all_buffers = buffer_group.obligatory + buffer_group.optional
            for buf in all_buffers:
                found = False
                for buffer_in in self.buffers_in:
                    if buffer_in.alias == buf:
                        found = True
                        break
                if not found:
                    raise Exception('buffer_group', 'buffer_group \'' + buffer_group.name + '\' contains non-existing buffer \'' + buf + '\'')

        #
        # <simulation>
        #
        simulation = xml.getElementsByTagName("simulation")
        if len(simulation) == 1:
            use_ros_sim_clock = simulation[0].getAttribute("use_ros_sim_clock")
            if not use_ros_sim_clock:
                raise Exception('use_ros_sim_clock', '<simulation> tag must contain \'use_ros_sim_clock\' attribute')
            self.use_ros_sim_clock = str_to_bool(use_ros_sim_clock)

            use_sim_clock = simulation[0].getAttribute("use_sim_clock")
            if not use_sim_clock:
                raise Exception('use_sim_clock', '<simulation> tag must contain \'use_sim_clock\' attribute')
            self.use_sim_clock = str_to_bool(use_sim_clock)

            trigger_gazebo = simulation[0].getAttribute("trigger_gazebo")
            if trigger_gazebo:
                self.trigger_gazebo = str_to_bool(trigger_gazebo)
        elif len(simulation) > 1:
            raise Exception('simulation', 'wrong number of <simulation> tags, must be 0 or 1')

def parseSubsystemXml(xml_str):
    dom = minidom.parseString(xml_str)
    subsystem_definition = dom.getElementsByTagName("subsystem_definition")
    if len(subsystem_definition) != 1:
        raise Exception('subsystem_definition', 'subsystem_definition is missing')

    sd = SubsystemDefinition()
    sd.parse(subsystem_definition[0])

    return sd


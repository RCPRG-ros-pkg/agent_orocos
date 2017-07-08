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

class Trigger(object):
    def __init__(self, trigger_type):
        self.trigger_type = trigger_type

class TriggerOnNewData(Trigger):
    def __init__(self, xml):
        super(TriggerOnNewData, self).__init__("new_data_on_buffer")
        self.name = xml.getAttribute("name")
        self.min = float(xml.getAttribute("min"))

class TriggerOnNoData(Trigger):
    def __init__(self, xml):
        super(TriggerOnNoData, self).__init__("no_data_on_buffer")
        self.name = xml.getAttribute("name")
        self.first_timeout = float(xml.getAttribute("first_timeout"))
        self.next_timeout = float(xml.getAttribute("next_timeout"))
        self.first_timeout_sim = float(xml.getAttribute("first_timeout_sim"))

class TriggerOnPeriod(Trigger):
    def __init__(self, xml):
        super(TriggerOnPeriod, self).__init__("period")
        self.value = float(xml.getAttribute("value"))

class TriggerOnInterval(Trigger):
    def __init__(self, xml):
        super(TriggerOnInterval, self).__init__("interval")
        self.min = float(xml.getAttribute("min"))
        self.first = float(xml.getAttribute("first"))
        self.first_sim = float(xml.getAttribute("first_sim"))
        self.next = float(xml.getAttribute("next"))
#        obligatory_data = xml.getElementsByTagName('obligatory_data')
#        self.obligatory_data = []
#        for od in obligatory_data:
#            self.obligatory_data.append( od.getAttribute("buffer_name") )

class TriggerMethods:
    def parse(self, xml):
        new_data_on_buffer = xml.getElementsByTagName('new_data_on_buffer')
        no_data_on_buffer = xml.getElementsByTagName('no_data_on_buffer')
        period = xml.getElementsByTagName('period')
        interval = xml.getElementsByTagName('interval')

        if not (len(period) == 0 or len(period) == 1):
            raise Exception('period', 'wrong number of <period> tags in <trigger_methods>, should be 0 or 1')

        if not (len(interval) == 0 or len(interval) == 1):
            raise Exception('interval', 'wrong number of <interval> tags in <trigger_methods>, should be 0 or 1')

        if len(new_data_on_buffer) + len(no_data_on_buffer) + len(period) == 0:
            raise Exception('<trigger_methods>', 'at least one trigger method should be specified')

        for m in new_data_on_buffer:
            if not self.new_data:
                self.new_data = []
            self.new_data.append( TriggerOnNewData(m) )

        for m in no_data_on_buffer:
            if not self.no_data:
                self.no_data = []
            self.no_data.append( TriggerOnNoData(m) )

        if len(period) == 1:
            self.period = TriggerOnPeriod(period[0])

        if len(interval) == 1:
            self.interval = TriggerOnInterval(interval[0])

    def __init__(self, xml=None):
        self.new_data = None
        self.no_data = None
        self.period = None
        self.interval = None
        if xml:
            self.parse(xml)

    def onNewData(self, alias=None):
        if not self.new_data:
            return None
        if not alias:
            return self.new_data
        for t in self.new_data:
            if t.name == alias:
                return t
        return None

    def onNoData(self, alias=None):
        if not self.no_data:
            return None
        if not alias:
            return self.no_data
        for t in self.no_data:
            if t.name == alias:
                return t
        return None

    def onPeriod(self):
        return self.period

class SubsystemDefinition:
    def __init__(self):
        self.buffers_in = []
        self.buffers_out = []
        self.trigger_methods = None
        self.predicates = []
        self.states = []
        self.initial_state_name = None
        self.behaviors = []
        self.period = None
        self.use_ros_sim_clock = False
        self.trigger_gazebo = False
        self.use_sim_clock = False

    def getInitialStateName(self):
        return self.initial_state_name

    def parseBuffers(self, xml):
        # <in>
        for p_in in xml.getElementsByTagName('in'):
            p = InputPort(p_in)
            self.buffers_in.append(p)

        # <out>
        for p_out in xml.getElementsByTagName('out'):
            p = OutputPort(p_out)
            self.buffers_out.append(p)

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

    def parseTriggerMethods(self, xml):
        self.trigger_methods = TriggerMethods(xml)

    def parse(self, xml):
        # <buffers>
        buffers = xml.getElementsByTagName("buffers")
        if len(buffers) != 1:
            raise Exception('buffers', 'wrong number of <buffers> tags, should be 1')
        self.parseBuffers(buffers[0])

        # <trigger>
        trigger_methods = xml.getElementsByTagName("trigger_methods")
        if len(trigger_methods) != 1:
            raise Exception('trigger_methods', 'wrong number of <trigger_methods> tags, should be 1')
        self.parseTriggerMethods(trigger_methods[0])
            
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


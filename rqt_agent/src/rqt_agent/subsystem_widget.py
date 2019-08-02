# Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import subprocess
import copy
import math

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Signal, Slot
from python_qt_binding.QtWidgets import QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel
import rospkg
import rospy
from rospy.exceptions import ROSException

import tempfile

import matplotlib.pyplot as plt
import numpy as np

from subsystem_msgs.srv import *
import subsystem_common

import behavior_graph
import fsm_graph
import subsystem_components
import subsystem_state_history
import dot_graph
import latex_equations


class SubsystemWidget(QWidget):
    """
    main class inherits from the ui window class.

    TODO: description.
    """

# ___________________________________________________________
# |[ipc_buffers]                                             |
# |                     subsystem_name                       |
# |  master_component                                        |
# |  subsystem_state       [components]                      |
# |                        [components]                      |
# |                        [components]                      |
# |                        [components]                      |
# |                        [components]                      |
# |                                                          |
# |                                                          |
# |                                                          |
# |                                                          |
# |[ipc_buffers]                                             |
# |__________________________________________________________|

    def layout_widgets(self, layout):
       return (layout.itemAt(i) for i in range(layout.count()))

    def resetBuffersLayout(self):
        self.buffer_groups = {}
        self.lower_subsystems = []

        print self.subsystem_name, ".resetBuffersLayout()"

        for buf in self.all_buffers:
            self.all_buffers[buf].hide()

        while True:
            w = self.lower_buffers_layout.takeAt(0)
            if w == None:
                break;
            del w

        while True:
            w = self.upper_buffers_layout.takeAt(0)
            if w == None:
                break;
            del w

    @Slot()
    def on_click_showBehaviorGraph(self):
        self.dialogBehaviorGraph.show()

    @Slot()
    def on_click_showStateHistory(self):
        self.dialogStateHistory.show()

    @Slot()
    def on_click_showComponentsList(self):
        self.dialogComponents.show()

    @Slot()
    def on_click_showStateMachineGraph(self):
        self.dialogStateMachineGraph.show()

    @Slot()
    def on_click_showPeriodHistogram(self):
        #self.dialogBehaviorGraph.show()
        if not self.period_histogram is None:

            ranges = [0.0001, 0.0002, 0.0003, 0.0004, 0.0006, 0.0008, 0.001, 0.0012,
                        0.0014, 0.0016, 0.002, 0.0024, 0.0028, 0.0032, 0.0038, 0.0050,
                        0.0060, 0.0080, 0.01, 0.02, 0.03, 0.05,]
            ranges_str = []
            for val in ranges:
                ranges_str.append( str(val*1000) )

            x = np.arange(len(self.period_histogram))
            y = self.period_histogram
            y_log = []
            for idx in range(len(y)):
                val = y[idx]
                if self.prev_period_histogram is None:
                    prev_val = 0
                else:
                    prev_val = self.prev_period_histogram[idx]
                val_rel = val - prev_val
                if val_rel == 0:
                    y_log.append( 0 )
                else:
                    y_log.append( val_rel )#math.log(val_rel) )
            plt.bar(x, y_log)
            plt.xticks(x[0:-1]+0.5, ranges_str)

            #plt.yticks(x[0:-1]+0.5, ranges_str)

            plt.show()

            self.prev_period_histogram = np.copy(self.period_histogram)

    def __init__(self, plugin=None, name=None):
        """
        @type selected_topics: list of tuples.
        @param selected_topics: [($NAME_TOPIC$, $TYPE_TOPIC$), ...]
        @type select_topic_type: int
        @param select_topic_type: Can specify either the name of topics or by
                                  the type of topic, to filter the topics to
                                  show. If 'select_topic_type' argument is
                                  None, this arg shouldn't be meaningful.
        """
        super(SubsystemWidget, self).__init__()

        self.initialized = False

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_agent'), 'resource', 'SubsystemWidget.ui')
        loadUi(ui_file, self)
        self._plugin = plugin

        print "created Subsystem widget"

        if name != None:
            self.SubsystemName.setText(name)
        else:
            self.SubsystemName.setText("<could not get subsystem name>")

        self.subsystem_name = name

        self.subsystem_info = None
        self.state = ''
        self.behavior = ''
        self.period_histogram = None
        self.prev_period_histogram = None

        self.all_buffers = {}
        self.resetBuffersLayout()

        self.components = {}

        self.graph_generated = None

        self.dialogBehaviorGraph = behavior_graph.BehaviorGraphDialog(self.subsystem_name, self)
        self.showBehaviorGraph.clicked.connect(self.on_click_showBehaviorGraph)

        self.dialogStateHistory = subsystem_state_history.StateHistoryDialog(self.subsystem_name, self)
        self.showStateHistory.clicked.connect(self.on_click_showStateHistory)

        self.dialogComponents = subsystem_components.ComponentsDialog(self.subsystem_name, self)
        self.showComponentsList.clicked.connect(self.on_click_showComponentsList)

        self.dialogStateMachineGraph = fsm_graph.StateMachineGraphDialog(self.subsystem_name, self)
        self.showStateMachineGraph.clicked.connect(self.on_click_showStateMachineGraph)

        self.showPeriodHistogram.clicked.connect(self.on_click_showPeriodHistogram)

    def isInitialized(self):
        return self.initialized

    def extractConnectionInfo(self, conn, comp_from, comp_to):
        if (not comp_to) or (not comp_from):
            print 'WARNING: wrong edge(1): ', conn, comp_from, comp_to
            return None

        if conn.find(comp_from.name) != 0:
            print 'WARNING: wrong edge(2): ', conn, comp_from.name, comp_to.name
            return None

        idx = len(comp_from.name)
        port_from = None
        for p in comp_from.ports:
            if conn.find(p.name, idx) == idx:
                port_from = p.name
                idx += len(p.name)
                break

        if not port_from:
            print 'WARNING: wrong edge(3): ', conn, comp_from.name, comp_to.name
            return None

        if conn.find(comp_to.name, idx) != idx:
            print 'WARNING: wrong edge(4): ', conn, comp_from.name, comp_to.name

        idx += len(comp_to.name)

        port_to = None
        for p in comp_to.ports:
            if conn.find(p.name, idx) == idx:
                port_to = p.name
                idx += len(p.name)
                break

        if not port_to:
            print 'WARNING: wrong edge(5): ', conn, comp_from.name, comp_to.name
            return None

        return (comp_from.name, port_from, comp_to.name, port_to)

    def exportBehaviorGraph(self, graph_name):
        self.behavior_graphs[graph_name].exportToPdf(self.subsystem_name+"_"+graph_name+".pdf")

    def exportBehaviorGraphs(self):
        behavior_graphs_list = ["<all>"]#, "<always running>"]
        for behavior in self.subsystem_info.behaviors:
            behavior_graphs_list.append(behavior.name)

        for graph_name in behavior_graphs_list:
            self.exportBehaviorGraph(graph_name)

    def genBehaviorGraph(self, subsystem_info, behavior_name, hide_converters=True):
        g = dot_graph.Graph(shape="rounded_box")
        behavior = None
        for b in subsystem_info.behaviors:
            if b.name == behavior_name:
                behavior = b
                break

        if hide_converters:
            conv_comp = set()
            for comp in subsystem_info.components:
                if comp.is_converter:
                    conv_comp.add(comp.name)

        sw_comp = set()
        for b in subsystem_info.behaviors:
            for r in b.running_components:
                sw_comp.add(r)

        all_comp = set()
        for comp in subsystem_info.components:
            all_comp.add(comp.name)

        always_running = all_comp - sw_comp

        conn_set = {}
        current_running = set()
        if behavior:
            for r in behavior.running_components:
                current_running.add(r)
        other_behaviors_comp = sw_comp - current_running
        running = always_running.union(current_running)

        if hide_converters:
            conv_connections = {}

        for c in subsystem_info.component_connections:
            if ((not c.component_from.strip()) or (not c.component_to.strip())) and not c.unconnected:
                continue
            if behavior:
                if (not c.component_from in current_running) and (not c.component_to in current_running):
                    continue
                if c.component_from in other_behaviors_comp or c.component_to in other_behaviors_comp:
                    continue
#                elif name == "<always running>":
#                    if (not c.component_from in always_running) or (not c.component_to in always_running):
#                        continue
            elif behavior_name == "<all>":
                pass
            else:
                raise Exception('getBehaviorConnectionsSet', 'wrong behavior name: ' + behavior_name)

            c_from = c.component_from
            c_to = c.component_to
            unconnected = c.unconnected
            c_name = c.name
            if hide_converters:
                if c_from in conv_comp:
                    if not c_from in conv_connections:
                        conv_connections[c_from] = (None, c_to)
                        continue
                    else:
                        conv_connections[c_from] = (conv_connections[c_from][0], c_to)
                        c_to = conv_connections[c_from][1]
                        c_from = conv_connections[c_from][0]
                        unconnected = False
                elif c_to in conv_comp:
                    if not c_to in conv_connections:
                        conv_connections[c_to] = (c_from, None)
                        continue
                    else:
                        conv_connections[c_to] = (c_from, conv_connections[c_to][1])
                        c_from = conv_connections[c_to][0]
                        c_to = conv_connections[c_to][1]
                        unconnected = False

            if not unconnected:
                conn_tuple = (c_from, c_to)
            else:
                if not c_from.strip():
                    conn_tuple = (None, c_to)
                else:
                    conn_tuple = (c_from, None)

            if c_name:
                cname = c_name
            elif c.port_from.strip():
                cname = c.port_from
                if not unconnected and cname.endswith("_OUTPORT"):
                    cname = cname[:-8]
            else:
                cname = c.port_to
                if not unconnected and cname.endswith("_INPORT"):
                    cname = cname[:-7]

            latex_name = c.latex
            if not latex_name:
                #latex_name = '\\text{' + cname + '}'
                latex_name = None
#                if not cname:
#                    continue
            if conn_tuple in conn_set:
                #conn_set[conn_tuple] = conn_set[conn_tuple] + "\\n" + cname
                conn_set[conn_tuple][0].append(cname)
                conn_set[conn_tuple][1].append(latex_name)
            else:
                conn_set[conn_tuple] = [[cname], [latex_name]]

        shown_components = set()
        for conn_tuple in conn_set:
            edge = g.getEdge(conn_tuple[0], conn_tuple[1])
            if edge == None:
                edge = dot_graph.Graph.Edge()
                edge.id_from = conn_tuple[0]
                edge.id_to = conn_tuple[1]
            for i in range(len(conn_set[conn_tuple][0])):
                edge.addLabel(conn_set[conn_tuple][0][i], conn_set[conn_tuple][1][i])
            if conn_tuple[0] != None:
                shown_components.add(conn_tuple[0])
            if conn_tuple[1] != None:
                shown_components.add(conn_tuple[1])
            g.edges.append(edge)

        prev_comp = None
        for c in subsystem_info.components:
            if not c.name in shown_components:
                continue
            node = dot_graph.Graph.Node()
            node.label = c.name
            node.latex_label = c.latex
            g.nodes[c.name] = node

            # add edges for exec sequence
            # this is not good
#            if prev_comp != None:
#                edge = dot_graph.Graph.Edge()
#                edge.id_from = prev_comp
#                edge.id_to = c.name
#                g.edges.append(edge)
#            prev_comp = c.name

        return g

    def genStateMachineGraph(self, subsystem_info):
        g = dot_graph.Graph()

        print "subsystem name: " + self.subsystem_name
        #edges_counter = 0
        for state in subsystem_info.state_machine:
            # domyslnie przyjmujemy, ze stan jest niepodlaczony
            # trzeba jeszcze obsluzyc jak faktycznie nie bedzie nastepnikow
            # print "state.name: ", state.name
            n = dot_graph.Graph.Node()
            n.label = state.name
            g.nodes[state.name] = n
            print "  state: " + state.name
            print "    behaviors:"
            for b in state.behavior_names:
                print "      " + b

            for next_state in state.next_states:
                #print "    next state", next_state.name
                e = dot_graph.Graph.Edge()
                e.id_from = state.name
                e.id_to = next_state.name
                #e.addLabel('s' + str(edges_counter), '\sigma_{' + str(edges_counter) + '}')
                #print "    s_" + str(edges_counter) + " = " + next_state.init_cond
                e.addLabel('s_{' + state.name + "," + next_state.name + "}", '\sigma_{' + latex_equations.toMathText(state.name) + "," + latex_equations.toMathText(next_state.name) + '}')
                print "    s_{" + state.name + "," + next_state.name + "} = " + next_state.init_cond
                #edges_counter += 1
                g.edges.append(e)
        for behavior in subsystem_info.behaviors:
            print "  behavior: " + behavior.name
            print "    terminal_condition: " + behavior.terminal_condition
            print "    error_condition: " + behavior.error_condition

        return g
            
    def exportStateMachineGraph(self):
        self.state_machine_graph.exportToPdf(self.subsystem_name+"_fsm.pdf")
        self.exportStateMachineConditionsToPdf(self.subsystem_info)

    def extractIdentifiers(self, s):
        id_positions = []
        iterating_identifier = False
        id_begin = None
        for i in range(len(s)):
            if not iterating_identifier:
                if s[i].isalpha() or s[i] == '_':
                    iterating_identifier = True
                    id_begin = i
            else:
                if not s[i].isalnum() and s[i] != '_':
                    iterating_identifier = False
                    ident = s[id_begin:i]
                    id_positions.append( (ident, id_begin) )
        if iterating_identifier:
            ident = s[id_begin:]
            id_positions.append( (ident, id_begin) )
        return id_positions

    def prepareUsedPredicatesInfo(self, subsystem_info):
        # extract relevant predicates for every possible transition
        transitions = {}

        for state in subsystem_info.state_machine:
            term_err_cond_ids = []
            for behavior_name in state.behavior_names:
                for b in subsystem_info.behaviors:
                    if b.name == behavior_name:
                        id_positions = self.extractIdentifiers(b.terminal_condition)
                        for id_pos in id_positions:
                            if id_pos[0] != 'and' and id_pos[0] != 'or' and not id_pos[0] in term_err_cond_ids:
                                term_err_cond_ids.append(id_pos[0])
                        id_positions = self.extractIdentifiers(b.error_condition)
                        for id_pos in id_positions:
                            if id_pos[0] != 'and' and id_pos[0] != 'or' and not id_pos[0] in term_err_cond_ids:
                                term_err_cond_ids.append(id_pos[0])
                        break

            for next_state in state.next_states:
                id_positions = self.extractIdentifiers(next_state.init_cond)
                init_cond_ids = []
                for id_pos in id_positions:
                    if id_pos[0] != 'and' and id_pos[0] != 'or' and not id_pos[0] in init_cond_ids:
                        init_cond_ids.append(id_pos[0])
                transitions[(state.name,next_state.name)] = term_err_cond_ids + init_cond_ids
        return transitions

    def exportStateMachineConditionsToPdf(self, subsystem_info):

        def convertToLatex(condition):
            cond = copy.copy(condition)
            id_positions = self.extractIdentifiers(cond)

            # calculate line breaks (on 'and' and on 'or' only)
            max_line_len = 50
            last_line_break = -(len(state.name) + len(next_state.name))/2
            last_and_or = 0
            line_breaks = []
            for id_pos in id_positions:
                if id_pos[0] == 'and' or id_pos[0] == 'or':
                    if id_pos[1]-last_line_break > max_line_len:
                        line_breaks.append(id_pos[1])
                        last_line_break = id_pos[1]
                    last_and_or = id_pos[1]

            for br in reversed(line_breaks):
                cond = cond[0:br] + "\\\\" + cond[br:]

            # create length-sorted list of unique identifiers
            ids = []
            for id_pos in id_positions:
                if not id_pos[0] in ids:
                    ids.append(id_pos[0])
            ids.sort(key = lambda s: len(s), reverse=True)
            
            for i in range(len(ids)):
                cond = cond.replace(ids[i], "{" + str(i) + "}")

            for i in range(len(ids)):
                if ids[i] == "and":
                    ident = " \\wedge "
                elif ids[i] == "or":
                    ident = " \\vee "
                elif ids[i] == "not":
                    ident = " \\neg "
                else:
                    ident = " \\text{" + ids[i] + "} "
                cond = cond.replace("{" + str(i) + "}", ident)
            return cond

        latex_formulas = []
        file_names = []

        print "subsystem name: " + self.subsystem_name
        for state in subsystem_info.state_machine:
            print "  state: " + state.name
            for next_state in state.next_states:
                init_cond = convertToLatex(next_state.init_cond)
                init_cond = "\\sigma_{" + latex_equations.toMathText(state.name) + "," + latex_equations.toMathText(next_state.name) + "} = " + init_cond
                latex_formulas.append(init_cond)
                file_names.append(self.subsystem_name + "_ " + state.name + "_" + next_state.name + "_init")

            term_cond = None
            err_cond = None
            for behavior_name in state.behavior_names:
                for b in subsystem_info.behaviors:
                    if b.name == behavior_name:
                        if term_cond == None:
                            term_cond = "(" + b.terminal_condition + ")"
                        else:
                            term_cond = term_cond + " or (" + b.terminal_condition + ")"
                        if err_cond == None:
                            err_cond = "(" + b.error_condition + ")"
                        else:
                            err_cond = err_cond + " or (" + b.error_condition + ")"
                        break

            term_cond = convertToLatex(term_cond)
            term_cond = "\\tau_{" + latex_equations.toMathText(state.name) + "} = " + term_cond
            latex_formulas.append(term_cond)
            file_names.append(self.subsystem_name + "_ " + state.name + "_" + next_state.name + "_term")

            err_cond = convertToLatex(err_cond)
            err_cond = "\\epsilon_{" + latex_equations.toMathText(state.name) + "} = " + err_cond
            latex_formulas.append(err_cond)
            file_names.append(self.subsystem_name + "_ " + state.name + "_" + next_state.name + "_err")

        with open("formulas.tex", 'w') as outfile:
            for i in range(len(latex_formulas)):
                outfile.write("% " + file_names[i] + "\n")
                #outfile.write("\\begin{gather*}\n" + latex_formulas[i] + "\n\\end{gather*}\n\n")
                outfile.write(latex_formulas[i] + "\n")

        # export to pdf
#        eps_file_list = []
#        for i in range(len(latex_formulas)):
#            handle, path = tempfile.mkstemp(suffix=".eps")
#            eps_file_list.append( (handle, path) )
#        latex_equations.exportToEpsList(eps_file_list, latex_formulas)
#        for i in range(len(latex_formulas)):
#            (handle, path) = eps_file_list[i]
#            subprocess.call(['epspdf', path, file_names[i]+".pdf"])

    def update_subsystem(self, msg):
        for value in msg.status[1].values:
            if value.key == 'master_component':
                self.state = value.value
            elif value.key[-2:] == 'Rx' or value.key[-2:] == 'Tx':
                if value.key[0:-2] in self.all_buffers:
                    if value.value == '<data ok>':
                        self.all_buffers[value.key[:-2]].setStyleSheet("background-color: green")
                    else:
                        self.all_buffers[value.key[:-2]].setStyleSheet("background-color: red")
                    self.all_buffers[value.key[:-2]].setToolTip(value.value)

        if self.graph_generated == None and self.initialized:

            draw_unconnected = False

            self.all_component_connections = []
            for conn in self.subsystem_info.component_connections:
                self.all_component_connections.append( (conn.component_from, conn.port_from, conn.component_to, conn.port_to) )

            self.dialogStateHistory.setUsedPredicatesInfo(self.prepareUsedPredicatesInfo(self.subsystem_info))
        # behaviors

            behavior_graphs_list = ["<all>"]#, "<always running>"]
            for behavior in self.subsystem_info.behaviors:
                behavior_graphs_list.append(behavior.name)

            self.behavior_graphs = {}
            for graph_name in behavior_graphs_list:
                self.behavior_graphs[graph_name] = self.genBehaviorGraph(self.subsystem_info, graph_name, hide_converters=False)
                graph_str = self.behavior_graphs[graph_name].exportToPlain()
                self.dialogBehaviorGraph.addGraph(graph_name, graph_str)

            self.dialogBehaviorGraph.showGraph("<all>")

        # state machine (fsm)
            self.state_machine_graph = self.genStateMachineGraph(self.subsystem_info)
            graph_str = self.state_machine_graph.exportToPlain()
            self.dialogStateMachineGraph.addGraph(graph_str)
            self.dialogStateMachineGraph.showGraph()
            
        # end of state machine
            
            
            self.graph_generated = True

        components_state = {}
        for value in msg.status[0].values:
            components_state[value.key] = value.value

        components_diag_msgs = {}
        for value in msg.status[1].values:
            components_diag_msgs[value.key] = value.value

        #
        # update dialogs
        #
        self.dialogComponents.updateState(components_state, components_diag_msgs)
        self.dialogBehaviorGraph.updateState(components_state)

        #rospy.wait_for_service('/' + name = '/getSubsystemInfo')
        if self.subsystem_info == None:
            try:
                self._getSubsystemInfo = rospy.ServiceProxy('/' + self.subsystem_name + '/getSubsystemInfo', GetSubsystemInfo)
                self.subsystem_info = self._getSubsystemInfo()
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            
            # print "\n\nSUBSYSTEM NAME:",self.subsystem_name, " START OF subsystem_info\n\n"

            # if self.subsystem_info != None:
                #print self.subsystem_info
                # print self.subsystem_info.state_machine
            
            # print "\n\nSUBSYSTEM NAME:",self.subsystem_name, " END OF subsystem_info\n\n"


            self.initialized = True

        mcd = subsystem_common.parseMasterComponentDiag(self.state)
        if len(mcd.history) > 0:
            self.SubsystemState.setText(mcd.history[0].state_name)
            self.dialogStateHistory.updateState(mcd)
            self.dialogStateMachineGraph.updateState(mcd)
            self.PeriodWall.setText(str(mcd.current_period*1000.0) + 'ms')
            self.period_histogram = mcd.period_histogram
        else:
            self.SubsystemState.setText("unknown")
            self.PeriodWall.setText("unknown")

    def getCommonBuffers(self, subsystem):
        if not self.isInitialized() or not subsystem.isInitialized():
            return None
        if (subsystem.subsystem_info == None) or (self.subsystem_info == None):
            return None
        common_buffers = None
        for this_index in range(len(self.subsystem_info.upper_inputs)):
            up_in = self.subsystem_info.upper_inputs[this_index]
            for index in range(len(subsystem.subsystem_info.lower_outputs)):
                lo_out = subsystem.subsystem_info.lower_outputs[index]
                if up_in == lo_out:
                    if common_buffers == None:
                        common_buffers = []
                    common_buffers.append(up_in)

        for this_index in range(len(self.subsystem_info.upper_outputs)):
            up_out = self.subsystem_info.upper_outputs[this_index]
            for index in range(len(subsystem.subsystem_info.lower_inputs)):
                lo_in = subsystem.subsystem_info.lower_inputs[index]
                if up_out == lo_in:
                    if common_buffers == None:
                        common_buffers = []
                    common_buffers.append(up_out)
        return common_buffers

    def getCommonString(self, str_list):
        idx = 0
        while True:
            character = None
            for s in str_list:
                if idx >= len(s):
                    return s
                if character == None:
                    character = s[idx]
                elif character != s[idx]:
                    return s[:idx]
            idx = idx + 1
        return None     # this is never reached

    def groupBuffers(self, buffer_list, subsystem_name):
        if buffer_list == None or len(buffer_list) == 0:
            print "Error in %s.groupBuffers(%s, %s): buffers list is None or empty"%(self.subsystem_name, buffer_list, subsystem_name)
            return False
        if subsystem_name in self.buffer_groups:
            # TODO: remove old buffer widgets
            return False
        self.buffer_groups[subsystem_name] = buffer_list

        lo_in = []
        up_in = []
        lo_out = []
        up_out = []

        for buf_name in buffer_list:
            if buf_name in self.subsystem_info.lower_inputs:
                lo_in.append(buf_name)
            elif buf_name in self.subsystem_info.upper_inputs:
                up_in.append(buf_name)
            elif buf_name in self.subsystem_info.lower_outputs:
                lo_out.append(buf_name)
            elif buf_name in self.subsystem_info.upper_outputs:
                up_out.append(buf_name)

        # buffer group should be either in lower part or upper part
        if (len(lo_in) > 0 or len(lo_out) > 0) and (len(up_in) > 0 or len(up_out) > 0):
            print "Error in %s.groupBuffers(%s, %s): mixed upper and lower buffers"%(self.subsystem_name, buffer_list, subsystem_name)
            return False

        # get most common part of buffers' names
        name_list = []
        common_name = self.getCommonString(buffer_list)
        for idx in range(len(buffer_list)):
            name_list.append( buffer_list[idx][len(common_name):] )

        print common_name, name_list

        vbox = QVBoxLayout()

        hbox1 = QHBoxLayout()
        hbox1.addStretch()
        if not (common_name) in self.all_buffers:
            self.all_buffers[common_name] = QLabel(common_name)
        hbox1.addWidget(self.all_buffers[common_name])
        self.all_buffers[common_name].show()
        hbox1.addStretch()

        hbox2 = QHBoxLayout()
        hbox2.addSpacing(20)
        for buf_name in name_list:
#            hbox2.addStretch()
            if common_name+buf_name in lo_in or common_name+buf_name in up_out:
                suffix = ' /\\'
            else:
                suffix = ' \\/'

            if not (common_name+buf_name) in self.all_buffers:
                self.all_buffers[common_name+buf_name] = QPushButton(buf_name + suffix)
            hbox2.addWidget( self.all_buffers[common_name+buf_name] )
            self.all_buffers[common_name+buf_name].show()
#            hbox2.addWidget( QPushButton(buf_name + suffix) )
#        hbox2.addStretch()
        hbox2.addSpacing(20)


        if len(lo_in) > 0 or len(lo_out) > 0:
            vbox.addLayout(hbox1)
            vbox.addLayout(hbox2)
            self.lower_buffers_layout.addLayout(vbox)
            self.lower_subsystems.append(subsystem_name)
        else:
            vbox.addLayout(hbox2)
            vbox.addLayout(hbox1)
            self.upper_buffers_layout.addLayout(vbox)

    def getLowerSubsystemPosition(self, subsystem_name):
        for i in range(len(self.lower_subsystems)):
            if self.lower_subsystems[i] == subsystem_name:
                return i
        return -1

    def getLowerSubsystems(self):
        return self.lower_subsystems

    def start(self):
        """
        This method needs to be called to start updating topic pane.
        """

    def shutdown_plugin(self):
        for topic in self._topics.values():
            topic['info'].stop_monitoring()
        self._timer_refresh_topics.stop()


    # TODO(Enhancement) Save/Restore tree expansion state
    def save_settings(self, plugin_settings, instance_settings):
        header_state = self.topics_tree_widget.header().saveState()
        instance_settings.set_value('tree_widget_header_state', header_state)

    def restore_settings(self, pluggin_settings, instance_settings):
        if instance_settings.contains('tree_widget_header_state'):
            header_state = instance_settings.value('tree_widget_header_state')
            if not self.topics_tree_widget.header().restoreState(header_state):
                rospy.logwarn("rqt_topic: Failed to restore header state.")



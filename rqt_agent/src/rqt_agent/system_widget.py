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

from __future__ import division
import os
import subprocess
import tempfile

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot, QRectF
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout,\
    QScrollArea, QGraphicsScene
from python_qt_binding.QtGui import QPixmap
import roslib
import rospkg
import rospy
from rospy.exceptions import ROSException

from .topic_info import TopicInfo
from .subsystem_widget import SubsystemWidget

class SystemWidget(QWidget):
    """
    main class inherits from the ui window class.

    You can specify the topics that the topic pane.

    SystemWidget.start must be called in order to update topic pane.
    """

    _column_names = ['topic', 'type', 'bandwidth', 'rate', 'value']

    def __init__(self, plugin=None):
        """
        """
        super(SystemWidget, self).__init__()

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_agent'), 'resource', 'SystemWidget.ui')
        loadUi(ui_file, self)

        self._plugin = plugin

        self._subsystems = {}

        self.all_subsystems = {}
        self._widgets = {}
        self._added_widgets = {}
        self.prev_subsystems = []

        self.structure_changed = False

        self.structure_root = None
        self.structure_graph = None

        #self.mainWidget = QWidget()
        #self.scrollArea = QScrollArea()
        #self.scrollArea.setWidgetResizable( True );
        #self.scrollArea.setWidget(self.mainWidget)

        self.graphicsView.setScene(QGraphicsScene(0,0,10,10))

        # init and start update timer
        self._timer_refresh_topics = QTimer(self)
        self._timer_refresh_topics.timeout.connect(self.refresh_topics)

    def checkStructureChange(self):
        result = False

        for subsystem_name in self.prev_subsystems:
            if not subsystem_name in self._widgets:
                result = True
                break;

        if result == False:
            for subsystem_name in self._widgets:
                if not subsystem_name in self.prev_subsystems:
                    result = True
                    break

        self.prev_subsystems = []
        for subsystem_name in self._widgets:
            self.prev_subsystems.append(subsystem_name)

        return result

    def start(self):
        """
        This method needs to be called to start updating topic pane.
        """
        self._timer_refresh_topics.start(100)

    def layout_widgets(self, layout):
       return (layout.itemAt(i) for i in range(layout.count()))

    @Slot()
    def refresh_topics(self):
        """
        refresh tree view items
        """
        #
        # update the list of subsystems
        #
        topic_list = rospy.get_published_topics()
        if topic_list is None:
            rospy.logerr('Not even a single published topic found. Check network configuration')
            return

        # start new topic dict
        new_subsystems = {}

        for topic_name, topic_type in topic_list:
            name_split = topic_name.split('/')

            if (len(name_split) == 3) and (name_split[0] == '') and (name_split[2] == 'diag') and (topic_type == "diagnostic_msgs/DiagnosticArray"):
                subsystem_name = name_split[1]
                # if topic is new
                if subsystem_name not in self._subsystems:
                    # create new TopicInfo
                    topic_info = TopicInfo(topic_name, topic_type)
                    new_subsystems[subsystem_name] = topic_info
                    topic_info.start_monitoring()
                else:
                    # if topic has been seen before, copy it to new dict and
                    # remove it from the old one
                    new_subsystems[subsystem_name] = self._subsystems[subsystem_name]
                    del self._subsystems[subsystem_name]

        # remove unused subsystems
        while True:
            repeat = False
            for s in self._subsystems:
                if not s in new_subsystems:
                    del self._subsystems[s]
                    repeat = True
                    break
            if not repeat:
                break

        # switch to new topic dict
        self._subsystems = new_subsystems

        #
        # update each subsystem
        #
        new_widgets = {}
        for subsystem_name in self._subsystems:
            msg = self._subsystems[subsystem_name].last_message

            if (msg != None) and (len(msg.status) == 2) and \
              msg.status[0].name == 'components' and msg.status[1].name == 'diagnostics':
                name_split = subsystem_name.split('/')

                if not subsystem_name in self.all_subsystems:
                    self.all_subsystems[subsystem_name] = SubsystemWidget(self._plugin, subsystem_name)

                if not subsystem_name in self._widgets:
                    new_widgets[subsystem_name] = self.all_subsystems[subsystem_name]
                else:
                    new_widgets[subsystem_name] = self._widgets[subsystem_name]

        self._widgets = new_widgets

        structure_changed = self.checkStructureChange()
        if structure_changed:
            self.structure_changed = True

        if self.structure_changed:
            allInitialized = True
            for subsystem_name, wg in new_widgets.iteritems():
                if not wg.isInitialized():
                    allInitialized = False
                    break
            if allInitialized:
                #print dir(self.verticalLayout)
                self.generateSystemStructureDot()

                # Remove all widgets from the layout
                for subsystem_name, wg in self._added_widgets.iteritems():
                    wg.setParent(None)

                for it in range(100):
                    if self.verticalLayout.isEmpty():
                        break
                    self.verticalLayout.removeWidget()
                    self.verticalLayout.takeAt(0)
                    print 'removed widget', it

                self._added_widgets = {}
                for subsystem_name, wg in self._widgets.iteritems():
                    self._added_widgets[subsystem_name] = wg

                for wg_name, wg in self._widgets.iteritems():
                    self.verticalLayout.addWidget(wg)
                    wg.show()
                    print 'added widget', wg_name

                self.structure_changed = False

        while True:
            repeat = False
            for s in self.all_subsystems:
                if not s in self._widgets:
                    del self.all_subsystems[s]
                    repeat = True
                    break
            if not repeat:
                break

        for subsystem_name in self._widgets:
            self._widgets[subsystem_name].update_subsystem(self._subsystems[subsystem_name].last_message)

    def generateSystemStructureDot(self):
        dot = 'digraph system {\n'
        all_buf_names = set()
        for subsystem_name, wg in self._widgets.iteritems():
            if not wg.isInitialized():
                dot += '  {} [style="filled,rounded" fillcolor=orange shape=box];\n'.format(subsystem_name)
                continue

            dot += '  {} [style=rounded shape=box];\n'.format(subsystem_name)

            for buf_name in wg.subsystem_info.upper_inputs:
                dot += '  {} -> {};\n'.format(buf_name, subsystem_name)
                all_buf_names.add(buf_name)

            for buf_name in wg.subsystem_info.lower_inputs:
                dot += '  {} -> {} [arrowhead=none arrowtail=normal dir=back];\n'.format(subsystem_name, buf_name)
                all_buf_names.add(buf_name)

            for buf_name in wg.subsystem_info.upper_outputs:
                dot += '  {} -> {} [arrowhead=none arrowtail=normal dir=back];\n'.format(buf_name, subsystem_name)
                all_buf_names.add(buf_name)

            for buf_name in wg.subsystem_info.lower_outputs:
                dot += '  {} -> {};\n'.format(subsystem_name, buf_name)
                all_buf_names.add(buf_name)

        for buf_name in all_buf_names:
            dot += '  {} [shape=box];\n'.format(buf_name)
        dot += '}\n'

        tmpdirname = tempfile.mkdtemp()
        in_read, in_write = os.pipe()
        os.write(in_write, dot)
        os.close(in_write)
        subprocess.call(['dot', '-Tpng', '-o{}/system.png'.format(tmpdirname)], stdin=in_read)
        os.close(in_read)

        # Clear the diagram
        self.graphicsView.scene().clear()
        self.graphicsView.scene().update()
        pixmap = QPixmap('{}/system.png'.format(tmpdirname))
        self.graphicsView.scene().addPixmap(pixmap)
        self.graphicsView.scene().setSceneRect(QRectF(pixmap.rect()))
        os.remove('{}/system.png'.format(tmpdirname))
        os.rmdir(tmpdirname) 

    def shutdown_plugin(self):
        for topic in self._topics.values():
            topic['info'].stop_monitoring()
        self._timer_refresh_topics.stop()

    def set_selected_topics(self, selected_topics):
        """
        @param selected_topics: list of tuple. [(topic_name, topic_type)]
        @type selected_topics: []
        """
        rospy.logdebug('set_selected_topics topics={}'.format(
                                                         len(selected_topics)))
        self._selected_topics = selected_topics

    # TODO(Enhancement) Save/Restore tree expansion state
    def save_settings(self, plugin_settings, instance_settings):
        header_state = self.topics_tree_widget.header().saveState()
        instance_settings.set_value('tree_widget_header_state', header_state)

    def restore_settings(self, pluggin_settings, instance_settings):
        if instance_settings.contains('tree_widget_header_state'):
            header_state = instance_settings.value('tree_widget_header_state')
            if not self.topics_tree_widget.header().restoreState(header_state):
                rospy.logwarn("rqt_topic: Failed to restore header state.")


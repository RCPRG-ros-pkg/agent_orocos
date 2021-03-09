# Copyright (c) 2021, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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
import math
import subprocess

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot, QRectF, QPointF, QSize, QRect, QPoint
from python_qt_binding.QtWidgets import QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel,\
    QListWidgetItem, QDialog, QGraphicsView, QGraphicsScene, QGraphicsPathItem, QTableWidgetItem, QHeaderView, QStyle, QCommonStyle
from python_qt_binding.QtGui import QColor, QPen, QBrush, QPainterPath, QPolygonF, QTransform,\
    QPainter, QIcon, QPixmap, QPaintEvent, QPalette, QStandardItem, QStandardItemModel
from python_qt_binding.QtSvg import QSvgGenerator
import roslib
import rospkg
import rospy
from rospy.exceptions import ROSException

import xml.dom.minidom as minidom
import tempfile

from rqt_topic.topic_info import TopicInfo

from subsystem_msgs.srv import *

class LoggerSelector:
    def __init__(self, list_view):
        self.__list_view = list_view
        self.__loggers_item_model = QStandardItemModel()
        self.__list_view.setModel(self.__loggers_item_model)
        self.__all_logger_names = []
        self.__loggers_vis_set = set()
        self.__loggers_item_model.itemChanged.connect(self.__loggersItemChanged)

    def __loggersItemChanged(self, item):
        #print('__loggersItemChanged {}'.format(item))
        #print item.index()  # returns PyQt5.QtCore.QModelIndex
        logger_name = item.text()
        check_state = item.checkState()
        if check_state == Qt.Checked:
            if not logger_name in self.__loggers_vis_set:
                self.__loggers_vis_set.add( logger_name )
            print('show logger "{}"'.format(logger_name))
        elif check_state == Qt.Unchecked:
            if logger_name in self.__loggers_vis_set:
                self.__loggers_vis_set.remove( logger_name )
            print('hide logger "{}"'.format(logger_name))
        else:
            raise Exception('Wrong check_state')

    def updateLoggersList(self, logger_list):
        assert isinstance(logger_list, list)
        for logger_name in logger_list:
            assert isinstance( logger_name, str )
            if not logger_name in self.__all_logger_names:
                self.__all_logger_names.append( logger_name )
                item = QStandardItem( logger_name )
                check_state = Qt.Unchecked
                item.setCheckState(check_state)
                item.setCheckable(True)
                self.__loggers_item_model.appendRow(item)

    def getActiveLoggers(self):
        return self.__loggers_vis_set

class LogVis:
    def __init__(self, list_view):
        self.__list_view = list_view
        self.__logs_item_model = QStandardItemModel()
        self.__list_view.setModel(self.__logs_item_model)
        #self.__all_logger_names = []
        #self.__loggers_vis_set = set()
        #self.__logs_item_model.itemChanged.connect(self.__loggersItemChanged)
        self.__active_loggers = set()
        self.__logs = []
        self.__new_logs = []

    def setActiveLoggers(self, active_loggers):
        self.__active_loggers = active_loggers

    def addLog(self, logger_name, log_str, log_time):
        self.__logs.append( (logger_name, log_str, log_time) )
        self.__new_logs.append( (logger_name, log_str, log_time) )

    def updateView(self):
        # TODO: sort new items wrt. time
        for logger_name, log_str, log_time in self.__new_logs:
            if log_time is None:
                time_str = '???'
            else:
                sec_10000 = log_time.secs%10000
                nsec4 = int(log_time.nsecs/100000)
                time_str = '{}.{:04d}'.format(sec_10000, nsec4)

            item = QStandardItem( '{}: {}: {}'.format(time_str, logger_name, log_str) )
            self.__logs_item_model.insertRow(0, item)
            #removeRows
        self.__new_logs = []

class LogsDialog(QDialog):

    @Slot()
    def closeClick(self):
        self.close()

    # TODO: remove:
    @Slot()
    def updateClick(self):
        print 'do nothing'
        #self.updateLoggersList()

    def __init__(self, subsystem_name, parent=None):
        super(LogsDialog, self).__init__(parent)

        self.parent = parent

        self.setWindowFlags(Qt.Window)

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_agent'), 'resource', 'SubsystemLogs.ui')
        loadUi(ui_file, self)

        self.setWindowTitle(subsystem_name + " - state history")

        self.pushButton_close.clicked.connect(self.closeClick)

        self.pushButton_update.clicked.connect(self.updateClick)

        self.logger_selector = LoggerSelector(self.listView)
        self.logs_vis = LogVis(self.listViewLogs)

    def parseLogItem(self, log_item):
        pos = log_item.rfind('Time(')
        if pos < 0:
            return log_item, None
        # else
        log_str = log_item[0:pos]
        time_str = log_item[pos+5:-1]
        fields = time_str.split(';')
        if len(fields) < 2:
            return log_item, None

        try:
            sec = int(fields[0])
            nsec = int(fields[1])
        except:
            return log_item, None
        return log_str, rospy.Time(sec, nsec)

    def update(self, logs):
        assert isinstance(logs, dict)
        self.logger_selector.updateLoggersList( logs.keys() )
        active_loggers = self.logger_selector.getActiveLoggers()
        self.logs_vis.setActiveLoggers( active_loggers )

        for logger_name, log_list in logs.iteritems():
            for log_item in log_list:
                log_str, log_time = self.parseLogItem(log_item)
                self.logs_vis.addLog( logger_name, log_str, log_time )
            #if logger_name in active_loggers:

        self.logs_vis.updateView()

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
import math

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot, QRectF, QPointF, QSize, QRect, QPoint
from python_qt_binding.QtWidgets import QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QListWidgetItem, QDialog, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem, QGraphicsPathItem, QTableWidgetItem, QHeaderView, QStyle, QCommonStyle
from python_qt_binding.QtGui import QColor, QPen, QBrush, QPainterPath, QPolygonF, QTransform, QPainter, QIcon, QPixmap, QPaintEvent, QPalette
from python_qt_binding.QtSvg import QSvgGenerator
import roslib
import rospkg
import rospy
from rospy.exceptions import ROSException

from subsystem_msgs.srv import *

class StateHistoryDialog(QDialog):

    @Slot()
    def closeClick(self):
        self.close()

    class MyStyle(QCommonStyle):
        def drawControl (self, element, opt, painter, widget=None):

#            print element
            if element == QStyle.CE_HeaderLabel:
                hv = widget
#                if not hv or hv.orientation() != Qt.Horizontal:
#                    return super(StateHistoryDialog.MyStyle, self).drawControl(element, opt, p, widget)
                header = opt

                if header.section < 3:
                    return super(StateHistoryDialog.MyStyle, self).drawControl(element, opt, painter, widget)

                painter.save()
                #// painter->translate(header->rect.topLeft())
                rect = header.rect.bottomLeft()
                painter.translate(QPoint(rect.x() + 10, rect.y() + 5))
#                print "drawControl"
                painter.rotate(-90)
                painter.drawText(0,0,header.text)
                painter.restore()
                return
#            return QProxyStyle::drawControl(element, option, painter, widget);
#            print "drawControl"
            return super(StateHistoryDialog.MyStyle, self).drawControl(element, opt, painter, widget)

#        def drawItemText( painter, rectangle, alignment, palette, enabled, text, textRole = QPalette.NoRole):
#            pass

    class MyHorizHeader(QHeaderView):
        def __init__(self, parent=None):
            super(StateHistoryDialog.MyHorizHeader, self).__init__(Qt.Horizontal, parent)
            style = StateHistoryDialog.MyStyle()
            self.setStyle(style)

        def sizeHint(self):
            # Get the base implementation size.
            baseSize = super(StateHistoryDialog.MyHorizHeader, self).sizeHint()
            # Override the height with a custom value.
            baseSize.setHeight( 150 );
#            baseSize.setWidth( 20 );
            return baseSize;

    def __init__(self, subsystem_name, parent=None):
        super(StateHistoryDialog, self).__init__(parent)

        self.parent = parent

        self.setWindowFlags(Qt.Window)

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_agent'), 'resource', 'StateHistory.ui')
        loadUi(ui_file, self)

        self.setWindowTitle(subsystem_name + " - state history")

        self.pushButton_close.clicked.connect(self.closeClick)

        hdr = self.MyHorizHeader(self.tableWidget)
        hdr.setMinimumSectionSize(40)
        hdr.setDefaultSectionSize(40)
        self.tableWidget.setHorizontalHeader(hdr)

        self.initialized = False
        item0 = QTableWidgetItem("state")
        item1 = QTableWidgetItem("reason")
        item2 = QTableWidgetItem("time")
        self.tableWidget.setHorizontalHeaderItem (0, item0)
        self.tableWidget.setHorizontalHeaderItem (1, item1)
        self.tableWidget.setHorizontalHeaderItem (2, item2)
        self.tableWidget.setColumnWidth(0, 100)
        self.tableWidget.setColumnWidth(1, 50)
        self.tableWidget.setColumnWidth(2, 75)

    def updateState(self, mcd):
        if self.tableWidget.rowCount() != len(mcd.history):
            self.tableWidget.setRowCount( len(mcd.history) )
        row = 0
        curr_pred_v = []
        for pv in mcd.current_predicates:
            if pv.value == True:
                curr_pred_v.append("t")
            elif pv.value == False:
                curr_pred_v.append("f")
            else:
                raise ValueError("wrong predicate value: '" + str(pv.value) + "' for predicate: '" + pv.name + "'")

        for ss in mcd.history:
            self.tableWidget.setItem(row, 0, QTableWidgetItem(ss.state_name))
            self.tableWidget.setItem(row, 1, QTableWidgetItem(ss.reason))
            self.tableWidget.setItem(row, 2, QTableWidgetItem(str(ss.switch_interval)))

            if not self.initialized:
                self.initialized = True
                predicates = []
                for pv in ss.predicates:
                    predicates.append( pv.name )

                self.tableWidget.setColumnCount(len(predicates)+3)
                idx = 3
                for p in predicates:
                    item = QTableWidgetItem(p)
                    self.tableWidget.setHorizontalHeaderItem(idx, item)
                    idx += 1

            idx = 0
            for pv in ss.predicates:
                if pv.value == True:
                    p_str = "t"
                elif pv.value == False:
                    p_str = "f"
                else:
                    raise ValueError("wrong predicate value: '" + str(pv.value) + "' for predicate: '" + pv.name + "'")
                if row == 0:
                    self.tableWidget.setItem(row, 3+idx, QTableWidgetItem(p_str + " ("+curr_pred_v[idx]+")"))
                else:
                    self.tableWidget.setItem(row, 3+idx, QTableWidgetItem(p_str))
                idx += 1
                
            row = row + 1


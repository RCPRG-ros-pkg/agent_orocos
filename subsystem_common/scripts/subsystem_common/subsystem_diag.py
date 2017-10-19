"""
This file contains Python classes and routines for parsing subsystem state string.
"""

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

#import diagnostic_msgs.msg

import xml.dom.minidom as minidom

class SubsystemDiag:
    class StateSwitchEvent:
        def __init__(self):
            self.state_name = None
            self.reason = None
            self.switch_interval = None
            self.predicates = None

    def __init__(self):
        self.history = []
        self.current_predicates = None
        self.current_period = None

def parseMasterComponentDiag(diag_xml):
    result = SubsystemDiag()

    dom = minidom.parseString(diag_xml)
    mcd = dom.getElementsByTagName("mcd")
    if len(mcd) != 1:
        return result

    hist = mcd[0].getElementsByTagName("h")
    if len(hist) == 1:
        ss_list = hist[0].getElementsByTagName("ss")
        for ss in ss_list:
            ss_ev = SubsystemDiag.StateSwitchEvent()
            ss_ev.state_name = ss.getAttribute("n")
            ss_ev.reason = ss.getAttribute("r")
            ss_ev.switch_interval = float(ss.getAttribute("t"))
            ss_ev.predicates = ss.getAttribute("e")
            result.history.append(ss_ev)

    current_predicates = mcd[0].getElementsByTagName("pr")
    if len(current_predicates) == 1:
        result.current_predicates = current_predicates[0].getAttribute("v")

    period = mcd[0].getElementsByTagName("p")
    if len(period) == 1:
        result.current_period = float(period[0].childNodes[0].data)

    return result


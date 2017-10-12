#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import tempfile

class Graph:
    class Node:
        def __init__(self):
            self.label = None
            self.latex_label = None

    class Edge:
        def __init__(self):
            self.id_from = None
            self.id_to = None
            self.label = []
            self.latex_label = []

        def addLabel(self, label, latex_label):
            self.label.append(label)
            self.latex_label.append(latex_label)

    def __init__(self):
        self.nodes = {}
        self.edges = []

    def getEdge(self, id_from, id_to):
        for e in self.edges:
            if e.id_from == id_from and e.id_to == id_to:
                return e
        return None

    def generateDotFile(self, draw_unconnected=False, use_latex=False):

        eps_file_list = []
        latex_formulas = []

        dot = "digraph transition {\n"

        for e in self.edges:
            if (e.id_from == None or e.id_to == None) and not draw_unconnected:
                continue

            # allow loops?
            #if c[0] == c[1]:
            #    continue
            if use_latex:
                edge_latex_str = ''
                sep = ''
                for i in range(len(e.latex_label)):
                    latex = e.latex_label[i]
                    if latex == None:
                        latex = '\\text{' + e.label[i].replace('_', '\_') + '}'
                    edge_latex_str += sep + latex
                    sep = ' \\\\ '

                if edge_latex_str in latex_formulas:
                    handle, path = eps_file_list[latex_formulas.index(edge_latex_str)]
                else:
                    handle, path = tempfile.mkstemp(suffix=".eps")
                    eps_file_list.append( (handle, path) )
                    latex_formulas.append( edge_latex_str )

                if e.id_from == None:
                    dot += "\"" + e.id_to + "_unconnected_in\" [shape=point label=\"\"];\n"
                    dot += e.id_to + "_unconnected_in -> " + e.id_to + " [label=<<TABLE BORDER=\"0\"><TR><TD><IMG src=\"" + path + "\"/></TD></TR></TABLE>>];\n"
                elif e.id_to == None:
                    dot += "\"" + e.id_from + "_unconnected_out\" [shape=point label=\"\"];\n"
                    dot += e.id_from + " -> " + e.id_from + "_unconnected_out [label=<<TABLE BORDER=\"0\"><TR><TD><IMG src=\"" + path + "\"/></TD></TR></TABLE>>];\n"
                else:
                    dot += e.id_from + " -> " + e.id_to + " [label=<<TABLE BORDER=\"0\"><TR><TD><IMG src=\"" + path + "\"/></TD></TR></TABLE>>];\n"

            else:
                edge_str = ''
                sep = ''
                for label in e.label:
                    edge_str += sep + label
                    sep = '\\n'
                if e.id_from == None:
                    dot += "\"" + e.id_to + "_unconnected_in\" [shape=point label=\"\"];\n"
                    dot += e.id_to + "_unconnected_in -> " + e.id_to + " [label=\"" + edge_str + "\"];\n"
                elif e.id_to == None:
                    dot += "\"" + e.id_from + "_unconnected_out\" [shape=point label=\"\"];\n"
                    dot += e.id_from + " -> " + e.id_from + "_unconnected_out [label=\"" + edge_str + "\"];\n"
                else:
                    dot += e.id_from + " -> " + e.id_to + " [label=\"" + edge_str + "\"];\n"

        if use_latex:
            for name in self.nodes:
                n = self.nodes[name]
                if not n.latex_label:
                    continue
                if n.latex_label in latex_formulas:
                    handle, path = eps_file_list[latex_formulas.index(n.latex_label)]
                else:
                    handle, path = tempfile.mkstemp(suffix=".eps")
                    eps_file_list.append( (handle, path) )
                    latex_formulas.append( n.latex_label )
                dot += n.label + " [label=\"\"; image=\"" + path + "\"];\n"

        dot     += "}\n"
        return dot, eps_file_list, latex_formulas


    def generateVisualLayout(self):
        dot = self.generateDotFile(draw_unconnected=False)


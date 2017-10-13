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

import os
import subprocess

def exportToEpsList(eps_file_list, latex_formulas):
    in_read, in_write = os.pipe()
    os.write(in_write, "\\documentclass{minimal}\n")
    os.write(in_write, "\\usepackage{amsmath}\n")
    os.write(in_write, "\\usepackage{mathtools}\n")
    os.write(in_write, "\\usepackage{lmodern}\n")
    os.write(in_write, "\\begin{document}\n")

    new_page = False
    for f in latex_formulas:
        if new_page:
            os.write(in_write, "\\clearpage\n")
        new_page = True
        os.write(in_write, "\\begin{gather*}" + f + "\\end{gather*}\n")

    os.write(in_write, "\\end{document}\n")
    os.close(in_write)

    # generate dvi file from latex document
    subprocess.call(['latex', '-output-directory=/tmp'], stdin=in_read)

    page_num = 1
    for (handle, path) in eps_file_list:
        subprocess.call(['dvips', '/tmp/texput.dvi', '-pp', str(page_num), '-o', '/tmp/texput.ps'])
        subprocess.call(['ps2eps', '/tmp/texput.ps', '-f'])
        with open('/tmp/texput.eps', 'r') as infile:
            data = infile.read()
        with open(path, 'w') as outfile:
            outfile.write(data)
        page_num += 1

    try:
        os.remove('/tmp/texput.dvi')
    except:
        pass

    try:
        os.remove('/tmp/texput.ps')
    except:
        pass

    try:
        os.remove('/tmp/texput.eps')
    except:
        pass

#def convertEpsListToPdf(eps_file_list):
#    for (handle, path) in eps_file_list:
#        head, tail = os.path.split(path)
#        subprocess.call(['epspdf', path, tail+".pdf"])

def toMathText(text):
    return '\\text{' + text.replace('_', '\_') + '}'

def removeEpsListFiles(eps_file_list):
    for (handle, file_name) in eps_file_list:
        os.close(handle)
        os.remove(file_name)


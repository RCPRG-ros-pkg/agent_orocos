#!/usr/bin/env python

## Runs FABRIC introspection tool.
# @file rqt_agent.py

import sys

from rqt_agent.my_module import FabricSystemViewPlugin
from rqt_gui.main import Main

plugin = 'rqt_agent'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))

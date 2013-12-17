#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __                           
#  +------+      / __ )(_) /_______________ _____  ___ 
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2011-2013 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

"""
Showns the Log TOC of available variables in the Crazyflie.
"""

__author__ = 'Bitcraze AB'
__all__ = ['MapTab']

import sys, time

from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtCore import Qt, pyqtSlot, pyqtSignal, QThread, SIGNAL

from cflib.crazyflie import Crazyflie

from cfclient.ui.tab import Tab
from cfclient.utils.GPS import GpsPoller

from cfclient.utils.guiconfig import GuiConfig

from cfclient.utils.periodictimer import PeriodicTimer

param_tab_class = uic.loadUiType(sys.path[0] + "/cfclient/ui/tabs/MapTab.ui")[0]

gpsd = None

class MapTab(Tab, param_tab_class):
    connectedSignal = pyqtSignal(str)
    disconnectedSignal = pyqtSignal(str)

    def __init__(self, tabWidget, helper, *args):
        super(MapTab, self).__init__(*args)
        self.setupUi(self)

        self.tabName = "Map"
        self.menuName = "Map"

        self.helper = helper
        self.tabWidget = tabWidget

        self.cf = helper.cf
        
        #Init the tree widget
        #self.logTree.setHeaderLabels(['Name', 'ID', 'Unpack', 'Storage'])

        self.cf.connectSetupFinished.add_callback(self.connectedSignal.emit)
        self.connectedSignal.connect(self.connected)

        # Clear the log TOC list when the Crazyflie is disconnected
        self.cf.disconnected.add_callback(self.disconnectedSignal.emit)
        self.disconnectedSignal.connect(self.disconnected)

    @pyqtSlot('QString')
    def disconnected(self, linkname):
        self.curLat.value = 7

    @pyqtSlot(str)
    def connected(self, linkURI):
        self.curLat.value = 8
        



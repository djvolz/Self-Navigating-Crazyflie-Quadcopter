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
__all__ = ['GpsTab']

import sys, time
import simplejson, urllib

from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtCore import Qt, pyqtSlot, pyqtSignal, QThread, SIGNAL

from cflib.crazyflie import Crazyflie

from cfclient.ui.tab import Tab
from cfclient.utils.GPS import GpsPoller

from cfclient.utils.guiconfig import GuiConfig

from cfclient.utils.periodictimer import PeriodicTimer
from cfclient.utils.input import JoystickReader
import showmap, math

param_tab_class = uic.loadUiType(sys.path[0] + "/cfclient/ui/tabs/GpsTab.ui")[0]

gpsd = None

class GpsTab(Tab, param_tab_class):
    connectedSignal = pyqtSignal(str)
    disconnectedSignal = pyqtSignal(str)
    
    destLatVal = 0
    destLongVal = 0
    def __init__(self, tabWidget, helper, *args):
        super(GpsTab, self).__init__(*args)
        self.setupUi(self)

        self.tabName = "GPS"
        self.menuName = "GPS"

        self.helper = helper
        self.tabWidget = tabWidget
        
        self.GPS = GpsPoller()
        self.GPS.start()
        
        self.gpsTimer = PeriodicTimer(1.0, self.updateCoords)
        self.gpsTimer.start()

        self.cf = helper.cf
        
        #Init the tree widget
        #self.logTree.setHeaderLabels(['Name', 'ID', 'Unpack', 'Storage'])

        self.cf.connectSetupFinished.add_callback(self.connectedSignal.emit)
        self.connectedSignal.connect(self.connected)
        
        self.updateDest.clicked.connect(self.destCoordsChanged)
        self.revertChanges.clicked.connect(self.destRevertChanges)

        # Clear the log TOC list when the Crazyflie is disconnected
        self.cf.disconnected.add_callback(self.disconnectedSignal.emit)
        self.disconnectedSignal.connect(self.disconnected)
        
        mapHTML = showmap.generateHTML(29.71984,-95.398087,29.732395,-95.394824)
        self.webView.setHtml(mapHTML)

    @pyqtSlot('QString')
    def disconnected(self, linkname):
        self.curLat.value = 0

    @pyqtSlot(str)
    def connected(self, linkURI):
        self.curLat.value = 0
        
    def destCoordsChanged(self):
        self.destLatVal = self.destLat.text().toFloat()[0]
        self.destLongVal = self.destLong.text().toFloat()[0]
        url="""
http://maps.googleapis.com/maps/api/directions/json?origin={0},{1}&destination={2},{3}&sensor=false
""".format(29.71984,-95.398087,29.732395,-95.394824)
        dirsResult=simplejson.load(urllib.urlopen(url))
        dirsResult=dirsResult["routes"][0]["legs"][0]["steps"]
        for step in dirsResult:
		    loc = step["end_location"]
		    JoystickReader.controller.getDestinationCoords(loc["lat"], loc["lng"])
        #latitude = self.GPS.getGpsd().fix.latitude
        #longitude = self.GPS.getGpsd().fix.longitude
        latitude = 29.7198
        longitude = -95.398087
        print("%0.5f"%self.destLatVal)
        print("%0.5f"%self.destLongVal)
        mapHTML = showmap.generateHTML(latitude,longitude,self.destLatVal,self.destLongVal)
        self.webView.setHtml(mapHTML)
    
    def destRevertChanges(self):
        self.destLat.setText(("%0.10f" %
                                   self.destLatVal))
        self.destLong.setText(("%0.10f" %
                                   self.destLongVal))

    def updateCoords(self):
        latitude = self.GPS.getGpsd().fix.latitude
        longitude = self.GPS.getGpsd().fix.longitude
        self.curLat.setText(("%0.10f" %
                                   latitude))
        self.curLong.setText(("%0.10f" %
                                   longitude))
        JoystickReader.controller.getCurrentCoords(latitude, longitude)  
        #GuiConfig().set("curLat", latitude)
        #GuiConfig().set("curLong", longitude)
  


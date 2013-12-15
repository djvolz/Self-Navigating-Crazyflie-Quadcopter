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
The flight control tab shows telimitry data and flight settings.
"""

__author__ = 'Bitcraze AB'
__all__ = ['FlightTab']

import sys
import math

import logging
logger = logging.getLogger(__name__)

from time import time

from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtCore import Qt, pyqtSlot, pyqtSignal, QThread, SIGNAL
from PyQt4.QtGui import QMessageBox

from cflib.crazyflie import Crazyflie

from cfclient.ui.widgets.ai import AttitudeIndicator
from cfclient.ui.widgets.compass import Compass

from cfclient.utils.guiconfig import GuiConfig
from cflib.crazyflie.log import Log

from cfclient.ui.tab import Tab

from cfclient.utils.logconfigreader import LogVariable, LogConfig

from math import pi, sin, cos

from cfclient.utils.input import JoystickReader

flight_tab_class = uic.loadUiType(sys.path[0] +
                                  "/cfclient/ui/tabs/flightTab.ui")[0]

MAX_THRUST = 65365.0


class FlightTab(Tab, flight_tab_class):

    uiSetupReadySignal = pyqtSignal()

    _motor_data_signal = pyqtSignal(object, int)
    _imu_data_signal = pyqtSignal(object, int)
    _althold_data_signal = pyqtSignal(object, int)
    _baro_data_signal = pyqtSignal(object, int)
    _heading_data_signal = pyqtSignal(object, int)

    _input_updated_signal = pyqtSignal(float, float, float, float)
    _rp_trim_updated_signal = pyqtSignal(float, float)
    _emergency_stop_updated_signal = pyqtSignal(bool)

    _log_error_signal = pyqtSignal(object, str)

    #UI_DATA_UPDATE_FPS = 10

    connectionFinishedSignal = pyqtSignal(str)
    disconnectedSignal = pyqtSignal(str)

    def __init__(self, tabWidget, helper, *args):
        super(FlightTab, self).__init__(*args)
        self.setupUi(self)

        self.tabName = "Flight Control"
        self.menuName = "Flight Control"

        self.tabWidget = tabWidget
        self.helper = helper

        self.disconnectedSignal.connect(self.disconnected)
        self.connectionFinishedSignal.connect(self.connected)
        # Incomming signals
        self.helper.cf.connectSetupFinished.add_callback(
                                     self.connectionFinishedSignal.emit)
        self.helper.cf.disconnected.add_callback(self.disconnectedSignal.emit)

        self._input_updated_signal.connect(self.updateInputControl)
        self.helper.inputDeviceReader.input_updated.add_callback(
                                     self._input_updated_signal.emit)
        self._rp_trim_updated_signal.connect(self.calUpdateFromInput)
        self.helper.inputDeviceReader.rp_trim_updated.add_callback(
                                     self._rp_trim_updated_signal.emit)
        self._emergency_stop_updated_signal.connect(self.updateEmergencyStop)
        self.helper.inputDeviceReader.emergency_stop_updated.add_callback(
                                     self._emergency_stop_updated_signal.emit)
        
        self.helper.inputDeviceReader.althold_updated.add_callback(
                    lambda enabled: self.helper.cf.param.set_value("flightmode.althold", enabled))

        self._imu_data_signal.connect(self._imu_data_received)
        self._baro_data_signal.connect(self._baro_data_received)
        self._althold_data_signal.connect(self._althold_data_received)
        self._motor_data_signal.connect(self._motor_data_received)
        self._heading_data_signal.connect(self._heading_data_received)

        self._log_error_signal.connect(self._logging_error)

        # Connect UI signals that are in this tab
        self.flightModeCombo.currentIndexChanged.connect(self.flightmodeChange)
        self.minThrust.valueChanged.connect(self.minMaxThrustChanged)
        self.maxThrust.valueChanged.connect(self.minMaxThrustChanged)
        self.thrustLoweringSlewRateLimit.valueChanged.connect(
                                      self.thrustLoweringSlewRateLimitChanged)
        self.slewEnableLimit.valueChanged.connect(
                                      self.thrustLoweringSlewRateLimitChanged)
        self.targetCalRoll.valueChanged.connect(self._trim_roll_changed)
        self.targetCalPitch.valueChanged.connect(self._trim_pitch_changed)
        self.maxAngle.valueChanged.connect(self.maxAngleChanged)
        self.maxYawRate.valueChanged.connect(self.maxYawRateChanged)
        self.uiSetupReadySignal.connect(self.uiSetupReady)
        self.clientXModeCheckbox.toggled.connect(self.changeXmode)
        self.isInCrazyFlightmode = False
        self.uiSetupReady()

        self.clientXModeCheckbox.setChecked(GuiConfig().get("client_side_xmode"))

        self.crazyflieXModeCheckbox.clicked.connect(
                             lambda enabled:
                             self.helper.cf.param.set_value("flightmode.x",
                                                            str(enabled)))
        self.helper.cf.param.add_update_callback(
                        group="flightmode", name="xmode",
                        cb=( lambda name, checked:
                        self.crazyflieXModeCheckbox.setChecked(eval(checked))))
        self.ratePidRadioButton.clicked.connect(
                    lambda enabled:
                    self.helper.cf.param.set_value("flightmode.ratepid",
                                                   str(enabled)))
        self.angularPidRadioButton.clicked.connect(
                    lambda enabled:
                    self.helper.cf.param.set_value("flightmode.ratepid",
                                                   str(not enabled)))
        self.helper.cf.param.add_update_callback(
                    group="flightmode", name="ratepid",
                    cb=(lambda name, checked:
                    self.ratePidRadioButton.setChecked(eval(checked))))
        
        self.helper.cf.param.add_update_callback(
                    group="flightmode", name="althold",
                    cb=(lambda name, enabled:
                    self.helper.inputDeviceReader.setAltHold(eval(enabled))))

        self.helper.cf.param.add_update_callback(
                        group="imu_sensors",
                        cb=self._set_available_sensors)
                
        self.logBaro = None
        self.logAltHold = None

        self.ai = AttitudeIndicator()
        #self.verticalLayout_4.addWidget(self.ai)
        #self.splitter.setSizes([1000,1])
        self.gridLayout.addWidget(self.ai, 0, 1)
        
        self.compass = Compass()
        self.gridLayout.addWidget(self.compass, 0, 2)

        self.targetCalPitch.setValue(GuiConfig().get("trim_pitch"))
        self.targetCalRoll.setValue(GuiConfig().get("trim_roll"))
        
        self.apitch = 0
        self.aroll = 0
        self.motor_power = 0

    def thrustToPercentage(self, thrust):
        return ((thrust / MAX_THRUST) * 100.0)

    def uiSetupReady(self):
        flightComboIndex = self.flightModeCombo.findText(
                             GuiConfig().get("flightmode"), Qt.MatchFixedString)
        if (flightComboIndex < 0):
            self.flightModeCombo.setCurrentIndex(0)
            self.flightModeCombo.currentIndexChanged.emit(0)
        else:
            self.flightModeCombo.setCurrentIndex(flightComboIndex)
            self.flightModeCombo.currentIndexChanged.emit(flightComboIndex)

    def _logging_error(self, log_conf, msg):
        QMessageBox.about(self, "Log error", "Error when starting log config"
                " [%s]: %s" % (log_conf.getName(), msg))

    def _motor_data_received(self, data, timestamp):
        self.actualM1.setValue(data["motor.m1"])
        self.actualM2.setValue(data["motor.m2"])
        self.actualM3.setValue(data["motor.m3"])
        self.actualM4.setValue(data["motor.m4"])
        self.motor_power = data["motor.m1"] + data["motor.m2"] + data["motor.m3"] + data["motor.m4"]
        
    def _baro_data_received(self, data, timestamp):
        self.actualASL.setText(("%.2f" % data["baro.aslLong"]))
        self.ai.setBaro(data["baro.aslLong"])
        
    def _althold_data_received(self, data, timestamp):       
        target = data["altHold.target"]
        if target>0:
            if not self.targetASL.isEnabled():
                self.targetASL.setEnabled(True) 
            self.targetASL.setText(("%.2f" % target))
            self.ai.setHover(target)    
        elif self.targetASL.isEnabled():
            self.targetASL.setEnabled(False)
            self.targetASL.setText("Not set")   
            self.ai.setHover(0)    
        
    def _imu_data_received(self, data, timestamp):
        self.aroll = data["stabilizer.roll"]
        self.apitch = data["stabilizer.pitch"]
        self.actualRoll.setText(("%.2f" % data["stabilizer.roll"]))
        self.actualPitch.setText(("%.2f" % data["stabilizer.pitch"]))
        self.actualYaw.setText(("%.2f" % data["stabilizer.yaw"]))
        self.actualThrust.setText("%.2f%%" %
                                  self.thrustToPercentage(
                                                  data["stabilizer.thrust"]))

        self.ai.setRollPitch(-data["stabilizer.roll"],
                             data["stabilizer.pitch"])
                           
    def _heading_data_received(self, data, timestamp):
    # hard and soft correction values generated by Processing Magnetometer_calibration script + calibrate_hardsoft.py
        # magn_ellipsoid_center = [1341.66, -537.690, 41.1584]
        # magn_ellipsoid_transform = [[0.934687, 0.0222809, 0.0151145], [0.0222809, 0.919365, -0.00622367], [0.0151145, -0.00622367, 0.996487]]                                
        magn_ellipsoid_center = [-170.956, -1056.30, 19.4655]
        magn_ellipsoid_transform = [[0.852266, 0.00526498, 0.0195745], [0.00526498, 0.888268, -0.00355351], [0.0195745, -0.00355351, 0.997333]]


        # values generated by calibrate_powered.py
        qx = [0.067946222436498283, -0.25034004667098259, 8.3336994198409666, -0.17762637163222378]
        qy = [-0.13945102271766135, 2.9074808469097495, 1.6764850422889934, 0.19244505046927501]
        qz = [0.018800599305554239, -0.79590273035713055, -3.1033531112103478, 0.13550993988096199]
                
        # matrix by vector multiplication
        def mv(a, b):
            out = [0,0,0]
            for x in range(0, 3):
                out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
            return out 
        
        # calculate adjustments related to how much power is sent to the motors      
        def adj(qs, power):
            p = float(power) / float(40000) # 10k * 4 motors
            return qs[0]*p**3+qs[1]*p**2+qs[2]*p+qs[3]

        x, y, z = data['heading.x'], data['heading.y'], data['heading.z']

        x = x - magn_ellipsoid_center[0]
        y = y - magn_ellipsoid_center[1]
        z = z - magn_ellipsoid_center[2]
        x, y, z = mv(magn_ellipsoid_transform, [x, y, z])

        x = x + adj(qx, self.motor_power)
        y = y + adj(qy, self.motor_power)
        z = z + adj(qz, self.motor_power)
        
        # correct magnetometer orientation relative to the CF orientation
        x, y, z = y, x, z * -1

        # calculate tilt-compensated heading angle        
        cosRoll = cos(math.radians(self.aroll))
        sinRoll = sin(math.radians(self.aroll))  
        cosPitch = cos(math.radians(self.apitch))
        sinPitch = sin(math.radians(self.apitch))
  
        Xh = x * cosPitch + z * sinPitch
        Yh = x * sinRoll * sinPitch + y * cosRoll - z * sinRoll * cosPitch
  
        heading = math.atan2(Yh, Xh)
        d_heading = math.degrees(heading) * -1 # for some reason getting inveted sign here
        JoystickReader.controller.getCFHeading(d_heading)
        
        # update compass widget
        self.compass.setAngle(d_heading)

        # lock heading to 0 degrees north
        if d_heading > 0:
            yaw_trim = -20
        else:
            yaw_trim = 20
        #self.helper.inputDeviceReader.update_trim_yaw_signal.emit(yaw_trim)

    def connected(self, linkURI):
        # IMU & THRUST
        lg = LogConfig("Stabalizer", 200)
        lg.addVariable(LogVariable("stabilizer.roll", "float"))
        lg.addVariable(LogVariable("stabilizer.pitch", "float"))
        lg.addVariable(LogVariable("stabilizer.yaw", "float"))
        lg.addVariable(LogVariable("stabilizer.thrust", "uint16_t"))

        self.log = self.helper.cf.log.create_log_packet(lg)
        if (self.log is not None):
            self.log.data_received.add_callback(self._imu_data_signal.emit)
            self.log.error.add_callback(self._log_error_signal.emit)
            self.log.start()
        else:
            logger.warning("Could not setup logconfiguration after "
                           "connection!")

        # MOTOR
        lg = LogConfig("Motors", 200)
        lg.addVariable(LogVariable("motor.m1", "uint32_t"))
        lg.addVariable(LogVariable("motor.m2", "uint32_t"))
        lg.addVariable(LogVariable("motor.m3", "uint32_t"))
        lg.addVariable(LogVariable("motor.m4", "uint32_t"))

        self.log = self.helper.cf.log.create_log_packet(lg)
        if (self.log is not None):
            self.log.data_received.add_callback(self._motor_data_signal.emit)
            self.log.error.add_callback(self._log_error_signal.emit)
            self.log.start()
        else:
            logger.warning("Could not setup logconfiguration after "
                           "connection!")
                        
        # HEADING
        lg = LogConfig("Heading", 200)
        lg.addVariable(LogVariable("heading.x", "int16_t"))
        lg.addVariable(LogVariable("heading.y", "int16_t"))
        lg.addVariable(LogVariable("heading.z", "int16_t"))

        self.log = self.helper.cf.log.create_log_packet(lg)
        if (self.log is not None):
            self.log.data_received.add_callback(self._heading_data_signal.emit)
            self.log.error.add_callback(self._log_error_signal.emit)
            self.log.start()
        else:
            logger.warning("Could not setup logconfiguration after "
                           "connection!")
            
    def _set_available_sensors(self, name, available):
        logger.info("[%s]: %s", name, available)
        available = eval(available)
        if ("HMC5883L" in name):
            if (not available):
                self.actualASL.setText("N/A")
                self.actualASL.setEnabled(False)
            else:
                self.actualASL.setEnabled(True)
                self.helper.inputDeviceReader.setAltHoldAvailable(available)
                if (not self.logBaro and not self.logAltHold):
                    # The sensor is available, set up the logging
                    lg = LogConfig("Baro", 200)
                    lg.addVariable(LogVariable("baro.aslLong", "float"))

                    self.logBaro = self.helper.cf.log.create_log_packet(lg)
                    if (self.logBaro is not None):
                        self.logBaro.data_received.add_callback(self._baro_data_signal.emit)
                        self.logBaro.error.add_callback(self._log_error_signal.emit)
                        self.logBaro.start()
                    else:
                        logger.warning("Could not setup logconfiguration after "
                                       "connection!")            
                    lg = LogConfig("AltHold", 200)
                    lg.addVariable(LogVariable("altHold.target", "float"))

                    self.logAltHold = self.helper.cf.log.create_log_packet(lg)
                    if (self.logAltHold is not None):
                        self.logAltHold.data_received.add_callback(self._althold_data_signal.emit)
                        self.logAltHold.error.add_callback(self._log_error_signal.emit)
                        self.logAltHold.start()
                    else:
                        logger.warning("Could not setup logconfiguration after "
                                       "connection!")                        

    def disconnected(self, linkURI):
        self.ai.setRollPitch(0, 0)
        self.actualM1.setValue(0)
        self.actualM2.setValue(0)
        self.actualM3.setValue(0)
        self.actualM4.setValue(0)
        self.actualRoll.setText("")
        self.actualPitch.setText("")
        self.actualYaw.setText("")
        self.actualThrust.setText("")
        self.actualASL.setText("")
        self.targetASL.setText("Not Set")
        self.targetASL.setEnabled(False)
        self.actualASL.setEnabled(False)
        self.logBaro = None
        self.logAltHold = None

    def minMaxThrustChanged(self):
        self.helper.inputDeviceReader.set_thrust_limits(
                            self.minThrust.value(), self.maxThrust.value())
        if (self.isInCrazyFlightmode == True):
            GuiConfig().set("min_thrust", self.minThrust.value())
            GuiConfig().set("max_thrust", self.maxThrust.value())

    def thrustLoweringSlewRateLimitChanged(self):
        self.helper.inputDeviceReader.set_thrust_slew_limiting(
                            self.thrustLoweringSlewRateLimit.value(),
                            self.slewEnableLimit.value())
        if (self.isInCrazyFlightmode == True):
            GuiConfig().set("slew_limit", self.slewEnableLimit.value())
            GuiConfig().set("slew_rate", self.thrustLoweringSlewRateLimit.value())

    def maxYawRateChanged(self):
        logger.debug("MaxYawrate changed to %d", self.maxYawRate.value())
        self.helper.inputDeviceReader.set_yaw_limit(self.maxYawRate.value())
        if (self.isInCrazyFlightmode == True):
            GuiConfig().set("max_yaw", self.maxYawRate.value())

    def maxAngleChanged(self):
        logger.debug("MaxAngle changed to %d", self.maxAngle.value())
        self.helper.inputDeviceReader.set_rp_limit(self.maxAngle.value())
        if (self.isInCrazyFlightmode == True):
            GuiConfig().set("max_rp", self.maxAngle.value())

    def _trim_pitch_changed(self, value):
        logger.debug("Pitch trim updated to [%f]" % value)
        self.helper.inputDeviceReader.set_trim_pitch(value)
        GuiConfig().set("trim_pitch", value)

    def _trim_roll_changed(self, value):
        logger.debug("Roll trim updated to [%f]" % value)
        self.helper.inputDeviceReader.set_trim_roll(value)
        GuiConfig().set("trim_roll", value)

    def calUpdateFromInput(self, rollCal, pitchCal):
        logger.debug("Trim changed on joystick: roll=%.2f, pitch=%.2f",
                     rollCal, pitchCal)
        self.targetCalRoll.setValue(rollCal)
        self.targetCalPitch.setValue(pitchCal)

    def updateInputControl(self, roll, pitch, yaw, thrust):
        self.targetRoll.setText(("%0.2f" % roll))
        self.targetPitch.setText(("%0.2f" % pitch))
        self.targetYaw.setText(("%0.2f" % yaw))
        self.targetThrust.setText(("%0.2f %%" %
                                   self.thrustToPercentage(thrust)))
        self.thrustProgress.setValue(thrust)

    def setMotorLabelsEnabled(self, enabled):
        self.actualM1.setEnabled(enabled)
        self.actualM2.setEnabled(enabled)
        self.actualM3.setEnabled(enabled)
        self.actualM4.setEnabled(enabled)

    def emergencyStopStringWithText(self, text):
        return ("<html><head/><body><p>"
                "<span style='font-weight:600; color:#7b0005;'>{}</span>"
                "</p></body></html>".format(text))

    def updateEmergencyStop(self, emergencyStop):
        if emergencyStop:
            self.setMotorLabelsEnabled(False)
            self.emergency_stop_label.setText(
                      self.emergencyStopStringWithText("Kill switch active"))
        else:
            self.setMotorLabelsEnabled(True)
            self.emergency_stop_label.setText("")

    def flightmodeChange(self, item):
        GuiConfig().set("flightmode", self.flightModeCombo.itemText(item))
        logger.info("Changed flightmode to %s",
                    self.flightModeCombo.itemText(item))
        self.isInCrazyFlightmode = False
        if (item == 0):  # Normal
            self.maxAngle.setValue(GuiConfig().get("normal_max_rp"))
            self.maxThrust.setValue(GuiConfig().get("normal_max_thrust"))
            self.minThrust.setValue(GuiConfig().get("normal_min_thrust"))
            self.slewEnableLimit.setValue(GuiConfig().get("normal_slew_limit"))
            self.thrustLoweringSlewRateLimit.setValue(
                                              GuiConfig().get("normal_slew_rate"))
            self.maxYawRate.setValue(GuiConfig().get("normal_max_yaw"))
        if (item == 1):  # Advanced
            self.maxAngle.setValue(GuiConfig().get("max_rp"))
            self.maxThrust.setValue(GuiConfig().get("max_thrust"))
            self.minThrust.setValue(GuiConfig().get("min_thrust"))
            self.slewEnableLimit.setValue(GuiConfig().get("slew_limit"))
            self.thrustLoweringSlewRateLimit.setValue(
                                                  GuiConfig().get("slew_rate"))
            self.maxYawRate.setValue(GuiConfig().get("max_yaw"))
            self.isInCrazyFlightmode = True

        if (item == 0):
            newState = False
        else:
            newState = True
        self.maxThrust.setEnabled(newState)
        self.maxAngle.setEnabled(newState)
        self.minThrust.setEnabled(newState)
        self.thrustLoweringSlewRateLimit.setEnabled(newState)
        self.slewEnableLimit.setEnabled(newState)
        self.maxYawRate.setEnabled(newState)

    @pyqtSlot(bool)
    def changeXmode(self, checked):
        self.helper.cf.commander.set_client_xmode(checked)
        GuiConfig().set("client_side_xmode", checked)
        logger.info("Clientside X-mode enabled: %s", checked)

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
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.

"""
Driver for reading data from the PyGame API. Used from Inpyt.py for reading input data.
Hacked to include AI 

You will need to modify the following files as shown below
+++ b/lib/cfclient/ui/main.py   
-        self.joystickReader = JoystickReader()
+        self.joystickReader = JoystickReader(cf=self.cf)


+++ b/lib/cfclient/utils/input.py   
+from cfclient.utils.aicontroller import AiController 

-    def __init__(self, do_device_discovery=True):
+    def __init__(self, do_device_discovery=True, cf=None):

-        self.inputdevice = PyGameReader()
+        self.inputdevice = AiController(cf)

You will also need to map the "exit" button to your controller.  This will server as 
the on/off switch for the AI.

You will also likely have to open the tools->parameters tab in the PC-Client which will load the TOC.  

"""

__author__ = 'Dog Copter'
__all__ = ['AiController']

import pygame
from pygame.locals import *

from PyQt4.QtCore import pyqtSignal

import time
import logging
import math

logger = logging.getLogger(__name__)

class AiController():
    """Used for reading data from input devices using the PyGame API."""
    
    def __init__(self,cf):
        self.cf = cf
        self.inputMap = None
        pygame.init()

        # AI variables
        self.timer1 = 0
        self.lastTime = 0

        # ---AI tuning variables---
        # This is the thrust of the motors duing hover.  0.5 reaches ~1ft depending on battery
        self.maxThrust = .85
        # Determines how fast to take off
        self.thrustInc = 0.021
        self.takeoffTime = 0.5
        # Determines how fast to land
        self.thrustDec = -0.019
        self.hoverTime = 2
        # Sets the delay between test flights
        self.repeatDelay = 2

        # parameters pulled from json with defaults from crazyflie pid.h
        # perl -ne '/"(\w*)": {/ && print $1,  "\n" ' lib/cflib/cache/27A2C4BA.json
        self.cfParams = {
            'pid_rate.pitch_kp': 0.0, 
            'pid_rate.pitch_kd': 0.0, 
            'pid_rate.pitch_ki': 0.0, 
            'pid_rate.roll_kp': 0, 
            'pid_rate.roll_kd': 0.0, 
            'pid_rate.roll_ki': 0.0, 
            'pid_rate.yaw_kp': 0, 
            'pid_rate.yaw_kd': 0.0, 
            'pid_rate.yaw_ki': 0, 
            # 'pid_attitude.pitch_kp': 0, 
            # 'pid_attitude.pitch_kd': 0.0, 
            # 'pid_attitude.pitch_ki': 0, 
            # 'pid_attitude.roll_kp': 0, 
            # 'pid_attitude.roll_kd': 0.0, 
            # 'pid_attitude.roll_ki': 0, 
            # 'pid_attitude.yaw_kp': 0.0, 
            # 'pid_attitude.yaw_kd': 0.0, 
            # 'pid_attitude.yaw_ki': 0.0, 
            'pid_attitude.pitch_kp': 3.5, 
            'pid_attitude.pitch_kd': 0.0, 
            'pid_attitude.pitch_ki': 2.0, 
            'pid_attitude.roll_kp': 3.5, 
            'pid_attitude.roll_kd': 0.0, 
            'pid_attitude.roll_ki': 2.0, 
            'pid_attitude.yaw_kp': 0.0, 
            'pid_attitude.yaw_kd': 0.0, 
            'pid_attitude.yaw_ki': 0.0, 
            'sensorfusion6.kp': 0.800000011921, 
            'sensorfusion6.ki': 0.00200000009499, 
            'imu_acc_lpf.factor': 32 }
            
        self.rollError = []
        self.pitchError = []
        self.yawError = []

        self.currentLat = []
        self.currentLong = []
        self.destinationLat = []
        self.destinationLong = []
        
        self.rollHistory = []
        self.pitchHistory = []
        self.yawHistory = []
        self.paramHistory = []
        
        rollDelta = 5;
        pitchDelta = 5;
        yawDelta = 5;
        rollDone = False
        pitchDone = False
        yawDone = False
        self.tuningPhase = 0
        self.tuned = False

    def readInput(self):
        """Read input from the selected device."""

        # First we read data from controller as normal
        # ----------------------------------------------------
        # We only want the pitch/roll cal to be "oneshot", don't
        # save this value.
        self.data["pitchcal"] = 0.0
        self.data["rollcal"] = 0.0
        for e in pygame.event.get():
          if e.type == pygame.locals.JOYAXISMOTION:
            index = "Input.AXIS-%d" % e.axis 
            try:
                if (self.inputMap[index]["type"] == "Input.AXIS"):
                    key = self.inputMap[index]["key"]
                    axisvalue = self.j.get_axis(e.axis)
                    # All axis are in the range [-a,+a]
                    axisvalue = axisvalue * self.inputMap[index]["scale"]
                    # The value is now in the correct direction and in the range [-1,1]
                    self.data[key] = axisvalue
            except Exception:
                # Axis not mapped, ignore..
                pass          

          if e.type == pygame.locals.JOYBUTTONDOWN:
            index = "Input.BUTTON-%d" % e.button 
            try:
                if (self.inputMap[index]["type"] == "Input.BUTTON"):
                    key = self.inputMap[index]["key"]
                    if (key == "estop"):
                        self.data["estop"] = not self.data["estop"]
                    elif (key == "exit"):
                        # self.data["exit"] = True
                        self.data["exit"] = not self.data["exit"]
                        logger.info("Toggling AI %d", self.data["exit"])
                    elif (key == "althold"):
                        self.data["althold"] = not self.data["althold"]
                        logger.info("Toggling AI %d", self.data["althold"])
                    else: # Generic cal for pitch/roll
                        self.data[key] = self.inputMap[index]["scale"]
            except Exception:
                # Button not mapped, ignore..
                pass


        # Second if AI is enabled overwrite selected data with AI
        # ----------------------------------------------------------
        if self.data["exit"]:
            self.augmentInputWithAi()

        # Return control Data
        return self.data


    # ELEC424 TODO:  Improve this function as needed
    def augmentInputWithAi(self):
        """
        Overrides the throttle input with a controlled takeoff, hover, and land loop.
        You will to adjust the tuning vaiables according to your crazyflie.  
        The max thrust has been set to 0.3 and likely will not fly.  
        I have found that a value  of 0.5 will reach about 1ft off the ground 
        depending on the battery's charge.
        """

        # Keep track of time
        currentTime = time.time()
        timeSinceLastAi = currentTime - self.lastTime
        self.timer1 = self.timer1 + timeSinceLastAi
        self.lastTime = currentTime
        
        # Basic AutoPilot steadly increase thrust, hover, land and repeat
        # -------------------------------------------------------------
        # delay before takeoff 
        if self.timer1 < 0:
            thrustDelta = 0
            # clear the error lists before takeoff
            self.rollError = []
            self.pitchError = []
            self.yawError = []
        # takeoff
        elif self.timer1 < self.takeoffTime :
            thrustDelta = self.thrustInc
        # hold
        elif self.timer1 < self.takeoffTime + self.hoverTime : 
            thrustDelta = 0
        # land
        elif self.timer1 < 2 * self.takeoffTime + self.hoverTime :
            thrustDelta = self.thrustDec
        # repeat
        else:
            self.timer1 = -self.repeatDelay
            thrustDelta = 0
            # Example Call to pidTuner
            self.pidTuner()
            if(self.tuned):
                print "Tuning is complete!"
                print self.cfParams
                return


        self.addThrust( thrustDelta )


        # override Other inputs as needed
        # --------------------------------------------------------------
        # self.data["roll"] = self.aiData["roll"]
        # self.data["pitch"] = self.aiData["pitch"]
        # self.data["yaw"] = self.aiData["yaw"]
        # self.data["pitchcal"] = self.aiData["pitchcal"]
        # self.data["rollcal"] = self.aiData["rollcal"]
        # self.data["estop"] = self.aiData["estop"]
        # self.data["exit"] = self.aiData["exit"]

    def addThrust(self, thrustDelta):
        # Increment thrust
        self.aiData["thrust"] = self.aiData["thrust"] + thrustDelta 
        # Check for max
        if self.aiData["thrust"] > self.maxThrust:
            self.aiData["thrust"] = self.maxThrust
        # check for min 
        elif self.aiData["thrust"] < 0:
            self.aiData["thrust"] = 0
        
        # overwrite joystick thrust values
        self.data["thrust"] = self.aiData["thrust"]


    # ELEC424 TODO: Implement this function
    def pidTuner(self):
        """ 
        example on how to update crazyflie params
        """
        rollTotalError = sum([abs(x) for x in self.rollError])
        self.rollHistory.append(rollTotalError)
        pitchTotalError = sum([abs(x) for x in self.pitchError])
        self.pitchHistory.append(pitchTotalError)
        yawTotalError = sum([abs(x) for x in self.yawError])
        self.yawHistory.append(yawTotalError)
        mode = 'pid_rate' #'pid_attitude'
        self.paramHistory.append((self.cfParams['pid_rate.roll_kp'], self.cfParams['pid_rate.pitch_kp'], self.cfParams['pid_rate.yaw_kp']))
        
        if((len(self.rollHistory) < 2) or (len(self.pitchHistory) < 2) or (len(self.yawHistory) < 2)):
            self.rollDelta = 5;
            self.pitchDelta = 5;
            self.yawDelta = 5;
            self.rollDone = False
            self.pitchDone = False
            self.yawDone = False
            return
        
        if(self.rollHistory[-1] > self.rollHistory[-2]):
            rollDelta = -rollDelta;
        self.cfParams['pid_rate.roll_kp'] = self.rollDelta + self.cfParams['pid_rate.roll_kp']
        print "roll", rollTotalError, " : ", self.cfParams['pid_rate.roll_kp']
        if(len(self.rollHistory) > 10):
            netSwitches = 0
            for i in xrange(1, 9):
                if(self.rollHistory[-i] > self.rollHistory[-(i+1)]):
                    netSwitches += 1
                else:
                    netSwitches -= 1
            if(netSwitches < 3):
                self.rollDone = True
        
        if(self.pitchHistory[-1] > self.pitchHistory[-2]):
            pitchDelta = -pitchDelta;
        self.cfParams['pid_rate.pitch_kp'] = self.pitchDelta + self.cfParams['pid_rate.pitch_kp']
        print "pitch", pitchTotalError, " : ", self.cfParams['pid_rate.pitch_kp']
        if(len(self.pitchHistory) > 10):
            netSwitches = 0
            for i in xrange(1, 9):
                if(self.pitchHistory[-i] > self.pitchHistory[-(i+1)]):
                    netSwitches += 1
                else:
                    netSwitches -= 1
            if(netSwitches < 3):
                self.pitchDone = True
                        
        if(self.yawHistory[-1] > self.yawHistory[-2]):
            yawDelta = -yawDelta;
        self.cfParams['pid_rate.yaw_kp'] = self.yawDelta + self.cfParams['pid_rate.yaw_kp']
        print "yaw", yawTotalError, " : ", self.cfParams['pid_rate.yaw_kp']
        if(len(self.yawHistory) > 10):
            netSwitches = 0
            for i in xrange(1, 9):
                if(self.yawHistory[-i] > self.yawHistory[-(i+1)]):
                    netSwitches += 1
                else:
                    netSwitches -= 1
            if(netSwitches < 3):
                self.yawDone = True        

        if(self.rollDone and self.pitchDone and self.yawDone):
            if(self.tuningPhase == 0):
                self.tuningPhase = 1
                self.rollDelta = .5;
                self.pitchDelta = .5;
                self.yawDelta = .5;
                self.rollDone = False
                self.pitchDone = False
                self.yawDone = False
            else:
                self.tuned == True
    
        self.updateCrazyFlieParam('pid_rate.roll_kp')
        self.updateCrazyFlieParam('pid_rate.pitch_kp')
        self.updateCrazyFlieParam('pid_rate.yaw_kp')
        self.updateCrazyFlieParam('pid_attitude.roll_kp')
        self.updateCrazyFlieParam('pid_attitude.pitch_kp')
        self.updateCrazyFlieParam('pid_attitude.yaw_kp')
    
    # update via param.py -> radiodriver.py -> crazyradio.py -> usbRadio )))
    def updateCrazyFlieParam(self, completename ):
        self.cf.param.set_value( unicode(completename), str(self.cfParams[completename]) )


    def getSignalQuality(self):
        linkQualityValue = []
        linkQualitySignal = pyqtSignal(int)
        # Connect link quality feedback
        self.cf.linkQuality.add_callback(self.linkQualitySignal.emit)
        self.linkQualitySignal.connect(
                    lambda percentage: self.linkQualityValue.append(percentage))
        print linkQualityValue

        #if(linkQualityValue < 60):
            # Have the crazyflie fly in the opposite direction.



    def calculateDistanceInMetersBetweenCoord(self, coord1Lat, coord1Long, coord2Lat, coord2Long):
        nRadius = 6371;  #Earth's radius in Kilometers
        latDiff = (coord2Lat - coord1Lat) * (pi/180);
        lonDiff = (coord2Long - coord1Long) * (pi/180);
        lat1InRadians = coord1Lat * (pi/180);
        lat2InRadians = coord2Lat * (pi/180);
        nA = pow( sin(latDiff/2), 2 ) + cos(lat1InRadians) * cos(lat2InRadians) * pow( sin(lonDiff/2), 2 );
        nC = 2 * atan2( sqrt(nA), sqrt( 1 - nA ));
        nD = nRadius * nC;

        # convert to meters
        return (nD*1000);

    def calculateAngleBegtweenCoordinates(self, coord1Lat, coord1Long, coord2Lat, coord2Long):
        if(coord1Long != Nan):
            deltaY = coord2Long - coord1Long;
            deltaX = coord2Lat - coord1Lat;

            angleInDegrees = atan2(deltaY, deltaX) * 180 / pi;
        
            return angleInDegrees

# - (void)calculateAngleBegtweenCoordinates
# {
#     if (self.sortedAnnotations.lastObject != nil) {
#         /* Sorted annotations is used because the maps annotations array isn't sorted and includes the user's location. */
#         RegionAnnotation *lastAnnotation = [self.sortedAnnotations lastObject];
        
        
#         CGFloat deltaY = lastAnnotation.coordinate.longitude - self.userLoc.coordinate.longitude;
#         CGFloat deltaX = lastAnnotation.coordinate.latitude - self.userLoc.coordinate.latitude;
        
#         CGFloat angleInDegrees = atan2(deltaY, deltaX) * 180 / M_PI;
        
#         NSLog(@"angleInDegrees: %f", angleInDegrees);
        
#         self.angleBetweenCoordinates = angleInDegrees;
#     } else {
#         NSLog(@"No Geofences");
#     }





    def start_input(self, deviceId, inputMap):
        """Initalize the reading and open the device with deviceId and set the mapping for axis/buttons using the
        inputMap"""
        # self.data = {"roll":0.0, "pitch":0.0, "yaw":0.0, "thrust":0.0, "pitchcal":0.0, "rollcal":0.0, "estop": False, "exit":False}
        # self.aiData = {"roll":0.0, "pitch":0.0, "yaw":0.0, "thrust":0.0, "pitchcal":0.0, "rollcal":0.0, "estop": False, "exit":False}
        self.data = {"roll":0.0, "pitch":0.0, "yaw":0.0, "thrust":0.0, "pitchcal":0.0, "rollcal":0.0, "estop": False, "althold": False, "exit":False}
        self.aiData = {"roll":0.0, "pitch":0.0, "yaw":0.0, "thrust":0.0, "pitchcal":0.0, "rollcal":0.0, "estop": False, "althold": False, "exit":False}
        self.inputMap = inputMap
        self.j = pygame.joystick.Joystick(deviceId)
        self.j.init()


    def enableRawReading(self, deviceId):
        """Enable reading of raw values (without mapping)"""
        self.j = pygame.joystick.Joystick(deviceId)
        self.j.init()

    def disableRawReading(self):
        """Disable raw reading"""
        # No need to de-init since there's no good support for multiple input devices
        pass

    def readRawValues(self):
        """Read out the raw values from the device"""
        rawaxis = {}
        rawbutton = {}

        for e in pygame.event.get():
            if e.type == pygame.locals.JOYBUTTONDOWN:
                rawbutton[e.button] = 1
            if e.type == pygame.locals.JOYBUTTONUP:
                rawbutton[e.button] = 0
            if e.type == pygame.locals.JOYAXISMOTION:
                rawaxis[e.axis] = self.j.get_axis(e.axis)

        return [rawaxis,rawbutton]

    def getAvailableDevices(self):
        """List all the available devices."""
        dev = []
        pygame.joystick.quit()
        pygame.joystick.init()
        nbrOfInputs = pygame.joystick.get_count()
        for i in range(0,nbrOfInputs):
            j = pygame.joystick.Joystick(i)
            dev.append({"id":i, "name" : j.get_name()})
        return dev
        
    def getRollError(self, vals):
        self.rollError.append(vals)
        #print vals
        #print "self is: ", self
    def getPitchError(self, vals):
        self.pitchError.append(vals)
        #print vals
    def getYawError(self, vals):
        self.yawError.append(vals)
        #print vals

    def getCurrentCoords(self, latitude, longitude):
        self.currentLat.append(latitude)
        self.currentLong.append(longitude)
        print "latitude" + latitude + " longitude" longitude

    def getDestinationCoords(self, latitude, longitude):
        self.destinationLat.append(latitude)
        self.destinationLong.append(longitude)
        print "latitude" + latitude + " longitude" longitude


import sys
import clr
import System.Collections.Generic
import System
clr.AddReference('System.Core')
clr.AddReference('IronPython')
clr.AddReference('System.Xml')
clr.AddReferenceByName('Utilities')
clr.AddReferenceByName('HSFUniverse')
clr.AddReferenceByName('UserModel')
clr.AddReferenceByName('MissionElements')
clr.AddReferenceByName('HSFSystem')

import System.Xml
import HSFSystem
import HSFSubsystem
import MissionElements
import Utilities
import HSFUniverse
import UserModel
from HSFSystem import *
from System.Xml import XmlNode
from Utilities import *
from HSFUniverse import *
from UserModel import *
from MissionElements import *
from System import Func, Delegate
from System.Collections.Generic import Dictionary
from IronPython.Compiler import CallTarget0

class power(HSFSubsystem.Subsystem):
    def __new__(cls, node, asset):
        instance = HSFSubsystem.Subsystem.__new__(cls)
        instance.Asset = asset
        instance.Name = instance.Asset.Name + '.' + node.Attributes['subsystemName'].Value.ToString().ToLower()

        instance.DOD_KEY = Utilities.StateVarKey[System.Double](instance.Asset.Name + '.' + 'depthofdischarge')
        instance.POWIN_KEY = Utilities.StateVarKey[System.Double](instance.Asset.Name + '.' + 'solarpanelpowerin')
        instance.addKey(instance.DOD_KEY)
        instance.addKey(instance.POWIN_KEY)
        instance.sun = Sun()
        # default values if variables not defined in xml file
        instance._batterySize = 100000
        instance._panelEfficiency = 0.25
        instance._panelArea = 0.18
        instance._panelDensity = 0.75
        instance._thermalAvgPower = 10.0
        instance._epsAvgPower = 1.0
        # values read from the xml file		
        if (node.Attributes['batterySize'] != None):
            instance._batterySize = float(node.Attributes['batterySize'].Value)
        if (node.Attributes['panelEfficiency'] != None):
            instance._panelEfficiency = float(node.Attributes['panelEfficiency'].Value)
        if (node.Attributes['panelArea'] != None):
            instance._panelArea = float(node.Attributes['panelArea'].Value)
        if (node.Attributes['panelDensity'] != None):
            instance._panelDensity = float(node.Attributes['panelDensity'].Value)
        if (node.Attributes['thermalSysAvgPower'] != None):
            instance._thermalAvgPower = float(node.Attributes['thermalSysAvgPower'].Value)
        if (node.Attributes['epsAvgPower'] != None):
            instance._epsAvgPower = float(node.Attributes['epsAvgPower'].Value)
        return instance

    def GetDependencyDictionary(self):
        dep = Dictionary[str, Delegate]()
        return dep

    def GetDependencyCollector(self):
        return Func[Event,  Utilities.HSFProfile[System.Double]](self.DependencyCollector)

    def CanPerform(self, event, universe):
        quatDepFuncKey = "PowerDynStatefromADCS." + self.Asset.Name
        quatDepFunc = self.SubsystemDependencyFunctions[quatDepFuncKey]
        dynInvokeResult = quatDepFunc.DynamicInvoke(event)
        es = event.GetEventStart(self.Asset)
        te = event.GetTaskEnd(self.Asset)
        ee = event.GetEventEnd(self.Asset)
        powerSubPowerOut = self._epsAvgPower + self._thermalAvgPower

        if (ee > SimParameters.SimEndSeconds):
            #Logger.Report("Simulation ended")
            return False

        olddod = self._newState.GetLastValue(self.Dkeys[0]).Value

        # collect power profile out
        powerOut = self.DependencyCollector(event)
        powerOut = powerOut + powerSubPowerOut

        # collect power profile in
        position = self.Asset.AssetDynamicState
        powerIn = self.CalcSolarPanelPowerProfile(es, te, self._newState, position, universe, dynInvokeResult)

        # calculate dod rate
        dodrateofchange = HSFProfile[System.Double]()
        dodrateofchange = ((powerOut - powerIn) / self._batterySize)

        exceeded= False
        freq = 1.0
        # function returns HSFProfile[System.Double]() object but python reads it as tuple with [0] element as
        # the desired object type
        dodProf = dodrateofchange.lowerLimitIntegrateToProf(es, te, freq, 0.0, exceeded, 0, olddod)
        self._newState.AddValue(self.DOD_KEY, dodProf[0])
        return True

    def CanExtend(self, event, universe, extendTo):
        quatDepFuncKey = "PowerDynStatefromADCS." + self.Asset.Name
        quatDepFunc = self.SubsystemDependencyFunctions[quatDepFuncKey]
        dynInvokeResult = quatDepFunc.DynamicInvoke(event)
        if event.GetAssetTask(self.Asset).Type == TaskType.FLYALONG:
            return False
        ee = event.GetEventEnd(self.Asset)
        if (ee > SimParameters.SimEndSeconds):
            return False

        sun = universe.Sun
        te = event.State.GetLastValue(self.DOD_KEY).Key
        if (event.GetEventEnd(self.Asset) < extendTo):
            event.SetEventEnd(self.Asset, extendTo)
        if te == SimParameters.SimEndSeconds:
            return False
        # get the dod initial conditions
        olddod = event.State.GetValueAtTime(self.DOD_KEY, te).Value

        # collect power profile out
        powerOut = self.DependencyCollector(event)
        # collect power profile in
        position = self.Asset.AssetDynamicState
        powerIn = self.CalcSolarPanelPowerProfile(te, ee, event.State, position, universe, dynInvokeResult)
        # calculated dod rate
        dodrateofchange = ((powerOut - powerIn) / self._batterySize)

        exceeded_lower = False
        exceeded_upper = False
        freq = 1.0
        dodProf = dodrateofchange.limitIntegrateToProf(te, ee, freq, 0.0, 1.0, exceeded_lower, exceeded_upper, 0, olddod)
        # dodProf is a tuple where the [0] element contains the HSFProfile[System.Double]() object desired
        dodProf = dodProf[0]
        if (exceeded_upper):
            return False
        if (dodProf.LastTime() != ee and ee == SimParameters.SimEndSeconds):
            dodProf[ee] = dodProf.LastValue()
        event.State.AddValue(self.DOD_KEY, dodProf)
        return True
      
    def GetSolarPanelPower(self, shadow, attitude, area, efficiency, time):
        if (str(shadow) == 'UMBRA'):
            return 0
        elif (str(shadow) == 'PENUMBRA'):
            return 0.5*self.CalcPowerInCosineArea(attitude,area,efficiency,time)
        else:
            return self.CalcPowerInCosineArea(attitude,area,efficiency,time)

    '''def CalcSolarPanelPowerProfile(self, start, end, state, position, universe):
        # create solar panel profile for this event
        freq = 1
        lastShadow = universe.Sun.castShadowOnPos(position, start)
        attitude = position
        solarPanelSolarProfile = Utilities.HSFProfile[System.Double](start, self.GetSolarPanelPower(lastShadow,attitude,self._panelArea,self._panelEfficiency,start))
        time = start
        while time <= end:
            shadow = universe.Sun.castShadowOnPos(position, time)
            solarPanelSolarProfile[time] = self.GetSolarPanelPower(shadow,attitude,self._panelArea,self._panelEfficiency,time)
            lastShadow = shadow
            time += freq
        state.AddValue(self.POWIN_KEY, solarPanelSolarProfile)
        return solarPanelSolarProfile'''

    def CalcSolarPanelPowerProfile(self, start, end, state, position, universe, attitude):
        # create solar panel profile for this event
        freq = 1
        lastShadow = universe.Sun.castShadowOnPos(position, start)
        solarPanelSolarProfile = Utilities.HSFProfile[System.Double](start, self.GetSolarPanelPower(lastShadow,attitude,self._panelArea,self._panelEfficiency,start))
        time = start
        while time <= end:
            shadow = universe.Sun.castShadowOnPos(position, time)
            solarPanelSolarProfile[time] = self.GetSolarPanelPower(shadow,attitude,self._panelArea,self._panelEfficiency,time)
            lastShadow = shadow
            time += freq
        state.AddValue(self.POWIN_KEY, solarPanelSolarProfile)
        return solarPanelSolarProfile

    def DependencyCollector(self, currentEvent):
        outProf = HSFProfile[float]()
        for dep in self.SubsystemDependencyFunctions:
            if not (dep.Key == "DepCollector" or dep.Key == ("PowerDynStatefromADCS."+self.Asset.Name)):
                temp = dep.Value.DynamicInvoke(currentEvent)
                outProf = outProf + temp
        return outProf

    '''def CalcPowerInCosineArea(self,attitude,area,efficiency,time):
        panelAxis = Matrix[System.Double](3,1)
        panelAxis[1] = 0.0
        panelAxis[2] = 0.0
        panelAxis[3] = -1.0
        attitudeAtTime = attitude.Quaternions(time)
        attitude = Quat(attitudeAtTime[1],attitudeAtTime[2],attitudeAtTime[3],attitudeAtTime[4])
        panAxisECI = Quat.Rotate(Quat.Conjugate(attitude),panelAxis)
        r_solar = self.sun.getEarSunVec(time)
        r_solarNorm = Vector.Norm(Vector(r_solar.ToString()))
        dotProd = Matrix[System.Double].Dot(r_solar/r_solarNorm,panAxisECI)
        return 1367.0*area*efficiency*dotProd*self._panelDensity'''

    def CalcPowerInCosineArea(self,attitude,area,efficiency,time):
        panelAxis = Matrix[System.Double](3,1)
        panelAxis[1] = 0.0
        panelAxis[2] = 0.0
        panelAxis[3] = -1.0
        dynStateTime = attitude[time]
        attitudeAtTime = dynStateTime[MatrixIndex(7,10),1]
        attitude = Quat(attitudeAtTime[1],attitudeAtTime[2],attitudeAtTime[3],attitudeAtTime[4])
        panAxisECI = Quat.Rotate(Quat.Conjugate(attitude),panelAxis)
        r_solar = self.sun.getEarSunVec(time)
        r_solarNorm = Vector.Norm(Vector(r_solar.ToString()))
        dotProd = Matrix[System.Double].Dot(r_solar/r_solarNorm,panAxisECI)
        return 1367.0*area*efficiency*dotProd*self._panelDensity
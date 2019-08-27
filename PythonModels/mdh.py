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
clr.AddReference("Microsoft.Office.Interop.Excel")
import System.Xml
import HSFSystem
import HSFSubsystem
import MissionElements
import Utilities
import HSFUniverse
import UserModel
import Microsoft.Office.Interop.Excel as Excel
from HSFSystem import *
from System.Xml import XmlNode
from Utilities import *
from HSFUniverse import *
from UserModel import *
from MissionElements import *
from System import Func, Delegate
from System.Collections.Generic import *
from IronPython.Compiler import CallTarget0

class mdh(HSFSubsystem.Subsystem):
    def __new__(cls, node, asset):
        instance = HSFSubsystem.Subsystem.__new__(cls)
        instance.Asset = asset
        instance.Name = instance.Asset.Name + '.' + node.Attributes['subsystemName'].Value.ToString().ToLower()
        if (node.Attributes['peakDataRate'] != None):
            instance._peakDataRate = float(node.Attributes['peakDataRate'].Value.ToString())
        if (node.Attributes['bufferSize'] != None):
            instance._bufferSize = float(node.Attributes['bufferSize'].Value.ToString())
        instance.LinkBudgetFile = str(node.Attributes["linkBudgetPath"].Value)
        currDirectory = System.AppDomain.CurrentDomain.BaseDirectory
        if "Debug" in currDirectory:
            pathToRemove = "Horizon\\bin\\Debug\\"
        if "Release" in currDirectory:
            pathToRemove = "Horizon\\bin\\Release\\"
        if currDirectory.endswith(pathToRemove):
            LinkBudgetPath = currDirectory.replace(pathToRemove,instance.LinkBudgetFile)
        excel = Excel.ApplicationClass()
        excel.Visible = False
        workbook = excel.Workbooks.Open(LinkBudgetPath,False)
        datasheet = workbook.Worksheets(14) # TODO: make this select the overview sheet in the AMSAT budget workbook
        dataRateCell = datasheet.Range["L5"]
        dataRateVal = dataRateCell.Value2/8.0 # Given in sheet in bps, convert to B/s
        workbook.Close()
        if dataRateVal:
            instance._peakDataRate = dataRateVal
        elif node.Attributes["peakDataRate"].Value:
            instance._peakDataRate = float(node.Attributes["peakDataRate"].Value)
        else:
            instance._peakDataRate
        instance.compRatio = float(node.Attributes['compressionRatio'].Value.ToString())
        instance.obcAvgPower = float(node.Attributes['obcAvgPower'].Value.ToString())
        instance.DATABUFFERRATIO_KEY = Utilities.StateVarKey[System.Double](instance.Asset.Name + '.' + 'databufferfillratio')
        instance.addKey(instance.DATABUFFERRATIO_KEY)
        return instance

    def GetDependencyDictionary(self):
        dep = Dictionary[str, Delegate]()
        depFunc1 = Func[Event,  Utilities.HSFProfile[System.Double]](self.POWERSUB_PowerProfile_MDHSUB)
        dep.Add("PowerfromMDH" + "." + self.Asset.Name, depFunc1)
        depFunc2 = Func[Event,  Utilities.HSFProfile[System.Double]](self.COMMSUB_DataRateProfile_MDHSUB)
        dep.Add("CommfromMDH" + "." + self.Asset.Name, depFunc2)
        depFunc3 = Func[Event,  System.Double](self.EVAL_DataRateProfile_MDHSUB)
        dep.Add("EvalfromMDH" + "." + self.Asset.Name, depFunc3)
        return dep

    def GetDependencyCollector(self):
        return Func[Event,  Utilities.HSFProfile[System.Double]](self.DependencyCollector)

    def CanPerform(self, event, universe):
        if (self._task.Type == TaskType.IMAGING):
            ts = event.GetTaskStart(self.Asset)
            te = event.GetTaskEnd(self.Asset)
            oldbufferratio = self._newState.GetLastValue(self.Dkeys[0]).Value
            newdataratein = HSFProfile[System.Double]()
            newdataratein = self.DependencyCollector(event) / self._bufferSize / self.compRatio
            exceeded = False
            newdataratio = HSFProfile[System.Double]()
            newdataratio = newdataratein.upperLimitIntegrateToProf(ts, te, 5, 1, exceeded, 0, oldbufferratio)
            if (exceeded == False):
                self._newState.AddValue(self.DATABUFFERRATIO_KEY, newdataratio[0])
                return True
          #  Logger.Report("SSDR")
            return False
        if(self._task.Type == TaskType.COMM):
             ts = event.GetTaskStart(self.Asset)
             te = event.GetTaskEnd(self.Asset)
             data = self._bufferSize * self._newState.GetLastValue(self.Dkeys[0]).Value
             if data == 0.0:
                 return False
             if( data / 2 > self._bufferSize/2):
                 dataqueout = data/2
             else:
                 dataqueout = data
             if (data - dataqueout < 0):
                 dataqueout = data
             if (dataqueout > 0):
                 self._newState.AddValue(self.DATABUFFERRATIO_KEY, KeyValuePair[System.Double, System.Double](te, (dataqueout - self._peakDataRate * (te-ts)) / self._bufferSize))
             return True
        return True

    def CanExtend(self, event, universe, extendTo):
        if event.GetAssetTask(self.Asset).Type == TaskType.FLYALONG:
            return False
        return super(mdh, self).CanExtend(event, universe, extendTo)

    def POWERSUB_PowerProfile_MDHSUB(self, event):
        prof1 = HSFProfile[System.Double]()
        prof1[event.GetEventStart(self.Asset)] = self.obcAvgPower
        return prof1

    def COMMSUB_DataRateProfile_MDHSUB(self, event):
        #datarate = (event.State.GetValueAtTime(self.DATABUFFERRATIO_KEY, event.GetTaskStart(self.Asset)).Value - event.State.GetValueAtTime(self.DATABUFFERRATIO_KEY, event.GetTaskEnd(self.Asset)).Value) / (event.GetTaskEnd(self.Asset) - event.GetTaskStart(self.Asset)) 
        #if datarate >= self._peakDataRate:
        datarate = 0.0
        if (event.State.GetValueAtTime(self.DATABUFFERRATIO_KEY, event.GetTaskStart(self.Asset)).Value - event.State.GetValueAtTime(self.DATABUFFERRATIO_KEY, event.GetTaskEnd(self.Asset)).Value) > 0.0:
            datarate = self._peakDataRate
        prof1 = HSFProfile[System.Double]()
        if (datarate != 0):
            prof1[event.GetTaskStart(self.Asset)] = datarate
            prof1[event.GetTaskEnd(self.Asset)] = 0
        return prof1

    def EVAL_DataRateProfile_MDHSUB(self, event):
        return (event.State.GetValueAtTime(DATABUFFERRATIO_KEY, event.GetTaskEnd(self.Asset)).Value - event.State.GetValueAtTime(DATABUFFERRATIO_KEY, event.GetTaskEnd(self.Asset)).Value)

    def DependencyCollector(self, currentEvent):
        return super(mdh, self).DependencyCollector(currentEvent)

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
#clr.AddReferenceByName('SystemState')
import System.Xml
import HSFSystem
import HSFSubsystem
import MissionElements
import Utilities
import HSFUniverse
import UserModel
# from MissionElements import SystemState
from HSFSystem import *
from System.Xml import XmlNode
from Utilities import *
from HSFUniverse import *
from UserModel import *
from MissionElements import *
from System import Func, Delegate, Math
from System.Collections.Generic import Dictionary, KeyValuePair
from IronPython.Compiler import CallTarget0

class payload(HSFSubsystem.Subsystem):
    def __new__(cls, node, asset):
        instance = HSFSubsystem.Subsystem.__new__(cls)
        instance.Asset = asset
        instance.Name = instance.Asset.Name + '.' + node.Attributes['subsystemName'].Value.ToString().ToLower()

        instance.PIXELS_KEY = Utilities.StateVarKey[System.Double](instance.Asset.Name + '.' + 'numpixels')
        instance.PAYLOADON_KEY = Utilities.StateVarKey[System.Boolean](instance.Asset.Name + '.' + 'payloadon')
        instance.addKey(instance.PIXELS_KEY)
        instance.addKey(instance.PAYLOADON_KEY)
        instance._subFieldNumPixels = 5000
        instance._fullFieldNumPixels = 1050000
        instance._subFieldCaptureTime = 90
        instance._fullFieldCaptureTime = 100
        instance._pixelDepth = 1
        if (node.Attributes['subFieldNumPixels'] != None):
            instance._subFieldNumPixels = float(node.Attributes['subFieldNumPixels'].Value.ToString())
        if (node.Attributes['fullFieldNumPixels'] != None):
            instance._fullFieldNumPixels = float(node.Attributes['fullFieldNumPixels'].Value.ToString())
        if (node.Attributes['subFieldCaptureTime'] != None):
            instance._subFieldCaptureTime = float(node.Attributes['subFieldCaptureTime'].Value.ToString())
        if (node.Attributes['fullFieldCaptureTime'] != None):
            instance._fullFieldCaptureTime = float(node.Attributes['fullFieldCaptureTime'].Value.ToString())
        if (node.Attributes['pixelDepth'] != None):
            instance._pixelDepth = float(node.Attributes['pixelDepth'].Value.ToString())

        return instance

    def GetDependencyDictionary(self):
        dep = Dictionary[str, Delegate]()
        depFunc1 = Func[Event,  Utilities.HSFProfile[System.Double]](self.POWERSUB_PowerProfile_PAYLOADSUB)
        dep.Add("PowerfromPayload" + "." + self.Asset.Name, depFunc1)
        depFunc2 = Func[Event,  Utilities.HSFProfile[System.Double]](self.MDHSUB_NewDataProfile_PAYLOADSUB)
        dep.Add("MDHfromPayload" + "." + self.Asset.Name, depFunc2)
        depFunc3 = Func[Event, System.Double](self.EVAL_targetvalue_PAYLOADSUB)
        dep.Add("EvalfromPayload" + "." + self.Asset.Name, depFunc3)
        return dep

    def GetDependencyCollector(self):
        return Func[Event,  Utilities.HSFProfile[System.Double]](self.DependencyCollector)

    def CanPerform(self, event, universe):
         if (self._task.Type == TaskType.IMAGING):
             value = self._task.Target.Value
             pixels = self._subFieldNumPixels
             timetocapture = self._subFieldCaptureTime
             if (value >= self._subFieldCaptureTime):
                    pixels = self._fullFieldNumPixels
                    timetocapture = self._fullFieldCaptureTime
             es = event.GetEventStart(self.Asset)
             ts = event.GetTaskStart(self.Asset)
             te = event.GetTaskEnd(self.Asset)
             if (ts > te):
                # Logger.Report("EOSensor lost access")
                 return False
             te = ts + timetocapture
             event.SetTaskEnd(self.Asset, te)

             position = self.Asset.AssetDynamicState
             timage = ts + timetocapture / 2

             self._newState.AddValue(self.PIXELS_KEY, KeyValuePair[System.Double, System.Double](timage, pixels))
             self._newState.AddValue(self.PIXELS_KEY, KeyValuePair[System.Double, System.Double](timage + 1, 0.0))

             self._newState.AddValue(self.PAYLOADON_KEY, KeyValuePair[System.Double, System.Boolean](ts, True))
             self._newState.AddValue(self.PAYLOADON_KEY, KeyValuePair[System.Double, System.Boolean](te, False))
             return True
         else:
             return True

    def CanExtend(self, event, universe, extendTo):
        return super(payload, self).CanExtend(event, universe, extendTo)

    def POWERSUB_PowerProfile_PAYLOADSUB(self, event):
        prof1 = HSFProfile[System.Double]()
        prof1[event.GetEventStart(self.Asset)] = 2 # per camera datasheet at http://www.3d-plus.com/data/doc/products/references/shortform_space_camera_2018.pdf
        if (event.State.GetValueAtTime(self.PAYLOADON_KEY, event.GetTaskStart(self.Asset)).Value):
            prof1[event.GetTaskStart(self.Asset)] = 4
            prof1[event.GetTaskEnd(self.Asset)] = 2	
        return prof1

    def MDHSUB_NewDataProfile_PAYLOADSUB(self, event):
        return event.State.GetProfile(self.PIXELS_KEY)

    def EVAL_targetvalue_PAYLOADSUB(self, event):
        return event.Task.Target.Value

    def DependencyCollector(self, currentEvent):
        return super(payload, self).DependencyCollector(currentEvent)

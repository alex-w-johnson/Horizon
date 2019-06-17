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
from System.Collections.Generic import Dictionary
from IronPython.Compiler import CallTarget0


class comm(HSFSubsystem.Subsystem):
    def __new__(cls, node, asset):
        instance = HSFSubsystem.Subsystem.__new__(cls)
        instance.Asset = asset
        instance.Name = instance.Asset.Name + '.' + node.Attributes['subsystemName'].Value.ToString().ToLower()
        #instance.LinkBudgetPath = str(System.AppDomain.CurrentDomain.BaseDirectory)+str(node.Attributes["linkBudgetPath"].Value)
        instance.LinkBudgetPath = r"C:\Users\Alex\source\repos\alex-w-johnson\Horizon\CANSat Link Budget.xls"
        excel = Excel.ApplicationClass()
        workbook = excel.Workbooks.Open(instance.LinkBudgetPath)
        datasheet = workbook.Worksheets.Item[13]
        dataRateCell = datasheet.Range["L5"]
        #print(dataRateCell)
        dataRateVal = dataRateCell.Value[str]
        print(dataRateVal)
        #L5
        instance.maxDataRate = float(node.Attributes["peakDataRate"].Value)
        instance.minElevAngle = float(node.Attributes["minElevAngle"].Value)
        instance.DATARATE_KEY = Utilities.StateVarKey[System.Double](instance.Asset.Name + '.' + 'datarate(B/s)')
        instance.addKey(instance.DATARATE_KEY)
        return instance
		
    def GetDependencyDictionary(self):
        dep = Dictionary[str, Delegate]()
        depFunc1 = Func[Event,  Utilities.HSFProfile[System.Double]](self.POWERSUB_PowerProfile_COMMSUB)
        dep.Add("PowerfromComm" + "." + self.Asset.Name, depFunc1)
        return dep

    def GetDependencyCollector(self):
        return Func[Event,  Utilities.HSFProfile[System.Double]](self.DependencyCollector)

    def CanPerform(self, event, universe):
        if (self._task.Type == TaskType.COMM):
            newProf = self.DependencyCollector(event)
            asset = self.Asset
            assetDynState = asset.AssetDynamicState
            task = event.GetAssetTask(asset)
            ts = event.GetTaskStart(asset)
            target = task.Target
            targDynState = target.DynamicState
            targPos = targDynState.PositionECI(ts)
            assetPos = assetDynState.PositionECI(ts)
            rho = targPos - assetPos
            rhoDotTargPos = Vector.Dot(rho,targPos)
            elevAngle = System.Math.Acos(rhoDotTargPos/Vector.Norm(rho)/Vector.Norm(targPos)) - (System.Math.PI / 2.0)
            elevAngle = elevAngle * 180.0 / System.Math.PI
            print(elevAngle)
            if elevAngle < self.minElevAngle:
                return False
            if (newProf.Empty() == False):
                event.State.SetProfile(self.DATARATE_KEY, newProf)
        return True

    def CanExtend(self, event, universe, extendTo):
        return super(comm, self).CanExtend(event, universe, extendTo)

    def POWERSUB_PowerProfile_COMMSUB(self, event):
        return event.State.GetProfile(self.DATARATE_KEY)

    def DependencyCollector(self, currentEvent):
        return super(comm, self).DependencyCollector(currentEvent)

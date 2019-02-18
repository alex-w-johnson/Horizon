import sys
import clr
import System.Collections.Generic
import System
clr.AddReference('System.Core')
clr.AddReference('IronPython')
clr.AddReference('System.Xml')
clr.AddReferenceByName('Utilities')
clr.AddReferenceByName('HSFUniverse')
clr.AddReferenceByName('Horizon')
clr.AddReferenceByName('MissionElements')
clr.AddReferenceByName('UserModel')
import Utilities
import HSFUniverse
import math
import MissionElements
import UserModel
from UserModel import XmlParser
from MissionElements import Asset
#per Maclean's rocketEOMS.py
from Utilities import *
from HSFUniverse import *
from Utilities import *
from System.Collections.Generic import Dictionary
from IronPython.Compiler import CallTarget0
from System import Array
from System import Xml

class eomSSTN(Utilities.EOMS):
    def __init__(self):
        self.Cd = float(scriptedNode["Geometry"].Attributes["Cd"].Value)
        self.CxArea = float(scriptedNode["Geometry"].Attributes["CxArea"].Value)
        # Mass Properties for Dynamics
        self.Ixx = float(scriptedNode["MassProp"].Attributes["Ixx"].Value)
        self.Iyy = float(scriptedNode["MassProp"].Attributes["Iyy"].Value)
        self.Izz = float(scriptedNode["MassProp"].Attributes["Izz"].Value)
        self.Mass = self.Ixx = float(scriptedNode["MassProp"].Attributes["Mass"].Value)

    def PythonAccessor(self, t, y):
        return super(eomSSTN, self).PythonAccessor(t, y)

    def CalculateForces(self,r_eci,v_eci):
        pass

    def CalculateMoments(self,r_eci,v_eci):
        pass

    def CalculateGravityForce(self,r_eci):
        pass

    def CalculateJ2Force(self,r_eci):
        pass

    def CalculateDragForce(self,r_eci,v_eci):
        pass

    def CalculateDragMoment(self,r_eci,v_eci):
        pass

    def CalculateMagMoment(self,r_eci):
        pass

    def CalculateGravGradMoment(self,r_eci):
        pass

    def GetAtmosDensity(self,r_eci):
        pass

    def GetRotationMatrix(self, q):
        pass
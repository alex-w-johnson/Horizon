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
        # Mass Properties for Dynamics
        self.Ixx = float(scriptedNode["MassProp"].Attributes["Ixx"].Value)
        self.Iyy = float(scriptedNode["MassProp"].Attributes["Iyy"].Value)
        self.Izz = float(scriptedNode["MassProp"].Attributes["Izz"].Value)
        self.Mass = float(scriptedNode["MassProp"].Attributes["Mass"].Value)

    def PythonAccessor(self, t, y, param, environment):
        xeci = y[1] # ECI frame, vernal equinox direction
        yeci = y[2] # ECI frame, Z cross X direction
        zeci = y[3] # ECI frame, North celestial pole direction
        vxeci = y[4] # X velocity in ECI
        vyeci = y[5] # Y velocity in ECI
        vzeci = y[6] # Z velocity in ECI
        q0beci = y[7] # Real scalar eta of quaternion (body in ECI)
        q1beci = y[8] # i-hat quaternion component (body in ECI)
        q2beci = y[9] # j-hat quaternion component (body in ECI)
        q3beci = y[10] # k-hat quaternion component (body in ECI)
        wxbeci = y[11] # x-axis body rate (roll rate)
        wybeci = y[12] # y-axis body rate (pitch rate)
        wzbeci = y[13] # z-axis body rate (yaw rate)
        wwxb = y[14] # x-axis reaction wheel speed (body frame)
        wwyb = y[15] # y-axis reaction wheel speed (body frame)
        wwzb = y[16] # z-axis reaction wheel speed (body frame)
        return super(eomSSTN, self).PythonAccessor(t, y)

    def CalculateForces(self,r_eci,v_eci):
        pass

    def CalculateMoments(self,r_eci,v_eci):
        pass

    def CalculateGravityForce(self,r_eci):
        mu = 398600.4418
        rnorm = System.Math.sqrt(self.xeci**2+self.yeci**2+self.zeci**2)
        fgravx = -mu*xeci/(rnorm^3)
        fgravy = -mu*yeci/(rnorm^3)
        fgravz = -mu*zeci/(rnorm^3)
        fgrav = Matrix[System.Double](3,1)
        fgrav[1] = fgravx
        fgrav[2] = fgravy
        fgrav[3] = fgravz
        return fgrav

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
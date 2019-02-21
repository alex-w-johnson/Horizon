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
from System.Collections.Generic import Dictionary
from IronPython.Compiler import CallTarget0
from System import Array
from System import Xml

class eomSSTN(Utilities.EOMS):
    def __init__(self):
        self.Cd = float(scriptedNode["Geometry"].Attributes["Cd"].Value)
        self.CxArea = float(scriptedNode["Geometry"].Attributes["CxArea"].Value)
        self.CoP = float(scriptedNode["Geometry"].Attributes["CoP"].Value)
        
        # Mass Properties for Dynamics
        self.Ixx = float(scriptedNode["MassProp"].Attributes["Ixx"].Value)
        self.Iyy = float(scriptedNode["MassProp"].Attributes["Iyy"].Value)
        self.Izz = float(scriptedNode["MassProp"].Attributes["Izz"].Value)
        self.Mass = float(scriptedNode["MassProp"].Attributes["Mass"].Value)
        self.Imat = CalcInertiaMatrix(self)
        self.CoM = float(scriptedNode["MassProp"].Attributes["CoM"].Value)
        
    def PythonAccessor(self, t, y):
        xeci = y[1]
        yeci = y[2]
        zeci = y[3]
        vxeci = y[4]
        vyeci = y[5]
        vzeci = y[6]
        qb0 = y[7]
        qb1 = y[8]
        qb2 = y[9]
        qb3 = y[10]
        wxbeci = y[11]
        wybeci = y[12]
        wzbeci = y[13]
        wwxb = y[14]
        wwyb = y[15]
        wwzb = y[16]
        Reci = Matrix[System.Double](3,1)
        Reci[1] = xeci
        Reci[2] = yeci
        Reci[3] = zeci
        Veci = Matrix[System.Double](3,1)
        Veci[1] = vxeci
        Veci[2] = vyeci
        Veci[3] = vzeci
        epsbeci = Matrix[System.Double](3,1)
        epsbeci[1] = qb0
        epsbeci[2] = qb1
        epsbeci[3] = qb2
        qbeci = Quat[System.Double](qb3,epsbeci)
        wbeci = Matrix[System.Double](3,1)
        wbeci[1] = wxbeci
        wbeci[2] = wybeci
        wbeci[3] = wzbeci
        wwb = Matrix[System.Double](3,1)
        wwb[1] = wwxb
        wwb[2] = wwyb
        wwb[3] = wwzb
        epscross = Matrix[System.Double](3,3)
        epscross[1,1] = qb3
        epscross[1,2] = -1*qb2
        epscross[1,3] = qb1
        epscross[2,1] = qb2
        epscross[2,2] = qb3
        epscross[2,3] = -1*qb0
        epscross[3,1] = -1*qb1
        epscross[3,2] = qb0
        epscross[3,3] = qb3
        epsbecidot = 0.5*epscross*wbeci
        
        
        # State transition matrix equations
        dy[1] = vxeci
        dy[2] = vyeci
        dy[3] = vzeci
        #dy[4] = accelerations
        
        d[7] = epsbecidot[1]
        d[8] = epsbecidot[2]
        d[9] = epsbecidot[3]
        dy[10] = -0.5*Utilities.Vector.Dot(epsbeci,wbeci)


        return super(eomSSTN, self).PythonAccessor(t, y)

    def CalcForces(self):
        pass

    def CalcMoments(self,r_eci,v_eci):
        pass

    def CalcGravityForce(self,r):
        mu = 398600.4418
        r3 = Matrix[System.Double].Norm(r)^3
        agrav = Matrix[System.Double](3,1)
        agrav[1] = -mu*r[1]/r3
        agrav[2] = -mu*r[2]/r3
        agrav[3] = -mu*r[3]/r3
        return agrav

    def CalcJ2Force(self,r):
        mu = 398600.4418
        J2 = 1.0826269e-3
        rE = 6378.137
        rnorm = Matrix[System.Double].Norm(r)
        aJ2 = Matrix[System.Double](3,1)
        aJ2[1] = -((3*J2*mu*(rE^2)*r[1])/(2*(rnorm^5)))*(1-((5*(r[3]^2))/(rnorm^2)))
        aJ2[2] = -((3*J2*mu*(rE^2)*r[2])/(2*(rnorm^5)))*(1-((5*(r[3]^2))/(rnorm^2)))
        aJ2[3] = -((3*J2*mu*(rE^2)*r[3])/(2*(rnorm^5)))*(3-((5*(r[3]^2))/(rnorm^2)))
        return aJ2

    def CalcDragForce(self,r_eci,v_eci):
        rho = CalcAtmosDens(self,r_eci)
        vnorm = Matrix[System.Double].Norm(v_eci)
        F_d = -1*rho*v_eci*vnorm*self.Cd*self.CxArea
        return F_d

    def CalcDragMoment(self,r_eci,v_eci):
        pass

    def CalcMagMoment(self,r_eci):
        pass

    def CalcGravGradMoment(self,r_eci,Qb_eci):
        mu = 398600.4418
        rnorm = Matrix[System.Double].Norm(r)
        rb = Matrix[System.Double].Transpose(Qb_eci)*r_eci
        T_g = 3*mu*Matrix[System.Double].Cross(rb,self.Imat*rb)
        return T_g

    def CalcAtmosDens(self,r_eci):
        rE = 6378.137
        h = Matrix[System.Double].Norm(r_eci) - rE
        # Based on exp. atmosphere model from Vallado Table 8-4
        atmMatrix = Matrix[System.Double](28,3)
        atmMatrix[1] = [ 0, 1.225, 7.249]
        atmMatrix[2] = [ 25, 3.899E-2, 6.349]
        atmMatrix[3] = [ 30, 1.774E-2, 6.682]
        atmMatrix[4] = [ 40, 3.972E-3, 7.554]
        atmMatrix[5] = [ 50, 1.057E-3, 8.382]
        atmMatrix[6] = [ 60, 3.206E-4, 7.714]
        atmMatrix[7] = [ 70, 8.770E-5, 6.549]
        atmMatrix[8] = [ 80, 1.905E-5, 5.799]
        atmMatrix[9] = [ 90, 3.396E-6, 5.382]
        atmMatrix[10] = [ 100, 5.297E-7, 5.877]
        atmMatrix[11] = [ 110, 9.661E-8, 7.263]
        atmMatrix[12] = [ 120, 2.438E-8, 9.473]
        atmMatrix[13] = [ 130, 8.484E-9, 12.636]
        atmMatrix[14] = [ 140, 3.845E-9, 16.149]
        atmMatrix[15] = [ 150, 2.070E-9, 22.523]
        atmMatrix[16] = [ 180, 5.464E-10, 29.740]
        atmMatrix[17] = [ 200, 2.789E-10, 37.105]
        atmMatrix[18] = [ 250, 7.248E-11, 45.546]
        atmMatrix[19] = [ 300, 2.418E-11, 53.628]
        atmMatrix[20] = [ 350, 9.518E-12, 53.298]
        atmMatrix[21] = [ 400, 3.725E-12, 58.515]
        atmMatrix[22] = [ 450, 1.585E-12, 60.828]
        atmMatrix[23] = [ 500, 6.967E-13, 63.822]
        atmMatrix[24] = [ 600, 1.454E-13, 71.835]
        atmMatrix[25] = [ 700, 3.614E-14, 88.667]
        atmMatrix[26] = [ 800, 1.170E-14, 124.64]
        atmMatrix[27] = [ 900, 5.245E-15, 181.05]
        atmMatrix[28] = [ 1000, 3.019E-15, 268.00]
        for r in atmMatrix:
            if h > atmMatrix[r][0] and h < atmMatrix[r+1][0]:
                rho = atmMatrix[r][1] * math.exp(-1*(h-atmMatrix[r][1])/atmMatrix[r][2])
            else:
                print('Invalid altitude input to exponential atmosphere calculation')
                break
        return rho

    def CalcRotMat(self, q):
        eta = q._eta
        eps = q._eps
        idmat = Matrix[System.Double].Eye(3)
        epstrans = Matrix[System.Double].Transpose(eps)
        Q = (2*eta^2 - 1)*idmat + 2*eps*epstrans - 2*eta*epscross
        return Q

    def CalcInertiaMatrix(self):
        InerMat = Matrix[System.Double](3,3)
        InerMat[1,1] = self.Ixx
        InerMat[2,2] = self.Iyy
        InerMat[3,3] = self.Izz
        return InerMat
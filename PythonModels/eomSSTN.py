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
from System.Collections.Generic import Dictionary
from IronPython.Compiler import CallTarget0
from System import Array
from System import Xml
import datetime #For using WMM
from System import DateTime #For using WMM


class eomSSTN(EOMS):
    # Base class is Utilities.EOMS b/c it is typecast to type DynamicEOMS in the EOMFactory, per Adam's suggestion - AJ
    def __new__(cls, node):
        instance = EOMS.__new__(cls)
        instance.Cd = float(node.Attributes["cd"].Value)
        instance.CxArea = float(node.Attributes["cxareaavg"].Value)
        instance.CoP = Matrix[System.Double](node.Attributes["cop"].Value)
        
        # Implemented to allow asset StateVarKeys to be created for individual assets (See Adam Frye's eomSwarm.py for example)
        if (node.Attributes["Name"] != None):
            instance.AssetName = str(node.Attributes["Name"].Value)

        # Mass Properties for Dynamics
        instance.Ivec = Matrix[System.Double](node.Attributes["moi"].Value)
        instance.Ixx = instance.Ivec[1]
        instance.Iyy = instance.Ivec[2]
        instance.Izz = instance.Ivec[3]
        instance.Mass = float(node.Attributes["mass"].Value)
        instance.Imat = Matrix[System.Double](3,3)
        instance.Imat[1,1] = instance.Ixx
        instance.Imat[2,2] = instance.Iyy
        instance.Imat[3,3] = instance.Izz
        instance.WHEELTORQUE_KEY = StateVarKey[Matrix[System.Double]](instance.AssetName + '.' + 'rxwheel_torque')
        instance.MAGTORQDIPOLE_KEY = StateVarKey[Matrix[System.Double]](instance.AssetName + '.' + 'magtorq_dipole')
        instance.IsWheelsVec = Matrix[System.Double](node.Attributes["iswheels"].Value)
        instance.IsWheels = Matrix[System.Double](3,3)
        instance.IsWheels[1,1] = instance.IsWheelsVec[1]
        instance.IsWheels[2,2] = instance.IsWheelsVec[2]
        instance.IsWheels[3,3] = instance.IsWheelsVec[3]
        return instance
    
    def PythonAccessor(self, t, y, param):
        xeci = y[1,1]
        yeci = y[2,1]
        zeci = y[3,1]
        vxeci = y[4,1]
        vyeci = y[5,1]
        vzeci = y[6,1]
        qb0 = y[7,1]
        qb1 = y[8,1]
        qb2 = y[9,1]
        qb3 = y[10,1]
        wxbeci = y[11,1]
        wybeci = y[12,1]
        wzbeci = y[13,1]
        wwxb = y[14,1]
        wwyb = y[15,1]
        wwzb = y[16,1]
        Reci = Matrix[System.Double](3,1)
        Reci[1] = xeci
        Reci[2] = yeci
        Reci[3] = zeci
        Veci = Matrix[System.Double](3,1)
        Veci[1] = vxeci
        Veci[2] = vyeci
        Veci[3] = vzeci
        epsbeci = Matrix[System.Double](3,1)
        epsbeci[1] = qb1
        epsbeci[2] = qb2
        epsbeci[3] = qb3
        qbeci = Quat(qb0,epsbeci)
        wbeci = Matrix[System.Double](3,1)
        wbeci[1] = wxbeci
        wbeci[2] = wybeci
        wbeci[3] = wzbeci
        wwb = Matrix[System.Double](3,1)
        wwb[1] = wwxb
        wwb[2] = wwyb
        wwb[3] = wwzb
        etaI = Matrix[System.Double](3,3)
        etaI[1,1] = qb0
        etaI[2,2] = qb0
        etaI[3,3] = qb0
        epsbecidot = 0.5*Matrix[System.Double].Cross(epsbeci,wbeci) + 0.5*etaI*wbeci

        # Current Julian Date
        jdCurrent = SimParameters.SimStartJD

        # ADCS control inputs
        T_control = param.GetValue(self.WHEELTORQUE_KEY) # Correct way to get parameters from ADCS? Ask Mehiel - AJ
        M_dipole = param.GetValue(self.MAGTORQDIPOLE_KEY)

        # State transition matrix equations
        dy = Matrix[System.Double](16,1)
        dy[1,1] = vxeci
        dy[2,1] = vyeci
        dy[3,1] = vzeci
        acceleration = self.CalcForces(self)
        dy[4,1] = acceleration[1]
        dy[5,1] = acceleration[2]
        dy[6,1] = acceleration[3]
        dy[7,1] = -0.5*Matrix[System.Double].Dot(epsbeci,wbeci)
        dy[8,1] = epsbecidot[1]
        dy[9,1] = epsbecidot[2]
        dy[10,1] = epsbecidot[3]
        T_dist = self.CalcMoments(Reci,Veci,qbeci,T_control,M_dipole)
        Imat = self.Imat
        omegaDot = Matrix[System.Double].Inverse(Imat)*(T_dist-Matrix[System.Double].Cross(wbeci,Imat*wbeci)) # Correct formula for body rate integration? Or need wheel momentum? -AJ
        dy[11,1] = omegaDot[1]
        dy[12,1] = omegaDot[2]
        dy[13,1] = omegaDot[3]
        IsWheels = self.IsWheels
        omegaWDot = Matrix[System.Double].Inverse(IsWheels)*(-T_control-Matrix[System.Double].Cross(wbeci,IsWheels*wwb)) # Fairly confident, ask Mehiel anyways - AJ
        dy[14,1] = omegaWDot[1]
        dy[15,1] = omegaWDot[2]
        dy[16,1] = omegaWDot[3]
        return dy
    
    def CalcForces(self):
        a_grav = self.CalcGravityForce(self,self.Reci)
        a_J2 = self.CalcJ2Force(self,self.Reci)
        a_drag = self.CalcDragForce(self,self.Reci,self.Veci)/self.Mass
        a_total = a_grav + a_J2 + a_drag
        return a_total

    def CalcMoments(self,r_eci,v_eci,qb_eci,T_control,M_dipole):
        T_drag = self.CalcDragMoment(self,self.Reci,self.Veci)
        T_mag = self.CalcMagMoment(self,self.Reci,self.jdCurrent,M_dipole,qb_eci)
        T_gravgrad = self.CalcGravGradMoment(self,r_eci,qb_eci)
        T_total = T_drag + T_mag + T_gravgrad + T_control
        return T_total

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
        rho = self.CalcAtmosDens(self,r_eci)
        vnorm = Matrix[System.Double].Norm(v_eci)
        F_d = -1*rho*1000.0*v_eci*vnorm*self.Cd*self.CxArea
        return F_d

    def CalcDragMoment(self,r_eci,v_eci):
        fdrag = self.CalcDragForce(self,r_eci,v_eci)
        T_d = Matrix[System.Double].Cross(self.CoP,fdrag)
        return T_d

    def CalcMagMoment(self,r_eci,JD,M_dipole,qb_eci):
        bField = self.CalcCurrentMagField(r_eci,JD)
        bFieldBody = Quat.Rotate(qb_eci,bField)
        magMoment = Matrix[System.Double].Cross(M_dipole,bFieldBody)
        return magMoment

    def CalcGravGradMoment(self,r_eci,qb_eci):
        mu = 398600.4418
        rnorm = Matrix[System.Double].Norm(r)
        r5 = rnorm*rnorm*rnorm*rnorm*rnorm
        rb = Quat.Rotate(qb_eci,r_eci)
        T_g = 3.0*mu*Matrix[System.Double].Cross(rb,self.Imat*rb)/rnorm
        return T_g

    def CalcAtmosDens(self,r_eci):
        rE = 6378.137
        h = Matrix[System.Double].Norm(r_eci) - rE
        # Based on exp. atmosphere model from Vallado Table 8-4
        atmMatrix = Matrix[System.Double](28,3)
        atmMatrix.SetRow(1,Matrix[System.Double]([ 0, 1.225, 7.249]))
        atmMatrix.SetRow(2,Matrix[System.Double]([ 25, 3.899E-2, 6.349]))
        atmMatrix.SetRow(3,Matrix[System.Double]([ 30, 1.774E-2, 6.682]))
        atmMatrix.SetRow(4,Matrix[System.Double]([ 40, 3.972E-3, 7.554]))
        atmMatrix.SetRow(5,Matrix[System.Double]([ 50, 1.057E-3, 8.382]))
        atmMatrix.SetRow(6,Matrix[System.Double]([ 60, 3.206E-4, 7.714]))
        atmMatrix.SetRow(7,Matrix[System.Double]([ 70, 8.770E-5, 6.549]))
        atmMatrix.SetRow(8,Matrix[System.Double]([ 80, 1.905E-5, 5.799]))
        atmMatrix.SetRow(9,Matrix[System.Double]([ 90, 3.396E-6, 5.382]))
        atmMatrix.SetRow(10,Matrix[System.Double]([ 100, 5.297E-7, 5.877]))
        atmMatrix.SetRow(11,Matrix[System.Double]([ 110, 9.661E-8, 7.263]))
        atmMatrix.SetRow(12,Matrix[System.Double]([ 120, 2.438E-8, 9.473]))
        atmMatrix.SetRow(13,Matrix[System.Double]([ 130, 8.484E-9, 12.636]))
        atmMatrix.SetRow(14,Matrix[System.Double]([ 140, 3.845E-9, 16.149]))
        atmMatrix.SetRow(15,Matrix[System.Double]([ 150, 2.070E-9, 22.523]))
        atmMatrix.SetRow(16,Matrix[System.Double]([ 180, 5.464E-10, 29.740]))
        atmMatrix.SetRow(17,Matrix[System.Double]([ 200, 2.789E-10, 37.105]))
        atmMatrix.SetRow(18,Matrix[System.Double]([ 250, 7.248E-11, 45.546]))
        atmMatrix.SetRow(19,Matrix[System.Double]([ 300, 2.418E-11, 53.628]))
        atmMatrix.SetRow(20,Matrix[System.Double]([ 350, 9.518E-12, 53.298]))
        atmMatrix.SetRow(21,Matrix[System.Double]([ 400, 3.725E-12, 58.515]))
        atmMatrix.SetRow(22,Matrix[System.Double]([ 450, 1.585E-12, 60.828]))
        atmMatrix.SetRow(23,Matrix[System.Double]([ 500, 6.967E-13, 63.822]))
        atmMatrix.SetRow(24,Matrix[System.Double]([ 600, 1.454E-13, 71.835]))
        atmMatrix.SetRow(25,Matrix[System.Double]([ 700, 3.614E-14, 88.667]))
        atmMatrix.SetRow(26,Matrix[System.Double]([ 800, 1.170E-14, 124.64]))
        atmMatrix.SetRow(27,Matrix[System.Double]([ 900, 5.245E-15, 181.05]))
        atmMatrix.SetRow(28,Matrix[System.Double]([ 1000, 3.019E-15, 268.00]))
        for r in range(1,28+1):
            if h == atmMatrix[28,1]:
                rho = atmMatrix[28,2] * math.exp(-1*(h-atmMatrix[28,1])/atmMatrix[28,3])
            if h > atmMatrix[r,1] and h < atmMatrix[r+1,1]:
                rho = atmMatrix[r,2] * math.exp(-1*(h-atmMatrix[r,1])/atmMatrix[r,3])
            else:
                print('Invalid altitude input to exponential atmosphere calculation')
                break
        return rho

    def CalcWheelInertiaMatrix(self):
        IswMat = Matrix[System.Double](3,3)
        IswMat[1,1] = self.IsWheelsVec[1]
        IswMat[2,2] = self.IsWheelsVec[2]
        IswMat[3,3] = self.IsWheelsVec[3]
        return IswMat

    def CalcCurrentYMDhms(JD):
        # calculates current utc year, month, day, hour, minute, and second and returns a list as [Y,M,D,h,m,s] per Vallado's "Inverse Julian Date" algorithm
        J2000 = 2451545.0 #JD for Jan. 1 2000
        Y2000 = 2000.0 
        T2000 = (JD - J2000)/365.25 # Number of Julian centuries (*100) from Jan. 1, 2000
        Y = Y2000 + math.floor(T2000)
        lyrs = math.floor(0.25*(Y-Y2000-1))
        days = (JD-J2000)-(365.0*(Y-Y2000)+lyrs)
        if days < 1.0:
            Y = Y-1
            lyrs = math.floor(0.25*(Y-Y2000-1))
            days = (JD-J2000)-(365.0*(Y-Y2000)+lyrs)
        dayofyr = math.floor(days)

        # determine month
        lmonth = [0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
        if Y % 4.0 == 0:
            lmonth[2] = 29

        # for loop to determine calendar month
        idx = 1
        tempint = 0
        while (dayofyr > tempint + lmonth[idx]) and (idx < 12):
            tempint += lmonth[idx]
            idx += 1
        M = idx
        D = dayofyr - tempint

        # find hours, minutes, seconds
        temp = (days - dayofyr) * 24.0
        h = math.floor(temp)
        temp = (temp-h) * 60.0
        m = math.floor(temp)
        s = (temp - m) * 60.0
        out[0] = Y
        out[1] = M
        out[2] = D
        out[3] = h
        out[4] = m
        out[5] = s
        return out # return Y M D h m s as list

    def CalcCurrentUTCDateTime(self,Y,M,D,h,m,s):
        currDateTime = System.Datetime(Y,M,D,h,m,s,kind=DateTimeKind.Utc)
        return currDateTime

    def CalcCurrentMagField(self,r_eci,JD):
        assetLLA = GeometryUtilities.ECI2LLA(r_eci,JD)
        assetLat = assetLLA[1]
        assetLong = assetLLA[2]
        assetAlt = assetLLA[3]
        YMDhms = self.CalcCurrentYMDhms(JD)
        Y = YMDhms[0]
        M = YMDhms[1]
        D = YMDhms[2]
        h = YMDhms[3]
        m = YMDhms[4]
        s = YMDhms[5]
        cDT = self.CalcCurrentUTCDateTime(Y,M,D,h,m,s)
        bvec = WMM.CalcBvec(assetLat,assetLong,assetAlt,cDT)
        bvecECI = GeometryUtilities.NED2ECIRotate(bvec,assetLLA,JD)
        bvecECIMat = Matrix(bvecECI);
        return bvecECIMat
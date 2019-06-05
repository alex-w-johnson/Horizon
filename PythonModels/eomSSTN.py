import sys
import clr
import System.Collections.Generic
import System
clr.AddReference('System.Core')
clr.AddReference('mscorlib')
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
from HSFUniverse import WMM
from HSFUniverse import ExponentialAtmosphere

class eomSSTN(EOMS):
    # Base class is Utilities.EOMS b/c it is typecast to type DynamicEOMS in the EOMFactory, per Adam's suggestion - AJ
    def __new__(cls, node):
        instance = EOMS.__new__(cls)
        instance.Cd = float(node.Attributes["cd"].Value)
        instance.CxArea = float(node.Attributes["cxareaavg"].Value)
        instance.CoP = Matrix[System.Double](node.Attributes["cop"].Value)
        instance.CoM = Matrix[float](node.Attributes["com"].Value)

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
        instance.WHEELTORQUE_KEY = StateVarKey[Matrix[System.Double]](instance.AssetName + '.' + 'rxwheeltorque')
        instance.MAGTORQDIPOLE_KEY = StateVarKey[Matrix[System.Double]](instance.AssetName + '.' + 'magtorqmdipole')
        instance.IsWheelsVec = Matrix[System.Double](node.Attributes["iswheels"].Value)
        instance.IsWheels = Matrix[System.Double](3,3)
        instance.ItWheelsVec = Matrix[float](node.Attributes["itwheels"].Value)
        instance.IsWheels[1,1] = instance.IsWheelsVec[1]
        instance.IsWheels[2,2] = instance.IsWheelsVec[2]
        instance.IsWheels[3,3] = instance.IsWheelsVec[3]
        instance.ItWheels = Matrix[float](3,3)
        #for idx in range(1,instance.ItWheelsVec.NumRows+1):
            #instance.ItWheels[idx,idx] = instance.ItWheelsVec[idx]
        instance.ItWheels[1,1] = instance.ItWheelsVec[1]
        instance.ItWheels[2,2] = instance.ItWheelsVec[2]
        instance.ItWheels[3,3] = instance.ItWheelsVec[3]
        instance.WheelMass = Matrix[float](node.Attributes["wheelsmass"].Value)
        instance.WheelOrigin = Matrix[float](node.Attributes["wheelorigin"].Value)
        instance.Wheel1Pos = Matrix[float](node.Attributes["poswheel1"].Value)
        instance.Wheel2Pos = Matrix[float](node.Attributes["poswheel2"].Value)
        instance.Wheel3Pos = Matrix[float](node.Attributes["poswheel3"].Value)
        com2w1 = instance.WheelOrigin+instance.Wheel1Pos-instance.CoM
        com2w2 = instance.WheelOrigin+instance.Wheel2Pos-instance.CoM
        com2w3 = instance.WheelOrigin+instance.Wheel3Pos-instance.CoM
        w1rnorm = Vector.Norm(Vector(com2w1.ToString()))
        w2rnorm = Vector.Norm(Vector(com2w2.ToString()))
        w3rnorm = Vector.Norm(Vector(com2w3.ToString()))
        w1r2 = w1rnorm*w1rnorm
        w2r2 = w2rnorm*w2rnorm
        w3r2 = w3rnorm*w3rnorm
        w1outer = com2w1*Matrix[float].Transpose(com2w1)
        w2outer = com2w2*Matrix[float].Transpose(com2w2)
        w3outer = com2w3*Matrix[float].Transpose(com2w3)
        w1parAxis = instance.WheelMass[1]*(w1r2*Matrix[float].Eye(3)-w1outer)
        w2parAxis = instance.WheelMass[2]*(w2r2*Matrix[float].Eye(3)-w2outer)
        w3parAxis = instance.WheelMass[3]*(w3r2*Matrix[float].Eye(3)-w3outer)
        wTotParAxis = w1parAxis + w2parAxis + w3parAxis
        Jbus = instance.Imat + instance.IsWheels + 2*instance.ItWheels + wTotParAxis
        #instance.Imat = Jbus
        instance.wmm = WMM()
        instance.atmos = ExponentialAtmosphere()
        
        # Residual dipole property
        instance.ResDipole = Matrix[System.Double](node.Attributes["residualdipole"].Value)
        return instance
    
    def PythonAccessor(self, t, y, param, environment):
        #print(t)
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
        epsbeci = Vector(3)
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
        etaI = qb0*Matrix[System.Double].Eye(3)
        epsbecidot = 0.5*Matrix[System.Double].CrossMatrix(epsbeci)*wbeci + 0.5*etaI*wbeci

        # Current Julian Date
        jdCurrent = UserModel.SimParameters.SimStartJD + t/86400.0

        # ADCS control inputs
        T_control = param.GetValue(self.WHEELTORQUE_KEY) # Correct way to get parameters from ADCS? Ask Mehiel - AJ
        M_dipole = param.GetValue(self.MAGTORQDIPOLE_KEY)+self.ResDipole
        #T_control = Matrix[System.Double]('[0;0;0]')
        #M_dipole = self.ResDipole
        # State transition matrix equations
        dy = Matrix[System.Double](16,1)
        dy[1,1] = vxeci
        dy[2,1] = vyeci
        dy[3,1] = vzeci
        #print('Calc Accels')
        acceleration = self.CalcForces(Reci,Veci)
        #print('Done')
        dy[4,1] = acceleration[1]
        dy[5,1] = acceleration[2]
        dy[6,1] = acceleration[3]
        #print('Calc QuatDot')
        dy[7,1] = -0.5*Matrix[System.Double].Dot(Matrix[System.Double](epsbeci.ToString()),wbeci)
        dy[8,1] = epsbecidot[1]
        dy[9,1] = epsbecidot[2]
        dy[10,1] = epsbecidot[3]
        
        #print('done')
        #print('Calc Disturbs')
        T_dist = self.CalcMoments(Reci,Veci,qbeci,T_control,M_dipole,jdCurrent) 
        #print(["Current Applied Torque:" + T_dist.ToString()])
        #print('done')
        Imat = self.Imat
        #print('Calc Ang Accels')
        omegaDot = Matrix[System.Double].Inverse(Imat)*(T_dist-Matrix[System.Double].Cross(wbeci,Imat*wbeci)) # Correct formula for body rate integration? Or need wheel momentum? -AJ        
        dy[11,1] = omegaDot[1]
        dy[12,1] = omegaDot[2]
        dy[13,1] = omegaDot[3]
        #print('done')
        IsWheels = self.IsWheels
        #print('Calc Wheel Dots')
        omegaWDot = Matrix[System.Double].Inverse(IsWheels)*(-T_control-Matrix[System.Double].Cross(wbeci,IsWheels*wwb)) # Fairly confident, ask Mehiel anyways - AJ
        #print('done')
        dy[14,1] = omegaWDot[1]
        dy[15,1] = omegaWDot[2]
        dy[16,1] = omegaWDot[3]
        #print(System.String(Matrix[float].Transpose(dy)))
        return dy
    
    def CalcForces(self,r_eci,v_eci):
        a_grav = self.CalcGravityForce(r_eci)
        a_J2 = self.CalcJ2Force(r_eci)
        a_drag = self.CalcDragForce(r_eci,v_eci)/self.Mass
        a_total = a_grav + a_J2 + a_drag
        return a_total

    def CalcMoments(self,r_eci,v_eci,qb_eci,T_control,M_dipole,jdCurrent):
        T_drag = self.CalcDragMoment(r_eci,v_eci)
        #print('drag')
        #print(T_drag)
        T_mag = self.CalcMagMoment(r_eci,jdCurrent,M_dipole,qb_eci)
        #print('mag')
        #print(T_mag)
        T_gravgrad = self.CalcGravGradMoment(r_eci,qb_eci)
        #print('grad')
        #print(T_gravgrad)
        #print(T_control)
        T_total = T_drag + T_mag + T_gravgrad + T_control
        #print(["T_tot = " + T_total.ToString()])
        return T_total

    def CalcGravityForce(self,r):
        mu = 398600.4418
        r3 = Matrix[System.Double].Norm(r)**3
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
        aJ2[1] = -((3*J2*mu*(rE**2)*r[1])/(2*(rnorm**5)))*(1-((5*(r[3]**2))/(rnorm**2)))
        aJ2[2] = -((3*J2*mu*(rE**2)*r[2])/(2*(rnorm**5)))*(1-((5*(r[3]**2))/(rnorm**2)))
        aJ2[3] = -((3*J2*mu*(rE**2)*r[3])/(2*(rnorm**5)))*(3-((5*(r[3]**2))/(rnorm**2)))
        return aJ2

    def CalcDragForce(self,r_eci,v_eci):
        rho = self.CalcAtmosDens(r_eci)
        #print(rho)
        vnorm = 1000.0*Matrix[System.Double].Norm(v_eci)
        F_d = -1*rho*1000.0*v_eci*vnorm*self.Cd*self.CxArea
        #print(F_d)
        return F_d

    def CalcDragMoment(self,r_eci,v_eci):
        fdrag = self.CalcDragForce(r_eci,v_eci)
        #print('Drag')
        T_d = Matrix[System.Double].Cross(self.CoP,fdrag)
        #print(T_d)
        return T_d

    def CalcMagMoment(self,r_eci,JD,M_dipole,qb_eci):
        bField = self.CalcCurrentMagField(r_eci,JD)
        #print(["B [T] = "+bField.ToString()])
        #print('bfield')
        #print(type(bField))
        bFieldBody = Quat.Rotate(qb_eci,bField)
        #print('bfieldBody')
        #print(type(bFieldBody))
        magMoment = Matrix[System.Double].Cross(M_dipole,bFieldBody)
        return magMoment

    def CalcGravGradMoment(self,r_eci,qb_eci):
        mu = 398600.4418
        rnorm = Matrix[System.Double].Norm(r_eci)
        r5 = rnorm**5
        rb = Quat.Rotate(qb_eci,r_eci)
        #print(rb)
        T_g = 3.0*mu*Matrix[System.Double].Cross(rb,self.Imat*rb)/r5
        return T_g

    def CalcAtmosDens(self,r_eci):
        rE = 6378.137
        h = Matrix[System.Double].Norm(r_eci) - rE
        #print(h)
        # Based on exp. atmosphere model from Vallado Table 8-4
        #print(System.DateTime.Now.ToString("HH:mm:ss"))
        rho = self.atmos.density(h)
        return rho

    def CalcCurrentYMDhms(self,JD):
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
        out = [Y,M,D,h,m,s]
        return out # return Y M D h m s as list

    def CalcCurrentUTCDateTime(self,Y,M,D,h,m,s):
        kind = System.DateTimeKind.Utc
        currDateTime = System.DateTime(Y,M,D,h,m,s,kind)
        return currDateTime

    def CalcCurrentMagField(self,r_eci,JD):
        r_eciVec = Vector(3)
        r_eciVec[1] = r_eci[1]
        r_eciVec[2] = r_eci[2]
        r_eciVec[3] = r_eci[3]
        assetLLA = GeometryUtilities.ECI2LLA(r_eciVec,JD)
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
        bvec = self.wmm.CalcBvec(assetLat,assetLong,assetAlt,cDT)
        bVec = Vector(3)
        bVec[1] = bvec[1]
        bVec[2] = bvec[2]
        bVec[3] = bvec[3]
        bvecECI = GeometryUtilities.NED2ECIRotate(bVec,assetLLA,JD)
        bvecECIMat = Matrix[System.Double](3,1)
        bvecECIMat[1] = bvecECI[1]
        bvecECIMat[2] = bvecECI[2]
        bvecECIMat[3] = bvecECI[3]
        #print(["B func [T] = "+bvecECIMat.ToString()])
        return bvecECIMat*1.0e-9
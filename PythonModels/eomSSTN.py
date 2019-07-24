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
        # Geometry data
        instance.geometry = Matrix[float](node.Attributes["geometry"].Value)
        instance.faceCentroids = instance.geometry[":",MatrixIndex(7,9)]
        instance.faceNormals = instance.geometry[":",MatrixIndex(10,12)]
        instance.faceAreas = instance.geometry.GetColumn(13)
        # Constants
        instance.ResDipole = Matrix[System.Double](node.Attributes["residualdipole"].Value)
        instance.invImat = Matrix[System.Double].Eye(3)
        instance.invImat[1,1] = 1.0/instance.Imat[1,1]
        instance.invImat[2,2] = 1.0/instance.Imat[2,2]
        instance.invImat[3,3] = 1.0/instance.Imat[3,3]
        instance.invIws = Matrix[System.Double].Eye(3)
        instance.invIws[1,1] = 1.0/instance.IsWheels[1,1]
        instance.invIws[2,2] = 1.0/instance.IsWheels[2,2]
        instance.invIws[3,3] = 1.0/instance.IsWheels[3,3]
        instance.mu = 398600.4418
        instance.rE = 6378.137
        instance.J2000 = 2451545.0
        instance.Y2000 = 2000.0
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
        if param.Mdata.IsEmpty:
            T_control = Matrix[System.Double]('[0;0;0]')
            M_dipole = self.ResDipole
        else:
            T_control = param.GetValue(self.WHEELTORQUE_KEY) # Correct way to get parameters from ADCS? Ask Mehiel - AJ
            M_dipole = param.GetValue(self.MAGTORQDIPOLE_KEY)+self.ResDipole
        # State transition matrix equations
        dy = Matrix[System.Double](16,1)
        dy[1,1] = vxeci
        dy[2,1] = vyeci
        dy[3,1] = vzeci
        #print('Calc Accels')
        acceleration = self.CalcForces(Reci,Veci,qbeci)
        #print('Done')
        dy[4,1] = acceleration[1]
        dy[5,1] = acceleration[2]
        dy[6,1] = acceleration[3]
        dy[7,1] = -0.5*Matrix[System.Double].Dot(Matrix[System.Double](epsbeci.ToString()),wbeci)
        dy[8,1] = epsbecidot[1]
        dy[9,1] = epsbecidot[2]
        dy[10,1] = epsbecidot[3]
        T_dist = self.CalcMoments(Reci,Veci,qbeci,T_control,M_dipole,jdCurrent)
        Imat = self.Imat
        omegaDot = self.invImat*(T_dist-Matrix[System.Double].Cross(wbeci,Imat*wbeci))
        dy[11,1] = omegaDot[1]
        dy[12,1] = omegaDot[2]
        dy[13,1] = omegaDot[3]
        IsWheels = self.IsWheels
        omegaWDot = self.invIws*(-T_control-Matrix[System.Double].Cross(wbeci,IsWheels*wwb))
        dy[14,1] = omegaWDot[1]
        dy[15,1] = omegaWDot[2]
        dy[16,1] = omegaWDot[3]
        return dy
    
    def CalcForces(self,r_eci,v_eci,qb_eci):
        a_total = a_grav = self.CalcGravityForce(r_eci)
        #a_total += self.CalcDragForce(r_eci,v_eci,qb_eci)/self.Mass/1000.0 # convert from m/s^2 to km/s^2
        return a_total

    def CalcMoments(self,r_eci,v_eci,qb_eci,T_control,M_dipole,jdCurrent):
        T_total = T_control
        #T_total += self.CalcDragMoment(r_eci,v_eci,qb_eci)
        #T_total += self.CalcMagMoment(r_eci,jdCurrent,M_dipole,qb_eci)
        #T_total += self.CalcGravGradMoment(r_eci,qb_eci)
        return T_total

    def CalcGravityForce(self,r):
        # Calc 2-body, J2, J3, J4, and J5 acceleration
        r3 = Matrix[System.Double].Norm(r)**3
        agrav = Matrix[System.Double](3,1)
        agrav[1] = -self.mu*r[1]/r3
        agrav[2] = -self.mu*r[2]/r3
        agrav[3] = -self.mu*r[3]/r3
        agrav += self.CalcJ2Force(r)
        #agrav += self.CalcJ3Force(r)
        #agrav += self.CalcJ4Force(r)
        #agrav += self.CalcJ5Force(r)
        return agrav

    def CalcJ2Force(self,r):
        J2 = 1.0826269e-3
        rnorm = Matrix[System.Double].Norm(r)
        aJ2 = Matrix[System.Double](3,1)
        aJ2[1] = -((3*J2*self.mu*(self.rE**2)*r[1])/(2*(rnorm**5)))*(1-((5*(r[3]**2))/(rnorm**2)))
        aJ2[2] = -((3*J2*self.mu*(self.rE**2)*r[2])/(2*(rnorm**5)))*(1-((5*(r[3]**2))/(rnorm**2)))
        aJ2[3] = -((3*J2*self.mu*(self.rE**2)*r[3])/(2*(rnorm**5)))*(3-((5*(r[3]**2))/(rnorm**2)))
        return aJ2

    def CalcJ3Force(self,r):
        J3 = -2.533e-06
        rnorm = Matrix[System.Double].Norm(r)
        aJ3 = Matrix[System.Double](3,1)
        gamma = -(5.0*J3/2.0)*(self.rE/rnorm)**3
        alpha = -( (3*(r[3]/rnorm))-7*(r[3]/rnorm)**3 )
        beta = Matrix[System.Double](3,1,0.0)
        beta[3] = 1 - 5*(r[3]/rnorm)**2
        aJ3 = -(gamma*self.mu/(rnorm**2)) * ( (alpha*r/rnorm) + 0.6*beta )
        return aJ3

    def CalcJ4Force(self,r):
        J4 = -1.620e-06
        rnorm = Matrix[System.Double].Norm(r)
        aJ4 = Matrix[System.Double](3,1)
        gamma = -5*J4*(self.rE/rnorm)**4/8.0
        fourthord = 63*(r[3]/rnorm)**4 #4th order term
        xy02order = 3-42*(r[3]/rnorm)**2 #0 and 2nd order terms for x and y components
        c12 = xy02order + fourthord
        c3 = -15 + 70*(r[3]/rnorm)**2 - fourthord
        aJ4[1] = c12*r[1]/rnorm
        aJ4[2] = c12*r[2]/rnorm
        aJ4[3] = c3*r[3]/rnorm
        aJ4 *= gamma*self.mu/(rnorm**2)
        return aJ4

    def CalcJ5Force(self,r):
        J5 = -2.273e-07
        rnorm = Matrix[System.Double].Norm(r)
        aJ5 = Matrix[System.Double](3,1)
        gamma = -J5*(self.rE/rnorm)**5/8
        c12=3*( (35*r[3]/rnorm) -(210*(r[3]/rnorm)**3) + (231*(r[3]/rnorm)**5) )
        c3 = 15- 315*((r[3]/rnorm)**2) + 945*((r[3]/rnorm)**4) - 693*((r[3]/rnorm)**6)
        aJ5[1]= c12*r[1]/rnorm
        aJ5[2] = c12*r[2]/rnorm
        aJ5[3] = c3*r[3]/rnorm
        aJ5 *= gamma*self.mu/(rnorm**2)
        return aJ5

    def CalcDragForce(self,r_eci,v_eci,qb_eci):
        rho = self.CalcAtmosDens(r_eci)
        vnorm = 1000.0*Matrix[System.Double].Norm(v_eci)
        F_d = Quat.Rotate(Quat.Conjugate(qb_eci),self.CalcTotalPanelDrag(rho,r_eci,v_eci,qb_eci,self.faceNormals,self.faceAreas))
        return F_d

    def CalcTotalPanelDrag(self,rho,r_eci,v_eci,qb_eci,SurfNormMat,SurfAreaVec):
        # Calcs total panel drag in newtons
        totalPanelDrag = Matrix[float](3,1)
        for pan in range(1,SurfNormMat.NumRows):
            totalPanelDrag += self.CalcPanelDragForce(rho,r_eci,v_eci,qb_eci,SurfNormMat[pan,":"],SurfAreaVec[pan])
        return totalPanelDrag

    def CalcPanelDragForce(self,rho,r_eci,v_eci,qb_eci,surfNormal,surfArea):
        # Calcs drag force on an individual panel in the body frame
        vRam = self.CalcRamVelocity(r_eci,v_eci)
        vRamBody = Quat.Rotate(qb_eci,vRam)
        vRamBodyNormalized = vRamBody/Matrix[float].Norm(vRamBody)
        ndotv = self.CalcRamIncDotProd(r_eci,v_eci,qb_eci,surfNormal)
        panDragBody = Matrix[float](3,1,0.0)
        if (ndotv < 0.0):
            panDragBody = 1000.0*Matrix[float].Norm(vRamBody)*ndotv*rho*(1000.0*vRamBody)*surfArea
        return panDragBody

    def CalcRamIncDotProd(self,r_eci,v_eci,qb_eci,surfNormal):
        # Calculates the normalized dot product b/w a panel normal vector and the ram velocity vector in the body frame
        vRam = self.CalcRamVelocity(r_eci,v_eci)
        vRamBody = Quat.Rotate(qb_eci,vRam)
        vRamBodyNormalized = vRamBody/Matrix[float].Norm(vRamBody)
        ndotv = Matrix[float].Dot(surfNormal,vRamBodyNormalized)
        return ndotv

    def CalcRamVelocity(self,r_eci,v_eci):
        omegaEarth = Matrix[float](3,1,0.0)
        omegaEarth[3] = 7.2921159e-5 #rad/s, ECI earth rotational speed
        vRam = v_eci - Matrix[float].Cross(omegaEarth,r_eci) #km/s
        return vRam
    
    def CalcCoP(self,r_eci,v_eci,qb_eci,SurfNormMat,SurfAreaMat,SurfCentMat):
        # Calculates CoP per Damaren De Ruiter pp. 231 (above Eq. 12.3)
        numerator = Matrix[float](3,1)
        denominator = 0.0
        for pan in range(1,SurfNormMat.NumRows):
            ndotv = self.CalcRamIncDotProd(r_eci,v_eci,qb_eci,SurfNormMat[pan,":"])
            if (ndotv < 0.0):
                rhoCM2CP = Matrix[float].Transpose(SurfCentMat[pan,":"]) - self.CoM
                denom = ndotv*SurfAreaMat[pan] 
                numerator += rhoCM2CP*denom
                denominator += denom
        cop = numerator/denominator
        return cop


    def CalcDragMoment(self,r_eci,v_eci,qb_eci):
        fdrag = Quat.Rotate(qb_eci,self.CalcDragForce(r_eci,v_eci,qb_eci))
        CoP = self.CalcCoP(r_eci,v_eci,qb_eci,self.faceNormals,self.faceAreas,self.faceCentroids)
        #T_d = Matrix[System.Double].Cross(self.CoP,fdrag)
        T_d = Matrix[float].Cross(CoP,fdrag)
        return T_d

    def CalcMagMoment(self,r_eci,JD,M_dipole,qb_eci):
        bField = self.CalcCurrentMagField(r_eci,JD)
        bFieldBody = Quat.Rotate(qb_eci,bField)
        magMoment = Matrix[System.Double].Cross(M_dipole,bFieldBody)
        return magMoment

    def CalcGravGradMoment(self,r_eci,qb_eci):
        rnorm = Matrix[System.Double].Norm(r_eci)
        r5 = rnorm**5
        rb = Quat.Rotate(qb_eci,r_eci)
        T_g = 3.0*self.mu*Matrix[System.Double].Cross(rb,self.Imat*rb)/r5
        return T_g

    def CalcAtmosDens(self,r_eci):
        h = Matrix[System.Double].Norm(r_eci) - self.rE
        # Based on exp. atmosphere model from Vallado Table 8-4
        rho = self.atmos.density(h)
        return rho

    def CalcCurrentYMDhms(self,JD):
        # calculates current utc year, month, day, hour, minute, and second and returns a list as [Y,M,D,h,m,s] per Vallado's "Inverse Julian Date" algorithm
        # Visit https://www.celestrak.com/software/vallado-sw.php for more information
        T2000 = (JD - self.J2000)/365.25 # Number of Julian centuries (*100) from Jan. 1, 2000
        Y = self.Y2000 + math.floor(T2000)
        lyrs = math.floor(0.25*(Y-self.Y2000-1))
        days = (JD-self.J2000)-(365.0*(Y-self.Y2000)+lyrs)
        if days < 1.0:
            Y = Y-1
            lyrs = math.floor(0.25*(Y-self.Y2000-1))
            days = (JD-self.J2000)-(365.0*(Y-self.Y2000)+lyrs)
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
        return bvecECIMat*1.0e-9 # convert from nT to Teslas
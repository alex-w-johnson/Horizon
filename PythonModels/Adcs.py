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

class adcs(HSFSubsystem.Subsystem):
    def __new__(cls, node, asset):
        instance = HSFSubsystem.Subsystem.__new__(cls)
        instance.Asset = asset
        instance.POINTVEC_KEY = Utilities.StateVarKey[Utilities.Matrix[System.Double]](instance.Asset.Name + '.' + 'eci_pointing_vector(xyz)')
        instance.SOLARELONG_KEY = Utilities.StateVarKey[System.Double](instance.Asset.Name + '.' + 'solar_elongation_angle')
        instance.EARTHELONG_KEY = Utilities.StateVarKey[System.Double](instance.Asset.Name + '.' + 'earth_elongation_angle')
        instance.WHEELTORQUE_KEY = Utilities.StateVarKey[Utilities.Matrix[System.Double]](instance.Asset.Name + '.' + 'rxwheel_torque')
        instance.MAGTORQDIPOLE_KEY = Utilities.StateVarKey[Utilities.Matrix[System.Double]](instance.Asset.Name + '.' + 'magtorq_dipole')
        instance.ISDESATURATING_KEY = Utilities.StateVarKey[System.Bool](instance.Asset.Name + '.' + 'isdesaturating')
        instance.addKey(instance.POINTVEC_KEY)
        instance.addKey(instance.SOLARELONG_KEY)
        instance.addKey(instance.EARTHELONG_KEY)
        instance.addKey(instance.WHEELTORQUE_KEY)
        instance.addKey(instance.MAGTORQDIPOLE_KEY)
        instance.addKey(instance.ISDESATURATING_KEY)
        instance.Asset.AssetDynamicState.IntegratorParameters.Add(instance.WHEELTORQUE_KEY)
        instance.Asset.AssetDynamicState.IntegratorParameters.Add(instance.MAGTORQDIPOLE_KEY)
        instance.kpvec = Vector(scriptedNode["Control"].Attributes["kpslew"])
        instance.kdvec = Vector(scriptedNode["Control"].Attributes["kdslew"])
        instance.antisolaraxis = Vector(scriptedNode["Guidance"].Attributes["antisolaraxis"])
        instance.numwheels = int(scriptedNode["Wheels"].Attributes["numwheels"])
        instance.masswheels = Vector(scriptedNode["Wheels"].Attributes["wheelsmass"])
        instance.iswheels = Vector(scriptedNode["Wheels"].Attributes["iswheels"])
        instance.itwheels = Vector(scriptedNode["Wheels"].Attributes["itwheels"])
        instance.poswheel1 = Vector(scriptedNode["Wheels"].Attributes["poswheel1"])
        instance.poswheel2 = Vector(scriptedNode["Wheels"].Attributes["poswheel2"])
        instance.poswheel3 = Vector(scriptedNode["Wheels"].Attributes["poswheel3"])
        instance.idlepowerwheels = Vector(scriptedNode["Wheels"].Attributes["idlepowwheels"])
        instance.maxpowerwheels = Vector(scriptedNode["Wheels"].Attributes["maxpowwheels"])
        instance.peaktorqwheels = Vector(scriptedNode["Wheels"].Attributes["peaktorque"])
        instance.maxspeedwheels = Vector(scriptedNode["Wheels"].Attributes["maxspeed"])
        instance.nummagtorx = int(scriptedNode["Magtorquers"].Attributes["nummagtorx"])
        instance.axmagtorx1 = Vector(scriptedNode["Magtorquers"].Attributes["axmagtorx1"])
        instance.axmagtorx2 = Vector(scriptedNode["Magtorquers"].Attributes["axmagtorx2"])
        instance.axmagtorx3 = Vector(scriptedNode["Magtorquers"].Attributes["axmagtorx3"])
        instance.peakbmagtorx = Vector(scriptedNode["Magtorquers"].Attributes["peakB"])
        instance.peakpowermagtorx = Vector(scriptedNode["Magtorquers"].Attributes["maxpowmagtorx"])
        instance.wmm = WMM()
        return instance

    def GetDependencyDictionary(self):
        dep = Dictionary[str, Delegate]()
        depFunc1 = Func[Event,  Utilities.HSFProfile[System.Double]](self.POWERSUB_PowerProfile_ADCSSUB)
        dep.Add("PowerfromADCS" + "." + self.Asset.Name, depFunc1)
        return dep

    def GetDependencyCollector(self):
        return Func[Event,  Utilities.HSFProfile[System.Double]](self.DependencyCollector)

    def POWERSUB_PowerProfile_ADCSSUB(self, event):
        prof1 = HSFProfile[System.Double]()
        prof1[event.GetEventStart(self.Asset)] = 30
        prof1[event.GetTaskStart(self.Asset)] = 60
        prof1[event.GetTaskEnd(self.Asset)] = 30
        return prof1

    def CanPerform(self, event, universe):
        # Event information
        es = event.GetEventStart(self.Asset)
        dt = SimParameters.SimStepSeconds
        
        # Load asset dynamic state
        dynamicStateEs = self.Asset.AssetDynamicState(es)
        posEs = dynamicStateEs.PositionECI(es)
        velEs = dynamicStateEs.VelocityECI(es)
        controlQuatsEs = dynamicStateEs.Quaternions(es)
        controlRatesEs = dynamicStateEs.EulerRates(es)
        wheelSpeedsEs = dynamicStateEs.WheelSpeeds(es)
        
        assetOrbState = Matrix[System.Double](6,1)
        assetOrbState[1] = posEs[1]
        assetOrbState[2] = posEs[2]
        assetOrbState[3] = posEs[3]
        assetOrbState[4] = velEs[1]
        assetOrbState[5] = velEs[2]
        assetOrbState[6] = velEs[3]

        # Load task parameters
        task = event.GetAssetTask(self.Asset)
        taskType = task.Type
        
        # Load target parameters
        target = task.Target
        targetDynStateEs = target.DynamicState(es)
        targetR_ot = targetDynStateEs.PositionECI

        if (taskType == TaskType.IMAGING):
            # Implement roll-constrained slew maneuver here THEN HOLD INERTIAL POINTING AFTER TASK START
            qComLam = self.CalcRollConstrainedQCommand(self,assetOrbState,targetR_ot)
            qBodLam = self.CalcBodyAttitudeLVLH(self,assetOrbState,controlQuatsEs)
            qBodCom = Quat.Conjugate(qComLam)*qBodLam
            propError = self.PropErrorCalc(self,self.kpvec,qBodCom._eps)
            deriError = self.DeriErrorCalc(self,self.kdvec,controlRatesEs)
            T_control = - propError - deriError
            pass
        elif (taskType == TaskType.COMM):
            # Point -X-axis towards target, point +Z-axis towards projection of Y_LVLH onto target pointing vector plane
            qCom0 = self.CalcCommsCommandFrame(self,assetOrbState,targetR_ot)
            qLam0 = self.CalcLVLHECIState(self,assetOrbState)
            qComLam = Quat.Conjugate(qLam0)*qCom0
            qBodLam  = self.CalcBodyAttitudeLVLH(self,assetOrbState,controlQuatsEs)
            qBodCom = Quat.Conjugate(qComLam)*qBodLam
            propError = self.PropErrorCalc(self,self.kpvec,qBodCom._eps)
            deriError = self.DeriErrorCalc(self,self.kdvec,controlRatesEs)
            T_control = - propError - deriError
            pass
        elif (taskType== TaskType.DESATURATE):
            for wheelIdx in range(1,self.numWheels):
                if wheelSpeedsEs[wheelIdx] > self.maxspeedwheels[wheelIdx]:
                    event.SetTaskStart(self.Asset, te)
            # Implement desaturation maneuver here
            # Point -X-axis to nadir, point +z-axis towards Y_LVLH
            qCom0 = self.CalcNadirCommandFrame(self,assetOrbState)
            qLam0 = self.CalcLVLHECIState(self,assetOrbState)
            qComLam = Quat.Conjugate(qLam0)*qCom0
            qBodLam  = self.CalcBodyAttitudeLVLH(self,assetOrbState,controlQuatsEs)
            qBodCom = Quat.Conjugate(qComLam)*qBodLam
            propError = self.PropErrorCalc(self,self.kpvec,qBodCom._eps)
            rEs = Vector.Norm(posEs)
            lvlhRate = Vector.Cross(posEs,velEs)/(rEs*rEs)
            deriError = self.DeriErrorCalc(self,self.kdvec,controlRatesEs - lvlhRate)
            T_control = - propError - deriError
            pass
        else:
            # Point -X-axis to nadir, point +z-axis towards Y_LVLH
            qCom0 = CalcNadirCommandFrame(self,assetOrbState)
            qLam0 = CalcLVLHECIState(self,assetOrbState)
            qComLam = Quat.Conjugate(qLam0)*qCom0
            qBodLam  = CalcBodyAttitudeLVLH(self,assetOrbState,controlQuatsEs)
            qBodCom = Quat.Conjugate(qComLam)*qBodLam
            propError = PropErrorCalc(self,self.kpvec,qBodCom._eps)
            rEs = Vector.Norm(posEs)
            lvlhRate = Vector.Cross(posEs,velEs)/(rEs*rEs)
            deriError = DeriErrorCalc(self,self.kdvec,controlRatesEs - lvlhRate)
            T_control = - propError - deriError
            return True
        return True

    def CanExtend(self, event, universe, extendTo):
        return super(adcs, self).CanExtend(event, universe, extendTo)

    def DependencyCollector(self, currentEvent):
        return super(adcs, self).DependencyCollector(currentEvent)

    def CalcRollConstrainedQCommand(self, state, r_ot):
        # Get state information from dynamic state
        r_oa = state[MatrixIndex(1,3),1]
        v_oa = state[MatrixIndex(4,6),1]
        
        # Determine LVLH Frame (+Z direction = nadir, +Y direction = -1* R cross V
        q_lam0 = self.CalcLVLHECIState(state)

        # Determine Command Frame
        C_com0 = Matrix[System.Double](3,3)
        rho = r_ot - r_oa
        rhoHat = rho / Matrix[System.Double].Norm(rho)
        z_com0 = rhoHat
        nadirProj = z_lam0 - Matrix[System.Double].Dot(z_com0,z_lam0)*z_com0
        x_com0 = -1.0*nadirProj
        y_com0 = Matrix[System.Double].Cross(z_com0,x_com0)
        C_com0 = Matrix[System.Double].SetColumn(C_com0,1,x_com0)
        C_com0 = Matrix[System.Double].SetColumn(C_com0,2,y_com0)
        C_com0 = Matrix[System.Double].SetColumn(C_com0,3,z_com0)
        q_com0 = Quat.Mat2Quat(C_com0)

        # Determine Command Quaternion w.r.t. LVLH Frame
        q_Command = Quat.Conjugate(q_lam0)*q_com0
        return q_Command

    # Calculates orientation of body frame with respect to lvlh frame
    def CalcBodyAttitudeLVLH(self,state,qb_eci):
        q_lam0 = self.CalcLVLHECIState(state)
        q_bodLam = Quat.Conjugate(q_lam0)*qb_eci
        return q_bodLam
        
    def PropErrorCalc(self,kp_vec,pStateError):
        kp_mat = Matrix[System.Double](3,3)
        kp_mat[1,1] = kp_vec[1]
        kp_mat[2,2] = kp_vec[2]
        kp_mat[3,3] = kp_vec[3]
        return kp_mat * pStateError

    def DeriErrorCalc(self,kd_vec,dStateError):
        kd_mat = Matrix[System.Double](3,3)
        kd_mat[1,1] = kd_vec[1]
        kd_mat[2,2] = kd_vec[2]
        kd_mat[3,3] = kd_vec[3]
        return kd_mat * dStateError

    def CalcLVLHECIState(self,state):
        r_oa = state[MatrixIndex(1,3),1]
        v_oa = state[MatrixIndex(4,6),1]
        x_lam0 = Matrix[System.Double](3,1)
        y_lam0 = Matrix[System.Double](3,1)
        z_lam0 = Matrix[System.Double](3,1)
        C_lam0 = Matrix[System.Double](3,3)
        nadirVec = -1.0*r_oa/Matrix.Norm(r_oa)
        z_lam0[1] = nadirVec[1]
        z_lam0[2] = nadirVec[2]
        z_lam0[3] = nadirVec[3]
        yVec = Matrix.Cross(r_oa,v_oa)
        y_lam0[1] = yVec[1]
        y_lam0[2] = yVec[2]
        y_lam0[3] = yVec[3]
        x_lam0 = Matrix.Cross(y_lam0,z_lam0)
        temp = Matrix.Horzcat(x_lam0,y_lam0)
        C_lam0 = Matrix.Horzcat(temp,z_lam0)
        q_lam0 = Quat.Mat2Quat(C_lam0)
        return q_lam0

    def CalcNadirCommandFrame(self,state):
        r_oa = state[MatrixIndex(1,3),1]
        v_oa = state[MatrixIndex(4,6),1]
        x_lam0 = Matrix[System.Double](3,1)
        y_lam0 = Matrix[System.Double](3,1)
        z_lam0 = Matrix[System.Double](3,1)
        C_com0 = Matrix[System.Double](3,3)
        nadirVec = -1.0*r_oa/Matrix.Norm(r_oa)
        z_lam0[1] = nadirVec[1]
        z_lam0[2] = nadirVec[2]
        z_lam0[3] = nadirVec[3]
        yVec = Matrix.Cross(r_oa,v_oa)
        y_lam0[1] = yVec[1]
        y_lam0[2] = yVec[2]
        y_lam0[3] = yVec[3]
        x_lam0 = Matrix.Cross(y_lam0,z_lam0)
        temp = Matrix.Horzcat(-1.0*z_lam0,-1.0*x_lam0)
        C_com0 = Matrix.Horzcat(temp,y_lam0)
        q_com0 = Quat.Mat2Quat(C_com0)
        return q_com0

    def CalcCommsCommandFrame(self,state,r_ot):
        r_oa = state[MatrixIndex(1,3),1]
        v_oa = state[MatrixIndex(4,6),1]
        rho = r_ot - r_oa
        rhoHat = rho / Matrix[System.Double].Norm(rho)
        C_com0 = Matrix[System.Double](3,3)
        x_com0 = Matrix[System.Double](3,1)
        y_com0 = Matrix[System.Double](3,1)
        z_com0 = Matrix[System.Double](3,1)
        x_com0 = -1.0*rhoNorm
        hOrbVec = Matrix[System.Double].Cross(r_oa,v_oa)
        y_lvlh0 = hOrbVec/Matrix[System.Double].Norm(hOrbVec)
        y_lvlhProj = y_lvlh0 - Matrix[System.Double].Dot(rhoHat,y_lvlh0)*rhoHat
        z_com0 = y_lvlhProj
        y_com0 = Matrix[System.Double].Cross(z_com0,x_com0)
        C_com0 = Matrix[System.Double].SetColumn(C_com0,1,x_com0)
        C_com0 = Matrix[System.Double].SetColumn(C_com0,2,y_com0)
        C_com0 = Matrix[System.Double].SetColumn(C_com0,3,z_com0)
        pass

    def CalcDesatCommandDipole(self, bBody, kpDesat, wWheel, wRef):
        # Cross-product dipole control law for desaturating reaction wheels
        # All vectors in BODY frame
        # Per Eq. 16 of "Reaction Wheels Desaturation Using Magnetorquers and Static Input Allocation" - 2015
        desatError = kpDesat*(wWheel-wRef)
        bBody2 = Vector.Norm(bBody) * Vector.Norm(bBody)
        tauM = -1.0*Vector.Cross(bBody,desatError)/bBody2
        return tauM

    def CalcDesatCommandWheelTorque(self, wWheel, IsWheel, omega_body):
        pass
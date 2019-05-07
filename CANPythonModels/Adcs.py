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
        ts = event.GetEventStart(self.Asset)
        state = self.Asset.AssetDynamicState
        pos = state.PositionECI(ts)
        controlState = state[MatrixIndex(7,13),1] # Control variables are body-eci quaternions and body rates
        if self._task.Type == TaskType.IMAGING:
            # Implement roll-constrained slew maneuver here
            pass
        if self._task.Type == TaskType.COMM:
            pass
        if self._task.Type == TaskType.DESATURATE:
            pass
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
        C_com0 = MatrixIndex[System.Double].SetColumn(1,x_com0)
        C_com0 = MatrixIndex[System.Double].SetColumn(2,y_com0)
        C_com0 = MatrixIndex[System.Double].SetColumn(3,z_com0)
        q_com0 = Quat.Mat2Quat(C_com0)

        # Determine Command Quaternion w.r.t. LVLH Frame
        q_Command = Quat.Conjugate(q_lam0)*q_com0
        return q_Command

    def CalcBodyAttitudeLVLH(self,state):
        q_lam0 = self.CalcLVLHECIState
        qb_eci = state[MatrixIndex(7,10),1]
        q_bodLam = Quat.Conjugate(q_lam0)*qb_eci
        return q_bodLam
        
    def PropErrorCalc(kp_vec,pStateError):
        kp_mat = Matrix[System.Double](3,3)
        kp_mat[1,1] = kp_vec[1]
        kp_mat[2,2] = kp_vec[2]
        kp_mat[3,3] = kp_vec[3]
        return kp_mat * pStateError

    def DeriErrorCalc(kd_vec,dStateError):
        kd_mat = Matrix[System.Double](3,3)
        kd_mat[1,1] = kd_vec[1]
        kd_mat[2,2] = kd_vec[2]
        kd_mat[3,3] = kd_vec[3]
        return kd_mat * dStateError

    def CalcLVLHECIState(state):
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

    def CalcRho0(self, event, time):
        r_oa = self.Asset.AssetDynamicState.PositionECI(time)
        r_ot = self._taks.Target.DynamicState.PositionECI(time)
        return r_ot - r_oa

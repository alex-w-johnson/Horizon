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
from HSFUniverse import DynamicState
from UserModel import *
from MissionElements import *
from System import Func, Delegate
from System.Collections.Generic import Dictionary
from IronPython.Compiler import CallTarget0
import eomSSTN
import math
from System import Array

class adcs(HSFSubsystem.Subsystem):
    def __new__(cls, node, asset):
        # N.B re: ChildNode indices (See CAN Model XML File for Details): 
        # 0 = Wheels 
        # 1 = Magtorquers 
        # 2 = Guidance 
        # 3 = Control
        instance = HSFSubsystem.Subsystem.__new__(cls)
        instance.Asset = asset
        instance.POINTVEC_KEY = StateVarKey[Matrix[float]](instance.Asset.Name + '.' + 'eci_pointing_vector(xyz)')
        instance.SOLARELONG_KEY = StateVarKey[float](instance.Asset.Name + '.' + 'solarboreelongation')
        instance.EARTHELONG_KEY = StateVarKey[float](instance.Asset.Name + '.' + 'earthboreangle')
        instance.RAMANGLE_KEY = StateVarKey[float](instance.Asset.Name + '.' + 'ramdirectionboreangle')
        instance.WHEELTORQUE_KEY = StateVarKey[Matrix[float]](instance.Asset.Name + '.' + 'rxwheeltorque')
        instance.MAGTORQDIPOLE_KEY = StateVarKey[Matrix[float]](instance.Asset.Name + '.' + 'magtorqmdipole')
        instance.isDesaturating = False
        instance.ISTRACKING_KEY = StateVarKey[bool](instance.Asset.Name + '.' + 'istracking')
        instance.addKey(instance.POINTVEC_KEY)
        instance.addKey(instance.SOLARELONG_KEY)
        instance.addKey(instance.EARTHELONG_KEY)
        instance.addKey(instance.RAMANGLE_KEY)
        instance.addKey(instance.WHEELTORQUE_KEY)
        instance.addKey(instance.MAGTORQDIPOLE_KEY)
        instance.addKey(instance.ISTRACKING_KEY)
        instance.Asset.AssetDynamicState.IntegratorParameters.Add(instance.WHEELTORQUE_KEY,Matrix[float]("[0.0; 0.0; 0.0]"))
        instance.Asset.AssetDynamicState.IntegratorParameters.Add(instance.MAGTORQDIPOLE_KEY,Matrix[float]("[0.0; 0.0; 0.0]"))
        instance.numwheels = int(node.ChildNodes[0].Attributes["numwheels"].Value)
        instance.masswheels = Matrix[float](node.ChildNodes[0].Attributes["wheelsmass"].Value)
        instance.iswheels = Matrix[float](node.ChildNodes[0].Attributes["iswheels"].Value)
        instance.itwheels = Matrix[float](node.ChildNodes[0].Attributes["itwheels"].Value)
        instance.numWheels = int(node.ChildNodes[0].Attributes["numwheels"].Value)
        instance.poswheel1 = Matrix[float](node.ChildNodes[0].Attributes["poswheel1"].Value)
        instance.poswheel2 = Matrix[float](node.ChildNodes[0].Attributes["poswheel2"].Value)
        instance.poswheel3 = Matrix[float](node.ChildNodes[0].Attributes["poswheel3"].Value)
        instance.idlepowerwheels = Matrix[float](node.ChildNodes[0].Attributes["idlepowwheels"].Value)
        instance.maxpowerwheels = Matrix[float](node.ChildNodes[0].Attributes["maxpowwheels"].Value)
        instance.peaktorqwheels = Matrix[float](node.ChildNodes[0].Attributes["peaktorque"].Value)
        instance.maxspeedwheels = Matrix[float](node.ChildNodes[0].Attributes["maxspeed"].Value)
        instance.nummagtorx = int(node.ChildNodes[1].Attributes["nummagtorx"].Value)
        instance.axmagtorx1 = Matrix[float](node.ChildNodes[1].Attributes["axmagtorx1"].Value)
        instance.axmagtorx2 = Matrix[float](node.ChildNodes[1].Attributes["axmagtorx2"].Value)
        instance.axmagtorx3 = Matrix[float](node.ChildNodes[1].Attributes["axmagtorx3"].Value)
        instance.peakbmagtorx = Matrix[float](node.ChildNodes[1].Attributes["peakB"].Value)
        instance.peakpowermagtorx = Matrix[float](node.ChildNodes[1].Attributes["maxpowmagtorx"].Value)
        instance.antisolaraxis = Matrix[float](node.ChildNodes[2].Attributes["antisolaraxis"].Value)
        instance.slewtime = float(node.ChildNodes[2].Attributes["slewtime"].Value)
        instance.dwelltime = float(node.ChildNodes[2].Attributes["dwelltime"].Value)
        instance.boreaxis = Matrix[float](node.ChildNodes[2].Attributes["boreaxis"].Value)
        instance.antennaaxis = Matrix[float](node.ChildNodes[2].Attributes["antennaaxis"].Value)
        instance.kpvec = Matrix[float](node.ChildNodes[3].Attributes["kpslew"].Value)
        instance.kdvec = Matrix[float](node.ChildNodes[3].Attributes["kdslew"].Value)
        instance.kpdesat = Matrix[float](node.ChildNodes[3].Attributes["kpdesat"].Value)
        instance.kddesat = Matrix[float](node.ChildNodes[3].Attributes["kddesat"].Value)
        instance.pointingbound = float(node.ChildNodes[3].Attributes["pointingbound"].Value)
        instance.beamwidth = float(node.ChildNodes[3].Attributes["beamwidth"].Value)
        instance.trackRate = float(node.ChildNodes[3].Attributes["trackrate"].Value)
        instance.wmm = WMM()
        instance.sun = Sun()
        return instance

    def GetDependencyDictionary(self):
        dep = Dictionary[str, Delegate]()
        depFunc1 = Func[Event,  HSFProfile[float]](self.POWERSUB_PowerProfile_ADCSSUB)
        dep.Add("PowerfromADCS" + "." + self.Asset.Name, depFunc1)
        depFunc2 = Func[Event, float](self.EVAL_tasktype_ADCSSUB)
        dep.Add("EvalfromADCS" + "." + self.Asset.Name, depFunc2)
        return dep

    def GetDependencyCollector(self):
        return Func[Event,  HSFProfile[float]](self.DependencyCollector)

    def POWERSUB_PowerProfile_ADCSSUB(self, event):
        prof1 = HSFProfile[float]()
        prof1[event.GetEventStart(self.Asset)] = 30
        prof1[event.GetTaskStart(self.Asset)] = 60
        prof1[event.GetTaskEnd(self.Asset)] = 30
        return prof1

    def EVAL_tasktype_ADCSSUB(self, event):
        task = event.GetAssetTask(self.Asset)
        if task.Type == TaskType.IMAGING:
            return 1.0E5
        elif task.Type == TaskType.FLYALONG:
            return 2.0
        elif task.Type == TaskType.DESATURATE:
            return 1.0
        else:
            return 0.0

    def CanPerform(self, event, universe):
        # TODO: Evaluate whether a task can be completed during an Access window by 
        # Finding the target's dynamic state during the entire event
        # Calculating the necessary time-varying attitude setpoint to successfully slew to the target
        # Simulate slewing to the target by propagating a "Slewing" dynamic state until endSlewTime (endSlewTime = es + slewtime{from XML file})
        #   Use a while loop to update command quats, rates, and torque commands at 1 Hz rate
        #   Propagate "Slewing" dynamic state using all of these updated values at same 1 Hz rate
        # If pointing error is less than pointing error required, then: 
        #   Update boolean HSFProfile "isTracking" to True from eventStartTime to taskEndTime
        #   Set taskStartTime to endSlewTime
        #   Calculate the necessary time-varying attitude setpoint to successfully slew to the target
        #   Use a while loop to update command quats, rates, and torque commands at 1 Hz rate
        #   Add torque commands as integrator parameters to assetDynState
        #   Propagate assetDynState at 1Hz rate up until taskEndTime (taskStartTime + dwellTime{per XML file})
        #   Return True
        # Else: 
        #   Reset dynamic state to values at event start time 
        #   Reset ADCS states to values at event start time 
        #   Add integrator parameters to maintain lvlh pointing
        #   Return False

        
        # Event information
        asset = self.Asset
        es = event.GetEventStart(asset)
        ts = event.GetTaskStart(asset)
        te = event.GetTaskEnd(asset)
        ee = event.GetEventEnd(asset)
        dt = SchedParameters.SimStepSeconds

        # Task information
        task = event.GetAssetTask(asset)
        taskType = task.Type

        # Target information
        target = task.Target
        targetDynState = target.DynamicState

        # Load asset dynamic state
        assetDynState = asset.AssetDynamicState
        assetVelEs = assetDynState.VelocityECI(es)
        assetQuatEs = assetDynState.Quaternions(es)
        assetQuatEs = Quat(assetQuatEs[1],assetQuatEs[2],assetQuatEs[3],assetQuatEs[4])
        assetRatesEs = assetDynState.EulerRates(es)
        assetRatesEs = Matrix[float].Transpose(assetRatesEs)
        assetWheelRates = assetDynState.WheelRates(es)

        # Calculate dynamic states at task start time
        assetPosEs = assetDynState.PositionECI(es)
        targetPosEs = targetDynState.PositionECI(es)
        
        # Calculate event time start
        time = es

        assetOrbState = Matrix[float](6,1)
        assetOrbState[1] = assetPosEs[1]
        assetOrbState[2] = assetPosEs[2]
        assetOrbState[3] = assetPosEs[3]
        assetOrbState[4] = assetVelEs[1]
        assetOrbState[5] = assetVelEs[2]
        assetOrbState[6] = assetVelEs[3]

        if (taskType == TaskType.IMAGING):
            # Implement roll-constrained slew maneuver here THEN HOLD INERTIAL POINTING AFTER TASK START
            # Check to see if can slew to target: if true, then update control inputs accordingly to dynamic EOMS
            
            # Check if desaturating
            if self.isDesaturating:
                return False

            # Check Wheel Speeds
            for wheelIdx in range(1,self.numWheels+1):
                if assetWheelRates[wheelIdx] > self.maxspeedwheels[wheelIdx]:
                    return False
            
            # Check slew time capability
            event.SetTaskEnd(asset,time + self.slewtime + self.dwelltime)
            te = event.GetTaskEnd(asset)
            event.SetEventEnd(asset,te)
            ee = event.GetEventEnd(asset)
            if (te > SimParameters.SimEndSeconds):
                return False
            targetPosMat = Matrix[float].Transpose(Matrix[float](targetPosEs.ToString()))
            dsType = assetDynState.Type
            dsEoms = assetDynState.Eoms
            dsIc = assetDynState.DynamicStateECI(time)
            slewDynState = DynamicState(asset.Name,dsType, dsEoms, dsIc)
            slewDynState.IntegratorParameters.Add(self.WHEELTORQUE_KEY,Matrix[float]("[0.0; 0.0; 0.0]"))
            slewDynState.IntegratorParameters.Add(self.MAGTORQDIPOLE_KEY,Matrix[float]("[0.0; 0.0; 0.0]"))
            # Reset time to 0.0 so state propagates from slew initial conditions
            timeSlew = 0.0
            while(timeSlew < self.slewtime):
                slewPosTime = slewDynState.PositionECI(timeSlew)
                slewPosTime = Matrix[float].Transpose(Matrix[float](slewPosTime.ToString()))
                slewVelTime = slewDynState.VelocityECI(timeSlew)
                slewQuatTime = slewDynState.Quaternions(timeSlew)
                slewQuatTime = Quat(slewQuatTime[1],slewQuatTime[2],slewQuatTime[3],slewQuatTime[4])
                slewRatesTime = Matrix[float].Transpose(slewDynState.EulerRates(timeSlew))
                assetOrbState = Matrix[float](6,1)
                assetOrbState[1] = slewPosTime[1]
                assetOrbState[2] = slewPosTime[2]
                assetOrbState[3] = slewPosTime[3]
                assetOrbState[4] = slewVelTime[1]
                assetOrbState[5] = slewVelTime[2]
                assetOrbState[6] = slewVelTime[3]
                qLam0 = self.CalcLVLHECIState(assetOrbState)
                tNormSlew = timeSlew/self.slewtime
                #print(tNormSlew)
                qComLam = self.CalcRollConstrainedQCommand(assetOrbState,targetPosMat)
                #print("Nominal Command: "+ qComLam.ToString())
                qBodLam = self.CalcBodyAttitudeLVLH(assetOrbState,slewQuatTime)
                #print("Body-LVLH: " + qBodLam.ToString())
                qComLam = Quat.Slerp(tNormSlew,qBodLam,qComLam)
                #print("Interp Command: " + qComLam.ToString())
                qBodCom = Quat.Conjugate(qComLam)*qBodLam
                qErr = Matrix[float].Transpose(Matrix[float](qBodCom._eps.ToString()))
                propError = self.PropErrorCalc(self.kpvec,qErr)
                #print("prop error: " + propError.ToString())
                deriError = self.DeriErrorCalc(self.kdvec,slewRatesTime)
                #print("deri error: " + deriError.ToString())
                T_control = -1.0*propError -deriError
                for i in range(1,self.numwheels+1):
                    if abs(T_control[i]) > self.peaktorqwheels[i]:
                        T_control[i] = math.copysign(self.peaktorqwheels[i],T_control[i])
                slewDynState.IntegratorParameters.Add(self.WHEELTORQUE_KEY,T_control)
                slewDynState.IntegratorParameters.Add(self.MAGTORQDIPOLE_KEY,Matrix[float]("[0.0; 0.0; 0.0]"))
                timeSlew += dt
                slewBoreAxis = Quat.Rotate(Quat.Conjugate(slewQuatTime), self.boreaxis)
                r_AT = targetPosMat - slewPosTime
                pointError = 180.0* System.Math.Acos(  Matrix[float].Dot( slewBoreAxis,r_AT )/(Vector.Norm( Vector(r_AT.ToString()) )*Vector.Norm(Vector(slewBoreAxis.ToString())))) /System.Math.PI
                #print("pointing error: " + pointError.ToString())
                trackingRate = Vector(2)
                trackingRate[1] = slewRatesTime[1]
                trackingRate[2] = slewRatesTime[2]
                trackRateError = Vector.Norm(trackingRate)
                #print("tracking error: " + trackRateError.ToString())
                #print(pointError.ToString())
                if (pointError <= self.pointingbound) and (trackRateError <= self.trackRate):
                    print("Imaging")
                    event.SetTaskStart(asset,time + timeSlew)
                    ts = event.GetTaskStart(asset)
                    event.SetTaskEnd(asset,ts + self.dwelltime)
                    te = event.GetTaskEnd(asset)
                    event.SetEventEnd(asset,te)
                    ee = event.GetEventEnd(asset)
                    while(time < te):
                        tNorm = (time - es)/ts
                        print("Normalized Maneuver Time: " + tNorm.ToString())
                        assetPosTime = assetDynState.PositionECI(time)
                        assetPosTime = Matrix[float].Transpose(Matrix[float](assetPosTime.ToString()))
                        assetVelTime = assetDynState.VelocityECI(time)
                        assetQuatTime = assetDynState.Quaternions(time)
                        assetQuatTime = Quat(assetQuatTime[1], assetQuatTime[2], assetQuatTime[3], assetQuatTime[4])
                        assetRatesTime = Matrix[float].Transpose(assetDynState.EulerRates(time))
                        assetOrbState = Matrix[float](6,1)
                        assetOrbState[1] = assetPosTime[1]
                        assetOrbState[2] = assetPosTime[2]
                        assetOrbState[3] = assetPosTime[3]
                        assetOrbState[4] = assetVelTime[1]
                        assetOrbState[5] = assetVelTime[2]
                        assetOrbState[6] = assetVelTime[3]
                        qLam0 = self.CalcLVLHECIState(assetOrbState)
                        qBodLam = self.CalcBodyAttitudeLVLH(assetOrbState,assetQuatTime)
                        if time < ts:
                            qComLam = self.CalcRollConstrainedQCommand(assetOrbState,targetPosMat)
                            print("Nominal Command: "+ qComLam.ToString())
                            qComLam = Quat.Slerp(tNorm,qBodLam,qComLam)
                            print("Interp Command: " + qComLam.ToString())
                            qComLamHold = qComLam
                            qCom0Hold = qLam0*qComLamHold
                        elif time >= ts and time < te:
                            qCom0 = qCom0Hold
                            #print(qCom0)
                            qComLam = Quat.Conjugate(qLam0)*qCom0
                            #print(qLam0*qComLam)
                            #print("qCommand: " + qComLam.ToString())
                        qBodCom = Quat.Conjugate(qComLam)*qBodLam
                        qErr = Matrix[float].Transpose(Matrix[float](qBodCom._eps.ToString()))
                        #print("qError: " + qErr.ToString())
                        propError = self.PropErrorCalc(self.kpvec,qErr)
                        #print("pError: " + propError.ToString())
                        r = Vector.Norm(Vector(assetPosTime.ToString()))
                        lvlhRate = Matrix[float].Cross(assetPosTime,Matrix[float](assetVelTime.ToString()))/(r*r)
                        lvlhRate = Quat.Rotate(qLam0,lvlhRate)
                        deriError = self.DeriErrorCalc(self.kdvec,assetRatesTime-lvlhRate)
                        #print("dError: " + deriError.ToString())
                        T_control = -1.0*propError -deriError
                        for i in range(1,self.numwheels+1):
                            if abs(T_control[i]) > self.peaktorqwheels[i]:
                                T_control[i] = math.copysign(self.peaktorqwheels[i],T_control[i])
                        assetDynState.IntegratorParameters.Add(self.WHEELTORQUE_KEY,T_control)
                        assetDynState.IntegratorParameters.Add(self.MAGTORQDIPOLE_KEY,Matrix[float]("[0.0; 0.0; 0.0]"))
                        # Update ADCS states
                        borePointing = Quat.Rotate(Quat.Conjugate(assetQuatTime),self.boreaxis)
                        borePointing = borePointing/Vector.Norm(Vector(borePointing.ToString()))
                        solPosNormVec = self.sun.getEarSunVec(time)/Vector.Norm(Vector(self.sun.getEarSunVec(time).ToString()))
                        solElongAng = System.Math.Acos(Matrix[float].Dot(borePointing,solPosNormVec))*180.0/System.Math.PI
                        nadirVec = -1.0*assetPosTime/Vector.Norm(Vector(assetPosTime.ToString()))
                        nadirBoreAng = System.Math.Acos(Matrix[float].Dot(borePointing,nadirVec))*180.0/System.Math.PI
                        ramVec = assetVelTime/Vector.Norm(assetVelTime)
                        ramVec = Matrix[float].Transpose(Matrix[float](ramVec.ToString()))
                        ramAng = System.Math.Acos(Matrix[float].Dot(borePointing,ramVec))*180.0/System.Math.PI
                        self._newState.AddValue(self.POINTVEC_KEY, HSFProfile[Matrix[float]](time,borePointing))
                        self._newState.AddValue(self.SOLARELONG_KEY, HSFProfile[float](time,solElongAng))
                        self._newState.AddValue(self.EARTHELONG_KEY, HSFProfile[float](time,nadirBoreAng))
                        self._newState.AddValue(self.RAMANGLE_KEY, HSFProfile[float](time,ramAng))
                        self._newState.AddValue(self.ISTRACKING_KEY,HSFProfile[bool](time, True))
                        self._newState.AddValue(self.WHEELTORQUE_KEY,HSFProfile[Matrix[float]](time,T_control))
                        self._newState.AddValue(self.MAGTORQDIPOLE_KEY,HSFProfile[Matrix[float]](time,Matrix[float]("[0.0; 0.0; 0.0]")))
                        time += dt
                    self._newState.AddValue(self.ISTRACKING_KEY,HSFProfile[bool](te, False))
                    return True   
            return False
        elif (taskType == TaskType.FLYALONG):
            # Point -X-axis to nadir, point +z-axis towards Y_LVLH
            #print("Flying Along")
            # Check if desaturating
            if self.isDesaturating:
                return False
            # Check Wheel Speeds
            for wheelIdx in range(1,self.numWheels+1):
                if assetWheelRates[wheelIdx] > self.maxspeedwheels[wheelIdx]:
                    return False
            event.SetTaskStart(asset,es)
            event.SetTaskEnd(asset,es+dt)
            event.SetEventEnd(asset,es+dt)
            qCom0 = self.CalcNadirCommandFrame(assetOrbState)
            qLam0 = self.CalcLVLHECIState(assetOrbState)
            qComLam = Quat.Conjugate(qLam0)*qCom0
            qBodLam  = self.CalcBodyAttitudeLVLH(assetOrbState,assetQuatEs)
            qBodCom = Quat.Conjugate(qComLam)*qBodLam
            qErr = Matrix[float](qBodCom._eps.ToString())
            qErr = Matrix[float].Transpose(qErr)
            propError = self.PropErrorCalc(self.kpvec,qErr)
            rEs = Vector.Norm(assetPosEs)
            lvlhRate = Vector.Cross(assetPosEs,assetVelEs)/(rEs*rEs)
            lvlhRate = Quat.Rotate(qLam0,lvlhRate)
            lvlhRate = Matrix[float](lvlhRate.ToString())
            lvlhRate = Matrix[float].Transpose(lvlhRate)
            deriError = self.DeriErrorCalc(self.kdvec,assetRatesEs - lvlhRate)
            T_control = -propError -deriError
            assetDynState.IntegratorParameters.Add(self.WHEELTORQUE_KEY, T_control)
            assetDynState.IntegratorParameters.Add(self.MAGTORQDIPOLE_KEY, Matrix[float]("[0.0; 0.0; 0.0]"))
            borePointing = Quat.Rotate(Quat.Conjugate(assetQuatEs),self.boreaxis)
            borePointing = borePointing/Vector.Norm(Vector(borePointing.ToString()))
            solPosNormVec = self.sun.getEarSunVec(es)/Vector.Norm(Vector(self.sun.getEarSunVec(es).ToString()))
            #print("solPosNormVec: " + solPosNormVec.ToString()+ "    boreAxis: "+ borePointing.ToString())
            solElongAng = System.Math.Acos(Matrix[float].Dot(borePointing,solPosNormVec))*180.0/System.Math.PI
            #print(Matrix[float].Dot(borePointing,solPosNormVec).ToString())
            nadirVec = -1.0*Matrix[float](assetPosEs.ToString())/Vector.Norm(assetPosEs)
            nadirBoreAng = System.Math.Acos(Matrix[float].Dot(borePointing,nadirVec))*180.0/System.Math.PI
            ramVec = assetVelEs/Vector.Norm(assetVelEs)
            ramVec = Matrix[float](ramVec.ToString())
            ramAng = System.Math.Acos(Matrix[float].Dot(borePointing,ramVec))*180.0/System.Math.PI
            self._newState.AddValue(self.POINTVEC_KEY,HSFProfile[Matrix[float]](es,borePointing))
            self._newState.AddValue(self.SOLARELONG_KEY,HSFProfile[float](es,solElongAng))
            self._newState.AddValue(self.EARTHELONG_KEY,HSFProfile[float](es,nadirBoreAng))
            self._newState.AddValue(self.RAMANGLE_KEY,HSFProfile[float](es,ramAng))
            self._newState.AddValue(self.ISTRACKING_KEY,HSFProfile[bool](es, False))
            self._newState.AddValue(self.WHEELTORQUE_KEY,HSFProfile[Matrix[float]](es,T_control))
            self._newState.AddValue(self.MAGTORQDIPOLE_KEY,HSFProfile[Matrix[float]](es,Matrix[float]("[0.0; 0.0; 0.0]")))
            return True
        elif (taskType == TaskType.DESATURATE):
            # Implement desaturation maneuver here
            # Point -X-axis to nadir, point +z-axis towards Y_LVLH
            # Check if desaturated already
            isDesated = True
            for idx in range(1,self.numWheels+1):
                isDesated = (isDesated and (assetWheelRates[idx] < 0.2*self.maxspeedwheels[idx]))
            if (not isDesated):
                self.isDesaturating = True
                event.SetTaskEnd(asset,es+dt)
                event.SetEventEnd(asset,es+dt)
                qCom0 = self.CalcNadirCommandFrame(assetOrbState)
                qLam0 = self.CalcLVLHECIState(assetOrbState)
                qComLam = Quat.Conjugate(qLam0)*qCom0
                qBodLam  = self.CalcBodyAttitudeLVLH(assetOrbState,assetQuatEs)
                qBodCom = Quat.Conjugate(qComLam)*qBodLam
                qErr = Matrix[float](qBodCom._eps.ToString())
                qErr = Matrix[float].Transpose(qErr)
                propError = self.PropErrorCalc(self.kpvec,qErr)
                rEs = Vector.Norm(assetPosEs)
                lvlhRate = Vector.Cross(assetPosEs,assetVelEs)/(rEs*rEs)
                lvlhRate = Quat.Rotate(qLam0,lvlhRate)
                lvlhRate = Matrix[float](lvlhRate.ToString())
                lvlhRate = Matrix[float].Transpose(lvlhRate)
                deriError = self.DeriErrorCalc(self.kdvec,assetRatesEs-lvlhRate)
                T_control = -propError -deriError
                print("Control "+ T_control.ToString())
                r_eci = Matrix[float](3,1)
                for i in range(1,4):
                    r_eci[i,1] = assetPosEs[i]
                mDipoleCommand = self.CalcDesatCommandDipole(self.CalcBodyMagField(r_eci,SimParameters.SimStartJD,assetQuatEs),T_control)
                #print("Dipole Calced")
                Tdipole = self.CalcMagMoment(r_eci,SimParameters.SimStartJD,mDipoleCommand,assetQuatEs)
                print("Dipole Torque: "+Tdipole.ToString())
                #print("Magtorq Calced")
                T_braking = self.CalcDesatCommandWheelTorque(Tdipole,assetWheelRates)
                print("Braking: "+ T_braking.ToString())
                #print("Braking Calced")
                assetDynState.IntegratorParameters.Add(self.WHEELTORQUE_KEY, T_braking)
                assetDynState.IntegratorParameters.Add(self.MAGTORQDIPOLE_KEY, mDipoleCommand)
                #assetDynState.IntegratorParameters.Add(self.WHEELTORQUE_KEY, T_control)
                #assetDynState.IntegratorParameters.Add(self.MAGTORQDIPOLE_KEY, Matrix[float]("[0.0; 0.0; 0.0]"))
                borePointing = Quat.Rotate(Quat.Conjugate(assetQuatEs),self.boreaxis)
                borePointing = borePointing/Vector.Norm(Vector(borePointing.ToString()))
                solPosNormVec = self.sun.getEarSunVec(es)/Vector.Norm(Vector(self.sun.getEarSunVec(es).ToString()))
                #print("solPosNormVec: " + solPosNormVec.ToString()+ "    boreAxis: "+ borePointing.ToString())
                solElongAng = System.Math.Acos(Matrix[float].Dot(borePointing,solPosNormVec))*180.0/System.Math.PI
                #print(Matrix[float].Dot(borePointing,solPosNormVec).ToString())
                nadirVec = -1.0*Matrix[float](assetPosEs.ToString())/Vector.Norm(assetPosEs)
                nadirBoreAng = System.Math.Acos(Matrix[float].Dot(borePointing,nadirVec))*180.0/System.Math.PI
                ramVec = assetVelEs/Vector.Norm(assetVelEs)
                ramVec = Matrix[float](ramVec.ToString())
                ramAng = System.Math.Acos(Matrix[float].Dot(borePointing,ramVec))*180.0/System.Math.PI
                self._newState.AddValue(self.POINTVEC_KEY,HSFProfile[Matrix[float]](es,borePointing))
                self._newState.AddValue(self.SOLARELONG_KEY,HSFProfile[float](es,solElongAng))
                self._newState.AddValue(self.EARTHELONG_KEY,HSFProfile[float](es,nadirBoreAng))
                self._newState.AddValue(self.RAMANGLE_KEY,HSFProfile[float](es,ramAng))
                self._newState.AddValue(self.ISTRACKING_KEY,HSFProfile[bool](es, False))
                self._newState.AddValue(self.WHEELTORQUE_KEY,HSFProfile[Matrix[float]](es,T_braking))
                self._newState.AddValue(self.MAGTORQDIPOLE_KEY,HSFProfile[Matrix[float]](es,mDipoleCommand))
                #print("IsDesaturating")
                return True
            elif isDesated:
                return False
        elif (taskType == TaskType.COMM):
            # Point -X-axis towards target, point +Z-axis towards projection of Y_LVLH onto target pointing vector plane
            for wheelIdx in range(1,self.numWheels+1):
                if assetWheelRates[wheelIdx] > self.maxspeedwheels[wheelIdx]:
                    return False
            # Set up scheduling evaluation for comms task
            event.SetTaskEnd(asset,time + self.slewtime + 600.0)
            te = event.GetTaskEnd(asset)
            event.SetEventEnd(asset,te)
            ee = event.GetEventEnd(asset)
            if (te > SimParameters.SimEndSeconds):
                return False
            targetPosMat = Matrix[float].Transpose(Matrix[float](targetPosEs.ToString()))
            dsType = assetDynState.Type
            dsEoms = assetDynState.Eoms
            dsIc = assetDynState.DynamicStateECI(time)
            slewDynState = DynamicState(asset.Name,dsType, dsEoms, dsIc)
            slewDynState.IntegratorParameters.Add(self.WHEELTORQUE_KEY,Matrix[float]("[0.0; 0.0; 0.0]"))
            slewDynState.IntegratorParameters.Add(self.MAGTORQDIPOLE_KEY,Matrix[float]("[0.0; 0.0; 0.0]"))
            # Reset time to 0.0 so state propagates from slew initial conditions
            timeSlew = 0.0
            while(timeSlew < self.slewtime):
                slewPosTime = slewDynState.PositionECI(timeSlew)
                slewPosTime = Matrix[float].Transpose(Matrix[float](slewPosTime.ToString()))
                slewVelTime = slewDynState.VelocityECI(timeSlew)
                slewQuatTime = slewDynState.Quaternions(timeSlew)
                slewQuatTime = Quat(slewQuatTime[1],slewQuatTime[2],slewQuatTime[3],slewQuatTime[4])
                slewRatesTime = Matrix[float].Transpose(slewDynState.EulerRates(timeSlew))
                assetOrbState = Matrix[float](6,1)
                assetOrbState[1] = slewPosTime[1]
                assetOrbState[2] = slewPosTime[2]
                assetOrbState[3] = slewPosTime[3]
                assetOrbState[4] = slewVelTime[1]
                assetOrbState[5] = slewVelTime[2]
                assetOrbState[6] = slewVelTime[3]
                targetPos = targetDynState.PositionECI(timeSlew)
                targetPosMat = Matrix[float].Transpose(Matrix[float](targetPos.ToString()))
                qCom0 = self.CalcCommsCommandFrame(assetOrbState,targetPosMat)
                qLam0 = self.CalcLVLHECIState(assetOrbState)
                qComLam = Quat.Conjugate(qLam0)*qCom0
                qBodLam = self.CalcBodyAttitudeLVLH(assetOrbState,slewQuatTime)
                qBodCom = Quat.Conjugate(qComLam)*qBodLam
                qErr = Matrix[float].Transpose(Matrix[float](qBodCom._eps.ToString()))
                propError = self.PropErrorCalc(self.kpvec,qErr)
                deriError = self.DeriErrorCalc(self.kdvec,slewRatesTime)
                T_control = -1.0*propError -deriError
                for i in range(1,self.numwheels+1):
                    if abs(T_control[i]) > self.peaktorqwheels[i]:
                        T_control[i] = math.copysign(self.peaktorqwheels[i],T_control[i])
                slewDynState.IntegratorParameters.Add(self.WHEELTORQUE_KEY,T_control)
                slewDynState.IntegratorParameters.Add(self.MAGTORQDIPOLE_KEY,Matrix[float]("[0.0; 0.0; 0.0]"))
                timeSlew += dt
                pointError = 180.0 * System.Math.Acos(Matrix[float].Dot( Quat.Rotate(slewQuatTime, self.boreaxis),(targetPosMat - slewPosTime) )/Vector.Norm( Vector(targetPosMat.ToString())-Vector(slewPosTime.ToString()) ))/System.Math.PI
                if (pointError < self.beamwidth):
                    event.SetTaskStart(asset,time + timeSlew)
                    ts = event.GetTaskStart(asset)
                    teNew = ts;
                    for timeindex in range(1,math.floor(te/dt)):
                        if not GeometryUtilities.hasLOS(assetDynState.PositionECI(timeindex),targetDynState.PositionECI(timeindex)):
                            teNew = ts + timeindex*dt
                    event.SetTaskEnd(asset,teNew)
                    te = event.GetTaskEnd(asset)
                    event.SetEventEnd(asset,te)
                    ee = event.GetEventEnd(asset)
                    while(time < ee):
                        assetPosTime = assetDynState.PositionECI(time)
                        assetPosTime = Matrix[float].Transpose(Matrix[float](assetPosTime.ToString()))
                        assetVelTime = assetDynState.VelocityECI(time)
                        assetQuatTime = assetDynState.Quaternions(time)
                        assetQuatTime = Quat(assetQuatTime[1], assetQuatTime[2], assetQuatTime[3], assetQuatTime[4])
                        assetRatesTime = assetDynState.EulerRates(time)
                        assetOrbState = Matrix[float](6,1)
                        assetOrbState[1] = assetPosTime[1]
                        assetOrbState[2] = assetPosTime[2]
                        assetOrbState[3] = assetPosTime[3]
                        assetOrbState[4] = assetVelTime[1]
                        assetOrbState[5] = assetVelTime[2]
                        assetOrbState[6] = assetVelTime[3]
                        targetPos = targetDynState.PositionECI(time)
                        targetPosMat = Matrix[float].Transpose(Matrix[float](targetPos.ToString()))
                        qCom0 = self.CalcCommsCommandFrame(assetOrbState,targetPosMat)
                        qLam0 = self.CalcLVLHECIState(assetOrbState)
                        qComLam = Quat.Conjugate(qLam0)*qCom0
                        qErr = Matrix[float].Transpose(Matrix[float](qBodCom._eps.ToString()))
                        propError = self.PropErrorCalc(self.kpvec,qErr)
                        deriError = self.DeriErrorCalc(self.kdvec,slewRatesTime)
                        T_control = -1.0*propError -deriError
                        for i in range(1,self.numwheels+1):
                            if abs(T_control[i]) > self.peaktorqwheels[i]:
                                T_control[i] = math.copysign(self.peaktorqwheels[i],T_control[i])
                        slewDynState.IntegratorParameters.Add(self.WHEELTORQUE_KEY,T_control)
                        slewDynState.IntegratorParameters.Add(self.MAGTORQDIPOLE_KEY,Matrix[float]("[0.0; 0.0; 0.0]"))
                        # Update ADCS states
                        borePointing = Quat.Rotate(Quat.Conjugate(assetQuatTime),self.boreaxis)
                        borePointing = borePointing/Vector.Norm(Vector(borePointing.ToString()))
                        solPosNormVec = self.sun.getEarSunVec(time)/Vector.Norm(Vector(self.sun.getEarSunVec(time).ToString()))
                        solElongAng = System.Math.Acos(Matrix[float].Dot(borePointing,solPosNormVec))*180.0/System.Math.PI
                        nadirVec = -1.0*assetPosTime/Vector.Norm(Vector(assetPosTime.ToString()))
                        nadirBoreAng = System.Math.Acos(Matrix[float].Dot(borePointing,nadirVec))*180.0/System.Math.PI
                        ramVec = assetVelTime/Vector.Norm(assetVelTime)
                        ramVec = Matrix[float].Transpose(Matrix[float](ramVec.ToString()))
                        ramAng = System.Math.Acos(Matrix[float].Dot(borePointing,ramVec))*180.0/System.Math.PI
                        self._newState.AddValue(self.POINTVEC_KEY, HSFProfile[Matrix[float]](time,borePointing))
                        self._newState.AddValue(self.SOLARELONG_KEY, HSFProfile[float](time,solElongAng))
                        self._newState.AddValue(self.EARTHELONG_KEY, HSFProfile[float](time,nadirBoreAng))
                        self._newState.AddValue(self.RAMANGLE_KEY, HSFProfile[float](time,ramAng))
                        self._newState.AddValue(self.ISTRACKING_KEY,HSFProfile[bool](time, True))
                        self._newState.AddValue(self.WHEELTORQUE_KEY,HSFProfile[Matrix[float]](time,T_control))
                        self._newState.AddValue(self.MAGTORQDIPOLE_KEY,HSFProfile[Matrix[float]](time,Matrix[float]("[0.0; 0.0; 0.0]")))
                        time += dt
                    self._newState.AddValue(self.ISTRACKING_KEY,HSFProfile[bool](te, False))
                    return True
                else:
                    return False
        return False

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
        rhoHat = rho / Vector.Norm(Vector(rho.ToString()))
        z_com0 = rhoHat
        rNorm = Vector.Norm(Vector(r_oa.ToString()))
        nadirVec = -1.0*r_oa/rNorm
        z_lam0 = Matrix[float](3,1)
        z_lam0[1] = nadirVec[1]
        z_lam0[2] = nadirVec[2]
        z_lam0[3] = nadirVec[3]
        nadirProj = z_lam0 - Matrix[System.Double].Dot(z_com0,z_lam0)*z_com0
        x_com0 = -1.0*nadirProj
        y_com0 = Matrix[System.Double].Cross(z_com0,x_com0)
        C_com0.SetColumn(1,x_com0)
        C_com0.SetColumn(2,y_com0)
        C_com0.SetColumn(3,z_com0)
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
        r_oa = state[MatrixIndex(1, 3),1]
        v_oa = state[MatrixIndex(4, 6),1]
        x_lam0 = Matrix[System.Double](3,1)
        y_lam0 = Matrix[System.Double](3,1)
        z_lam0 = Matrix[System.Double](3,1)
        C_lam0 = Matrix[System.Double](3,3)
        rNorm = Vector.Norm(Vector(r_oa.ToString()))
        nadirVec = -1.0*r_oa/rNorm
        z_lam0[1] = nadirVec[1]
        z_lam0[2] = nadirVec[2]
        z_lam0[3] = nadirVec[3]
        yVec = -1.0*Matrix[float].Cross(r_oa,v_oa)
        ynorm = Vector.Norm(Vector(yVec.ToString()))
        yVec = yVec/ynorm
        y_lam0[1] = yVec[1]
        y_lam0[2] = yVec[2]
        y_lam0[3] = yVec[3]
        x_lam0 = Matrix[float].Cross(y_lam0,z_lam0)
        C_lam0.SetColumn(1,x_lam0)
        C_lam0.SetColumn(2,y_lam0)
        C_lam0.SetColumn(3,z_lam0)
        q_lam0 = Quat.Mat2Quat(C_lam0)
        return q_lam0

    def CalcNadirCommandFrame(self,state):
        r_oa = state[MatrixIndex(1,3),1]
        v_oa = state[MatrixIndex(4,6),1]
        x_lam0 = Matrix[System.Double](3,1)
        y_lam0 = Matrix[System.Double](3,1)
        z_lam0 = Matrix[System.Double](3,1)
        C_com0 = Matrix[System.Double](3,3)
        rNorm = Vector.Norm(Vector(r_oa.ToString()))
        #print("rNorm" + rNorm.ToString())
        nadirVec = -1.0*r_oa/rNorm
        #print(nadirVec)
        z_lam0[1] = nadirVec[1]
        z_lam0[2] = nadirVec[2]
        z_lam0[3] = nadirVec[3]
        yVec = -1.0*Matrix[float].Cross(r_oa,v_oa)
        ynorm = Vector.Norm(Vector(yVec.ToString()))
        yVec = yVec/ynorm
        y_lam0[1] = yVec[1]
        y_lam0[2] = yVec[2]
        y_lam0[3] = yVec[3]
        #print(y_lam0)
        x_lam0 = Matrix[float].Cross(y_lam0,z_lam0)
        C_com0.SetColumn(1,-z_lam0)
        C_com0.SetColumn(2,-x_lam0)
        C_com0.SetColumn(3,y_lam0)
        #print(C_com0)
        #print(C_com0.ToString())
        q_com0 = Quat.Mat2Quat(C_com0)
        #print(q_com0)
        return q_com0

    def CalcCommsCommandFrame(self,state,r_ot):
        r_oa = state[MatrixIndex(1,3),1]
        v_oa = state[MatrixIndex(4,6),1]
        rho = r_ot - r_oa
        rhoHat = rho / Vector.Norm(Vector(rho.ToString()))
        C_com0 = Matrix[System.Double](3,3)
        x_com0 = Matrix[System.Double](3,1)
        y_com0 = Matrix[System.Double](3,1)
        z_com0 = Matrix[System.Double](3,1)
        x_com0 = -1.0*rhoNorm
        hOrbVec = Matrix[System.Double].Cross(r_oa,v_oa)
        y_lvlh0 = -hOrbVec/Vector.Norm(Vector(hOrbVec.ToString()))
        y_lvlhProj = y_lvlh0 - Matrix[System.Double].Dot(rhoHat,y_lvlh0)*rhoHat
        z_com0 = y_lvlhProj
        y_com0 = Matrix[System.Double].Cross(z_com0,x_com0)
        C_com0 = Matrix[System.Double].SetColumn(C_com0,1,x_com0)
        C_com0 = Matrix[System.Double].SetColumn(C_com0,2,y_com0)
        C_com0 = Matrix[System.Double].SetColumn(C_com0,3,z_com0)
        pass

    def CalcDesatCommandDipole(self, bBody, T_command):
        # Cross-product dipole control law for desaturating reaction wheels
        # All vectors in BODY frame
        # Per Eq. 16 of "Reaction Wheels Desaturation Using Magnetorquers and Static Input Allocation" - 2015
        bBody2 = Vector.Norm(Vector(bBody.ToString())) * Vector.Norm(Vector(bBody.ToString()))
        tauM = Matrix[System.Double].Cross(bBody,T_command)/bBody2
        # Saturated magtorquer condition
        for i in range(1,4):
            if System.Math.Abs(tauM[i]) > self.peakbmagtorx[i]:
                tauM[i] = System.Math.Sign(tauM[i])*self.peakbmagtorx[i]
        return tauM

    def CalcDesatCommandWheelTorque(self, T_dipole, omegaWheels):
        magBrakeRatio = 0.7 # Derived from Simulink simulation results, there's probably an entire research paper to back this up
        # TODO: Check we have enough power to brake this hard while desaturating
        T_brake = Matrix[System.Double](3,1)
        # Direction to apply braking torque
        for i in range(1,4):
            absValTmag = System.Math.Abs(T_dipole[i])
            T_brake[i] = magBrakeRatio*absValTmag*omegaWheels[i]/self.maxspeedwheels[i]
            if System.Math.Abs(T_brake[i]) > self.peaktorqwheels[i]:
                T_brake[i] = System.Math.Sign(T_brake[i])*self.peaktorqwheels[i]
            #print(T_brake)
        return T_brake

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

    def CalcMagMoment(self,r_eci,JD,M_dipole,qb_eci):
        bFieldBody = self.CalcBodyMagField(r_eci,JD,qb_eci)
        magMoment = Matrix[System.Double].Cross(M_dipole,bFieldBody)
        return magMoment

    def CalcBodyMagField(self,r_eci,JD,qb_eci):
        bField = self.CalcCurrentMagField(r_eci,JD)
        bFieldBody = Quat.Rotate(qb_eci,bField)
        return bFieldBody
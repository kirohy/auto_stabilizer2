#!/usr/bin/env python

import time
import math
import sys

from hrpsys import rtm

from hrpsys.OpenHRP import *
import OpenHRP

from auto_stabilizer import *
from auto_stabilizer.AutoStabilizerService_idl import *

def findComp(name):
    timeout_count = 0
    comp = None
    while timeout_count < 10:
        comp = rtm.findRTC(name)
        if comp != None and comp.isActive():
            break
        print("find Comp wait for " + name)
        time.sleep(1)
        timeout_count += 1
    if comp == None:
        print("Cannot find component: %s" % name)
    return comp

class AutoStabilizer_Configurator(object):
    Groups = {}

    rh_svc = None
    seq_svc = None
    sh_svc = None
    ast_svc = None
    kf_svc = None
    rmfo_svc = None
    log_svc = None

    def __init__(self, *args, **kwargs):
        self.Groups = {}

        self.rh_svc = rtm.findService(findComp("RobotHardware0"),"RobotHardwareService","RobotHardwareService","service0")._narrow(OpenHRP.RobotHardwareService)
        self.seq_svc = rtm.findService(findComp("seq"),"SequencePlayerService","SequencePlayerService","service0")._narrow(OpenHRP.SequencePlayerService)
        self.sh_svc = rtm.findService(findComp("sh"),"StateHolderService","StateHolderService","service0")._narrow(OpenHRP.StateHolderService)
        self.ast_svc = rtm.findService(findComp("ast"),"AutoStabilizerService","AutoStabilizerService","service0")._narrow(OpenHRP.AutoStabilizerService)
        self.kf_svc = rtm.findService(findComp("kf"),"KalmanFilterService","KalmanFilterService","service0")._narrow(OpenHRP.KalmanFilterService)
        self.rmfo_svc = rtm.findService(findComp("rmfo"),"RemoveForceSensorLinkOffsetService","RemoveForceSensorLinkOffsetService","service0")._narrow(OpenHRP.RemoveForceSensorLinkOffsetService)
        self.log_svc = findComp("log").service("service0")._narrow(OpenHRP.DataLoggerService)


    def servoOn(self, jname='all'):
        self.servoOff()

        c = False
        while (c != 'Y' and c != 'y'):
            c = raw_input("press 'Y' for servo ON and power ON. >> ")

        # urata system flag resetting for jaxon
        self.rh_svc.power(jname ,OpenHRP.RobotHardwareService.SWITCH_ON)
        time.sleep(0.01)
        self.rh_svc.power(jname ,OpenHRP.RobotHardwareService.SWITCH_OFF)
        time.sleep(0.02)

        self.rh_svc.setServoGainPercentage(jname,100) # for setting 100 to initial servo position gain

        # reset JointGroups
        for k, v in self.Groups.items():
            self.seq_svc.removeJointGroup(k)
        for k, v in self.Groups.items():
            self.seq_svc.waitInterpolationOfGroup(k)
        for k, v in self.Groups.items():
            self.seq_svc.addJointGroup(k, v)

        # move to idle mode for filter type RTCs
        for rtc_name in ["ast"]:
            rtc = rtm.findRTC(rtc_name)
            if rtc:
                rtc.stop();
                rtc.start();

        time.sleep(2) # withtout this. command angle jumps
        self.sh_svc.goActual()
        time.sleep(0.1)
        self.rh_svc.power(jname, OpenHRP.RobotHardwareService.SWITCH_ON)
        time.sleep(0.2)
        self.rh_svc.servo(jname, OpenHRP.RobotHardwareService.SWITCH_ON)
        time.sleep(2) # wait for gain transition
        return True

    def servoOff(self,jname='all'):
        c = False
        while (c != 'Y' and c != 'y'):
            c = raw_input("press 'Y' for servo OFF and power OFF. >> ")

        self.rh_svc.servo('all', OpenHRP.RobotHardwareService.SWITCH_OFF)
        time.sleep(0.2)
        if jname == 'all':
            self.rh_svc.power('all', OpenHRP.RobotHardwareService.SWITCH_OFF)
        return True

    def handServoOn(self):
        self.handServoOff()

        c = False
        while (c != 'Y' and c != 'y'):
            c = raw_input("press 'Y' for HAND servo ON and power ON. >> ")

        # reset JointGroups
        for k in ["rhand", "lhand"]:
            self.seq_svc.removeJointGroup(k)
        for k in ["rhand", "lhand"]:
            self.seq_svc.waitInterpolationOfGroup(k)
        for k in ["rhand", "lhand"]:
            self.seq_svc.addJointGroup(k, self.Groups[k])

        # go Actual (hand only)
        commandAngle = self.sh_svc.getCommand().jointRefs;
        actualAngle = self.rh_svc.getStatus().angle
        jointId = len(self.Groups["rleg"] + self.Groups["lleg"] + self.Groups["torso"] + self.Groups["head"] + self.Groups["rarm"] + self.Groups["larm"])
        commandAngle[jointId:] = actualAngle[jointId:]
        self.seq_svc.setJointAngles(commandAngle, 1.0)
        self.seq_svc.waitInterpolation()

        for j in self.Groups["rhand"] + self.Groups["lhand"]:
            self.rh_svc.power(j ,OpenHRP.RobotHardwareService.SWITCH_ON)
            time.sleep(0.01)
            self.rh_svc.servo(j ,OpenHRP.RobotHardwareService.SWITCH_ON)
        return True

    def handServoOff(self):
        c = False
        while (c != 'Y' and c != 'y'):
            c = raw_input("press 'Y' for HAND servo OFF and power OFF. >> ")

        for j in self.Groups["rhand"] + self.Groups["lhand"]:
            self.rh_svc.servo(j ,OpenHRP.RobotHardwareService.SWITCH_OFF)
            time.sleep(0.01)
            self.rh_svc.power(j ,OpenHRP.RobotHardwareService.SWITCH_OFF)
        return True

    def setResetPose(self):
        return True

    def setCollisionFreeResetPose (self):
        return True

    def servoOnWithResetPose(self):
        if self.servoOn() == True:
            self.setCollisionFreeResetPose()
            print "go to collision-free-reset-pose"
            self.seq_svc.waitInterpolation()
            self.setResetPose()
            print "go to reset-pose"
            self.seq_svc.waitInterpolation()

    def removeForceSensorOffsetRMFO(self, sensor_names=[], tm=8.0):
        return self.rmfo_svc.removeForceSensorOffset(sensor_names, tm)

    def setupLogger(self):
        self.log_svc.add("TimedDoubleSeq","sh_qOut")
        rtm.connectPorts(rtm.findRTC("sh").port("qOut"),rtm.findRTC("log").port("sh_qOut"))
        self.log_svc.add("TimedDoubleSeq","RobotHardware0_q")
        rtm.connectPorts(rtm.findRTC("RobotHardware0").port("q"),rtm.findRTC("log").port("RobotHardware0_q"))
        self.log_svc.add("TimedDoubleSeq","RobotHardware0_dq")
        rtm.connectPorts(rtm.findRTC("RobotHardware0").port("dq"),rtm.findRTC("log").port("RobotHardware0_dq"))
        self.log_svc.add("TimedDoubleSeq","RobotHardware0_tau")
        rtm.connectPorts(rtm.findRTC("RobotHardware0").port("tau"),rtm.findRTC("log").port("RobotHardware0_tau"))
        self.log_svc.add("TimedLongSeqSeq","RobotHardware0_servoState")
        rtm.connectPorts(rtm.findRTC("RobotHardware0").port("servoState"),rtm.findRTC("log").port("RobotHardware0_servoState"))
        for sen in ["rfsensor", "lfsensor", "rhsensor", "lhsensor"]:
            self.log_svc.add("TimedDoubleSeq","RobotHardware0_" + sen)
            rtm.connectPorts(rtm.findRTC("RobotHardware0").port(sen),rtm.findRTC("log").port("RobotHardware0_" + sen))
        self.log_svc.add("TimedOrientation3D","kf_rpy")
        rtm.connectPorts(rtm.findRTC("kf").port("rpy"),rtm.findRTC("log").port("kf_rpy"))
        self.log_svc.add("TimedDoubleSeq","ast_q")
        rtm.connectPorts(rtm.findRTC("ast").port("q"),rtm.findRTC("log").port("ast_q"))
        self.log_svc.add("TimedPoint3D","ast_genCogOut")
        rtm.connectPorts(rtm.findRTC("ast").port("genCogOut"),rtm.findRTC("log").port("ast_genCogOut"))
        self.log_svc.add("TimedPoint3D","ast_genDcmOut")
        rtm.connectPorts(rtm.findRTC("ast").port("genDcmOut"),rtm.findRTC("log").port("ast_genDcmOut"))
        self.log_svc.add("TimedPoint3D","ast_genZmpOut")
        rtm.connectPorts(rtm.findRTC("ast").port("genZmpOut"),rtm.findRTC("log").port("ast_genZmpOut"))
        self.log_svc.add("TimedPoint3D","ast_tgtZmpOut")
        rtm.connectPorts(rtm.findRTC("ast").port("tgtZmpOut"),rtm.findRTC("log").port("ast_tgtZmpOut"))
        self.log_svc.add("TimedPoint3D","ast_actCogOut")
        rtm.connectPorts(rtm.findRTC("ast").port("actCogOut"),rtm.findRTC("log").port("ast_actCogOut"))
        self.log_svc.add("TimedPoint3D","ast_actDcmOut")
        rtm.connectPorts(rtm.findRTC("ast").port("actDcmOut"),rtm.findRTC("log").port("ast_actDcmOut"))
        for ee in ["rleg", "lleg", "rarm", "larm"]:
            self.log_svc.add("TimedDoubleSeq","ast_tgt" + ee + "WrenchOut")
            rtm.connectPorts(rtm.findRTC("ast").port("tgt" + ee + "WrenchOut"),rtm.findRTC("log").port("ast_tgt" + ee + "WrenchOut"))
            self.log_svc.add("TimedDoubleSeq","ast_act" + ee + "WrenchOut")
            rtm.connectPorts(rtm.findRTC("ast").port("act" + ee + "WrenchOut"),rtm.findRTC("log").port("ast_act" + ee + "WrenchOut"))
        self.log_svc.maxLength(500*60)
        self.log_svc.clear()

    def init(self):
        for j in self.Groups["rhand"] + self.Groups["lhand"]:
            self.rh_svc.setServoErrorLimit(j, 0.0)

        for j in self.Groups["rleg"] + self.Groups["lleg"] + self.Groups["torso"] + self.Groups["head"] + self.Groups["rarm"] + self.Groups["larm"]:
            self.rh_svc.setJointControlMode(j,OpenHRP.RobotHardwareService.TORQUE)
            self.rh_svc.setServoTorqueGainPercentage(j,100)

        self.setupLogger()




class JAXON_JVRC_Configurator(AutoStabilizer_Configurator):
    def __init__(self, *args, **kwargs):
        super(JAXON_JVRC_Configurator, self).__init__(*args, **kwargs)
        self.Groups = {'rleg': ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5'],
                       'lleg': ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5'],
                       'torso': ['CHEST_JOINT0', 'CHEST_JOINT1', 'CHEST_JOINT2'],
                       'head': ['HEAD_JOINT0', 'HEAD_JOINT1'],
                       'rarm': ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'RARM_JOINT6', 'RARM_JOINT7'],
                       'larm': ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6', 'LARM_JOINT7'],
                       "rhand": ["RARM_F_JOINT0", "RARM_F_JOINT1"],
                       "lhand": ["LARM_F_JOINT0", "LARM_F_JOINT1"]}

    def setResetPose(self):
        self.seq_svc.setJointAngles([-8.002327e-06,0.000427,-0.244732,0.676564,-0.431836,-0.000427,-8.669072e-06,0.000428,-0.244735,0.676565,-0.431834,-0.000428,0.0,0.0,0.0,0.0,0.0,0.0,0.698132,-0.349066,-0.087266,-1.39626,0.0,0.0,-0.349066,0.0,0.698132,0.349066,0.087266,-1.39626,0.0,0.0,-0.349066]+[0.0]*4, 5.0)
        return True

    def setCollisionFreeResetPose (self):
        self.seq_svc.setJointAngles([0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.523599,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.523599,0.0,0.0,0.0,0.0,0.0]+[0.0]*4,10.0)
        return True

    def setAstParametersJAXON(self):
        # ast setting
        astp=self.ast_svc.getAutoStabilizerParam()[1]
        astp.controllable_joints = self.Groups["rleg"] + self.Groups["lleg"] + self.Groups["torso"] + self.Groups["head"] + self.Groups["rarm"] + self.Groups["larm"]
        # remove hand joints
        astp.dq_weight[len(self.Groups["rleg"] + self.Groups["lleg"]):len(self.Groups["rleg"] + self.Groups["lleg"] + self.Groups["torso"])] = [1e2]*len(self.Groups["torso"]) # reduce chest joint move
        self.ast_svc.setAutoStabilizerParam(astp)
        # kf setting
        kfp=self.kf_svc.getKalmanFilterParam()[1]
        kfp.R_angle=1000
        self.kf_svc.setKalmanFilterParam(kfp)

    def init(self):
        super(JAXON_JVRC_Configurator, self).init()
        self.setAstParametersJAXON()

    def startABSTIMP (self):
        self.ast_svc.startAutoBalancer()
        self.setResetPose()
        self.ast_svc.startImpedanceController("larm")
        self.ast_svc.startImpedanceController("rarm")
        self.ast_svc.startStabilizer()

if __name__ == '__main__':
    rtm.nshost = "localhost"
    rtm.nsport = "15005"
    rtm.initCORBA()

    hcf = JAXON_JVRC_Configurator()

    if len(sys.argv) > 1 and sys.argv[1] == "init":
        hcf.init()
        hcf.startABSTIMP()

#
# Copyright (c) 2015 CNRS
#
# This file is part of Pinocchio
# Pinocchio is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
# Pinocchio is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Lesser Public License for more details. You should have
# received a copy of the GNU Lesser General Public License along with
# Pinocchio If not, see
# <http://www.gnu.org/licenses/>.
#~ universe
#~ root
#~ leg_left_1_joint
#~ leg_left_2_joint
#~ leg_left_3_joint
#~ leg_left_4_joint
#~ leg_left_5_joint
#~ leg_left_6_joint
#~ leg_right_1_joint
#~ leg_right_2_joint
#~ leg_right_3_joint
#~ leg_right_4_joint
#~ leg_right_5_joint
#~ leg_right_6_joint
#~ torso_1_joint
#~ torso_2_joint
#~ arm_left_1_joint
#~ arm_left_2_joint
#~ arm_left_3_joint
#~ arm_left_4_joint
#~ arm_left_5_joint
#~ arm_left_6_joint
#~ arm_left_7_joint
#~ arm_right_1_joint
#~ arm_right_2_joint
#~ arm_right_3_joint
#~ arm_right_4_joint
#~ arm_right_5_joint
#~ arm_right_6_joint
#~ arm_right_7_joint
#~ head_1_joint
#~ head_2_joint

from pinocchio.robot_wrapper import RobotWrapper
import numpy as np
class ReemcWrapper(RobotWrapper):

    def __init__(self,filename):
        RobotWrapper.__init__(self,filename)
        a=-0.001616164386097708
        self.q0= np.matrix( [
            0, 0, 0.817, 0, 0, 0, 1,                      # Free flyer
            0, 0.05,  -0.40+a, 0.75, -0.35-a, -0.05,      # left leg
            0, -0.05, -0.40+a, 0.75, -0.35-a, 0.05,      # right leg
            0,0,                                               # chest
            0, 0.15, 0, 0.3, 0, 0.5, 0,         # left arm
            0, 0.15, 0, 0.3, 0, 0.5, 0,          # right arm
            0.0, 0.0                                      # head
            ] ).T

        self.opCorrespondances = { "lh": "arm_left_7_joint",
                                   "rh": "arm_right_7_joint",
                                   "rf": "leg_right_6_joint",
                                   "lf": "leg_left_6_joint",
                                   }
        for op,name in self.opCorrespondances.items():
            idx = self.__dict__[op] = self.index(name)
            #self.__dict__['_M'+op] = types.MethodType(lambda s,q: s.position(q,idx),self)

    # --- SHORTCUTS ---
    def Mrh(self,q):
        return self.position(q,self.rh)
    def Jrh(self,q):
        return self.jacobian(q,self.rh)
    def wJrh(self,q):
        return se3.jacobian(self.model,self.data,self.rh,q,False)
    def vrh(self,q,v):
        return self.velocity(q,v,self.rh)

    def Jlh(self,q):
        return self.jacobian(q,self.lh)
    def Mlh(self,q):
        return self.position(q,self.lh)

    def Jlf(self,q):
        return self.jacobian(q,self.lf)
    def Mlf(self,q):
        return self.position(q,self.lf)

    def Jrf(self,q):
        return self.jacobian(q,self.rf)
    def Mrf(self,q):
        return self.position(q,self.rf)


__all__ = [ 'ReemcWrapper' ]

#~ from IPython import embed #to be del
#~ robot = ReemcWrapper("/home/tflayols/devel-src/reemc_wrapper/reemc/reemc.urdf")
#~ robot.initDisplay()
#~ robot.loadDisplayModel("world/pinocchio","pinocchio")
#~ robot.display(robot.q0)
#~ robot.viewer.gui.refresh()
#~ 
#~ a=-0.001616164386097708
#~ q= np.matrix( [
            #~ 0, 0, 0.817, 0, 0, 0, 1,                      # Free flyer
            #~ 0, 0.05,  -0.40+a, 0.75, -0.35-a, -0.05,      # left leg
            #~ 0, -0.05, -0.40+a, 0.75, -0.35-a, 0.05,      # right leg
            #~ 0,0,                                               # chest
            #~ 0, 0.15, 0, 0.3, 0, 0.5, 0,         # left arm
            #~ 0, 0.15, 0, 0.3, 0, 0.5, 0,          # right arm
            #~ 0.0, 0.0                                      # head
            #~ ] ).T
#~ print robot.Mlf(q).translation[0]-robot.com(q)[0]

#to find a I did:
#~ inc = 0.1
#~ a=0.0
#~ while (True):
    #~ a+=inc#0.0000000000000000001
    #~ q[9]=robot.q0[9]+a
    #~ q[11]=robot.q0[11]-a
    #~ q[15]=robot.q0[15]+a
    #~ q[17]=robot.q0[17]-a
    #~ if ((robot.Mlf(q).translation[0]-robot.com(q)[0]<0.0 and inc >=0.0) or (robot.Mlf(q).translation[0]-robot.com(q)[0]>0.0 and inc <=0.0)):
        #~ print robot.Mlf(q).translation[0]-robot.com(q)[0]
        #~ inc=inc/-10
        #~ embed()



#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
from mav_manager.srv import Vec4

class korg_mapper(object):

    def __init__(self):

        self.goToZ_val = 0.5
        self.flyVel_val = 0.3
        self.fly_dir = "random"  # "x", "y", or "random" and define any vector below
        # fly_dir_vec does not have to be a norm vector
        self.fly_dir_vec = np.array([1,-1,0])

        rospy.init_node('korg_mapper')
        self.sub = rospy.Subscriber("~nanokontrol", Joy, self.cb)

        self.robot = rospy.get_param('~mav_name', 'hornet2')
        self.srv_motors = rospy.ServiceProxy('/'+self.robot+'/mav_services/motors', SetBool)
        self.srv_takeoff = rospy.ServiceProxy('/'+self.robot+'/mav_services/takeoff', Trigger)
        self.srv_land = rospy.ServiceProxy('/'+self.robot+'/mav_services/land', Trigger)
        self.srv_gohome = rospy.ServiceProxy('/'+self.robot+'/mav_services/goHome', Trigger)
        self.srv_hover = rospy.ServiceProxy('/'+self.robot+'/mav_services/hover', Trigger)
        self.srv_estop = rospy.ServiceProxy('/'+self.robot+'/mav_services/estop', Trigger)
        self.srv_goToRelative = rospy.ServiceProxy('/'+self.robot+'/mav_services/goToRelative', Vec4)
        self.srv_setDesVelInWorldFrame = rospy.ServiceProxy('/'+self.robot+'/mav_services/setDesVelInWorldFrame', Vec4)
        self.srv_goTo = rospy.ServiceProxy('/'+self.robot+'/mav_services/goTo', Vec4)

        # enabling/disabling buttons - needs two clicks to enable
        self.enable_slider_9 = None    # first confirmation
        self.enable_slider_9_double = None   # second confirmation


    def cb(self, data):
        # for each button pressed, call corresponding services
        motors_on = None
        motors_off = None
        takeoff = None
        land = None
        gohome = None
        hover = None
        estop = None
        defined_play = None
        defined_forward = None

        button_vec = data.buttons
        #rospy.loginfo("buttons vector obtained: %s", button_vec)
        axes_vec = data.axes
        #rospy.loginfo("axes vector obtained: %s", axes_vec)


        if button_vec[0] == 1:
            motors_on = True
            self.motors_on()
            return
        if button_vec[1] == 1:
            motors_off = True
            self.motors_off()
            return
        if button_vec[2] == 1:
            takeoff = True
            self.takeoff()
            return
        if button_vec[3] == 1:
            land = True
            self.land()
            return
        if button_vec[21] == 1:
            gohome = True
            self.gohome()
            return
        if button_vec[22] == 1:
            hover = True
            self.hover()
            return
        if button_vec[23] == 1:
            estop = True
            self.estop()
            return
        if button_vec[19] == 1:
            defined_play = True
            self.goToRelZ(self.goToZ_val)
            return
        if button_vec[20] == 1:
            defined_forward = True
            if self.fly_dir == "x":
                self.flyVelX(self.flyVel_val)
                return
            if self.fly_dir == "y":
                self.flyVelY(self.flyVel_val)
                return
            if self.fly_dir == "random":
                self.flyVelRandom(self.flyVel_val, self.fly_dir_vec)
                return

        if button_vec[16] == 1:
            if self.enable_slider_9 is not True:
                self.enable_slider_9 = True
                rospy.loginfo("Slider 9 request (1)!! STOP - before proceeding, turn the slider up and down and leave at the lowest position, then press again.")
            else:
                self.enable_slider_9_double = True
                rospy.loginfo("Slider 9 unlocked (2)!!")
        if button_vec[17] == 1:
            self.enable_slider_9 = False
            self.enable_slider_9_double = False
            rospy.loginfo("Slider 9 disabled")

        if self.enable_slider_9_double is True:
            incre = axes_vec[8] + 1.2
            self.goToZ(incre)
            rospy.loginfo("slide bar 9 - go to height %s", incre)
            return


    def goToZ(self, z=0.0):
        rospy.loginfo("go to height in world [0.0, 0.0, %s, 0.0]", z)
        try:
            self.srv_goTo([0.0, 0.0, z, 0.0])
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: %s", str(exc))

    def goToRelZ(self, z=1.0):
        rospy.loginfo("go to relative [0.0, 0.0, %s, 0.0]", z)
        try:
            self.srv_goToRelative([0.0, 0.0, z, 0.0])
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: %s", str(exc))

    def flyVelX(self, vx):
        rospy.loginfo("go at velocity [%s, 0.0, 0.0, 0.0]", vx)
        try:
            self.srv_setDesVelInWorldFrame([vx, 0.0, 0.0, 0.0])
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: %s", str(exc))

    def flyVelY(self, vy):
        rospy.loginfo("go at velocity [0.0, %s, 0.0, 0.0]", vy)
        try:
            self.srv_setDesVelInWorldFrame([0.0, vy, 0.0, 0.0])
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: %s", str(exc))

    def flyVelRandom(self, vy, vec):
        dir_norm = np.linalg.norm(vec)
        dir_vec = (vec/dir_norm)*vy
        rospy.loginfo("go at velocity %s", dir_vec)
        try:
            self.srv_setDesVelInWorldFrame([dir_vec[0], dir_vec[1], dir_vec[2], 0.0])
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: %s", str(exc))

    def motors_on(self):
        rospy.loginfo("motors on")
        try:
            self.srv_motors(True)
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: %s", str(exc))

    def motors_off(self):
        rospy.loginfo("motors off")
        try:
            self.srv_motors(False)
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: %s", str(exc))

    def takeoff(self):
        rospy.loginfo("takeoff")
        try:
            self.srv_takeoff()
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: %s", str(exc))

    def land(self):
        rospy.loginfo("land")
        try:
            self.srv_land()
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: %s", str(exc))

    def gohome(self):
        rospy.loginfo("go home")
        try:
            self.srv_gohome()
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: %s", str(exc))

    def hover(self):
        rospy.loginfo("hover")
        try:
            self.srv_hover()
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: %s", str(exc))

    def estop(self):
        rospy.loginfo("estop")
        try:
            self.srv_estop()
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: %s", str(exc))


if __name__ == '__main__':
    node = korg_mapper()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting Down."


#!/usr/bin/env python

from std_msgs.msg import Bool

import time
import pprint
import openvr
import rospy


class KeyPress(object):
    '''
    Get the HTC Vive controllers keypresses and print them to screen.

    You need to do:
    export LD_LIBRARY_PATH=$HOME/.steam/steam/steamapps/common/SteamVR/bin/
        linux64:$HOME/.steam/steam/steamapps/common/tools/bin/linux64:$LD_LIBRARY_PATH

    before executing it.

    Expected output (if prints are on):
    Left controller:
    {   'grip_button': False,
        'menu_button': False,
        'trackpad_pressed': False,
        'trackpad_touched': True, ??????
        'trackpad_x': 0.032959990203380585,
        'trackpad_y': -0.8471328020095825,
        'trigger': 0.0,
        'ulButtonPressed': 0L,
        'ulButtonTouched': 4294967296L,
        'unPacketNum': 2037L}
    Right controller:
    {   'grip_button': False,
        'menu_button': False,
        'trackpad_pressed': False,
        'trackpad_touched': False,
        'trackpad_x': 0.0,
        'trackpad_y': 0.0,
        'trigger': 0.0,
        'ulButtonPressed': 0L,
        'ulButtonTouched': 0L,
        'unPacketNum': 1146L}


    Original source code author:
    Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
    '''

    def __init__(self):
        '''
        Initialization of KeyPress object.
        '''
        rospy.init_node('ctrl_keypress')

        self.reading_rate_hz = 10
        self.show_only_new_events = True
        self.last_unPacketNum_left = 0
        self.last_unPacketNum_right = 0

        self.rtrigger_pressed = rospy.Publisher(
            'ctrl_keypress/rtrigger', Bool, queue_size=1)
        self.ltrigger_pressed = rospy.Publisher(
            'ctrl_keypress/ltrigger', Bool, queue_size=1)
        self.rtake_off = rospy.Publisher(
            'ctrl_keypress/rtake_off', Empty, queue_size=1)
        self.r_land = rospy.Publisher(
            'ctrl_keypress/rland', Empty, queue_size=1)
        self.rtrackpad = rospy.Publisher(
            'ctrl_keypress/rtrackpad', Empty, queue_size=1)

        print("===========================")
        print("Initializing OpenVR...")

        try:
            # was: openvr.init(openvr.VRApplication_Scene),
            # but that gives 306 error. Don't need HMD.
            openvr.init(openvr.VRApplication_Other)

        except openvr.OpenVRError as e:
            print("Error when initializing OpenVR (try {} / {})".format(
                  retries + 1, max_init_retries))
            print(e)
            time.sleep(2.0)

        print("Success!")
        print("===========================")
        self.vrsystem = openvr.VRSystem()

        self.left_id, self.right_id = None, None
        print("===========================")
        print("Waiting for controllers...")
        try:
            while (self.left_id is None) or (self.right_id is None):
                self.left_id, self.right_id = (
                    self.get_controller_ids(self.vrsystem))
                if self.left_id and self.right_id:
                    break
                print("Waiting for controllers...")
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("Control+C pressed, shutting down...")
            openvr.shutdown()

        print("Left controller ID: " + str(self.left_id))
        print("Right controller ID: " + str(self.right_id))
        print("===========================")

        self.pp = pprint.PrettyPrinter(indent=4)

    def print_events(self):
        print("===========================")
        print("Printing controller events!")
        try:
            while not rospy.is_shutdown():
                time.sleep(1.0 / self.reading_rate_hz)

                (result, pControllerState) = (
                    self.vrsystem.getControllerState(self.left_id))
                d = self.from_controller_state_to_dict(pControllerState)

                if (self.show_only_new_events and self.last_unPacketNum_left
                        != d['unPacketNum']):
                    self.last_unPacketNum_left = d['unPacketNum']
                    # print("Left controller:")
                    # self.pp.pprint(d)
                    if d['trigger'] == 1.0:
                        print 'left trigger'
                        self.ltrigger_pressed.publish(True)

                (result, pControllerState) = (
                    self.vrsystem.getControllerState(self.right_id))
                d = self.from_controller_state_to_dict(pControllerState)

                if (self.show_only_new_events and self.last_unPacketNum_right
                        != d['unPacketNum']):
                    self.last_unPacketNum_right = d['unPacketNum']
                    # print("Right controller:")
                    # self.pp.pprint(d)
                    if d['trigger'] == 1.0:
                        print 'right trigger'
                        self.rtrigger_pressed.publish(True)
                    if d['ulButtonPressed'] == 1.0:
                        print 'right land button'
                        self.land.publish(Empty())
                    if d['menu_button'] == 1.0:
                        print 'right take-off button'
                        self.rtake_off.publish(Empty())
                    if d['trackpad_pressed] == 1.0:
                        print 'right trackpad'
                        self.rtrackpad.publish(Empty())


        except KeyboardInterrupt:
            print("Control+C pressed, shutting down...")
            openvr.shutdown()

    def get_controller_ids(self, vrsys=None):
        if vrsys is None:
            vrsys = openvr.self.vrsystem()
        else:
            vrsys = vrsys
        left = None
        right = None
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            device_class = vrsys.getTrackedDeviceClass(i)
            if device_class == openvr.TrackedDeviceClass_Controller:
                role = vrsys.getControllerRoleForTrackedDeviceIndex(i)
                if role == openvr.TrackedControllerRole_RightHand:
                    right = i
                if role == openvr.TrackedControllerRole_LeftHand:
                    left = i
        return left, right

    def from_controller_state_to_dict(self, pControllerState):
        # docs: https://github.com/ValveSoftware/openvr/wiki/Iself.vrsystem::GetControllerState
        d = {}
        d['unPacketNum'] = pControllerState.unPacketNum
        # on trigger .y is always 0.0 says the docs
        d['trigger'] = pControllerState.rAxis[1].x
        # 0.0 on trigger is fully released
        # -1.0 to 1.0 on joystick and trackpads
        d['trackpad_x'] = pControllerState.rAxis[0].x
        d['trackpad_y'] = pControllerState.rAxis[0].y
        # These are published and always 0.0
        # for i in range(2, 5):
        #     d['unknowns_' + str(i) + '_x'] = pControllerState.rAxis[i].x
        #     d['unknowns_' + str(i) + '_y'] = pControllerState.rAxis[i].y
        d['ulButtonPressed'] = pControllerState.ulButtonPressed
        d['ulButtonTouched'] = pControllerState.ulButtonTouched
        # To make easier to understand what is going on
        # Second bit marks menu button
        d['menu_button'] = bool(pControllerState.ulButtonPressed >> 1 & 1)
        # 32 bit marks trackpad
        d['trackpad_pressed'] = bool(pControllerState.ulButtonPressed >> 32 & 1)
        d['trackpad_touched'] = bool(pControllerState.ulButtonTouched >> 32 & 1)
        # third bit marks grip button
        d['grip_button'] = bool(pControllerState.ulButtonPressed >> 2 & 1)
        # System button can't be read, if you press it
        # the controllers stop reporting
        return d


if __name__ == '__main__':
    # button_pressed = rospy.Publisher('htc_ctrl_button', Bool, queue_size=1)
    keypress = KeyPress()
    keypress.print_events()

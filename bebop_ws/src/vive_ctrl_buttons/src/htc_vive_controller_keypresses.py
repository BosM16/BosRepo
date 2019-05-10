#!/usr/bin/env python

from std_msgs.msg import Bool, Empty

import time
import pprint
import openvr
import rospy

from fabulous.color import (highlight_red, highlight_green, highlight_blue,
                            white, green)


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
        self.last_unPacketNum_left = 0
        self.last_unPacketNum_right = 0
        self.vive_loc_ready = False

        self.rtrigger_pressed = rospy.Publisher(
            'ctrl_keypress/rtrigger', Bool, queue_size=1)
        self.ltrigger_pressed = rospy.Publisher(
            'ctrl_keypress/ltrigger', Bool, queue_size=1)
        self.rmenu_button = rospy.Publisher(
            'ctrl_keypress/rmenu_button', Bool, queue_size=1)
        self.rtrackpad = rospy.Publisher(
            'ctrl_keypress/rtrackpad', Bool, queue_size=1)

        rospy.Subscriber(
                    'vive_localization/ready', Empty, self.vive_localization_ready)

        openvr.init(openvr.VRApplication_Other)
        self.vrsystem = openvr.VRSystem()
        # Let system choose id's at first to make sure both controllers are
        # found.
        self.left_id, self.right_id = None, None
        print white(' Waiting for Vive controllers ...')
        try:
            while (not rospy.is_shutdown()) and (
                    self.left_id is None or self.right_id is None):

                self.left_id, self.right_id = (
                    self.get_controller_ids(self.vrsystem))
                if self.left_id and self.right_id:
                    break
                print white(' Waiting for Vive controllers ...')
                time.sleep(1.0)
        except KeyboardInterrupt:
            print white('----Control+C pressed, shutting down... ----')
            openvr.shutdown()

        # print '==========================='
        # print " Trigger id's: "
        # print(" * Right controller ID: " + str(self.right_id))
        # print(" * Left controller ID: " + str(self.left_id))
        # print("===========================")

        self.pp = pprint.PrettyPrinter(indent=4)

    def identify_leftright(self):
        '''
        Let user identify which is right and which is left controller by
        pulling triggers. Controller hand is displayed in terminal.
        '''
        while not (rospy.is_shutdown() or self.vive_loc_ready):
            rospy.sleep(0.1)

        if not rospy.is_shutdown():
            print highlight_green(' Pull each trigger ')

        identify_right = True
        identify_left = True
        while (identify_right or identify_left) and not rospy.is_shutdown():

            (result, pControllerState) = (
                self.vrsystem.getControllerState(self.left_id))
            d = self.from_controller_state_to_dict(pControllerState)
            # print '\n left controller:', self.last_unPacketNum_left, d['unPacketNum']
            # print highlight_green('trigger value: ', d['trigger'])
            if (self.last_unPacketNum_left != d['unPacketNum']):
                self.last_unPacketNum_left = d['unPacketNum']
                # print("Left controller:")
                # self.pp.pprint(d)
                if d['trigger'] == 1.0:
                    print highlight_blue(' Left  trigger ')
                    identify_left = False

            (result, pControllerState) = (
                self.vrsystem.getControllerState(self.right_id))
            d = self.from_controller_state_to_dict(pControllerState)
            # print 'right controller: ', self.last_unPacketNum_right, d['unPacketNum']
            # print highlight_green('trigger value: ', d['trigger'])
            if (self.last_unPacketNum_right != d['unPacketNum']):
                self.last_unPacketNum_right = d['unPacketNum']
                if d['trigger'] == 1.0:
                    print highlight_blue(' Right trigger ')
                    identify_right = False

            rospy.sleep(1.0 / self.reading_rate_hz)

    def publish_events(self):
        if not rospy.is_shutdown():
            print white(" Monitoring controller events! ")
        try:
            while not rospy.is_shutdown():
                rospy.sleep(1.0 / self.reading_rate_hz)

                (result, pControllerState) = (
                    self.vrsystem.getControllerState(self.left_id))
                d = self.from_controller_state_to_dict(pControllerState)

                # print '\n left controller:', self.last_unPacketNum_left, d['unPacketNum']
                # print highlight_green('trigger value: ', d['trigger'])
                # print 'trackpad', d['trackpad_pressed']
                # print 'trigger', d['trigger']
                if (self.last_unPacketNum_left != d['unPacketNum']):
                    self.last_unPacketNum_left = d['unPacketNum']
                    # print("Left controller:")
                    # self.pp.pprint(d)
                    if d['trigger'] == 1.0:
                        self.ltrigger_pressed.publish(True)
                    if d['trigger'] == 0.0:
                        self.ltrigger_pressed.publish(False)

                (result, pControllerState) = (
                    self.vrsystem.getControllerState(self.right_id))
                d = self.from_controller_state_to_dict(pControllerState)

                # print 'right controller: ', self.last_unPacketNum_right, d['unPacketNum']
                # print highlight_green('trigger value: ', d['trigger'])
                # print 'trackpad', d['trackpad_pressed']
                # print 'trigger', d['trigger']
                if (self.last_unPacketNum_right != d['unPacketNum']):
                    self.last_unPacketNum_right = d['unPacketNum']
                    # print("Right controller:")
                    # self.pp.pprint(d)
                    if d['trigger'] == 1.0:
                        self.rtrigger_pressed.publish(True)
                    if d['trigger'] == 0.0:
                        self.rtrigger_pressed.publish(False)
                    if d['menu_button'] == 1.0:
                        self.rmenu_button.publish(True)
                    if d['menu_button'] == 0.0:
                        self.rmenu_button.publish(False)
                    if d['trackpad_pressed'] == 1.0:
                        self.rtrackpad.publish(True)
                    if d['trackpad_pressed'] == 0.0:
                        self.rtrackpad.publish(False)

        except KeyboardInterrupt:
            print white("Control+C pressed, shutting down...")
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
                if not right:
                    right = i
                elif not left:
                    left = i
        return left, right

    def from_controller_state_to_dict(self, pControllerState):
        # docs: https://github.com/ValveSoftware/openvr/wiki/...
        #       Iself.vrsystem::GetControllerState
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
        d['trackpad_pressed'] = bool(
                                    pControllerState.ulButtonPressed >> 32 & 1)
        d['trackpad_touched'] = bool(
                                    pControllerState.ulButtonTouched >> 32 & 1)
        # third bit marks grip button
        d['grip_button'] = bool(pControllerState.ulButtonPressed >> 2 & 1)
        # System button can't be read, if you press it
        # the controllers stop reporting
        return d

    def vive_localization_ready(self, empty):
        '''Sets variable vive_loc_ready to true when vive localizaiton is
        running.
        '''
        self.vive_loc_ready = True


if __name__ == '__main__':
    # button_pressed = rospy.Publisher('htc_ctrl_button', Bool, queue_size=1)
    keypress = KeyPress()
    keypress.identify_leftright()
    keypress.publish_events()

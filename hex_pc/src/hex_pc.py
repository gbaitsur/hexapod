#!/usr/bin/env python

# NODE DESCRIPTION


# publishes [pppppp]
# subscribes to [sssssss]




# -*- coding:utf-8 -*-
import sys, os



# import PyQt4 QtCore and QtGui modules
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4 import uic

import threading

import rospy

from math import degrees, atan2, sqrt, radians

from std_msgs.msg import String
from geometry_msgs.msg import Twist





(Ui_MainWindow, QMainWindow) = uic.loadUiType(os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + '/ui/hex_control.ui')


class MainWindow(QMainWindow):
    """MainWindow inherits QMainWindow"""

    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ui.centralWidget.keyReleaseEvent = self.key_released
        self.ui.centralWidget.keyPressEvent = self.key_pressed
        self.ui.centralWidget.grabKeyboard()

        self.ui.btn_storage.clicked.connect(self.storage_position)
        self.ui.btn_init.clicked.connect(self.initial_position)

        self.ui.btn_forward.pressed.connect(self.teleop_button_pressed)
        self.ui.btn_back.pressed.connect(self.teleop_button_pressed)
        self.ui.btn_left.pressed.connect(self.teleop_button_pressed)
        self.ui.btn_right.pressed.connect(self.teleop_button_pressed)
        self.ui.btn_ccw.pressed.connect(self.teleop_button_pressed)
        self.ui.btn_cw.pressed.connect(self.teleop_button_pressed)

        self.ui.btn_forward.released.connect(self.teleop_button_released)
        self.ui.btn_back.released.connect(self.teleop_button_released)
        self.ui.btn_left.released.connect(self.teleop_button_released)
        self.ui.btn_right.released.connect(self.teleop_button_released)
        self.ui.btn_ccw.released.connect(self.teleop_button_released)
        self.ui.btn_cw.released.connect(self.teleop_button_released)


        self.text_command_publisher = rospy.Publisher('pc_text_commands', String, queue_size=10)
        self.twist_publisher = rospy.Publisher('teleop', Twist, queue_size=10)


        self.pressed_buttons = list()
        self.pressed_keys = list()

        self.clearance = -10
        self.roll = 0
        self.pitch = 0
        self.translation_effort = 0
        self.translation_direction = 0
        self.rotation_effort = 0

        self.quitting = False

        self.with_joystick = False
        try:
            self.joy = ps3()
            self.with_joystick = True
        except:
            pass




        self.check_keyboard()
        self.send_teleop()

        self.setWindowTitle("hex_control")




        self.show()


    def __del__(self):
        self.ui = None

    def check_keyboard(self):
        if self.quitting or rospy.is_shutdown():
            return

        Key_Escape = 0x01000000
        Key_Tab = 0x01000001
        Key_Backtab = 0x01000002
        Key_Backspace = 0x01000003
        Key_Return = 0x01000004
        Key_Enter = 0x01000005
        Key_Insert = 0x01000006
        Key_Delete = 0x01000007
        Key_Pause = 0x01000008
        Key_Print = 0x01000009
        Key_SysReq = 0x0100000a
        Key_Clear = 0x0100000b
        Key_Home = 0x01000010
        Key_End = 0x01000011
        Key_Left = 0x01000012
        Key_Up = 0x01000013
        Key_Right = 0x01000014
        Key_Down = 0x01000015
        Key_PageUp = 0x01000016
        Key_PageDown = 0x01000017
        Key_Shift = 0x01000020
        Key_Control = 0x01000021
        Key_Meta = 0x01000022
        Key_Alt = 0x01000023
        Key_AltGr = 0x01001103
        Key_CapsLock = 0x01000024
        Key_NumLock = 0x01000025
        Key_ScrollLock = 0x01000026
        Key_F1 = 0x01000030
        Key_F2 = 0x01000031
        Key_F3 = 0x01000032
        Key_F4 = 0x01000033
        Key_F5 = 0x01000034
        Key_F6 = 0x01000035
        Key_F7 = 0x01000036
        Key_F8 = 0x01000037
        Key_F9 = 0x01000038
        Key_F10 = 0x01000039
        Key_F11 = 0x0100003a
        Key_F12 = 0x0100003b
        Key_F13 = 0x0100003c
        Key_F14 = 0x0100003d
        Key_F15 = 0x0100003e
        Key_F16 = 0x0100003f
        Key_F17 = 0x01000040
        Key_F18 = 0x01000041
        Key_F19 = 0x01000042
        Key_F20 = 0x01000043
        Key_F21 = 0x01000044
        Key_F22 = 0x01000045
        Key_F23 = 0x01000046
        Key_F24 = 0x01000047
        Key_F25 = 0x01000048
        Key_F26 = 0x01000049
        Key_F27 = 0x0100004a
        Key_F28 = 0x0100004b
        Key_F29 = 0x0100004c
        Key_F30 = 0x0100004d
        Key_F31 = 0x0100004e
        Key_F32 = 0x0100004f
        Key_F33 = 0x01000050
        Key_F34 = 0x01000051
        Key_F35 = 0x01000052
        Key_Super_L = 0x01000053
        Key_Super_R = 0x01000054
        Key_Menu = 0x01000055
        Key_Hyper_L = 0x01000056
        Key_Hyper_R = 0x01000057
        Key_Help = 0x01000058
        Key_Direction_L = 0x01000059
        Key_Direction_R = 0x01000060
        Key_Space = 0x20
        Key_Any = Key_Space
        Key_Exclam = 0x21
        Key_QuoteDbl = 0x22
        Key_NumberSign = 0x23
        Key_Dollar = 0x24
        Key_Percent = 0x25
        Key_Ampersand = 0x26
        Key_Apostrophe = 0x27
        Key_ParenLeft = 0x28
        Key_ParenRight = 0x29
        Key_Asterisk = 0x2a
        Key_Plus = 0x2b
        Key_Comma = 0x2c
        Key_Minus = 0x2d
        Key_Period = 0x2e
        Key_Slash = 0x2f
        Key_0 = 0x30
        Key_1 = 0x31
        Key_2 = 0x32
        Key_3 = 0x33
        Key_4 = 0x34
        Key_5 = 0x35
        Key_6 = 0x36
        Key_7 = 0x37
        Key_8 = 0x38
        Key_9 = 0x39
        Key_Colon = 0x3a
        Key_Semicolon = 0x3b
        Key_Less = 0x3c
        Key_Equal = 0x3d
        Key_Greater = 0x3e
        Key_Question = 0x3f
        Key_At = 0x40
        Key_A = 0x41
        Key_B = 0x42
        Key_C = 0x43
        Key_D = 0x44
        Key_E = 0x45
        Key_F = 0x46
        Key_G = 0x47
        Key_H = 0x48
        Key_I = 0x49
        Key_J = 0x4a
        Key_K = 0x4b
        Key_L = 0x4c
        Key_M = 0x4d
        Key_N = 0x4e
        Key_O = 0x4f
        Key_P = 0x50
        Key_Q = 0x51
        Key_R = 0x52
        Key_S = 0x53
        Key_T = 0x54
        Key_U = 0x55
        Key_V = 0x56
        Key_W = 0x57
        Key_X = 0x58
        Key_Y = 0x59
        Key_Z = 0x5a

        self.translation_effort = 0
        self.rotation_effort = 0

        if self.with_joystick == True:
            self.joy.update()

            if self.joy.l2:
                self.roll = self.joy.a_joystick_right_x * 4
                self.pitch = self.joy.a_joystick_right_y * 4
            else:
                self.rotation_effort = -self.joy.a_joystick_right_x

            self.translation_direction = degrees(atan2(self.joy.a_joystick_left_x, -self.joy.a_joystick_left_y))

            x = self.joy.a_joystick_left_x * sqrt(1 - 0.5 * self.joy.a_joystick_left_y ** 2)
            y = self.joy.a_joystick_left_y * sqrt(1 - 0.5 * self.joy.a_joystick_left_x ** 2)
            self.translation_effort = sqrt(x ** 2 + y ** 2)

            if self.joy.up:
                self.clearance += 1
                self.clearance = min(self.clearance, 90)

            if self.joy.down:
                self.clearance -= 1
                self.clearance = max(self.clearance, -10)

            if self.joy.start:
                self.clearance = -10
                self.mode = "init"

        if int(Key_Q) in self.pressed_keys:
            self.clearance += 1
            self.clearance = min(self.clearance, 90)

        if int(Key_Z) in self.pressed_keys:
            self.clearance -= 1
            self.clearance = max(self.clearance, -10)

        if int(Key_A) in self.pressed_keys and int(Key_Shift) in self.pressed_keys:
            self.roll -= 0.1

        if int(Key_D) in self.pressed_keys and int(Key_Shift) in self.pressed_keys:
            self.roll += 0.1

        if int(Key_W) in self.pressed_keys and int(Key_Shift) in self.pressed_keys:
            self.pitch -= 0.1

        if int(Key_S) in self.pressed_keys and int(Key_Shift) in self.pressed_keys:
            self.pitch += 0.1

        if int(Key_W) in self.pressed_keys and int(Key_Shift) not in self.pressed_keys:
            self.translation_direction = 0
            self.translation_effort = 1

        if int(Key_S) in self.pressed_keys and int(Key_Shift) not in self.pressed_keys:
            self.translation_direction = 180
            self.translation_effort = 1

        if int(Key_A) in self.pressed_keys and int(Key_Shift) not in self.pressed_keys:
            self.rotation_effort = 1

        if int(Key_D) in self.pressed_keys and int(Key_Shift) not in self.pressed_keys:
            self.rotation_effort = -1


        self.t = threading.Timer(0.01, self.check_keyboard)
        self.t.start()

    def key_pressed(self, e):
        k = e.key()
        if not k in self.pressed_keys:
            self.pressed_keys.append(k)

    def key_released(self, e):
        k = e.key()
        try:
            self.pressed_keys.remove(k)
        except:
            pass

    def closeEvent(self, QCloseEvent):
        self.ui.centralWidget.releaseKeyboard()
        self.quitting = True



    def storage_position(self):
        self.text_command_publisher.publish("storage_position")

    def initial_position(self):
        self.text_command_publisher.publish("initial_position")



    def teleop_button_pressed(self):
        self.pressed_buttons.append(self.sender())


    def teleop_button_released(self):
        self.pressed_buttons.remove(self.sender())

    def send_teleop(self):
        if self.quitting or rospy.is_shutdown():
            return


        teleop = Twist()

        if self.ui.btn_forward in self.pressed_buttons:
            teleop.linear.x = 0.01
        elif self.ui.btn_back in self.pressed_buttons:
            teleop.linear.x = -0.01

        if self.ui.btn_right in self.pressed_buttons:
            teleop.linear.y = -0.01
        elif self.ui.btn_left in self.pressed_buttons:
            teleop.linear.y = 0.01


        if self.ui.btn_cw in self.pressed_buttons:
            teleop.angular.z = radians(-0.01)
        elif self.ui.btn_ccw in self.pressed_buttons:
            teleop.angular.z = radians(0.01)

        self.twist_publisher.publish(teleop)


        self.t = threading.Timer(0.1, self.send_teleop)
        self.t.start()




def run():


    # create application

    rospy.init_node('hex_pc', anonymous=True)


    app = QApplication(sys.argv)
    app.setApplicationName('hex_control')

    # create widget
    w = MainWindow()

    # execute application
    sys.exit(app.exec_())




if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
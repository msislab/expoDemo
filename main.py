# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import numpy as np
import rbpodo as rb
import time
from sys import exit

ROBOT_IP_G = "192.168.0.20"
ROBOT_IP_S = "192.168.0.21"

fatal_error = False;
error_msg = "No Error"


robotG_speed = 0.3 # 0 to 1
robotS_speed = 0.3 # 0 to 1

pcb_safe_height = 0
gripper_safe_height = 0

robotG_home = np.array(700, 45, 430 , 90, 0, 0)
robotS_home = np.array(250, -30, 380 , 90, 0, 90)

pcb1home = np.array(575.406, 24.9550, 263.38 +gripper_safe_height, 90, 0, 0.5)
pcb2home = np.array(575.406, 24.9550, 263.38 +gripper_safe_height, 90, 0, 0.5)
PickPosition = np.array(575.406, 24.9550, 263.38 +pcb_safe_height, 90, 0, 90)
P1 = np.array(575.406, 24.9550, 263.38 +pcb_safe_height, 90, 0, 90)
P2 = np.array(575.68, 81.68, 263.38 + pcb_safe_height, 90, 0, 90)
P3 = np.array(668.08, 80.93, 263.38 + pcb_safe_height, 90, 0, 90)
P4 = np.array(668.08, 24.14, 263.55 + pcb_safe_height, 90, 0, 90)

def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.

def gripper_init(robot,rc):
    force = 5
    speed = 100
    acc = 100
    try:
        script = f"gripper_macro 6,2,0,0,0,0,0,0,0,0"
        robot.eval(rc, script, True)
        time.sleep(10)
        script = f"gripper_macro 6,2,1,0,force,speed,acc,0,0,0"
        robot.eval(rc, script, True)

        script = f"gripper_macro 6,2,2,0,{str(100)},0,0,0,0,0"
        robot.eval(rc, script, True)
        print(time.sleep(10))
        if ( gripper_pos(robot,rc)-100) > 1:
            error_msg = "Gripper Error, Commanded and Feedback position not Equal"
            print(error_msg)
            fatal_error = True
            exit(error_msg)



    except Exception as e:
        print(e)
    finally:
        pass


def gripper_move(robot,rc, pos):
    try:
        script = f"gripper_macro 6,2,2,0,{str(pos)},0,0,0,0,0"
        robot.eval(rc, script, True)

    except Exception as e:
        print(e)
    finally:
        pass

def gripper_pos(robot,rc):

    return round(robot.get_system_variable(rc, rb.SystemVariable.JRT_JEGB)[1])


def robotG_move_home(robotG, rc):


def _main():
    try:

        #initialize cobots
        robotG = rb.Cobot(ROBOT_IP_G)
        robotS = rb.Cobot(ROBOT_IP_S)

        rc = rb.ResponseCollector()

        gripper_init(robotG, rc)
        time.sleep(5)


        #ready the gripper
        pos = gripper_move(robotG, 50)

        #move robot to home.
        robotG.move_j(robotG_home)
        if robot.wait_for_move_started(rc, 0.1).type() == rb.ReturnType.Success:
            robot.wait_for_move_finished(rc)
        rc.error().throw_if_not_empty()


        robotG.set_operation_mode(rc, rb.OperationMode.Real)
        robotG.set_speed_bar(rc, 0.3) #30% speed


        robot.move_l(rc,np.array([346,0,432,90,0,90]),200,400 )
        if robot.wait_for_move_started(rc, 0.1).type() == rb.ReturnType.Success:
            robot.wait_for_move_finished(rc)
        rc.error().throw_if_not_empty()

        robot.move_l(rc, np.array([346, 105.5 , 291, 90, 0, 90]), 200, 400)
        if robot.wait_for_move_started(rc, 0.1).type() == rb.ReturnType.Success:
            robot.wait_for_move_finished(rc)
        rc.error().throw_if_not_empty()

        res, cb_info = robot.get_control_box_info(rc)
        if res.is_success():
            print(f"Control Box Info: {cb_info}")

        # Move Shank to tighten to 40mm code 00=0
        robot.set_dout_bit_combination(rc, 0, 1, 0, rb.Endian.LittleEndian)
        # Tighten Bit code 10 = 2
        robot.set_dout_bit_combination(rc, 0, 1,2, rb.Endian.LittleEndian)
        # Move Shank to loosen to 48mm code 01 =1
        robot.set_dout_bit_combination(rc, 0, 1, 1, rb.Endian.LittleEndian)
        # loosen code  11 =3
        robot.set_dout_bit_combination(rc, 0, 1, 3, rb.Endian.LittleEndian)


        robot.flush(rc)
    except Exception as e:
        print(e)
    finally:
        pass
    print_hi('PyCharm')

if __name__ == '__main__':
    _main()

#    robot.set_dout_bit_combination(rc, 0, 15, 5, rb.Endian.LittleEndian)
#    time.sleep(1)
#   01=
#

#    robot.set_dout_bit_combination(rc, 0, 15, 10, rb.Endian.LittleEndian)
#   time.sleep(1)

#    robot.set_dout_bit_combination(rc, 0, 15, 0, rb.Endian.LittleEndian)
#    time.sleep(1)
#res = robot.set_tcp_info(rc, np.array([0, 0, 0, 0, 0, 0]))
#        if not res.is_success():
#            print("failed to set tcp")
#            exit(-1)
#        rc = rc.error().throw_if_not_empty()
#JRT_JEGB_42140
# datatype enum class GripperModel
# f"gripper_macro 6,2,0,0,0,0,0,0,0,0"

# See PyCharm help at https://www.jetbrains.com/help/pycharm/

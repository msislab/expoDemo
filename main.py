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

jaws_FC = 0
jaws_HC = 38
jaws_HO = 50
jaws_FO = 100

local_speed = 100
local_acc = 100

robot_speed = 0.3 # 0 to 1

pcb_safe_height = 0
gripper_safe_height = 0

robotG_home_j = np.array([175.39, -18.47, -90.83, 19.30, -89.94, 275.53])
robotS_home = np.array([250, -30, 380, 90, 0, 90])

pcb1home_j = np.array([170.50, -42.57, -69.27, 21.83, -89.96, 279.93 ])
pcb1home = np.array([984.85, -52.51, 290.21, 90, -0.5, -1.8])

pcb2home_j = np.array([176.31, -42.34, -69.03, 21.37, -89.95, 274.62 ])
pcb2home = np.array([986.75, 47.30, 292.13, 90, 0, -1.1])

PickPosition = np.array([575.406, 24.9550, 263.38 , 90, 0, 90])

#screwing locations from pcb1
P1 = np.array([575.406, 24.9550, 263.38 +pcb_safe_height, 90, 0, 90])
P2 = np.array([575.68, 81.68, 263.38 + pcb_safe_height, 90, 0, 90])
P3 = np.array([668.08, 80.93, 263.38 + pcb_safe_height, 90, 0, 90])
P4 = np.array([668.08, 24.14, 263.55 + pcb_safe_height, 90, 0, 90])
#screwing locations from pcb2

def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.

def gripper_pos(robot,rc):
    return round(robot.get_system_variable(rc, rb.SystemVariable.JRT_JEGB)[1])

def gripper_init(robot,rc):
    force = 5
    speed = 100
    acc = 100
    try:
        script = f"gripper_macro 6,2,0,0,0,0,0,0,0,0"
        robot.eval(rc, script, True)
        time.sleep(7)
        script = f"gripper_macro 6,2,1,0,5,100,100,0,0,0"
        robot.eval(rc, script, True)

        script = f"gripper_macro 6,2,2,0,{str(50)},0,0,0,0,0"
        robot.eval(rc, script, True)
        while (gripper_pos(robot, rc) != 50):
            time.sleep(0.5)

        #if ( gripper_pos(robot,rc)-100) > 1:
        #    error_msg = "Gripper Error, Commanded and Feedback position not Equal"
        #    print(error_msg)
        #    fatal_error = True
        #    exit(error_msg)



    except Exception as e:
        print(e)
    finally:
        pass

#add while wait and timeout
def gripper_move(robot,rc, pos):
    try:
        script = f"gripper_macro 6,2,2,0,{str(pos)},0,0,0,0,0"
        robot.eval(rc, script, True)

    except Exception as e:
        print(e)
    finally:
        pass



def robotG_move_home(robotG, rc):
    # move robot to home.

    robotG.move_j(rc,robotG_home_j,local_speed,local_acc)
    rc.error().throw_if_not_empty()
    if robotG.wait_for_move_started(rc, 0.1).type() == rb.ReturnType.Success:
        robotG.wait_for_move_finished(rc)
    rc.error().throw_if_not_empty()

def robotG_move_pcb1(robot, rc, pickOrDrop):

    if pickOrDrop ==0:
        gripper_move(robot, rc, jaws_HO)
        while (gripper_pos(robot, rc) != jaws_HO):
            time.sleep(0.5)
        print("Jaws Position: "+str(gripper_pos(robot, rc)))

    robot.move_j(rc,pcb1home_j,local_speed,local_acc)
    time.sleep(0.25)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        pass
    print("Arrived at pcb1home+20mm")

    #approach the pcb
    robot.move_l(rc,pcb1home,local_speed,local_acc)
    time.sleep(0.25)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    print("Arrived at pcb1home")

    if pickOrDrop == 0:
        #Gripper jaws to CLOSE %.
        gripper_move(robot,rc,jaws_HC)
        time.sleep(0.25)
        while(gripper_pos(robot,rc)!=jaws_HC):
            time.sleep(0.5)
        print("Jaws Position: "+str(gripper_pos(robot, rc)))
    elif pickOrDrop == 1:
        gripper_move(robot, rc, jaws_HO)
        time.sleep(0.25)
        while (gripper_pos(robot, rc) != jaws_HO):
            time.sleep(0.5)
    print("Jaws position: "+str(gripper_pos(robot, rc)))

    #add safe moving height
    pcb1home[2] = pcb1home[2] +20
    robot.move_l(rc,pcb1home,local_speed,local_acc)
    time.sleep(0.25)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    print("Arrived at pcb1home+20mm")
    #revert by to original
    pcb1home[2] = pcb1home[2] - 20


def robotG_move_pcb2(robot, rc, pickOrDrop):

    if pickOrDrop ==0:
        # Gripper jaws to CLOSE %.
        gripper_move(robot, rc, jaws_HO)
        while (gripper_pos(robot, rc) != jaws_HO):
            time.sleep(0.5)
        print("Jaws position: "+str(gripper_pos(robot, rc)))

    #pcb2home_j is safe pcb approach
    robot.move_j(rc,pcb2home_j,local_speed,local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    print("Arrived at pcb2home +20mm")

    # approach the pcb
    robot.move_l(rc,pcb2home,local_speed,local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    print("Arrived at pcb2home")

    if pickOrDrop == 0:
        # Gripper jaws to CLOSE %.
        gripper_move(robot, rc, jaws_HC)
        while (gripper_pos(robot, rc) != jaws_HC):
            time.sleep(0.5)
        print("Jaws Position: "+str(gripper_pos(robot, rc)))
    elif pickOrDrop == 1:
        gripper_move(robot, rc, jaws_HO)
        while (gripper_pos(robot, rc) != jaws_HO):
            time.sleep(0.5)
        print("Jaws Position: "+str(gripper_pos(robot, rc)))

    pcb2home[2] = pcb2home[2] + 20
    robot.move_l(rc, pcb2home, local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    print("Arrived at pcb2home +20mm")
    pcb2home[2] = pcb2home[2] - 20
def _main():
    # initialize cobots
    robotG = rb.Cobot(ROBOT_IP_G)
    #robotS = rb.Cobot(ROBOT_IP_S)

    rc = rb.ResponseCollector()

    robotG.set_operation_mode(rc, rb.OperationMode.Real)
    robotG.set_speed_bar(rc, robot_speed)  # 30% speed

    # robotS.set_operation_mode(rc, rb.OperationMode.Real)
    # robotS.set_speed_bar(rc, robot_speed) #30% speed

    position = np.array([0, 0, 0, 0, 0, 0])
    res, position = robotG.get_tcp_info(rc)
    print (res)

    print ("(["+str(position[0])+", "+str(position[1])+", "+str(position[2])+
           ", "+str(position[3])+", "+str(position[4])+", "+str(position[5])+"])")

    #robotG.get_system_variable(rc, rb.SystemVariable.JRT_JEGB)[1]
    robot.get_system_variable(rc, rb.SystemVariable.SD_
    
def _main1():
    try:

        #initialize cobots
        robotG = rb.Cobot(ROBOT_IP_G)
        robotS = rb.Cobot(ROBOT_IP_S)

        rc = rb.ResponseCollector()

        robotG.set_operation_mode(rc, rb.OperationMode.Real)
        robotG.set_speed_bar(rc, robot_speed) #30% speed


        #robotS.set_operation_mode(rc, rb.OperationMode.Real)
        #robotS.set_speed_bar(rc, robot_speed) #30% speed

        gripper_init(robotG, rc)

        #ready the gripper
        gripper_move(robotG, rc,jaws_HO)
        print("Gripper set to 50%")
        #move robot to home.
        robotG_move_home(robotG,rc)
        print("arrived at home")

        robotG_move_pcb1(robotG,rc,0)

        robotG_move_pcb2(robotG,rc,1)

        robotG_move_pcb2(robotG, rc, 0)
        robotG_move_pcb1(robotG, rc, 1)

        robotG_move_home(robotG, rc)




        # Move Shank to tighten to 40mm code 00=0
        #robot.set_dout_bit_combination(rc, 0, 1, 0, rb.Endian.LittleEndian)
        # Tighten Bit code 10 = 2
        #robot.set_dout_bit_combination(rc, 0, 1,2, rb.Endian.LittleEndian)
        # Move Shank to loosen to 48mm code 01 =1
        #robot.set_dout_bit_combination(rc, 0, 1, 1, rb.Endian.LittleEndian)
        # loosen code  11 =3
        #robot.set_dout_bit_combination(rc, 0, 1, 3, rb.Endian.LittleEndian)


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

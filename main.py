# This is a sample Python script.
import sys

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

pick = 0
drop = 1
pcb1 = 0
pcb2 = 1
fasten = 0
loosen = 1

loop_sleep_sec = 0.5

local_speed = 100
local_acc = 100

robot_speed = 0.7 # 0 to 1

pcb_safe_height = 0
gripper_safe_height = 0

robotS_home_j = np.array ([34.64, -35.65, 122.06, 3.59, 90.04, -34.66])
robotS_home = ([590.257, 47.2, 436.718, 89.992, 0.011, 0.992])

screwFeederHome_j = np.array ([15.681, -15.192, 122.239, -17.048, 90.031, -15.715])
screwFeeder = np.array ([390.27, -4.48, 333.5 , 90.0, -0.0, 90.0])

robotG_home_j = ([173.8, 1.21, -113.95, 22.731, -89.93, 277.11])
robotG_home = ([0 ,0 ,0 ,0 ,0 , 0])


pcb1home_j = np.array ([170.477, -42.137, -68.748, 20.876, -89.956, 279.955])
pcb1home = np.array ([984.93, -53.28, 285.5, 90.0, 0.01, 0.5])

pcb2home_j = np.array ([176.31, -42.34, -69.03, 21.37, -89.95, 274.62])
pcb2home = np.array ([985.03, 47.07, 285.71, 90.0, -0.0, 0.5])

p = np.zeros((18, 6), dtype=np.float16)

#screwing locations from pcb1
p[0] = np.array([575.406, 24.9550, 263.38, 90, 0, 90])
p[1] = np.array([575.68, 81.68, 263.38, 90, 0, 90])
p[2] = np.array([668.08, 80.93, 263.38, 90, 0, 90])
p[3] = np.array([668.08, 24.14, 263.55, 90, 0, 90])

#screwing locations from pcb2
p[4] = np.array ([576.74, -73.58, 245.0, 90.0, -0.0, 90.0])
p[5] = np.array ([576.559, -16.968, 245.001, 90.0, 0.001, 90.001])
p[6] = np.array ([669.09, -17.3, 245.0, 90.0, -0.0, 90.0])
p[7] = np.array ([669.15, -73.499, 245.0, 90.0, 0.0, 90.0])

#test points
p[8] = np.array([367.50, 124, 245.0, 90.0, 0.0, 90.0])
p[9] = np.array([364.9, 180.46, 245.0, 90.0, 0.0, 90.0])
p[10] = np.array ([458.6, 180.8, 245.0, 90.0, 0.0, 90.0])
p[11] = np.array ([458.8, 124.4, 245.0, 90.0, 0.0, 90.0])

#Screw drop points from p[12] to p[13]
p[12] = np.array ([288.144, 151.854, 285.714, 90.0, -0.0, 90.0]) #xyz
p[13] = np.array ([47.495, -20.605, 138.863, -28.258, 90.032, -47.533]) #joints

p[14] = np.array ([288.144, 151.854, 251.86, 90.0, -0.0, 90.0])
p[15] = np.array ([47.495, -16.811, 142.449, -35.637, 90.028, -47.536])

p[16] = np.array ([288.144, 213.429, 251.86, 90.0, -0.0, 90.0])
p[17] = np.array ([54.36, -10.929, 138.473, -37.545, 90.026, -54.403])

#p_j[0] =np.array
#p_j[1] =np.array
#p_j[2] =np.array
#p_j[3] =np.array
#p_j[4] =np.array([3.337, 19.502, 106.249, -35.751, 90.012, -3.378])
#p_j[5] =np.array([9.066, 18.958, 107.033, -35.991, 90.011, -9.107])
#p_j[6] =np.array([7.77, 29.635, 90.665, -30.3, 90.008, -7.806])
#p_j[7] =np.array([2.897, 30.139, 89.847, -29.985, 90.008, -2.933])

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
        script = f"gripper_macro 6,2,1,0,5,50,100,0,0,0"
        robot.eval(rc, script, True)
        while (gripper_pos(robot, rc) != 50):
            time.sleep(0.5)

        #if ( gripper_pos(robot,rc)-100) < 1:
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
    time.sleep(0.25)
    while robotG.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
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
def dropScrew(robot, rc):
    #SHANK TO 55MM 1111= 15
    robot.set_dout_bit_combination(rc, 0, 3, 15, rb.Endian.BigEndian)
    time.sleep(3)
    robot.move_j(rc, p[13], local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    print ("RobotS drop first position")

    robot.move_j(rc, p[15], local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    print("RobotS drop 2nd position")

    robot.move_j(rc, p[17], local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    print("RobotS drop 3rd position")

    robot.move_j(rc, p[13], local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    print("RobotS drop first position")


#only to be used with robot with Screw Driver
def pickScrew(robot,rc):
    # move to screw feeder location
    # Move Shank to tighten to 30mm code 0000=0
    robot.set_dout_bit_combination(rc, 0, 3, 0, rb.Endian.BigEndian)
    robot.move_j(rc, screwFeederHome_j, local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    print ("RobotS arrived at screwfeeder home position")

    robot.move_l(rc, screwFeeder, local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    print("RobotS arrived at screw pick position")

    # Pick Screw code 1000=8
    robot.set_dout_bit_combination(rc, 0, 3, 8, rb.Endian.BigEndian)
    time.sleep(4)

    screwFeeder[2] = screwFeeder[2] + 20
    robot.move_l(rc, screwFeeder, local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    screwFeeder[2] = screwFeeder[2] - 20

    print("RobotS arrived  back at screwfeeder home position")

    screwFeeder[0] = screwFeeder[0] + 100
    robot.move_l(rc, screwFeeder, local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    screwFeeder[0] = screwFeeder[0] - 100

    #MOVE SHANK TO 35MM
    robot.set_dout_bit_combination(rc, 0, 3, 0, rb.Endian.BigEndian)
    time.sleep(1)


def robotS_move_home(robot, rc):
    robot.move_j(rc, robotS_home_j, local_acc, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    print("Arrived at robotS_home")

def screw_pcb(robot, rc, point):
    robot.set_dout_bit_combination(rc, 0, 3, 0, rb.Endian.BigEndian)
    time.sleep(3)
    point[2] = point[2] + 50
    robot.move_l(rc, point, local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    point[2] = point[2] - 50

    robot.move_l(rc, point, local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)

    robot.set_dout_bit_combination(rc, 0, 3, 4, rb.Endian.BigEndian)
    time.sleep(8)
    point[2] = point[2] + 200
    robot.move_l(rc, point, local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    point[2] = point[2] - 200

def unscrew(robot, rc, point):
    #MOVE SHANK TO 35MM
    robot.set_dout_bit_combination(rc, 0, 3, 0, rb.Endian.BigEndian)
    time.sleep(3)
    point[2] = point[2] + 50
    robot.move_l(rc, point, local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    point[2] = point[2] - 50

    robot.move_l(rc, point, local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    #LOOSEN THE SCREW 1100=12
    robot.set_dout_bit_combination(rc, 0, 3, 12, rb.Endian.BigEndian)
    time.sleep(8)
    point[2] = point[2] + 200
    robot.move_l(rc, point, local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    point[2] = point[2] - 200

    # MOVE SHANK TO 45MM 0010=2
    robot.set_dout_bit_combination(rc, 0, 3, 2, rb.Endian.BigEndian)
    time.sleep(4)

def _main():
    # initialize cobots
    robot = rb.Cobot(ROBOT_IP_S)


    rc = rb.ResponseCollector()

    robot.set_operation_mode(rc, rb.OperationMode.Real)
    robot.set_speed_bar(rc, robot_speed) #30% speed

    #exit("good exit")
    position = np.array([0, 0, 0, 0, 0, 0])
    res, position = robot.get_tcp_info(rc)
    print (res)
    print("X,Y,Z,Rx,Ry,Rz")
    print ("(["+str(position[0])+", "+str(position[1])+", "+str(position[2])+
           ", "+str(position[3])+", "+str(position[4])+", "+str(position[5])+"])\n")

    #robotG.get_system_variable(rc, rb.SystemVariable.JRT_JEGB)[1]
    res, J0 = robot.get_system_variable(rc, rb.SystemVariable.SD_J0_ANG)
    res, J1 = robot.get_system_variable(rc, rb.SystemVariable.SD_J1_ANG)
    res, J2 = robot.get_system_variable(rc, rb.SystemVariable.SD_J2_ANG)
    res, J3 = robot.get_system_variable(rc, rb.SystemVariable.SD_J3_ANG)
    res, J4 = robot.get_system_variable(rc, rb.SystemVariable.SD_J4_ANG)
    res, J5 = robot.get_system_variable(rc, rb.SystemVariable.SD_J5_ANG)
    print("J0,J1,J2,J3,J4,J5")
    print("([" + str(J0) + ", " + str(J1) + ", " + str(J2) +
          ", " + str(J3) + ", " + str(J4) + ", " + str(J5) + "])\n")
    print (res)

def _main8():
    try:

        #initialize cobots
        robotG = rb.Cobot(ROBOT_IP_G)
        robotS = rb.Cobot(ROBOT_IP_S)

        rc = rb.ResponseCollector()

        robotG.set_operation_mode(rc, rb.OperationMode.Real)
        robotG.set_speed_bar(rc, robot_speed)

        robotS.set_operation_mode(rc, rb.OperationMode.Real)
        robotS.set_speed_bar(rc, robot_speed)
        #robotS.get_system_variable(rc, rb.RobotState.)

        # screw home position
        robotS_move_home(robotS, rc)
        print("Robot have arrived at ")

        gripper_init(robotG, rc)
        # ready the gripper
        gripper_move(robotG, rc, jaws_HO)
        print("Gripper position: 50%")
        # move robot to home.
        robotG_move_home(robotG, rc)
        print("Arrived at robotG_home")

        while robotG.get_tcp_info(rc)[1][0] != robotG_home[0]:
            print(". ")

        #res, points = robotS.get_tcp_info(rc,)
        robotG_move_pcb1(robotG, rc, pick)
        robotG_move_pcb2(robotG, rc, drop)
        robotG_move_home(robotG, rc)

        while robotG.get_tcp_info(rc)[1][0] != robotG_home[0]:
            print(". ")
        # screw at pcb2
        pickScrew(robotS, rc)
        screw_pcb(robotS, rc, p[4])
        pickScrew(robotS, rc)
        screw_pcb(robotS, rc, p[5])
        #pickScrew(robotS, rc)
        #screw_pcb(robotS, rc, p[6])
        #pickScrew(robotS, rc)
        #screw_pcb(robotS, rc, p[7])

        #unscrew(robotS, rc, p[4])
        # dropScrew(robotS, rc)
        #unscrew(robotS, rc, p[5])
        #dropScrew(robotS, rc)
        # unscrew(robotS, rc, p[6])
        # dropScrew(robotS, rc)
        # unscrew(robotS, rc, p[7])
        # dropScrew(robotS, rc)

        #screwing done move to home
        robotS_move_home(robotS, rc)

        while robotG.get_tcp_info(rc)[1][0] != robotG_home[0]:
            print(". ")

        robotG_move_pcb2(robotG, rc, pick)
        robotG_move_pcb1(robotG, rc, drop)

        # screw Robot home position
        robotG_move_home(robotG, rc)
        robotS.flush(rc)
        robotG.flush(rc)

    except Exception as e:
        print(e)

    finally:
        pass

    print_hi('Program Ended')


def main_final():
    toggle = 1



if __name__ == '__main__':
    _main()

#    robot.set_dout_bit_combination(rc, 0, 15, 5, rb.Endian.LittleEndian)
#    time.sleep(1)

#    robot.set_dout_bit_combination(rc, 0, 15, 10, rb.Endian.LittleEndian)
#   time.sleep(1)

#    robot.set_dout_bit_combination(rc, 0, 15, 0, rb.Endian.LittleEndian)
#    time.sleep(1)

#JRT_JEGB_42140
# datatype enum class GripperModel
# f"gripper_macro 6,2,0,0,0,0,0,0,0,0"

#todo: update robotG_home position
#todo: check pcb1 pick and drop points.
#todo: check screw point of picb1 and pcb2


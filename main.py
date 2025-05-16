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

robot_speed = 0.9 # 0 to 1
MAX_Linear_Speed = 1200 #2500
MAX_Linear_Acceleration = 2500 #1300 #2500
MAX_Joint_Speed = 1200
MAX_Joint_Acceleration = 1300

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

local_speed2 = 200
local_acc2 = 400

pcb_safe_height = 0
gripper_safe_height = 0

#updated
robotS_home_j = np.array ([3.6, -16.2, 111.8, -5.6, 90.0, -3.6])
robotS_home = ([388.829, -85.597, 443.756, 89.998, 0.033, 90.029])

screwFeederHome_j = np.array ([15.681, -15.192, 122.239, -17.048, 90.031, -15.715])
screwFeeder = np.array ([390.27, -4.48, 332.5 , 90.0, -0.0, 90.0])

robotG_home_j = np.array ([173.8, 1.21, -113.95, 22.73, -89.93, 277.11])
robotG_home = np.array([590.257, 47.2, 436.718, 89.992, 0.011, 0.992])


pcb1home_j = np.array ([170.601, -41.221, -67.731, 18.954, -89.958, 279.842])
pcb1home = np.array ([985.87, -51.08, 286.66, 90.0, 0.01, 0.51])

pcb2home_j = np.array ([176.47, -41.047, -67.5, 18.549, -89.949, 273.97])
pcb2home = np.array ([985.83, 50.0, 286.3, 90.0, 0.0, 0.5])

p = np.zeros((18, 6), dtype=np.float16)

#camera points
cam2_j = np.array ([176.042, -43.705, -60.331, 14.02, -90.043, 272.851])
cam2_p = np.array ([1017.146, 40.354, 354.58, 89.91, 0.02, -1.05])

cam1_j = ([170.458, -43.629, -60.468, 14.09, -90.044, 278.435])
cam1_p = np.array ([1015.59, -58.69, 354.58, 89.91, 0.02, -1.05])

#screwing locations from pcb1
p[0] = np.array([575.406, 24.9550, 263.38, 90, 0, 90])
p[1] = np.array([575.68, 81.68, 263.38, 90, 0, 90])
p[2] = np.array([668.08, 80.93, 263.38, 90, 0, 90])
p[3] = np.array([668.08, 24.14, 263.55, 90, 0, 90])

#screwing locations from pcb2
p[4] = np.array ([576.79, -73.909, 245.0, 90.0, -0.0, 90.0])
p[5] = np.array ([576.56, -17.22, 245.0, 90.0, 0.0, 90.0])
p[6] = np.array ([669.09, -17.3, 245.0, 90.0, -0.0, 90.0])
p[7] = np.array ([669.15, -74.2, 245.0, 90.0, -0.0, 90.0])

#test points
p[8] = np.array([367.50, 124, 245.0, 90.0, 0.0, 90.0])
p[9] = np.array([364.9, 180.46, 245.0, 90.0, 0.0, 90.0])
p[10] = np.array ([458.6, 180.8, 245.0, 90.0, 0.0, 90.0])
p[11] = np.array ([458.8, 124.4, 245.0, 90.0, 0.0, 90.0])

#Screw drop points from p[12] to p[13]
p[12] = np.array ([466.662, -74.405, 308.039, 90.0, 0.0, 90.0]) #xyz
p[13] = np.array ([4.374, 0.744, 118.335, -29.079, 90.021, -4.414]) #joints

p[14] = np.array ([466.66, -81.07, 258.0, 90.0, -0.0, 90.0])
p[15] = ([3.546, 5.754, 122.8, -38.555, 90.017, -3.589])

p[16] = np.array ([466.66, -171.5, 258.0, 90.0, 0.0, 90.0])
p[17] = np.array ([-7.423, 8.726, 119.31, -38.036, 90.015, 7.38])

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
        gripper_move(robot, rc, 50)
        while (gripper_pos(robot, rc) != 50):
            time.sleep(0.5)
            print(". ")

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

def robotG_move(robot, rc, point):
    robot.move_l(rc, point, local_speed2, local_acc2)
    print("Robot G moving")
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        print(". ")
        time.sleep(0.5)
    time.sleep(3)

def robotG_move_home(robotG, rc):
    # move robot to home.

    robotG.move_j(rc,robotG_home_j,local_speed,local_acc)
    rc.error().throw_if_not_empty()
    time.sleep(0.25)
    print("Robot G moving to home")
    while robotG.get_robot_state(rc)[1] == rb.RobotState.Moving:
        print(". ")
        time.sleep(0.5)
    rc.error().throw_if_not_empty()

def robotG_move_pcb1(robot, rc, pickOrDrop):

    if pickOrDrop == pick:
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

    if pickOrDrop == pick:
        #Gripper jaws to CLOSE %.
        gripper_move(robot,rc,jaws_HC)
        time.sleep(0.25)
        while(gripper_pos(robot,rc)!=jaws_HC):
            time.sleep(0.5)
        print("Jaws Position: "+str(gripper_pos(robot, rc)))
    elif pickOrDrop == drop:
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
def dropScrew(robot, rc) :
    #SHANK TO 55MM 1111= 15
    robot.set_dout_bit_combination(rc, 0, 3, 15, rb.Endian.BigEndian)
    time.sleep(3)
    robot.move_j(rc, p[13], local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    print ("RobotS drop first position")

    robot.move_l(rc, p[14], local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    print("RobotS drop 2nd position")

    robot.move_l(rc, p[16], local_speed, local_acc)
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
    time.sleep(3)

    screwFeeder[2] = screwFeeder[2] + 100
    robot.move_l(rc, screwFeeder, local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    screwFeeder[2] = screwFeeder[2] - 100

    print("RobotS arrived  back at screwfeeder home position")
    # MOVE SHANK TO 35MM
    robot.set_dout_bit_combination(rc, 0, 3, 0, rb.Endian.BigEndian)

    #screwFeeder[0] = screwFeeder[0] + 100
    #robot.move_l(rc, screwFeeder, local_speed, local_acc)
    #print ("moving to safe screw space")
    #while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
    #    print(". ")
    #    time.sleep(0.5)
    #screwFeeder[0] = screwFeeder[0] - 100

    #MOVE SHANK TO 35MM
    #robot.set_dout_bit_combination(rc, 0, 3, 0, rb.Endian.BigEndian)
    #time.sleep(3)


def robotS_move_home(robot, rc):
    robot.move_j(rc, robotS_home_j, local_acc, local_acc)
    print("RobotS moving to home")
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        print(". ")
        time.sleep(0.25)
    print("Arrived at robotS_home")

def screw_pcb(robot, rc, point):
    #move shank to 35mm
    robot.set_dout_bit_combination(rc, 0, 3, 0, rb.Endian.BigEndian)
    time.sleep(1)

    point[2] = point[2] + 50
    robot.move_l(rc, point, local_speed2, local_acc2)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    point[2] = point[2] - 50

    robot.move_l(rc, point, local_speed2, local_acc2)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)

    robot.set_dout_bit_combination(rc, 0, 3, 4, rb.Endian.BigEndian)
    time.sleep(7.5)
    point[2] = point[2] + 100
    robot.move_l(rc, point, local_speed2, local_acc2)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    point[2] = point[2] - 100

def unscrew(robot, rc, point):
    #MOVE SHANK TO 35MM
    robot.set_dout_bit_combination(rc, 0, 3, 0, rb.Endian.BigEndian)
    time.sleep(3)
    point[2] = point[2] + 20
    robot.move_l(rc, point, local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    point[2] = point[2] - 20

    point[2] = point[2] + 5
    robot.move_l(rc, point, local_speed, local_acc)
    while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
        time.sleep(0.5)
    point[2] = point[2] - 5
    time.sleep(0.25)
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

def _main3():
    # initialize cobots
    robot = rb.Cobot(ROBOT_IP_G)

    rc = rb.ResponseCollector()

    robot.set_operation_mode(rc, rb.OperationMode.Real)
    robot.set_speed_bar(rc, 0.7) #30% speed

    robotG_move(robot, rc, cam2_p)

    # unscrew(robot, rc, p[4])
    # dropScrew(robot, rc)
    # unscrew(robot, rc, p[5])
    # dropScrew(robot, rc)
    # unscrew(robot, rc, p[6])
    # dropScrew(robot, rc)
    # unscrew(robot, rc, p[7])
    # dropScrew(robot, rc)

    # robotS_move_home(robot, rc)

    #robotS_move_home(robot, rc)
    #robotG_move_pcb1(robot, rc, pick)
    #robotG_move_pcb2(robot, rc, drop)
    #robotG_move_home(robot, rc)
    #robotG_move_cam(robot, rc, cam1_p)
    #robotG_move_cam(robot, rc, cam2_p)
    #robotG_move_home(robot, rc)


    #pickScrew(robot, rc)
    #screw_pcb(robot, rc, p[4])
    #pickScrew(robot, rc)
    #screw_pcb(robot, rc, p[5])
    #pickScrew(robot, rc)
    #screw_pcb(robot, rc, p[6])
    #pickScrew(robot, rc)
    #screw_pcb(robot, rc, p[7])
    #dropScrew(robot, rc)
    #robotG_move_home(robot,rc)

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

def _main():
    try:

        #initialize cobots
        robotG = rb.Cobot(ROBOT_IP_G)
        robotS = rb.Cobot(ROBOT_IP_S)

        rc = rb.ResponseCollector()

        robotG.set_operation_mode(rc, rb.OperationMode.Real)
        robotG.set_speed_bar(rc, robot_speed)

        robotS.set_operation_mode(rc, rb.OperationMode.Real)
        robotS.set_speed_bar(rc, robot_speed)

        # screw home position
        robotS_move_home(robotS, rc)
        print("RobotS have arrived home ")

        # move robot to home.
        robotG_move_home(robotG, rc)
        print("Arrived at robotG_home")

        gripper_init(robotG, rc)
        # ready the gripper
        gripper_move(robotG, rc, jaws_HO)
        print("Gripper position: 50%")

        #res, points = robotS.get_tcp_info(rc,)
        robotG_move(robotG, rc, cam1_p)

        robotG_move_pcb1(robotG, rc, pick)
        robotG_move_pcb2(robotG, rc, drop)

        robotG_move(robotG, rc, cam2_p)

        print("Moving RobotG to home")
        robotG_move_home(robotG, rc)
        while round(robotG.get_tcp_info(rc)[1][0]) != round(robotG_home[0]):
            print(". ")

        pickScrew(robotS, rc)
        screw_pcb(robotS, rc, p[4])
        pickScrew(robotS, rc)
        screw_pcb(robotS, rc, p[5])
        pickScrew(robotS, rc)
        screw_pcb(robotS, rc, p[6])
        pickScrew(robotS, rc)
        screw_pcb(robotS, rc, p[7])

        # screwing done move to home
        print ("Moving RobotS to home..")
        robotS_move_home(robotS, rc)
        while round(robotS.get_tcp_info(rc)[1][0]) != round(robotS_home[0]):
            print(". ")
            time.sleep(0.25)

        robotG_move (robotG, rc, cam2_p)

        print ("RobotG to home... ")
        robotG_move_home(robotG, rc)
        while round(robotG.get_tcp_info(rc)[1][0]) != round(robotG_home[0]):
            print(". ")
            time.sleep(0.25)

        unscrew(robotS, rc, p[4])
        dropScrew(robotS, rc)
        unscrew(robotS, rc, p[5])
        dropScrew(robotS, rc)
        unscrew(robotS, rc, p[6])
        dropScrew(robotS, rc)
        unscrew(robotS, rc, p[7])
        dropScrew(robotS, rc)

        robotS_move_home(robotS, rc)

        while round(robotS.get_tcp_info(rc)[1][0]) != round(robotS_home[0]):
            print(". ")

        robotG_move (robotG, rc, cam2_p)
        robotG_move_pcb2 (robotG, rc, pick)
        robotG_move_pcb1 (robotG, rc, drop)
        robotG_move(robotG, rc, cam1_p)

        print("RobotG Moving to home")

        robotG_move_home(robotG, rc)
        #while round(robotG.get_tcp_info(rc)[1][0]) != round(robotG_home[0]):
        #    print(". ")

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


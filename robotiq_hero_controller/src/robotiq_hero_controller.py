#!/usr/bin/env python

PKG = 'robotiq_hero_controller'
import roslib; roslib.load_manifest(PKG)
import rospy
from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg
#from time import sleep
from std_msgs.msg import Float64

finger_A_raw_value = 0
finger_B_raw_value = 0
finger_C_raw_value = 0

finger_C_position = 255
finger_B_position = 255
finger_A_position = 255

def map_raw_value_to_finger_position(value, fromLow, fromHigh, toLow, toHigh):
    result = (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow   
    return result

def finger_A_cb(msg):
    global finger_A_position
    global finger_A_raw_value
    finger_A_raw_value = int(msg.data)
    temp_A_pos = map_raw_value_to_finger_position(finger_A_raw_value, 75, 192, 0, 255)
    
    if temp_A_pos < 0:
        temp_A_pos = 0
    elif temp_A_pos > 255:
        temp_A_pos = 255

    #print finger_A_raw_value
    finger_A_position = temp_A_pos
    #print 'finger A: ' + str(finger_A_position)

def finger_B_cb(msg):
    global finger_B_position
    global finger_B_raw_value
    finger_B_raw_value = int(msg.data)
    temp_B_pos = map_raw_value_to_finger_position(finger_B_raw_value, 160, 192, 0, 255)

    if temp_B_pos < 0:
        temp_B_pos = 0
    elif temp_B_pos > 255:
        temp_B_pos = 255

    finger_B_position = temp_B_pos
    
    #print 'finger B: ' + str(finger_B_position)
    
def finger_C_cb(msg):
    global finger_C_position
    global finger_C_raw_value
    finger_C_raw_value = int(msg.data)
    temp_C_pos = map_raw_value_to_finger_position(finger_C_raw_value, 75, 195, 0, 255)

    if temp_C_pos < 0:
        temp_C_pos = 0
    elif temp_C_pos > 255:
        temp_C_pos = 255

    finger_C_position = temp_C_pos
    #finger_C_position = int(msg.data)
    #finger_c_position = c_position
    #print 'finger C: ' + str(finger_C_position)


def gen_robotiq_controller_command():
    global finger_A_position
    global finger_B_position
    global finger_C_position

    command = outputMsg.SModel_robot_output();

    command.rACT = 1 # Activate Gripper
    command.rMOD = 0 # 0 for basic mode
    command.rGTO = 1 # 1 indicates move fingers to requested position
    command.rATR = 0 # Emergency Release
    command.rGLV = 0 # Disable Robotiq Glove mode
    command.rICF = 1 # Individual Finger control in Basic Mode
    command.rICS = 0 # Individual Finger control in Scissor Mode

    # Finger A
    command.rPRA = finger_A_position # Position
    command.rSPA = 255 # Speed 
    command.rFRA = 30  # Force

    # Finger B
    command.rPRB = finger_B_position # Position
    command.rSPB = 255 # Speed
    command.rFRB = 30  # Force

    # Finger C
    command.rPRC = finger_C_position # Position
    command.rSPC = 255 # Speed
    command.rFRC = 30  # Force

    # Scissor Mode
    command.rPRS = 0
    command.rSPS = 0
    command.rFRS = 0

    return command


def robotiq_main_loop():
    
    rospy.init_node('robotiq_hero_controller')

    # Subscribe to HERO Glove to get finger position
    # Finger A is Robotiq's thumb
    rospy.Subscriber("/hero/finger_1/joint", Float64, finger_A_cb)
    rospy.Subscriber("/hero/finger_2/joint", Float64, finger_C_cb)
    rospy.Subscriber("/hero/finger_3/joint", Float64, finger_B_cb)

    pub = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output, queue_size=10)

    robotiq_finger_position = outputMsg.SModel_robot_output();

    while not rospy.is_shutdown():
        
        #print 'finger A: ' + str(finger_A_position) + ', finger B: ' + str(finger_B_position) + ', finger C: ' + str(finger_C_position)
        
        robotiq_finger_position = gen_robotiq_controller_command()

        pub.publish(robotiq_finger_position)

        rospy.sleep(0.1)


if __name__ == '__main__':
    robotiq_main_loop()


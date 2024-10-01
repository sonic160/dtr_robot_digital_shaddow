#!/usr/bin/env python3
# Software License Agreement (BSD License)
# 
# This script controls a motor to turn following a unit-pulse signal. And monitor the response.


import threading, rospy, Board, time
from cm.msg import msg_cm as RosJointState
# import argparse
import time
import numpy as np
from math import sqrt
import inverse_kinematics
import random

random.seed(50)

# 机械臂根据逆运动学算出的角度进行移动
ik = inverse_kinematics.IK()


class ArmIK:
    servo3Range = (0, 1000, 0, 240.0)  # 脉宽， 角度
    servo4Range = (0, 1000, 0, 240.0)
    servo5Range = (0, 1000, 0, 240.0)
    servo6Range = (0, 1000, 0, 240.0)

    def __init__(self):
        self.setServoRange()

    def setServoRange(self, servo3_Range=servo3Range, servo4_Range=servo4Range, servo5_Range=servo5Range,
                      servo6_Range=servo6Range):
        # 适配不同的舵机
        self.servo3Range = servo3_Range
        self.servo4Range = servo4_Range
        self.servo5Range = servo5_Range
        self.servo6Range = servo6_Range
        self.servo3Param = (self.servo3Range[1] - self.servo3Range[0]) / (self.servo3Range[3] - self.servo3Range[2])
        self.servo4Param = (self.servo4Range[1] - self.servo4Range[0]) / (self.servo4Range[3] - self.servo4Range[2])
        self.servo5Param = (self.servo5Range[1] - self.servo5Range[0]) / (self.servo5Range[3] - self.servo5Range[2])
        self.servo6Param = (self.servo6Range[1] - self.servo6Range[0]) / (self.servo6Range[3] - self.servo6Range[2])

    def transformAngelAdaptArm(self, theta3, theta4, theta5, theta6):
        # 将逆运动学算出的角度转换为舵机对应的脉宽值
        servo3 = int(round(theta3 * self.servo3Param + (self.servo3Range[1] + self.servo3Range[0]) / 2))

        servo4 = int(round(-theta4 * self.servo4Param + (self.servo4Range[1] + self.servo4Range[0]) / 2))

        servo5 = int(round((self.servo5Range[1] + self.servo5Range[0]) / 2 + theta5 * self.servo5Param))

        servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2]) / 2 + theta6)) * self.servo6Param)

        return {"servo3": servo3, "servo4": servo4, "servo5": servo5, "servo6": servo6}

    def setPitchRanges(self, coordinate_data, alpha, alpha1, alpha2, d=0.01):
        # 给定坐标coordinate_data和俯仰角alpha,以及俯仰角范围的范围alpha1, alpha2，自动寻找最接近给定俯仰角的解
        # 如果无解返回False,否则返回舵机角度、俯仰角
        # 坐标单位m， 以元组形式传入，例如(0, 0.5, 0.1)
        # alpha为给定俯仰角, 单位度
        # alpha1和alpha2为俯仰角的取值范围

        x, y, z = coordinate_data
        a_range = abs(int(abs(alpha1 - alpha2) / d)) + 1
        for i in range(a_range):
            if i % 2:
                alpha_ = alpha + (i + 1) / 2 * d
            else:
                alpha_ = alpha - i / 2 * d
                if alpha_ < alpha1:
                    alpha_ = alpha2 - i / 2 * d
            ##            print(alpha_)
            result = ik.getRotationAngle((x, y, z), alpha_)
            if result:
                theta3, theta4, theta5, theta6 = result['theta3'], result['theta4'], result['theta5'], result['theta6']
                servos = self.transformAngelAdaptArm(theta3, theta4, theta5, theta6)
                return result, servos, alpha_

        return False


class CMDataPublisher:
    def __init__(self, node, io_block_flag: list, freq=10):
        self.node = node
        rate = self.node.get_param('~rate', freq)
        self.r = rospy.Rate(rate)
        self.io_block_flag = io_block_flag

        self.msg = RosJointState()
        self.msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint']
        self.msg.header.frame_id = 'not_relervant'
        self.msg.position = [0]*6
        self.msg.temperature = [0]*6
        self.msg.voltage = [0]*6

        # Start publisher
        self.joint_states_pub = rospy.Publisher('/condition_monitoring', RosJointState, queue_size=50)
        rospy.loginfo("Starting Joint State Publisher at " + str(rate) + "Hz")


    def safe_read(self, monitored_motor: int):
        ''' Read the position safely. This function verifies if the IO is occupied before perform the reading operation.
        If IO is blocked, it will waits until it is released. During the reading operation, it will block the IO.        
        '''
        # Check if the io is blocked:
        while self.io_block_flag[0]:
            print('Thread_CM: Waiting for the IO to be released!')
            pass

        # Block the IO and perform the reading action.
        self.io_block_flag[0] = True
        # Get CM data.
        motor_idx = monitored_motor-1
        self.msg.position[motor_idx] = Board.getBusServoPulse(motor_idx+1) # Position
        self.msg.temperature[motor_idx] = Board.getBusServoTemp(motor_idx+1) # Temperature
        self.msg.voltage[motor_idx] = Board.getBusServoVin(motor_idx+1) # Voltage
        # Release the IO
        self.io_block_flag[0] = False


    def get_and_pub_cm_data(self):             
        # Log current time.
        self.msg.header.stamp = rospy.Time.now()
        # Get CM data.
        for monitored_motor in range(1, 7):
            self.safe_read(monitored_motor)
        # Publish the data.
        self.joint_states_pub.publish(self.msg)        


class ControlMotor:
    def __init__(self, node, io_block_flag: list):
        self.node = node
        self.io_block_flag = io_block_flag

        # Prepare initial values of the msg.
        self.msg = RosJointState()
        self.msg.name = ['Target value']
        self.msg.header.frame_id = 'not_relervant'
        self.msg.position = [0, 0, 0, 0, 0, 0]
        self.msg.temperature = [0, 0, 0, 0, 0, 0]
        self.msg.voltage = [0]

        self.monitor_pos_pub = rospy.Publisher('/position_monitoring', RosJointState, queue_size=1)


    def safe_control_motor(self, target_value: int, duration: int, monitored_motor: int):
        ''' Send the control command to a given motor safely. It verifies the IO is not occupied before sending the control command.
        During the sending operation, it will block the IO.
        '''
        # Check if the io is blocked:
        while self.io_block_flag[0]:
            print('Thread_Control: Waiting for the IO to be released!')
            pass

        # Block the IO and perform the reading action.
        self.io_block_flag[0] = True
        # Set target value.
        Board.setBusServoPulse(monitored_motor, target_value, duration)
        # Release the IO
        self.io_block_flag[0] = False


    def send_and_pub_control_signal(self, trajectory: list, duration_list: list):       
        # Log the current time.
        self.msg.header.stamp = rospy.Time.now()
        
        # Loop over the motors.
        for monitored_motor in range(1, 7):
            motor_idx = monitored_motor - 1
            target_value = trajectory[motor_idx]
            duration = duration_list[motor_idx]
            self.safe_control_motor(target_value, duration, monitored_motor)            
        # Sleep for 2 seconds. The time needed for the robot to finish one trajectory.
        time.sleep(2)
                
        # Publish the control command per trajectory.        
        self.msg.position = trajectory
        self.msg.temperature = duration_list
        self.monitor_pos_pub.publish(self.msg)
        # Log the information.
        rospy.loginfo('Publish control command: Position target: {}, Duration: {}ms'.format(self.msg.position, self.msg.temperature))       


def node_condition_monitoring(node, io_block_flag, freq=100):
    cm_data_publisher = CMDataPublisher(node, io_block_flag, freq)
    while not rospy.is_shutdown():
        cm_data_publisher.get_and_pub_cm_data()
        cm_data_publisher.r.sleep()

def node_control_robot(node, io_block_flag: list, trajectories=[[500, 500, 500, 500, 500, 500]], durations_lists=[[1000, 1000, 1000, 1000, 1000, 1000]]):
    # Initialize ros node.
    robot_controller = ControlMotor(node, io_block_flag)
    # Sleep for 5 seconds. Time needed to start the listener on the PC side.
    time.sleep(5)    
    # Loop over the trajectories. Send the control signals.
    for trajectory, duration_list in zip(trajectories, durations_lists):
        robot_controller.send_and_pub_control_signal(trajectory, duration_list)
  

if __name__ == '__main__':
    AK = ArmIK()
    print(ik.getLinkLength())

    # 原本直接使用 trajectories 指定6个motor的转角
    # 修改为通过在工作范围内随机选择一个位置，通过逆运动学计算获得motor转角，将这个信息发送到控制信号模块，durations_lists 的信息不改变

    # Initial position and actions_number
    actions_number = 2
    # 俯仰角
    angleOrient = (-180, -180, 0)
    Px = 0.0
    Py = 0.26
    Pz = 0.04
    # Working range
    #range_Px = [-0.1, 0.2]
    #range_Py = [0.10, 0.26]
    #range_Pz = [0.04, 0.06]

    range_Px = [-0.2, -0.1]
    range_Py = [0, 0.05]
    range_Pz = [0.15, 0.30]

    # randomly set pincer position: trajectories with precision = f.2
    work_range = [
        (round(random.uniform(range_Px[0], range_Px[1]), 2),
         round(random.uniform(range_Py[0], range_Py[1]), 2),
         round(random.uniform(range_Pz[0], range_Pz[1]), 2))
        for _ in range(actions_number)
    ]

    # trajectories = [[500, 500, 500, 500, 500, 500]]
    trajectories = [[450, 500, 111, 512, 282, 283],
                    [450, 500, 500, 500, 500, 500],
            ] 
    for coord in work_range:
        target = AK.setPitchRanges(coord, angleOrient[0], angleOrient[1], angleOrient[2])
        if target:
            servo_data = target[1]
            trajectory = [
                450, 500,  # Fixed values for servos 1 and 2
                servo_data['servo3'],
                servo_data['servo4'],
                servo_data['servo5'],
                servo_data['servo6']
            ]
            trajectories.append(trajectory)

    durations_lists = [[2000, 2000, 2000, 2000, 2000, 2000] for _ in range(len(trajectories))]

    # Define the io block flag.
    io_block_flag = [False]

    try:
        # Initialize ROS node in the main thread
        rospy.init_node('node_test_motor_position', anonymous=True)

        # Create two threads
        monitoring_freq = 10
        thread1 = threading.Thread(target=node_condition_monitoring, args=(rospy, io_block_flag, monitoring_freq))
        thread2 = threading.Thread(target=node_control_robot, args=(rospy, io_block_flag, trajectories, durations_lists))

        # Start the threads
        thread1.start()
        thread2.start()

        # Wait for both threads to finish
        thread1.join()
        thread2.join()

    except rospy.ROSInterruptException:
        pass

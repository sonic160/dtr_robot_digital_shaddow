#!/usr/bin/env python3
# Software License Agreement (BSD License)
# 
# This script defines and test failure simulators.
# We controls a motor to turn following a unit-pulse signal. And monitor the response.


import threading, rospy, Board, time
from cm.msg import msg_cm as RosJointState
from trajectory_control import node_condition_monitoring
from trajectory_generation_test import generate_n_trajs
import sys
import random
import numpy as np


class FailureSimulator:
    ''' ### Description
    Base class for the failure simulator.
    '''
    def __init__(self, trajectory_idxes: list=[], motor_idxes: list=[], failure_simulators: list = []):
        self.trajectory_indexes = trajectory_idxes
        self.motor_indexes = motor_idxes
        self.failure_simulators = failure_simulators
        self.current_simulator_idx = 0
        self.current_trajectory = 0


    def simulate_failure(self, target: int=0, duration: int=1000):
        ''' ### Description
        This function simulates the failure of a give motor. It calles the failure simulator defined in self.failure_simulators.

        ### Parameters 
        - target: The original target value of the motor.
        - duration: The original duration of the motor.

        ### Returns
        - failed_target: The new target value of the motor after injecting failure.
        - failed_duration: The new duration of the motor after injecting failure.
        '''
        # Get the simulator related to the current failure.
        simulator = self.failure_simulators[self.current_simulator_idx]
        # Run the simulation.
        failed_target, failed_duration = simulator.simulate_failure(target, duration)
        # Update the current simulator index.
        if self.current_simulator_idx < len(self.failure_simulators) - 1:
            self.current_simulator_idx += 1
        
        return failed_target, failed_duration
    

class StuckSimulator(FailureSimulator):
    ''' ### Description
    This class defines a simulator for the stuck failure mode. It is inherited from FailureSimulator.    
    '''
    def simulate_failure(self, target: int=0, duration: int=1000) -> tuple:
        ''' ### Description
        This function simulates stuck. It sets the target and duration to -1.
        In the main function, this will result in the control signal not sent.
        
        ### Parameters 
        - target: The original target value of the motor.
        - duration: The original duration of the motor.

        ### Returns
        - failed_target: The new target value of the motor after injecting failure.
        - failed_duration: The new duration of the motor after injecting failure.        
        '''
        failed_target = -1
        failed_duration = -1

        return failed_target, failed_duration
    

class SteadyStateErrorSimulator(FailureSimulator):
    ''' ### Description
    This class defines a simulator for the steady state error failure mode. It is inherited from FailureSimulator.    
    '''
    def __init__(self, error_factor: int=3):
        super().__init__()
        self.error_factor = error_factor
    
    def simulate_failure(self, target: int=0, duration: int=1000) -> tuple:
        ''' ### Description
        This function simulates steady state error. It sets the target to the original target value multiplied by the error factor.
        In the main function, this will result in the control signal not sent.
        
        ### Parameters 
        - target: The original target value of the motor.
        - duration: The original duration of the motor.

        ### Returns
        - failed_target: The new target value of the motor after injecting failure.
        - failed_duration: The new duration of the motor after injecting failure.
        '''
        # print(self.error_factor)
        
        # Extract lower and upper limits of the errors.
        ll = self.error_factor[0]
        ul = self.error_factor[1]
        
        # Generate error.
        error = random.randint(ll, ul)
        if random.random()<.5:
            error = -1*error        
        
        failed_target = target + error
        failed_duration = duration

        return failed_target, failed_duration
    

class SpeedDegradationSimulator(FailureSimulator):
    ''' ### Description
    This class defines a simulator for the degradation of rotation speed failure mode. It is inherited from FailureSimulator.    
    '''
    def __init__(self, percentage_loss: float=.2):
        super().__init__()
        self.percentage_loss = percentage_loss
    
    def simulate_failure(self, target: int=0, duration: int=1000) -> tuple:
        ''' ### Description
        This function simulates the degradation of rotation speed. It sets the duration to the original duration multiplied by the 1 + percentage loss.
        
        ### Parameters 
        - target: The original target value of the motor.
        - duration: The original duration of the motor.

        ### Returns
        - failed_target: The new target value of the motor after injecting failure.
        - failed_duration: The new duration of the motor after injecting failure.
        '''
        failed_target = target
        failed_duration = duration*(1+self.percentage_loss)

        return failed_target, failed_duration
    

def node_control_robot(node, io_block_flag: list, 
                       trajectories: list=[[500, 500, 500, 500, 500, 500]], 
                       durations_lists: list=[[1000, 1000, 1000, 1000, 1000, 1000]],
                       failure_simulator=None):
    # Initialize ros node.
    robot_controller = ControlMotor(node, io_block_flag, failure_simulator)
    # Sleep for 5 seconds. Time needed to start the listener on the PC side.
    time.sleep(5)    

    if robot_controller.failure_simulator is None:
        # Loop over the trajectories. Send the control signals.
        for trajectory, duration_list in zip(trajectories, durations_lists):
            robot_controller.send_and_pub_control_signal(trajectory, duration_list)
    else:
        for idx_trajectory in range(len(trajectories)):
            robot_controller.failure_simulator.current_trajectory = idx_trajectory
            trajectory, duration_list = trajectories[idx_trajectory], durations_lists[idx_trajectory]
            robot_controller.send_and_pub_control_signal(trajectory, duration_list)
    

class ControlMotor:
    def __init__(self, node, io_block_flag: list, failure_simulator=None):
        self.node = node
        self.io_block_flag = io_block_flag
        self.failure_simulator = failure_simulator

        # Prepare initial values of the msg.
        self.msg = RosJointState()
        self.msg.name = ['Target value']
        self.msg.header.frame_id = 'not_relervant'
        self.msg.position = [0, 0, 0, 0, 0, 0]
        self.msg.temperature = [0, 0, 0, 0, 0, 0]
        self.msg.voltage = [0]

        self.monitor_pos_pub = rospy.Publisher('/position_monitoring', RosJointState)


    def safe_control_motor(self, target_value: int, duration: int, monitored_motor: int):
        ''' Send the control command to a given motor safely. It verifies the IO is not occupied before sending the control command.
        During the sending operation, it will block the IO.
        '''
        # Check if the io is blocked:
        while self.io_block_flag[0]:
            print(f'Thread_Control: Waiting for the IO to be released! Motor{monitored_motor}')
            pass

        # Block the IO and perform the reading action.
        self.io_block_flag[0] = True
        # Set target value.
        if target_value>=0 and target_value<=1000 and duration>0: # Ignore unreasonable inputs.
            Board.setBusServoPulse(monitored_motor, target_value, duration)
        # Release the IO
        self.io_block_flag[0] = False

    def send_and_pub_control_signal(self, trajectory: list, duration_list: list):       
        # Log the current time.
        self.msg.header.stamp = rospy.Time.now()
        
        # Loop over the motors.
        for monitored_motor in range(1, 7):
            # Get the target value and duration.
            motor_idx = monitored_motor - 1
            target_value = trajectory[motor_idx]
            duration = duration_list[motor_idx]

            # If need to simulate failure:
            if self.failure_simulator is not None:
                if self.failure_simulator.current_trajectory in self.failure_simulator.trajectory_indexes:
                    # Get the index of the current trajectory in trajectory_indexes list.
                    idx_trajectory = self.failure_simulator.trajectory_indexes.index(self.failure_simulator.current_trajectory)
                    if monitored_motor in self.failure_simulator.motor_indexes[idx_trajectory]:
                        target_value, duration = self.failure_simulator.simulate_failure(target_value, duration)
            
            # Publish the command.
            self.safe_control_motor(target_value, duration, monitored_motor)            
        
        # Sleep for 2 seconds. The time needed for the robot to finish one trajectory.
        time.sleep(2)
                
        # Publish the control command per trajectory.        
        self.msg.position = trajectory
        self.msg.temperature = duration_list
        self.monitor_pos_pub.publish(self.msg)
        # Log the information.
        rospy.loginfo('Publish control command: Position target: {}, Duration: {}ms'.format(self.msg.position, self.msg.temperature))       


if __name__ == '__main__':
    # This is a test script for simulating failures on motors by software.
    
    # Generate n trajectories.
    number_of_runs = 2
    number_of_movement_per_traj = 5    
    trajectories, durations_lists = generate_n_trajs(number_of_runs, number_of_movement_per_traj)
    
    # Define failure label:
    # 0: No failure
    # 1-4: Motor 6-3 stucks
    # 5-8: Motor 6-3 steady-state error
    failure_label = 0

    # Default value for failure_simulator.
    # When no failure or stuck failure, set to None.
    # For stuck failure, we collect original response, and inject failure during postprocessing.
    failure_simulator = None
    
    # Simulate a steady-state error.
    if failure_label > 4 & failure_label < 9:
        # Define failure generators.
        error_range = [5, 20]
        
        # Calculate failed motor.
        failed_motor_idx = 11-failure_label
        
        # All the trajectories are failed.
        total_rows = number_of_runs*(number_of_movement_per_traj + 2)
        failed_trajectories = np.arange(0, total_rows).tolist()
        
        # In each trajectory, only failed_motor_idx motor failed.
        failed_motor_idx = [[failed_motor_idx]] * total_rows
        
        failure_simulators = []
        for i in range(total_rows):
            failure_simulators.append(SteadyStateErrorSimulator(error_factor=error_range))
            
        failure_simulator = FailureSimulator(trajectory_idxes=failed_trajectories,
                                             motor_idxes=failed_motor_idx,
                                             failure_simulators=failure_simulators)
    # failure_simulator = None
    
    # failed_trajectories = np.arange(1, total_rows + 1)
    
    # pattern = [1,6,5,4,3]
    # pattern_length = len(pattern)
    
    
    
    # for i in range(0, total_rows,  number_of_movement_per_traj):
    #     value = pattern[(i // number_of_movement_per_traj) % pattern_length]
       
        
    # mult_pattern = [1,1,1,1,1,1,1,6,6,6,6,6,1,1,5,5,5,5,5,1,1,4,4,4,4,4,1,1,3,3,3,3,3,1,1] 
    # failed_trajectories = failed_trajectories.tolist()
    # print("failed_trajectories", failed_trajectories)
    # failed_motors= [[motor_number] for motor_number in mult_pattern] * int(total_rows/7)
    # print("failed_motors", failed_motors)
    
    # failure_simulators = []
    # for _ in range(int(total_rows/7)):
    #     a = random.uniform(0.15, 0.35) * random.choice([-1,1])
    #     failure_simulators.append(SteadyStateErrorSimulator(error_factor=a))
    #     failure_simulators.append(SteadyStateErrorSimulator(error_factor=a))
    #     failure_simulators.append(SteadyStateErrorSimulator(error_factor=a))
    #     failure_simulators.append(SteadyStateErrorSimulator(error_factor=a))
    #     failure_simulators.append(SteadyStateErrorSimulator(error_factor=a))
    #     failure_simulators.append(SteadyStateErrorSimulator(error_factor=a))
    #     failure_simulators.append(SteadyStateErrorSimulator(error_factor=a))
    
    # failure_simulator = FailureSimulator(trajectory_idxes=failed_trajectories,
    #                                      motor_idxes=failed_motors,
    #                                      failure_simulators=failure_simulators)
    # print("failure_simulators", failure_simulators)

    # Define the io block flag.
    io_block_flag = [False]

    try:
        # Initialize ROS node in the main thread
        rospy.init_node('node_test_motor_position', anonymous=True)

        # Create two threads
        monitoring_freq = 10
        thread1 = threading.Thread(target=node_condition_monitoring, args=(rospy, io_block_flag, monitoring_freq))
        thread2 = threading.Thread(target=node_control_robot, args=(rospy, io_block_flag, trajectories, durations_lists, failure_simulator))

        # Start the threads
        thread1.start()
        thread2.start()

        # Wait for both threads to finish
        thread1.join()
        thread2.join()

    except rospy.ROSInterruptException:
        pass


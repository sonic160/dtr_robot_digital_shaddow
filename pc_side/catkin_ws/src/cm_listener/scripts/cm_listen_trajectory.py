#!/usr/bin/env python
import rospy
from cm_listener.msg import msg_cm as RosJointState
import pandas as pd
# import matplotlib.pyplot as plt
import os
from datetime import datetime


# This file contains a script for receiving the monitored position movements for the six motors,
# following a given trajectory. The commands are also received.
# The collected data will be saved as three seperate .csv files.


class DataCollector:
    ''' ### Description
    Class for collecting data from a contion-monitoring Ros topic.

    ### Initialization
    - base_path: Specify the path to the condition_monitoring_python_ros file.
    '''
    def __init__(self, base_path):
        self.position_data = []
        self.command_position = []
        self.command_duration = []
        self.base_path = base_path


    def position_monitoring(self, msg):
        ''' ### Description
        Method for collecting the position data. The collected data will be saved in self.position_data.

        ### Parameters
        - msg: A structure for the defined Ros message. Must be cm/msg type.
        '''
        # Get the position monitoring data.
        position = msg.position

        # Get the time stamp.
        timestamp = msg.header.stamp.to_sec()

        # Add the current message into a list.
        self.position_data.append({
            'timestamp': timestamp, 
            'motor_1': position[0],
            'motor_2': position[1],
            'motor_3': position[2],
            'motor_4': position[3],
            'motor_5': position[4],
            'motor_6': position[5]
        })


    def command_monitoring(self, msg):
        ''' ### Description
        Method for collecting the command data. The collected data will be saved in self.command_position and command_duration.

        ### Parameters
        - msg: A structure for the defined Ros message. Must be cm/msg type.
        '''
        # Get the position monitoring data.
        trajectory = msg.position
        # It is defined that when the msg is sent from the command topic, the duration of the command is 
        # saved in msg.temperature
        duration_list = msg.temperature

        # Get the time stamp.
        timestamp = msg.header.stamp.to_sec()

        # Add the current message into a list.
        self.command_position.append({
            'timestamp': timestamp, 
            'motor_1': trajectory[0],
            'motor_2': trajectory[1],
            'motor_3': trajectory[2],
            'motor_4': trajectory[3],
            'motor_5': trajectory[4],
            'motor_6': trajectory[5]
        })

        self.command_duration.append({
            'timestamp': timestamp, 
            'motor_1': duration_list[0],
            'motor_2': duration_list[1],
            'motor_3': duration_list[2],
            'motor_4': duration_list[3],
            'motor_5': duration_list[4],
            'motor_6': duration_list[5]
        })


    def run(self):
        ''' ### Description
        Initiate a ros node for collecting the data. Keep this node running until exit signals are sent from keyboard.
        Then, save the data.
        '''
        # Create Ros node.
        rospy.init_node('data_collector_node', anonymous=True)
        
        # Subscribe to the condition-monitoring and position-monitoring topics.
        sub_cm = rospy.Subscriber('condition_monitoring', RosJointState, self.position_monitoring)
        sub_pos_cmd = rospy.Subscriber('position_monitoring', RosJointState, self.command_monitoring)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down data collector node.")
            self.save_data_to_dataframe()


    def generate_data_path(self):
        ''' ### Description
        This function generate the path for saving the data under the base_path.
        It also generate the file names by logging the date and time for the current moment.

        ### Return
        - subfolder_path: Path for saving the data.        
        '''
        # Check if the folder exists, if not, create it        
        base_path = self.base_path        
        if not os.path.exists(base_path):
            os.makedirs(base_path)

        # Create a subfolder with the current time as the folder name
        current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
        subfolder_path = os.path.join(base_path, current_time)
        if not os.path.exists(subfolder_path):
            os.makedirs(subfolder_path)
        
        return subfolder_path


    def save_data_to_dataframe(self):
        ''' ### Description
        Save the collected data.
        '''

        # Generate the folder to save the data.
        path = self.generate_data_path()

        # Position monitoring data.
        if self.position_data:
            df = pd.DataFrame(self.position_data)
            df.to_csv(path + '/trajectory_monitoring_position.csv', index=False)
            print("trajectory_monitoring_position.csv'")

        # Command data.
        if self.command_position:
            df_cmd = pd.DataFrame(self.command_position)
            df_cmd.to_csv(path + '/trajectory_monitoring_cmd.csv', index=False)
            print("trajectory_monitoring_cmd.csv'")

        # Command duration.
        if self.command_duration:
            df_cmd_duration = pd.DataFrame(self.command_duration)
            df_cmd_duration.to_csv(path + '/trajectory_monitoring_cmd_duration.csv', index=False)
            print("trajectory_monitoring_cmd_duration.csv'")
         


if __name__ == '__main__':
    # Specify the base path for saving the data.
    base_path = '/home/zhiguo/github_repo/dtr_robot_digital_shaddow/pc_side/collected_data/failure_simulation_data/robot_b'
    # Initialize the data collector.
    data_collector = DataCollector(base_path=base_path)
    # Receiving the collected data.
    data_collector.run()
    # Save data.
    data_collector.save_data_to_dataframe()
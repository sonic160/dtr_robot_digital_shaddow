import numpy as np


def MotorCommand(number_of_motives_in_traj, zero_amount, allowed_amplitude):
    ''' ## Description
    This function generates a random trajectory with a given number of moves.

    ### Inputs
    - number_of_motives_in_traj: int, number of movements in a trajectory you want to generate.
    - zero_amount: float
    - allowed_amplitude: list, Defined the maximum amplitude of one movement can reach, in the robot control unit so [0, 1000] corresponds to 240 degrees.

    ### Returns
    - command_triplets: ndarray of dimension (number_of_motives+2, 3), where each row is a triplet of the form [target_position, rising_time, holding_time]
    '''
    motive_len = 2000 # Each movement lasts 2000 ms, so 2 seconds.

    # Define limit of movement based on the rotation speed of the motor.
    # From the datasheet, it is 0.272 degrees per ms.
    speed_cap_degree = .272
    speed_cap = speed_cap_degree*1000/240 # Transform into robot control units.

    # Select the amount of zeros movements in the dataset.
    percentage_zero_amount = zero_amount * 100
    toggle_value = np.random.randint(1, 101)

    # Define a command for moving the motor to a nominal position: [pos, rising_time, holding_time]
    starting_cmd = [500, 1000, 1000]

    if toggle_value >= percentage_zero_amount:
        # Initialization.
        command_triplets = np.zeros((number_of_motives_in_traj+2, 3)) # Commands.
        old_point = 0 # Index of the previous movement.

        # Start a loop for the movements.
        # A starting and an ending point will be added to the sequence later.
        for i in range(1, number_of_motives_in_traj+1):
            # Generate a random number to get the rising time, and then calculate the length of the plateau.
            # Here, we limit the rise time to be between 40% and 50% of the movement length, which corresponds to 800ms and 1000ms
            arrival_to_plateau_proportion = np.random.randint(3, 6) / 10 
            arrival_to_point_length = round(motive_len * arrival_to_plateau_proportion) # Get the rising time.           
            # Log the rising time.
            command_triplets[i, 1] = arrival_to_point_length
            if i != 0:
                old_point = starting_cmd[0] # By setting the old value to be zero, we make sure we have a initial movement to get the robot back to nominal position, which is a upstraight position.

            # Calculate the length of the plateau.    
            plateau_length = motive_len - arrival_to_point_length
            command_triplets[i, 2] = plateau_length

            # Now, generate a random movement from the previous position.
            if np.random.rand() < 0.5: # We assume random move in both directions.
                point = old_point + np.random.rand()*allowed_amplitude
            else:
                point = old_point - np.random.rand()*allowed_amplitude

            # Apply the speed cap.
            min_point = old_point - speed_cap*arrival_to_point_length
            max_point = old_point + speed_cap*arrival_to_point_length
            point = np.clip(point, min_point, max_point)

            # Log the target value.
            command_triplets[i, 0] = point
        
        end_cmd = [point, 1000, 2000] # Add an ending point, to hold the motor position for 3 seconds.
        command_triplets[0], command_triplets[-1] = starting_cmd, end_cmd
    else:
        command_triplets = np.array([[0, 0, 0]])
	
    return command_triplets

def generate_trajectory(number_of_motives_in_traj, zero_amount):
    ''' ## Description
    This function generate the commands for the six motors randomly.

    ## Inputs
    - number_of_motives_in_traj: int, number of movements in a trajectory you want to generate.
    - zero_amount: float

    ## Returns
    - cmd_triplets_list: list, list of ndarray of dimension (number_of_motives+2, 3), where each row is a triplet of the form [target_position, rising_time, holding_time].
    '''

    # Initial values.
    cmd_triplets_list = []

    # Define the allowed amplitude of each motor.
    allowed_amplitude_degree = [80, 80, 80, 80, 80, 80]
    allowed_amplitude = [x*1000/240 for x in allowed_amplitude_degree]
    danger_zone = False

    # Loop for the six motors.
    for i in range(6):
        if danger_zone:
            allowed_amplitude = [50, 50, 50, 50, 50, 70]
            allowed_amplitude = [x*1000/240 for x in allowed_amplitude]

        triplets = MotorCommand(number_of_motives_in_traj, zero_amount, allowed_amplitude[i])
        if i == 0 and -50*1000/24 < triplets[0, 0] < 50*1000/24:
            danger_zone = True

        cmd_triplets_list.append(triplets)

    return cmd_triplets_list


def generate_and_get_matrices(number_of_motives_in_traj, zero_amount=0):
    ''' ## Description
    This function is the entry point of generating a random trajectory, and format that in the matrix form required by the control algorithm of the robot.

    ## Inputs
    - number_of_motives_in_traj: int, number of movements in a trajectory you want to generate.
    - zero_amount: float    
    '''

    # Generate the trajectory for each motor.
    cmd_triplets_list = generate_trajectory(number_of_motives_in_traj, zero_amount)

    traj_matrix = np.zeros((len(cmd_triplets_list[0]), len(cmd_triplets_list)),dtype = int)
    duration_matrix = np.zeros((len(cmd_triplets_list[0]), len(cmd_triplets_list)),dtype = int)

    for motor_index, motor in enumerate(cmd_triplets_list):
        motor_points = [int((point[0])) for point in motor]
        motor_durations = [int(point[1]) for point in motor]

        for i, point in enumerate(motor_points):
            traj_matrix[i, motor_index] = round(point)
        for i, duration in enumerate(motor_durations):
            duration_matrix[i, motor_index] = int(duration)

    return traj_matrix.tolist(), duration_matrix.tolist()


def generate_n_trajs(n_trajs, number_of_motives_in_traj):
    ''' ## Description
    This function generate n trajectories, and concetenate the results into two big matrix to be implemented in the robots.

    ## Inputs:
    - n_trajs: int, number of trajectories you want to generate.
    - number_of_motives_in_traj: int, number of movements in a trajectory you want to generate.

    ## Returns:
    - trajectories: ndarray of dimension *n_trajs*number_of_motives_in_traj, 6).
    - duration_list: ndarray of the same dimension as trajectories.
    '''

    all_trajectories = []
    all_durations = []
    # Do a loop to generate trajectory by trajectory.
    for i in range(n_trajs):
        # Generate a single trajectory.
        trajectories,durations_lists = generate_and_get_matrices(number_of_motives_in_traj)
        all_trajectories.append(trajectories)
        all_durations.append(durations_lists)       
              
    trajectories = np.vstack(all_trajectories)
    durations_lists = np.vstack(all_durations)

    return trajectories, durations_lists


if __name__ == '__main__':
    # # Test: Generate one trajectory of 5 movements.    
    # trajectories,durations_lists = generate_and_get_matrices(5)
    # print("trajectories")
    # print((np.array(trajectories)).shape)
    # print(trajectories)
    # print("duration lists")
    # print(durations_lists)

    # Test: Generate n trajectories of 5 movements, and concatenate into a big matrix.
    # This is a test script for simulating failures on motors by software.

    
    # Define normal trajectories.
    number_of_trajs = 30
    number_of_movements = 5

    trajectories, durations_lists = generate_n_trajs(number_of_trajs, number_of_movements)
    
    total_rows = trajectories.shape[0]
    print("len trajectories", len(trajectories))

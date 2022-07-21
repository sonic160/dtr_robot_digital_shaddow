import numpy as np


def realisticsinglemotorcommand(max_number_of_motives, len_time_series, zero_amount, allowed_amplitude):
    speed_cap = 1

    motor_command = np.zeros(len_time_series)

    percentage_zero_amount = zero_amount * 100
    toggle_value = np.random.randint(1, 101)
    average_smallest_motive_length = len_time_series / max_number_of_motives

    if toggle_value >= percentage_zero_amount:
        number_of_motives_in_traj = max_number_of_motives
        triplets = np.zeros((number_of_motives_in_traj +2, 3))

        remaining_points = len_time_series
        current_index = 0
        old_point = 0
        for i in range(1,number_of_motives_in_traj+1):
            arrival_to_plateau_proportion = np.random.randint(2, 9) / 10
            motive_len = 200
            arrival_to_point_length = round(motive_len * arrival_to_plateau_proportion)
            triplets[i, 1] = arrival_to_point_length
            
            if i != 0:
                old_point = motor_command[current_index]
                
            plateau_length = motive_len - arrival_to_point_length
            triplets[i, 2] = plateau_length

            if np.random.rand() < 0.1:
                point = np.random.rand() * allowed_amplitude
            else:
                point = -np.random.rand() * allowed_amplitude

            min_point = old_point - speed_cap * arrival_to_point_length
            max_point = old_point + speed_cap * arrival_to_point_length

            point = np.clip(point, min_point, max_point)
            triplets[i, 0] = point

            motor_command[current_index:current_index + arrival_to_point_length] = np.linspace(old_point, point, arrival_to_point_length)
            motor_command[current_index + arrival_to_point_length:current_index + motive_len] = point

            remaining_points -= motive_len
            current_index = len_time_series - remaining_points


            triplets[0],triplets[number_of_motives_in_traj+1] = [-30,100,100],[-30,100,100]
    else:
        triplets = np.array([[0, 0, 0]])
        motor_command[0] = 0.0001
        motor_command[1] = 0.0002
	
    return motor_command, triplets

def generate_trajectory(max_number_of_motives, len_time_series, zero_amount):
    trajectory = []
    triplets_cell = []
    allowed_amplitude = [50, 50, 50, 50, 50,70]
    danger_zone = False

    for i in range(6):
        if danger_zone:
            allowed_amplitude = [50, 50, 50, 50, 50, 70]

        motor_command, triplets = realisticsinglemotorcommand(
            max_number_of_motives, len_time_series, zero_amount, allowed_amplitude[i]
        )
        if i == 0 and -50 < triplets[0, 0] < 50:
            danger_zone = True

        trajectory.append(motor_command)
        triplets_cell.append(triplets)

    return trajectory, triplets_cell
"""
def build_matrices(triplets_cell):
    traj_matrix = []
    duration_matrix = []

    for motor in triplets_cell:
        motor_points = [int((point[0] + 120)/0.24) for point in motor] 
        print(motor_points)
        motor_durations = [int(point[1]*10) for point in motor]

        traj_matrix.append(motor_points)
        duration_matrix.append(motor_durations)

    return traj_matrix, duration_matrix
"""

def build_matrices(triplets_cell):
    traj_matrix = np.zeros((len(triplets_cell[0]), len(triplets_cell)),dtype = int)
    duration_matrix = np.zeros((len(triplets_cell[0]), len(triplets_cell)),dtype = int)

    for motor_index, motor in enumerate(triplets_cell):
        motor_points = [int((point[0] + 120)/0.24) for point in motor]
        motor_durations = [int(point[1]*10) for point in motor]

        for i, point in enumerate(motor_points):
            traj_matrix[i, motor_index] = round(point)
            print(traj_matrix[i, motor_index])
        for i, duration in enumerate(motor_durations):
            duration_matrix[i, motor_index] = int(duration)

    return traj_matrix.tolist(), duration_matrix.tolist()

# Function to generate and return matrices
def generate_and_get_matrices(max_number_of_motives, len_time_series, zero_amount):
    trajectory, csv_file_equivalent = generate_trajectory(
        max_number_of_motives, len_time_series, zero_amount
    )
    return build_matrices(csv_file_equivalent)


trajectories,durations_lists = generate_and_get_matrices(8,1600,0)
print("trajectories")
print((np.array(trajectories)).shape)
print(trajectories)
print("duration lists")
print(durations_lists)

   


import sys
import random

motor_number = int(sys.argv[1])
failure_type = int(sys.argv[2])
 
 
 
failed_trajectories = sorted(random.sample(range(8),6))
  
failed_motors = [motor_number] *len(failed_trajectories) 


print(failed_trajectories)
print(failed_motors)

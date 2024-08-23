import matplotlib.pyplot as plt
import numpy as np
import math

joint_delay_arrays = np.zeros((6,16000))
time_arrays = np.zeros((6,16000))
joint_min_step_delays = np.zeros(6)
joint_max_step_delays = np.zeros(6)
joint_accel_increments = np.zeros(6)

joint_min_step_delays[0] = 100
joint_max_step_delays[0] = 300
joint_accel_increments[0] = 0.05

joint_min_step_delays[1] = 100
joint_max_step_delays[1] = 300
joint_accel_increments[1] = 0.05
    
def set_delays(num_steps, motor_num):
    
    half_steps = num_steps * 0.5
    for i in range(int(half_steps)):
        joint_delay_arrays[motor_num, i] = math.floor(max(joint_max_step_delays[motor_num] - joint_accel_increments[motor_num] * i, joint_min_step_delays[motor_num]))
        joint_delay_arrays[motor_num, num_steps - i - 1] = math.floor(max(joint_max_step_delays[motor_num] - joint_accel_increments[motor_num] * i, joint_min_step_delays[motor_num]))
    for i in range(num_steps-1):
        time_arrays[motor_num, i+1] = time_arrays[motor_num, i] + joint_delay_arrays[motor_num, i] 

def get_velocity_array(in_array):

    for i in range(len(in_array)):
        steps_per_rad = 1600 / (2 * math.pi)

num_steps_1 = 8000
set_delays(num_steps_1, 0)

num_steps_2 = 6000
set_delays(num_steps_2, 1)

fig, ax = plt.subplots()
ax.plot(time_arrays[0, 0:num_steps_1], joint_delay_arrays[0, 0:num_steps_1])
ax.plot(time_arrays[1, 0:num_steps_2], joint_delay_arrays[1, 0:num_steps_2])
ax.grid()
plt.show()



        
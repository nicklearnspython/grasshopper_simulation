"""
============
Ball Balance
============

Balance a ball at a particular height.

"""
# --- IMPORTS ---

import numpy as np
import matplotlib.pyplot as plt


# --- CONSTANTS ---

hover_time = 0.0
dt = 0.01 # [sec]
total_time = 30 # [sec]
sample_num = int(total_time / dt)
mass = 10
gravity = -9.81
#bounce_friction = 0.8


K = 3
Kp = K * 3
Ki = K * 3

desired_altitude_time = 10 # [sec]
ref_vel = 20
ref_vel_path = [ref_vel] * int(sample_num/2 + 1) + [ref_vel] * int(sample_num/2)
ref_pos = 100

t_temp = np.arange(0, total_time+dt, dt)

#ref_pos_ideal_path = ref_pos / np.log(desired_altitude_time + 1) * np.log(t_temp+1)
#ref_vel_ideal_path = ref_pos / np.log(desired_altitude_time + 1) / (t_temp + 1)

ref_pos_ideal_path = ref_pos * (1 - 1 / (1 + t_temp))
ref_vel_ideal_path = ref_pos / ((t_temp+1)**2)


test1 = ref_pos - ref_pos / (1 + t_temp/0.01)
test2 = ref_pos - ref_pos / (1 + t_temp/2)
test3 = ref_pos - ref_pos / (1 + t_temp/10)


p0 = 0.0 # [m]
v0 = 0.0 # [m/s]


# --- FUNCTIONS ---

class Velocity_Profiles:
    def __init__():
        takeoff_profile = ref_pos / np.log(desired_altitude_time + 1) / (t_temp + 1)
        hover_profile = np.zeros(int(20 / dt))
        decending_profile = ref_pos - takeoff_profile
        landing_profile = 0



def sigmoid(x):
    # Sigmoid Function
    return x / np.sqrt(1 + np.square(x)) if x / np.sqrt(1 + np.square(x)) > 1 else 1


def control_vel_thrust(ref, vel, Kp, integrator):
    # Control Velocity using a proportional and Integral Controller
    integrator += (ref - vel) * dt
    return (ref - vel) * Kp + integrator * Ki, integrator


def control_vel_path_thrust(ref, vel, Kp, integrator):
    # Control Velocity using a proportional and Integral Controller
    integrator += Ki * (ref - vel ) * dt
    return (ref - vel) * Kp + integrator, integrator


def control_vel_path_lin_pos_thrust(ref_v, vel, ref_p, pos, integrator, hover_time):
    # Control Velocity using a proportional and Integral Controller
    # Once the ball is close to the desired altitude switch to linearized 
    # control about the position
    ref_v = path_decider(ref_v, vel, ref_p, pos, hover_time)

    integrator += Ki * (ref_v - vel ) * dt
    return (ref_v - vel) * Kp + integrator, integrator, ref_v, hover_time


def path_decider(ref_v, vel, ref_p, pos, hover_time):
    # decides the velocity path for the controller to follow 
    # Input:
    # Output: ref_vel_path
    if hover_time > 10 and pos < 5:
        ref_v = 0.5 * (0 - pos)
    elif hover_time > 15:
        ref_v = -(ref_v + vel)
    elif ref_p - pos < 5:
        ref_v = 0.8 * (ref_p - pos)
        hover_time += dt
    return ref_v


def posandvel(pm1, vm1, thrust):
    # Calculate position and velocity

    v = vm1 + (thrust/mass + gravity) * dt
    p = pm1 + v * dt

    #if p < 0:
    #    p = p * -1
    #    v = v * -bounce_friction

    return p, v


def pos(pm1, vm1):
    # calculate position
    positions = 0



# --- CODE ---

deciseconds = range(sample_num)
integrator = 0.0
pos_control = 0.0

t = [0]
p = [p0]
v = [v0]
c = [0]
path = [ref_vel]

for index, decisecond in enumerate(deciseconds):

    thrust, integrator, patht, hover_time = control_vel_path_lin_pos_thrust(ref_vel_ideal_path[index], v[-1], ref_pos, p[-1], integrator, hover_time)
    
    pt, vt = posandvel(p[-1], v[-1], thrust)
    t = np.append(t, t[-1] + dt)
    v = np.append(v, vt)
    p = np.append(p, pt)  
    c = np.append(c, thrust)
    path = np.append(path, patht)



plt.plot(t, test1-test3, label="test1")
plt.plot(t, test2, label="test2")
plt.plot(t, test3, label="test3")

plt.plot(t, ref_pos_ideal_path, label="pos")
plt.plot(t, ref_vel_ideal_path, label = "vel")

"""

plt.plot(t,p,label='Position [m]')
plt.plot(t,v,label='Velocity [m/s]')
#plt.plot(t,c,label='Control Effort [N]')
plt.plot(t,c/mass,label='Control Effort / Mass [m/s^2]')
plt.plot(t,path,label='Velocity Path [m/s]')

"""

plt.xlabel("Time [s]")

plt.legend(loc='best')
plt.grid()
plt.show()



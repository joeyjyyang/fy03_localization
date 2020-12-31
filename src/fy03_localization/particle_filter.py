#!/usr/bin/env python

from matplotlib.pyplot import *
from random import *
from math import *
import scipy as scipy
import scipy.stats
from numpy import *

# Prediction step.
# Get motion data (control inputs) from IMU,
# and predict the new positions of all particles.
def predict(x_vel: float, y_vel: float, x_vals: list, y_vals: list, u: list, dt = 1.) -> list:
    linear_acceleration_x = u[0]
    linear_acceleration_y = u[1]

    linear_velocity_x = x_vel + (linear_acceleration_x * dt)
    linear_velocity_y = y_vel + (linear_acceleration_y * dt)
    
    linear_displacement_x = linear_velocity_x * dt
    linear_displacement_y = linear_velocity_y * dt

    for i in range(0, len(x_vals)):
        x_vals[i] += linear_displacement_x
        y_vals[i] += linear_displacement_y

    return linear_displacement_x, linear_displacement_y, linear_velocity_x, linear_velocity_y

def string_to_list(input_str: str):
    iterator = 0
    ret_list = []
    temp_val = ""
    while(input_str[iterator] != "\n"):
        if(input_str[iterator] == ","):
            ret_list.append(float(temp_val))
            temp_val = ""
        else:
            temp_val = temp_val + input_str[iterator]
        iterator += 1
    ret_list.append(float(temp_val))
    return ret_list

    
def random_particles(num_particles: int, x_range: tuple, y_range:tuple) -> list:
    ##make particles
    x_out = []
    y_out = []
    for i in range(0,num_particles):
        x_out.append(uniform(x_range[0], x_range[1]))
        y_out.append(uniform(y_range[0], y_range[1]))
    return x_out, y_out

def plot_particles(x_vals: list, y_vals: list, weights = 1):
    ##plot particles
    scatter(x_vals, y_vals, weights)
    show()

def find_current_estimate(x_vals: list, y_vals: list, weights: list) -> tuple:
    estimate_x = 0
    estimate_y = 0
    for i in range(0, len(x_vals)):
        estimate_x += x_vals[i]*weights[i]
        estimate_y += y_vals[i]*weights[i]
    return (estimate_x, estimate_y)

def normalize_weights(weights: list) -> list:
    total_weight = 0
    normalized_weights = []
    for i in weights:
        total_weight += i
    for j in range(0, len(weights)):
        normalized_weights.append(weights[j]/total_weight)
    return normalized_weights

def p2p_distance(point1: list, point2: tuple):
    x_diff = float(point1[0]-point2[0])
    y_diff = float(point1[1]-point2[1])
    distance = sqrt((x_diff**2) + (y_diff**2))
    return distance

def assign_weights(measurement: list, x_vals: list, y_vals: list, cov: float) -> list:
    location = 0
    weights = []
    distance = 0
    temp_weight = 0
    for i in range(0, len(x_vals)):
        distance = p2p_distance(measurement, (x_vals[i], y_vals[i]))
        #weights.append(1/(distance+cov))
        temp_weight = (scipy.stats.norm.pdf(distance, 0, cov))
        if temp_weight == 1:
            temp_weight = 0
        weights.append(temp_weight)
        weights[i] = weights[i] + 0.000000001
        
    return weights

def re_sample(x_values: list, y_values: list, normalized_weights: list, num_particles: int, cov: float):
    cum_sum = 0
    counter = 0
    x_return = []
    y_return = []
    weights_return = []
    for i in range(0, len(normalized_weights)):
        cum_sum += normalized_weights[i]*float(num_particles)
        while(counter<cum_sum):
            x_return.append(random.normal(x_values[i], cov))
            y_return.append(random.normal(y_values[i], cov))
            counter += 1
    for i in range(0, len(x_return)):
            weights_return.append(1)

    return x_return, y_return, weights_return
        
def main():
    current_pos = [5,5]
    num_particles = 10000
    accel_vector = []
    x_displace = 0
    y_displace = 0
    x_vel = 0
    y_vel = 0
    x_range = (0,10)
    y_range = (0,10)
    x_vals = []
    y_vals = []
    accel_tuple = ()
    weights = []
    current_estimate = ()
    normalized_weights = []
    UWB_covariance = 0.3
    myfile = open("traj1.txt", 'r')

    x_vals, y_vals = random_particles(num_particles, x_range, y_range)
    plot_particles(x_vals, y_vals)

    ##readfile
    accel_tuple = myfile.readline()
    print(accel_tuple)


    ##while file not at end

    for i in myfile:
        print(i)
        accel_vector = string_to_list(i)
        print(accel_vector)

        x_displace, y_displace, x_vel, y_vel = predict(x_vel, y_vel, x_vals, y_vals, accel_vector)


        current_pos[0] = current_pos[0] + x_displace
        current_pos[1] = current_pos[1] + y_displace
        ##make

        #print(x_vals, y_vals)

        ##plot
 

        weights = assign_weights(current_pos, x_vals, y_vals, UWB_covariance)
        normalized_weights = normalize_weights(weights)

        current_estimate = find_current_estimate(x_vals, y_vals, normalized_weights)
        print(str(current_estimate[0]) + ', ' + str(current_estimate[1]))

        plot_particles(x_vals, y_vals, weights)

        x_vals, y_vals, weights = re_sample(x_vals, y_vals, normalized_weights, num_particles, UWB_covariance)
        plot_particles(x_vals, y_vals, weights)

main()

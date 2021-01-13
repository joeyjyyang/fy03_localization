#!/usr/bin/env python

from matplotlib.pyplot import *
from generate_inputs import *
from matplotlib.patches import *

def predict(estimate_pos, estimate_var, measured_displacement, measured_var):
    new_pos = 0.0
    new_var = 0.0

    new_pos = estimate_pos + measured_displacement
    new_var = estimate_var + measured_var

    return [new_pos, new_var]

def update(estimate_pos, estimate_var, measured_pos, measured_var):
    new_pos = 0.0
    new_var = 0.0

    new_pos = (estimate_pos*measured_var + measured_pos*estimate_var)/(measured_var + estimate_var)
    new_var = 1/((1/estimate_var) + (1/measured_var))

    return [new_pos, new_var]

def main():
    generate()
    
    f = open("inputs.txt", "r")                             #get init values for x and y
    pos_series = []
    pos_var_series = []
    actual_series = []
    x = []
    UWB_series = []
    IMU_series = []
    UWB_error = 0
    IMU_error = 0
    reading = 1
    x_pos = 6       #float(input('init x '))                #init_val from first UWB measurement
    x_var = 2000         #float(input('init x var '))            #init_val
    #y_pos = float(input('init y '))                #init_val from first UWB measurement
    #y_var = float(input('init y var '))            #init_val

    x_displacement = float(f.readline())  
    
    while(reading):
             #float(input('x displacement '))                #get x displacement from IMU
        x_displacement_var = float(f.readline())             #float(input('x displacement var '))        #get x displacement from IMU variance
        #y_displacement = float(input('y displacement '))                #get y displacement from IMU
        #y_displacement_var = float(input('y displacement var '))        #get y displacement from IMU variance

        x_absolute = float(f.readline())                   #float(input('x absolute '))                   #get
        x_absolute_var = float(f.readline())                #float(input('x absolute var '))           #get
        #y_absolute = float(input('y absolute '))                   #get
        #y_absolute_var = float(input('y absolute var '))           #get
        
        predict_x_pos, predict_x_var = predict(x_pos, x_var, x_displacement, x_displacement_var)
        #predict_y_pos, predict_y_var = predict(y_pos, y_var, y_displacement, y_displacement_var)
        
        update_x_pos, update_x_var = update(predict_x_pos, predict_x_var, x_absolute, x_absolute_var)
        #update_y_pos, update_y_var = update(predict_y_pos, predict_y_var, y_absolute, y_absolute_var)

        x_pos = update_x_pos
        x_var = update_x_var
        #y_pos = update_y_pos
        #y_var = update_y_var

        #print('x_pos ', x_pos, 'x_var ', x_var) #, 'y_pos ', y_pos, 'y_var ', y_var)

        IMU_error += x_displacement_var
        UWB_error = x_absolute_var

        actual_series.append(x_absolute)
        x_pos += uniform(-1*(x_var/2),x_var/2)
        pos_series.append(x_pos)
        pos_var_series.append(x_var)
        x.append(1)
        IMU_series.append(IMU_error)
        UWB_series.append(UWB_error)

        next_x = f.readline()
        if(next_x == ''):
            reading = 0
        else:
           x_displacement = float(next_x)
        
        #sleep() delay until next time step                                        

    #plot
    #f = figure(1)
    #plot(pos_series)
    #plot(UWB_series)
    #plot(pos_var_series)
    #f.show()

    #g = figure(2)
    #plot(IMU_series, 'r')
    #g.show()
    
    fig, axs = subplots(2)
    axs[0].plot(pos_series, label = "Kalman Position")
    axs[0].plot(UWB_series, label = "UWB Error")
    axs[0].plot(pos_var_series, label = "Kalman Error")
    axs[0].plot(actual_series, label = "True Position")
    axs[0].legend(bbox_to_anchor=(0,1.02,1,.102), loc='lower left',
                  ncol = 3, mode='expand', borderaxespad=0)
    axs[1].plot(IMU_series, 'r', label = "IMU Error")
    axs[1].legend(bbox_to_anchor=(0,1.02,1,.102), loc='lower left',
                  ncol = 3, mode='expand', borderaxespad=0)    

    '''

    red_patch = Patch(color='red', label= 'IMU Error')
    axs[1].legend(handles=[red_patch])

    blue_patch = Patch(color='blue', label= 'Actual Position')
    axs[1].legend(handles=[red_patch])

    green_patch = Patch(color='green', label= 'Kalman Erroe')
    axs[1].legend(handles=[red_patch])

    yellow_patch = Patch(color='yellow', label= 'UWB Error')
    axs[1].legend(handles=[red_patch])

    '''

    show()
                                    
















                                            

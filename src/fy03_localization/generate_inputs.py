from random import * 

def generate():
    f = open("inputs.txt", "w")
    accel = 0
    positive = 0
    negative = 0
    vel = 0
    pos = 0
    number_inputs = int(input('How many time steps would you like to simulate?: '))
    
    for i in range(0,number_inputs):
        var_UWB = uniform(0.5,1)
        var_IMU = uniform(0.1,0.4)
        if((positive == 0) and (negative == 0)):
            accel_t = uniform(-0.02,0.02)
        elif(positive == 1):
            accel_t = uniform(-0.02,0)
        else:
            accel_t = uniform(0,0.02)
            
        if(vel > 0.05):
            positive = 1
        else:
            positive = 0

            
        if(vel < -0.05):
            negative = 1
        else:
            negative = 0

        #print(accel_t, vel)
        
        accel = accel_t
        vel += accel
        displace = (vel + 0.5*accel)
        pos = pos + displace
        f.write(str(displace) + '\n' + str(var_IMU) + '\n' + str(pos) + '\n' + str(var_UWB) + '\n')
    f.close()
        

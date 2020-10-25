#!/usr/bin/env python

import rospy, time, serial, os

serialReadLine = ""

# initialize serial port connections
serialPortDWM1001 = serial.Serial(
    port       = "/dev/ttyACM0",
    baudrate   = 115200,
    #parity = serial.PARITY_ODD
    #stopbits = serial.STOPBITS_TWO
    #bytesize = serial.SEVENBITS
)

def run():
    # allow serial port to be detected by user
    os.popen("sudo chmod 777 /dev/ttyACM0", "w")

    # close the serial port in case the previous run didn't closed it properly
    serialPortDWM1001.close()
    # sleep for one sec
    time.sleep(1)
    # open serial port
    serialPortDWM1001.open()

    # check if the serial port is opened
    if(serialPortDWM1001.isOpen()):
        rospy.loginfo("Port opened: "+ str(serialPortDWM1001.name) )
         
        # start sending commands to the board so we can initialize the board
        # reset incase previuos run didn't close properly
        serialPortDWM1001.write(b'reset')
        # send ENTER two times in order to access api
        serialPortDWM1001.write(b'\r')
        # sleep for half a second
        time.sleep(0.5)
        serialPortDWM1001.write(b'\r')
        # sleep for half second
        time.sleep(0.5)
        # send a third one - just in case
        serialPortDWM1001.write(b'\r')

        # give some time to DWM1001 to wake up
        time.sleep(2)
        # send command lec, so we can get positions is CSV format
        serialPortDWM1001.write(b'lec')
        serialPortDWM1001.write(b'\r')
        rospy.loginfo("Reading DWM1001 coordinates")
    
    else:
        rospy.loginfo("Can't open port: "+ str(serialPortDWM1001.name))
 
def publish():
    # just read everything from serial port
    serialReadLine = serialPortDWM1001.read_until()
    rospy.loginfo(serialReadLine)
    
def end(rate):
    rospy.loginfo("Quitting, and sending reset command to dev board")
    serialPortDWM1001.write(b'reset')
    serialPortDWM1001.write(b'\r')
    rate.sleep()
    
    if "reset" in serialReadLine:
        rospy.loginfo("succesfully closed ")
        serialPortDWM1001.close()

if __name__ == '__main__':
    
    rospy.init_node("fy03_localization_node")
    rate = rospy.Rate(15)
    run()
    
    try:
        while not rospy.is_shutdown():
            publish()
            rate.sleep()
   
    except rospy.ROSInterruptException:
        rospy.loginfo("end")
        end(rate)



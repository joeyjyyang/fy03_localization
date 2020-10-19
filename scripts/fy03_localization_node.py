#!/usr/bin/env python

import rospy, time, serial, os

serialReadLine = ""

def run():
    # allow serial port to be detected by user
    os.popen("sudo chmod 777 /dev/ttyACM0", "w")

    # initialize serial port connections
    serialPortDWM1001 = serial.Serial(
        port       = "/dev/ttyACM0",
	baudrate   = 115200,
	parity = serial.PARITY_ODD
	stopbits = serial.STOPBITS_TWO
	bytesize = serial.SEVENBITS
    )

    # close the serial port in case the previous run didn't closed it properly
    serialPortDWM1001.close()
    # sleep for one sec
    time.sleep(1)
    # open serial port
    serialPortDWM1001.open()

    # check if the serial port is opened
    if(serialPortDWM1001.isOpen()):
        rospy.loginfo("Port opened: "+ str(serialPortDWM1001.name) )
        ''' 
        # start sending commands to the board so we can initialize the board
        # reset incase previuos run didn't close properly
        serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
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
        serialPortDWM1001.write(DWM1001_API_COMMANDS.LEC)
        serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)'''
        rospy.loginfo("Reading DWM1001 coordinates")
        rospy.loginfo("test")
    
    else:
        rospy.loginfo("Can't open port: "+ str(serialPortDWM1001.name))
    
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node("fy03_localization_node")
    
    try:
 	run() 
    except rospy.ROSInterruptException:
        pass

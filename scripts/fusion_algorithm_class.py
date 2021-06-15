from numpy import *

class FusionAlgorithm:
	IMU_ERROR = (0.04/3)**2 # The variance of this kind of sensor, calculated from 4% max error = 3*sigma
	UWB_ERROR = (0.1/3)**2 # The variance of this kind of sensor, 3*sigma = 10 cm
	UWB_UNCERTAINTY = UWB_ERROR * 100 * 2  # UWB velocity variance is a constant

	def __init__(self):
		self.A = array([[1, 0.01], [0, 1]])
		self.B = array([[0.01*0.01*0.5], [0.01]])
		self.x_P = array([[FusionAlgorithm.IMU_ERROR, 0.01*FusionAlgorithm.IMU_ERROR], [0.01*FusionAlgorithm.IMU_ERROR, 0.0001*FusionAlgorithm.IMU_ERROR]])
		self.x_K = array([[0, 0], [0, 0]])
		self.R = array([[FusionAlgorithm.UWB_UNCERTAINTY, 14.1*FusionAlgorithm.UWB_UNCERTAINTY], [14.1*FusionAlgorithm.UWB_UNCERTAINTY, 200*FusionAlgorithm.UWB_UNCERTAINTY]])
		self.X = array([[0],[0]])
		self.y_P = array([[FusionAlgorithm.IMU_ERROR, 0.01*FusionAlgorithm.IMU_ERROR], [0.01*FusionAlgorithm.IMU_ERROR, 0.0001*FusionAlgorithm.IMU_ERROR]])
		self.y_K = array([[0, 0], [0, 0]])
		self.Y = array([[0],[0]])
		self.moving_avg_x = []
		self.moving_avg_y = [] 
    
	def take_avg(self, u):
		if len(self.moving_avg_x) < 500:
			self.moving_avg_x.append(u[0])
			self.moving_avg_y.append(u[1])
			self.IMU_data(u)
		else:
			self.moving_avg_x.pop(0)
			self.moving_avg_y.pop(0)
			self.moving_avg_x.append(u[0])
			self.moving_avg_y.append(u[1])
			filtered_x = sum(self.moving_avg_x)/len(self.moving_avg_x)
			filtered_y = sum(self.moving_avg_y)/len(self.moving_avg_y)
			self.IMU_data([filtered_x, filtered_y])
    
	# Prediction step.

	#The IMU runs 100 times per second
	#The UWB runs 10 times per second
	#The IMU runs 10 times per UWB data point

	def IMU_data(self, u, dt = 0.01):						##consider replacing dt with an actual time step from the system clock
		self.X = self.A.dot(self.X) + self.B.dot(u[0])
		self.x_P = (self.A.dot(self.x_P)).dot(self.A.transpose())

		self.Y = self.A.dot(self.Y) + self.B.dot(u[1])
		self.y_P = (self.A.dot(self.y_P)).dot(self.A.transpose())


	# happens 10 times per second
	# after 10 IMU data points

	def UWB_data(self, p_pos, pos, v, dt = 0.1):						##consider replacing dt with an actual time step from the system clock
		#current velocity = dx/dt
		#Assume the speed is a constant over 0.1s 
		UWB_vel_x = v[0]
		UWB_vel_y = v[1]
		print("IMU_vel x,y: " + str(self.X[1][0]) + ", " + str(self.Y[1][0]) + " UWB_vel x,y: " + str(UWB_vel_x) + ", " + str(UWB_vel_y) + "\n")


		x = array([[pos[0]], [UWB_vel_x]])
		y = array([[pos[1]], [UWB_vel_y]])
		self.x_K = divide(self.x_P, (self.x_P+self.R))
		self.y_K = divide(self.y_P, (self.y_P+self.R))
		self.X = self.X + self.x_K.dot(x - self.X)
		self.Y = self.Y + self.y_K.dot(y - self.Y)
		self.x_P = (identity(2) - self.x_K).dot(self.x_P)
		self.y_P = (identity(2) - self.y_K).dot(self.y_P)
		return self.X[0][0], self.Y[0][0]

		##add axis allignment step every 15-30 seconds, use x,y accel/vel from IMU and UWB displacement to allign.

from math import * 

class FusionAlgorithm:
	IMU_ERROR = (0.04/3)**2 # The variance of this kind of sensor, calculated from 4% max error = 3*sigma
	UWB_ERROR = (0.1/3)**2 # The variance of this kind of sensor, 3*sigma = 10 cm
	UWB_UNCERTAINTY = UWB_ERROR * 100 * 2  # UWB velocity variance is a constant
	def __init__(self):

		# Constants
		self.linear_velocity_x = 0 # IMU dead-reckoning linear velocity x value
		self.linear_velocity_y = 0 # IMU dead-reckoning linear velocity y value
		self.linear_velocity_x_matrix = [0] # Tracks up to 10 most recent velocity vals
		self.linear_velocity_y_matrix = [0] # Tracks up to 10 most recent velocity vals
		self.coordinate_angle_offset = 0 # Tracks up the offset angle to covert UWB coordinates to IMU ones
		self.IMU_uncertainty_x = 0 
		self.IMU_uncertainty_y = 0 
		self.IMU_uncertainty_x_matrix = [0] # Tracks up to 10 most recent velocity variances
		self.IMU_uncertainty_y_matrix = [0] # Tracks up to 10 most recent velocity variances
		self.alignment = 0 # The value controlling the frequeny of recalculating offset angle in axis alignment

	def take_avg(self, u, dt = 0.01):
		length = len(self.linear_velocity_x_matrix)
		#current velocity = previous velocity + (acceleration * time step)
		self.linear_velocity_x = self.linear_velocity_x_matrix[length-1] + u[0]*dt
		self.linear_velocity_y = self.linear_velocity_y_matrix[length-1] + u[1]*dt

		#Variance calculation
		self.IMU_uncertainty_x += dt*dt*(FusionAlgorithm.IMU_ERROR*(u[0]**2))
		self.IMU_uncertainty_y += dt*dt*(FusionAlgorithm.IMU_ERROR*(u[1]**2))

		#add current variance to list for kalman filter calculation
		self.IMU_uncertainty_x_matrix.append(self.IMU_uncertainty_x)
		self.IMU_uncertainty_y_matrix.append(self.IMU_uncertainty_y)

		#add curent velocity to list of velocities
		self.linear_velocity_x_matrix.append(self.linear_velocity_x)
		self.linear_velocity_y_matrix.append(self.linear_velocity_y)

	# Update step.
	# Get observation of position from UWB and update weights of particles.

	#Calculate the offset angle for axis alignment
	def calc_offset_angle(self, UWBx, UWBy, IMUx, IMUy):	
		theta1 = atan2(IMUy, IMUx)		
		theta2 = atan2(UWBy, UWBx)
		self.coordinate_angle_offset = theta1 - theta2

	def UWB2IMU(self, UWBx, UWBy):
		magnitude = sqrt(UWBx*UWBx + UWBy*UWBy)
		theta = atan2(UWBy, UWBx)
		new_theta = theta + self.coordinate_angle_offset
		return cos(new_theta)*magnitude, sin(new_theta)*magnitude
	
	# 1D Kalman filter that use 10 IMU values to achieve better estimation UWB value
	def kalman_filter(self, UWBx, matrix, uncertainty):
		x = UWBx
		p = FusionAlgorithm.UWB_UNCERTAINTY
		i = 0
		# Assume the uncertainty of nth IMU value in the list is n/matrix.length of the last IMU value,
		# which means ignore the previous self.IMU_uncertainty 
		for z in matrix:
			if p == 0:
				K = 0
			else: 
				K = float(p)/(float(p) + float(uncertainty[i]))
			x = x + float(K) * (z - x)
			p = float(1-K) * p 
			i += 1
		return x, p

	#happens 10 times per second
	#after 10 IMU data points

	def UWB_data(self, p_pos, pos, v, dt = 0.1):						##consider replacing dt with an actual time step from the system clock
		#current velocity = dx/dt
		#Assume the speed is a constant over 0.1s 
		UWB_vel_x = v[0]
		UWB_vel_y = v[1]

		#Axis alignment every 20 second
		if (self.alignment == 200):
			self.alignment = 0
			self.calc_offset_angle(UWB_vel_x, UWB_vel_y, self.linear_velocity_x, self.linear_velocity_y)
		self.alignment += 1

		print("IMU_vel x,y: " + str(self.linear_velocity_x) + ", " + str(self.linear_velocity_y) + " UWB_vel x,y: " + str(UWB_vel_x) + ", " + str(UWB_vel_y) + "\n")
				
		##add self correction step where last n (prob 50-100) UWB time steps are compared with IMU velocity to determine IMU velocity factor in real time



		UWB_vel_x, UWB_vel_y = self.UWB2IMU(UWB_vel_x, UWB_vel_y) 
		# Use Kalman filter
		self.linear_velocity_x, self.IMU_uncertainty_x = self.kalman_filter(UWB_vel_x, self.linear_velocity_x_matrix, self.IMU_uncertainty_x_matrix)
		self.linear_velocity_y, self.IMU_uncertainty_y = self.kalman_filter(UWB_vel_y, self.linear_velocity_y_matrix, self.IMU_uncertainty_y_matrix)
		
		#Take Average 
		# self.linear_velocity_x = (self.linear_velocity_x+UWB_vel_x)/2		##could replace with sophisticated EKF step
		# self.linear_velocity_y = (self.linear_velocity_y+UWB_vel_y)/2		##alternatively could use tuning parameters to weigh the average (e.g 0.7x+0.3y)	

		#length = len(self.linear_velocity_x_matrix)
		#x_vel_ave = 0
		#y_vel_ave = 0
		#for i in range(length):
		#	x_vel_ave = x_vel_ave + self.linear_velocity_x_matrix[i]   	##v1 * t1 + v2 * t2 + v3 * t3 = x
		#	y_vel_ave = y_vel_ave + self.linear_velocity_y_matrix[i]	##t1 = t2 = t3 = t
				##(v1 + v2 + v3)t = x
											
		
		#x_vel_ave = x_vel_ave/length
		#y_vel_ave = y_vel_ave/length

		# # Use kalman filter 
		estimate_variance_x = FusionAlgorithm.UWB_UNCERTAINTY + dt*dt * self.IMU_uncertainty_x
		estimate_variance_y = FusionAlgorithm.UWB_UNCERTAINTY + dt*dt * self.IMU_uncertainty_y
		K_x = estimate_variance_x / (estimate_variance_x + FusionAlgorithm.UWB_UNCERTAINTY)
		K_y = estimate_variance_y / (estimate_variance_y + FusionAlgorithm.UWB_UNCERTAINTY) 
		x_fused = p_pos[0] + self.linear_velocity_x*dt + K_x * (pos[0] - p_pos[0] - self.linear_velocity_x*dt)
		y_fused = p_pos[1] + self.linear_velocity_y*dt + K_y * (pos[1] - p_pos[1] - self.linear_velocity_y*dt)

		# Take arbitary numbers
		# self.x_fused = 0.5*self.x_tag + 0.5*(self.x_tag_prev + self.linear_velocity_x*dt)	##could replace with sophisticated EKF step
		# self.y_fused = 0.5*self.y_tag + 0.5*(self.y_tag_prev + self.linear_velocity_y*dt)

		#print("x: " + str(self.x_fused) + "y: " + str(self.y_fused))

		self.linear_velocity_x_matrix = [self.linear_velocity_x]
		self.linear_velocity_y_matrix = [self.linear_velocity_y]
		self.IMU_uncertainty_x_matrix = [self.IMU_uncertainty_x]
		self.IMU_uncertainty_y_matrix = [self.IMU_uncertainty_y]
       		return x_fused, y_fused

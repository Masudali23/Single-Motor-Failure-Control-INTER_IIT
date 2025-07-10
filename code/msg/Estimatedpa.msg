#Required inputs for custom INDI Controller
#Array containing values of Principal Axis(desired and current axis in FRD and NED frame)

uint64 timestamp		# time since system start (microseconds)
float32[3] n_b			# PA in inertial frame
float32[3] n_des_i		# Desired PA in NED
float32[3] n_des_b		# Desired PA in FRD
float32 thrust_design

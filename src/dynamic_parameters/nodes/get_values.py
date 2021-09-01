#!/usr/bin/python3

import rospy
import time

def main():
	while not rospy.is_shutdown():
		gains = rospy.get_param('LQR')
		N_max = gains[0]
		min_dist = gains[1]
		rc_max = gains[2]
		rc_min = gains[3]
		v_max = gains[4]
		n_ret_vel = gains[5]
		Ts= gains[6]
		q11= gains[7]
		q22 = gains[8]
		r11 = gains[9]
		#print("Received values: ", Kp, Ki, Kd)


if __name__ == "__main__":
	try:
		time.sleep(1)
		main()
	except KeyboardInterrupt:
		exit()
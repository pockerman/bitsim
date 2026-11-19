"""
Plot the true and the estimated state that extended_kalman_filter writes to state.csv, and save
the plots as x_coordinate.png, y_coordinate.png and orientation.png.

Usage: python plot.py [state.csv] [--show]
"""
import sys
import matplotlib.pyplot as plt
import csv
import math
import numpy as np

def main(filename):
	
	with open(filename, 'r', newline='') as csvfile:
		csv_file_reader = csv.reader(csvfile, delimiter=",")
		
		time = []
		x_position_true = []
		y_position_true = []
		orientation_true = []
		x_position = []
		y_position = []
		orientation = []
		for row in csv_file_reader:
			if not row[0].startswith('#'):
				try:
					time.append(float(row[0]))
					x_position_true.append(float(row[1]))
					y_position_true.append(float(row[2]))
					orientation_true.append(float(row[3]))
					x_position.append(float(row[4]))
					y_position.append(float(row[5]))
					orientation.append(float(row[6]))
				except:
					continue
		#plot_values(time, value_func)

		plt.plot(time, x_position_true)
		plt.plot(time, x_position)
		
		plt.xlabel('time')
		plt.ylabel('X position')
		plt.title('Comparison of X-coordinate')
		plt.legend(['true', 'EKF'])
		plt.savefig('x_coordinate.png')
		if SHOW:
			plt.show()
		plt.clf()

		plt.plot(time, y_position_true)
		plt.plot(time, y_position)
		
		plt.xlabel('time')
		plt.ylabel('Y position')
		plt.title('Comparison of Y-coordinate')
		plt.legend(['true', 'EKF'])
		plt.savefig('y_coordinate.png')
		if SHOW:
			plt.show()
		plt.clf()

		plt.plot(time, orientation_true)
		plt.plot(time, orientation)
		
		plt.xlabel('time')
		plt.ylabel('Orientation')
		plt.title('Comparison of Orientation')
		plt.legend(['true', 'EKF'])
		plt.savefig('orientation.png')
		if SHOW:
			plt.show()
		plt.clf()
       
		
SHOW = '--show' in sys.argv

if __name__ == '__main__':
	args = [a for a in sys.argv[1:] if a != '--show']
	main(args[0] if args else 'state.csv')
	
	



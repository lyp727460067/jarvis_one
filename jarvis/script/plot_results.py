#!/usr/bin/python
#
# Plots the results from the 2D pose graph optimization. It will draw a line
# between consecutive vertices.  The commandline expects two optional filenames:
#
#   ./plot_results.py --initial_poses optional --optimized_poses optional
#
# The files have the following format:
#   ID x y yaw_radians

import matplotlib.pyplot as plot
import numpy
import sys


poses_original = numpy.genfromtxt("/tmp/ate.txt", usecols = (0, 1))

poses_optimized = numpy.genfromtxt("/tmp/rpe.txt", usecols = (0, 1))
# poses_optimized[:,0] = poses_optimized[:,0]- poses_optimized[0,0]
# poses_original[:,0] = poses_original[:,0]- poses_original[0,0]
# Plots the results for the specified poses.
print(poses_original[:, 1])
print (str(max(poses_original[:, 1])))
index = []
for num in range(0,  len(poses_original)):
    index.append(num)
index1 = []
for num in range(0,  len(poses_optimized)):
    index1.append(num)
plot.figure()
if poses_original is not None:
  plot.plot(index, poses_original[:, 1]*1000, '-', label="ate",
            alpha=0.5, color="green")

if poses_optimized is not None:
  plot.plot(index1, poses_optimized[:, 1]*1000, '-', label="rpe",
            alpha=0.5, color="blue")

plot.axis('equal')
plot.legend()
# Show the plot and wait for the user to close.
plot.show()

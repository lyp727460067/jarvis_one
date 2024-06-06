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
import numpy as np
import sys
from optparse import OptionParser


parser = OptionParser()

parser.add_option("--proc_memlog", dest="proc_memlog",
                  default="", help="proc_memlog")
(options, args) = parser.parse_args()
# ParseFromLog()
options.proc_memlog="/home/lyp/data/0605/mem/proc_memlog.back.txt"
# Read the original and optimized poses files.
def ParseFromLog(file):
  time_start = 0.0
  delte_num  = 0
  mem_cost= []
  cpu_cost= []
  f=open(file, encoding='UTF-8')
  txt=[]
  info= ""
  for line in f:
    line_strip = line.strip()
    time_strng_index = line_strip.find("VmRSS:")
    # print(time_strng_index )
    if time_strng_index!= -1:
      time_string_index_string =  line_strip[time_strng_index:]
      time_index = time_string_index_string.find(":")
      v_string = time_string_index_string[time_index:]
      line_strip = v_string.split()
      mem_cost.append(int(line_strip[1])/104)
      # time = time_string_index_string[time_index:]
      # print(line_strip[1])

    time_strng_index1 = line.find("jarvis_pic_main")
    if time_strng_index1!= -1:
      line_strip = line_strip.split()
      pesent =  line_strip[6] 
      pesent =pesent.rstrip('%'); 
      
      cpu_cost.append(int(pesent))
      # time = time_string_index_string[time_index:]

  # print(cpu_cost)
  return mem_cost,cpu_cost 



def  MyPlot(plot,befor_ceres_time1,tilte, x_offset=0):
  mean  = sum(befor_ceres_time1)/len(befor_ceres_time1)
  info = tilte+"\n"
  info += "mean : "+str(int(mean))+"\n"
  info += "max: "+str(max(befor_ceres_time1))+"\n"
  info += "min: "+str(min(befor_ceres_time1))+"\n"

  plot.axhline(y=0,color="blue")
  xmin, xmax, ymin, ymax = plot.axis()
  plot.text(xmin+x_offset, ymax ,info, fontsize=15)
  return mean

# print("read befor")
if options.proc_memlog != '':
  mem_cost,cpu_cost = ParseFromLog(options.proc_memlog)
  index = []
  cpu_index = []
  for num in range(0,  len(mem_cost)):
    index.append(num)
  for num in range(0,  len(cpu_cost)):
    cpu_index.append(num)


  plot.plot(index, mem_cost, '-', label="mem_cost",
            alpha=1, color="green")
  MyPlot(plot,mem_cost,"mem_cost", x_offset=100)
  plot.plot(index, cpu_cost, '.', label="cpu_cost",
            alpha=1, color="red")

  MyPlot(plot,cpu_cost,"cpu_cost")

# if len(befor_ceres_time) !=0:
#   plot.plot(befor_ceres_time_durion, befor_ceres_time, '-', label="befor_ceres",
#             alpha=1, color="green")
#   mean = MyPlot(plot,befor_ceres_time,"befor_ceres")
#   plot.axhline(y=mean,color="green")

# if len(after_ceres_time) !=0:
#   plot.plot(after_ceres_time_durion, after_ceres_time, '-', label="after_ceres",
#             alpha=1, color="red")
#   mean = MyPlot(plot,after_ceres_time,"after_ceres",300)
#   plot.axhline(y=mean,color="red")






plot.axis('equal')
plot.legend()
# Show the plot and wait for the user to close.
plot.show()
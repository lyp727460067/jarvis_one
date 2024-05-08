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
parser.add_option("--befor_ceres", dest="befor_ceres_time",
                  default="", help="time befro feed stero time")
parser.add_option("--after_ceres", dest="after_ceres_time",
                  default="", help="after_ceres_time")
parser.add_option("--filter", dest="filter",
                  default="", help="filter")
(options, args) = parser.parse_args()

# Read the original and optimized poses files.
befor_ceres_time= [] 
befor_ceres_time_durion= [] 

after_ceres_time= [] 
after_ceres_time_durion= [] 

def ParseFromLog(file):
  time_start = 0.0
  delte_num  = 0
  feed_stere_time_cost= []
  feed_stere_time= []
  f=open(file, encoding='UTF-8')
  txt=[]
  info= ""
  for line in f:
    line_strip = line.strip()
    time_strng_index = line_strip.find(options.filter)
    if time_strng_index!= '-1':
      time_string_index_string =  line_strip[time_strng_index:]
      time_index = time_string_index_string.find(":")
      time = time_string_index_string[time_index+2:]
      if time =='':
        continue
      if delte_num  < 10:
        delte_num =  delte_num+1
        continue
      info= info+time
      info+=" "
      # print(time)
      feed_stere_time_cost.append(int(time)) 
      feed_stere_time.append(time_start)
      time_start= time_start+0.5
  print(info)
  return feed_stere_time_cost,feed_stere_time 


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

print("read befor")
if options.befor_ceres_time != '':
  befor_ceres_time,befor_ceres_time_durion = ParseFromLog(options.befor_ceres_time)

print("read after")
if options.after_ceres_time != '':
  after_ceres_time,after_ceres_time_durion = ParseFromLog(options.after_ceres_time)




if len(befor_ceres_time) !=0:
  plot.plot(befor_ceres_time_durion, befor_ceres_time, '-', label="befor_ceres",
            alpha=1, color="green")
  mean = MyPlot(plot,befor_ceres_time,"befor_ceres")
  plot.axhline(y=mean,color="green")

if len(after_ceres_time) !=0:
  plot.plot(after_ceres_time_durion, after_ceres_time, '-', label="after_ceres",
            alpha=1, color="red")
  mean = MyPlot(plot,after_ceres_time,"after_ceres",300)
  plot.axhline(y=mean,color="red")






plot.axis('equal')
plot.legend()
# Show the plot and wait for the user to close.
plot.show()

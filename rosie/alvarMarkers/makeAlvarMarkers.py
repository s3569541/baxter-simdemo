#!/usr/bin/python

import os
import sys

#baseval = int(sys.argv[1])

for i in range(6):
  baseval = i*10
  # generate png
  print 'generating png for cube #',i,'with id starting at',baseval
  with open('/tmp/marker.in', 'w') as f:
    def writeMarker(id,x,y):
      f.write(str(id)+'\n')
      f.write(str(x)+'\n')
      f.write(str(y)+'\n')
    writeMarker(99999,5,0) 
    writeMarker(0+baseval,20,1) 
    writeMarker(1+baseval,20,11) 
    writeMarker(2+baseval,20,21) 
    writeMarker(3+baseval,20,31) 
    writeMarker(4+baseval,10,11) 
    writeMarker(5+baseval,30,11) 
    writeMarker(99998,35,0) 
    f.write('-1')
  stream = os.popen('/opt/ros/melodic/lib/ar_track_alvar/createMarker -p < /tmp/marker.in')
  output = stream.read()
  output

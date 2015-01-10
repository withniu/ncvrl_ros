#!/usr/bin/env python

import re

if __name__ == '__main__':
  import sys
  filename = sys.argv[1]
  
  f_cam = open("camera.txt", 'w')
  f_pts = open("structure.pcd", 'w')
  
  f_pts.write('# .PCD v.7 - Point Cloud Data file format\n')
  f_pts.write('VERSION .7\n')
  f_pts.write('FIELDS x y z rgb\n')
  f_pts.write('SIZE 4 4 4 4\n')
  f_pts.write('TYPE F F F F\n')
  f_pts.write('COUNT 1 1 1 1\n')
  f_pts.write('HEIGHT 1\n')
  f_pts.write('VIEWPOINT 0 0 0 1 0 0 0\n')

  with open(filename, 'r') as f:
    f.readline()  # Skip header
    f.readline()	# Skip empty line
    data = f.readline().split()
    num_cam = int(data[0])
    for idx in range(0, num_cam):
      line = f.readline()
      data = line.split()
      #m = re.search('/(.*?).jpg', data[0])
      m = re.search('([0-9]+).jpg', data[0])   
      ts = m.group(0)
#ts = ts[1:ts.rindex('.')]
      ts = ts[:ts.rindex('.')]
      if abs(float(data[6])) < 30 and abs(float(data[7])) < 30 and abs(float(data[8])) < 30:
      	f_cam.write('%s, %s, %s, %s\n' % (ts, data[6], data[7], data[8]))
	
    f.readline() 	# Skip empty line

    data = f.readline().split()
    num_pts = int(data[0])


    f_pts.write('WIDTH %d\n' % num_pts)
    f_pts.write('POINTS %d\n' % num_pts)
    f_pts.write('DATA ascii\n')
	
    for idx in range(0, num_pts):
      line = f.readline()
      data = line.split()
      r = int(data[3])
      g = int(data[4])
      b = int(data[5])
      if abs(float(data[0])) < 30 and abs(float(data[1])) < 30 and abs(float(data[2])) < 30:
        f_pts.write('%s, %s, %s, %e\n' % (data[0], data[1], data[2], float(r << 16 | g << 8 | b)))
      else:
        f_pts.write('%s, %s, %s, %e\n' % ('0', '0', '0', float(r << 16 | g << 8 | b)))
  f_cam.close()
  f_pts.close()

#!/usr/bin/env python

import numpy
scale = 0.1354 
#117
#tf = numpy.array(((0.9237, -0.3831, -0.0086, 1.7055),
#		  (0.3823, 0.9198, 0.0882, 1.1780),
#   		  (-0.0259, -0.0848, 0.9961, 2.8442),
#		  (0, 0, 0, 1.0000)), dtype=numpy.float64)
#523
tf = numpy.array(((0.9246,   -0.3804,    0.0191,    0.3933),
		  (0.3779,    0.9225,    0.0782,    0.6388),
   		(-0.0473,   -0.0651,    0.9968,    1.5820),
		  (0, 0, 0, 1.0000)), dtype=numpy.float64)


if __name__ == '__main__':
    import sys
    filename = sys.argv[1]
    
    f_out = open(filename[:filename.rfind('.')] + '_transformed.pcd', 'w')
    
    with open(filename, 'r') as f:
        for idx in range(0, 9):
	    f_out.write(f.readline())    # Skip header
        
	line = f.readline()	# Skip empty line
	f_out.write(line)
        data = line.split()
	num_pts = int(data[1].rstrip('\n'))
	f_out.write(f.readline())	# Skip ascii
	for idx in range(0, num_pts):
	    line = f.readline()
	    data = line.split(',')

	    x = float(data[0])
	    y = float(data[1])
	    z = float(data[2])
	    rgb = float(data[3])

	    point = numpy.array((x * scale, y * scale, z * scale, 1.0), dtype=numpy.float64)
	    point_tf = numpy.dot(tf, numpy.transpose(point))

	    f_out.write('%f, %f, %f, %e\n' % (point_tf[0] / point_tf[3], point_tf[1] / point_tf[3], point_tf[2] / point_tf[3], rgb))
	
    f_out.close()

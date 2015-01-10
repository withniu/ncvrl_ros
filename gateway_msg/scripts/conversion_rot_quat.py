import numpy
from tf.transformations import quaternion_from_matrix
from tf.transformations import rotation_matrix


#R = rotation_matrix(0.123, (1, 2, 3))
#R = numpy.array(((-0.72744993, 0.01187622, 0.68605798),
#		(-0.68602504, 0.00729723, -0.72754133),
#		(-0.01364676, -0.99990285, 0.00283901)), 
#		dtype=numpy.float64)
#R = numpy.array(((-0.707, 0.0, 0.707),
#		(-0.707, 0.0, -0.707),
#		(-0.0, -1, 0.0)), 
#		dtype=numpy.float64)

# Apriltag 1000hz
R = numpy.array(((-0.72251177,  0.01262191,  0.69124339),
		(-0.69108341,  0.01502285, -0.72261887),
		(-0.01950527, -0.99980748, -0.00213139)), 
		dtype=numpy.float64)
M = numpy.identity(4)
M[:3, :3] = R

print M
q = quaternion_from_matrix(M)

print q


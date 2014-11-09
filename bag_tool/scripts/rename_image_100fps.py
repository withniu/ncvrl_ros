#!/usr/bin/env python

import glob
import re
import os

stamp_ref = 1413303981.13749303
index_ref = 1

images = glob.glob('./*.png')

for image in images:
    m = re.search('./image(.+?).png', image)
    index = int(m.group(1))
    stamp = stamp_ref + (index - index_ref) * 1.0 / 100.0
    secs = int(stamp);
    nsecs = int((stamp - int(stamp)) * 1000000000)

    filename = image.replace('image' + m.group(1), str(secs) + str(nsecs).zfill(9)) 
    
    os.rename(image, filename)   

#    print image + ':' + 'image' + m.group(1) + ':' + str(secs) + '.' + str(nsecs).zfill(9)
    



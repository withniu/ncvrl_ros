#!/usr/bin/env python

import re

if __name__ == '__main__':
  import sys
  filename_sfm = sys.argv[1]
  filename_vicon = sys.argv[2]

  f_mapping = open('mapping.txt', 'w')    

  sync_map = dict()
  with open(filename_vicon, 'r') as f:
    dataset = f.readlines()

    for line in dataset:
      data = line.split(',')
           #key = str(format(float(data[0]) * 1e9, '.0f'))
      key = data[0].replace(".", "")
      print key
      sync_map[key] = [data[2], data[3], data[4], data[5], data[6], data[7]]
#sync_map[key] = [data[2], data[3], data[4]]
    
#    print dataset

  with open(filename_sfm, 'r') as f:
    queryset = f.readlines()
#    print queryset

    for line in queryset:
      query = line.split(',')
       # print query[0]
      if query[0] in sync_map:
        data = sync_map[query[0]]
#f_mapping.write('%s,%s,%s,%s,%s,%s,%s\n' %(query[0], query[1], query[2], query[3].rstrip('\n'), data[0], data[1], data[2]))
        f_mapping.write('%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' %(query[0], query[1], query[2], query[3].rstrip('\n'), data[0], data[1], data[2], data[3], data[4], data[5]))
      else:
        print query[0] + ' failed'
        pass         
  f_mapping.close()

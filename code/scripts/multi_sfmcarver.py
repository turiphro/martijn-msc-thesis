#!/usr/bin/env python

#// Useful Python script to run sfmcarver multiple times
#// with different settings

# set variables
datasets    = ['memorial', 'sculpture1', 'car_and_wall1', 'car_and_wall2', 'phonebox1', 'lampposts_on_wall1', 'sainsburys1', 'sainsburys2', 'houses1', 'sciencepark2']
methods     = [0, 1, 2, 3]
params1     = [ [0], [0, 0.5], [0.005, 0.01, 0.05, 0.1], [0.001, 0.005, 0.01] ] # list of lists (one foreach method)
resolutions = [250, 500, 1000, 2500]

datasets = ['chess']
methods = [2]
params1 = [ [0.001, 0.005, 0.01]]

ext         = '.ot'  # .ot or .bt for binary
skip_done   = True

# paths
path_vsfm  = '../../data/vsfm'
path_pict  = '../../data/pictures'
path_carve = '../../data/carve'
path_bin   = '../build/sfmcarver'

# check files and settings before we start
import sys, os
OK = True
for d in range(len(datasets)):
  curr_vsfm = path_vsfm  + '/' + datasets[d] + '.nvm'
  curr_pict = path_pict  + '/' + datasets[d]
  if not os.path.exists(curr_vsfm):
    print '[!!] Error: path does NOT exist: ' + curr_vsfm
    OK = False
  if not os.path.exists(curr_pict):
    print '[!!] Error: path does NOT exist: ' + curr_pict
    OK = False
  for m in range(len(methods)):
    if methods[m] < 0 or methods[m] > 3:
      print '[!!] Error: invalid method: ' + str(methods[m])
      OK = False
    for r in range(len(resolutions)):
      for p1 in range(len(params1[m])):
        curr_out  = path_carve + '/' + str(methods[m]) + '/' \
                  + datasets[d] + '_' + str(resolutions[r])  \
                  + '_' + str(params1[m][p1]) + ext
        if not skip_done and os.path.exists(curr_out):
          print '[!!] Warning: path DOES already exist: ' + curr_out
          OK = False
        if resolutions[r] < 1 or resolutions[r] > 10000:
          print '[!!] Error: invalid resolution: ' + str(resolutions[r])
          OK = False
        if params1[m][p1] < 0 or params1[m][p1] > 1:
          print '[!!] Error: invalid param1: ' + str(params1[m][p1])
          OK = False

if not OK:
  sys.exit()
else:
  print "All file paths and settings OK."
  print

# run
for d in range(len(datasets)):
  for m in range(len(methods)):
    for r in range(len(resolutions)):
      for p1 in range(len(params1[m])):
        print '+++++++++++++++++++++++++++++++++++++'
        print '  DATASET:    ' + datasets[d]         + ' ('+str( d+1)+'/'+str(len(datasets))+')'
        print '  METHOD:     ' + str(methods[m])     + ' ('+str( m+1)+'/'+str(len(methods))+')'
        print '  RESOLUTION: ' + str(resolutions[r]) + ' ('+str( r+1)+'/'+str(len(resolutions))+')'
        print '  PARAM1:     ' + str(params1[m][p1]) + ' ('+str(p1+1)+'/'+str(len(params1[m]))+')'
        print '+++++++++++++++++++++++++++++++++++++'
        curr_vsfm = path_vsfm  + '/' + datasets[d] + '.nvm'
        curr_pict = path_pict  + '/' + datasets[d]
        curr_out  = path_carve + '/' + str(methods[m]) + '/' \
                  + datasets[d] + '_' + str(resolutions[r])  \
                  + '_' + str(params1[m][p1]) + ext
        command = path_bin + ' ' + curr_vsfm + ' ' + curr_pict + ' ' \
                + curr_out + ' ' + str(methods[m]) + ' ' \
                + str(resolutions[r]) + ' ' + str(params1[m][p1])
        
        if skip_done and os.path.exists(curr_out):
          print ' SKIPPING (already exists): ' + curr_out
          continue

        print '  Running: '
        print '  ' + command
        print '+++++++++++++++++++++++++++++++++++++'
        os.system(command)

print 
print 'Finished!'


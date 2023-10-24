import coptercontrol

#

# 64x64 ~4m
# 230x230 ~1m

# camera resolution
# 640x480

camera = [
  640,
  480,
]

import time

was = ""

def box_center(coords):
  c = [
    coords[0] + coords[2] / 2,
    coords[1] + coords[3] / 2,
  ]

  return c

def to_camera(point):
  camera_center = box_center([
    0, 0,
    camera[0],
    camera[1],
  ])

  diff = [
    (point[0] - camera_center[0]) / camera_center[0],
    (point[1] - camera_center[1]) / camera_center[1],
  ]

  return diff

def size_to_z(size):
  max = 220
  min = 64

  cam = 480

  inv = size / min
  z = 4 / inv

  #print(size)
  #print(inv)
  #print(z)
  
  return z / 2


# step 1. takeoff
takeoff()
rospy.loginfo('step 1. takeoff, done ')
rospy.sleep(3.0)






while 1:
  f = open('../drift.txt', 'r')
  line = f.readline()
  f.close()

  time.sleep(0.5) 

  if was == line:
    continue

  was = line

  coords = line.split('_')

  for i in range(0, 4):
    coords[i] = int(coords[i])
  
  rel = to_camera(coords)
  z = size_to_z(coords[2])

  print("depth = {}".format(z))

  target = [
    rel[0] / 10,
    z - 1,
    -rel[1] / 10
  ]

  # print(coords)
  print(target)

  curpos = get_telemetry('body')
  point = [
    curpos[0] - target[2],
    curpos[1] + target[0],
    curpos[2] + target[1]
  ]

  move_to(point[0], point[1], point[2], 0.1)
  rospy.sleep(4.0)

  if z < 2:
    landing()


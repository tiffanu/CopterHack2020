
import numpy as np
import math

local_pos = np.array([1, 1, 1])
local_rot = np.array([0, 0.0, 0.0])

def positionVectorToTranslationMatrix(vec):
  ret = [
    [1, 0, 0, vec[0]],
    [0, 1, 0, vec[1]],
    [0, 0, 1, vec[2]],
    [0, 0, 0, 1]  
  ]

  return ret

# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta) :
     
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
         
         
                     
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                 
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                     
                     
    R = np.dot(R_z, np.dot( R_y, R_x ))
 
    return R

def localToGlobal():
  
    worldCoordToLocal = positionVectorToTranslationMatrix(local_pos)
    worldCoordToLocal = np.linalg.inv(worldCoordToLocal) 
    localCoordToWorld = np.linalg.inv(worldCoordToLocal)

    worldRotToLocal = eulerAnglesToRotationMatrix(local_rot)
    rot = np.linalg.inv(worldRotToLocal)

    localRotToWorld = np.array([
      [
        [rot[0][0], rot[0][1], rot[0][2], 0],
        [rot[1][0], rot[1][1], rot[1][2], 0],
        [rot[2][0], rot[2][1], rot[2][2], 0],
        [0, 0, 0, 1]
      ],
    ])

    return localCoordToWorld.dot(localRotToWorld)
  #  return localRotToWorld.dot(localCoordToWorld)

vec = np.array([1, 1, 2, 1])
op = localToGlobal()

res = np.matmul(op, vec)

print(res)
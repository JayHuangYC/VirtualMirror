'''import cv, numpy
mat = cv.CreateMat(3, 5, cv.CV_32FC1)
cv.Set(mat, 7)
a = numpy.asarray(mat)
print a
'''

"""
These are some functions to help work with kinect camera calibration and projective
geometry. 

Tasks:
- Convert the kinect depth image to a metric 3D point cloud
- Convert the 3D point cloud to texture coordinates in the RGB image

Notes about the coordinate systems:
 There are three coordinate systems to worry about. 
 1. Kinect depth image:
    u,v,depth
    u and v are image coordinates, (0,0) is the top left corner of the image
                               (640,480) is the bottom right corner of the image
    depth is the raw 11-bit image from the kinect, where 0 is infinitely far away
      and larger numbers are closer to the camera
      (2047 indicates an error pixel)
      
 2. Kinect rgb image:
    u,v
    u and v are image coordinates (0,0) is the top left corner
                              (640,480) is the bottom right corner
                              
 3. XYZ world coordinates:
    x,y,z
    The 3D world coordinates, in meters, relative to the depth camera. 
    (0,0,0) is the camera center. 
    Negative Z values are in front of the camera, and the positive Z direction points
       towards the camera. 
    The X axis points to the right, and the Y axis points up. This is the standard 
       right-handed coordinate system used by OpenGL.
    

"""
import numpy as np


def depth2xyzuv(depth, u=None, v=None):
  """
  Return a point cloud, an Nx3 array, made by projecting the kinect depth map 
    through intrinsic / extrinsic calibration matrices
  Parameters:
    depth - comes directly from the kinect 
    u,v - are image coordinates, same size as depth (default is the original image)
  Returns:
    xyz - 3D world coordinates in meters (Nx3)
    uv - image coordinates for the RGB image (Nx3)
  
  You can provide only a portion of the depth image, or a downsampled version of
    the depth image if you want; just make sure to provide the correct coordinates
    in the u,v arguments. 
    
  Example:
    # This downsamples the depth image by 2 and then projects to metric point cloud
    u,v = mgrid[:480:2,:640:2]
    xyz,uv = depth2xyzuv(freenect.sync_get_depth()[::2,::2], u, v)
    
    # This projects only a small region of interest in the upper corner of the depth image
    u,v = mgrid[10:120,50:80]
    xyz,uv = depth2xyzuv(freenect.sync_get_depth()[v,u], u, v)
  
  if u is None or v is None:
    u,v = np.mgrid[:480,:640]
  """
  # Build a 3xN matrix of the d,u,v data
  C = np.vstack((u.flatten(), v.flatten(), depth.flatten(), 0*u.flatten()+1))

  # Project the duv matrix into xyz using xyz_matrix()
  X,Y,Z,W = np.dot(xyz_matrix(),C)
  X,Y,Z = X/W, Y/W, Z/W
  print "X=", X
  print "Y=", Y
  print "Z=", Z
  print "W=", W

  # Project the duv matrix into U,V rgb coordinates using rgb_matrix() and xyz_matrix()
  #U,V,_,W = np.dot(uv_matrix(), np.dot(xyz_matrix(),C))
  #U,V = U/W, V/W
  super_xyz = np.vstack((X,Y,Z,0*Z+1))
  print super_xyz
  
  U,V,W = np.dot(uv_matrix(), super_xyz)
  U,V = U/W, V/W
  print "W=",W
  print "U=",U
  print "V=",V
  
  xyz = np.vstack((X,Y,Z)).transpose()
  #Z<0 -> Z>0
  xyz = xyz[Z>0,:]
  
  uv = np.vstack((U,V)).transpose()    
  #Z<0 -> Z>0
  uv = uv[Z>0,:]       

  # Return both the XYZ coordinates and the UV coordinates
  print "xyz=",xyz
  print "uv=",uv
  return xyz, uv


#p.24
def uv_matrix():
  """
  Returns a matrix you can use to project XYZ coordinates (in meters) into
      U,V coordinates in the kinect RGB image
  """
  # +(0,2),-(1,2),-(2,2)
  rot = np.array([[0.99999892027314474,-0.00051357037360716667,-0.0013768434973232286], 
                  [0.00051845618194043942,0.99999356237038495,+0.0035505522069703959],
                  [0.001375011175291316,-0.0035512622063665761,+0.99999274891421563]])
  
  trans = np.array([[1.9985e-02, -7.44237e-04,-1.0916736e-02]])
  #-trans. -> +trans.
  m = np.hstack((rot, trans.transpose()))
  '''m = np.vstack((m, np.array([[0,0,0,1]])))
  KK = np.array([[520.97092069697146, 0.0, 318.40565581396697, 0],
                 [0.0, 517.85544366622719, 263.46756370601804, 0],
                 [0, 0, 0, 1],
                 [0, 0, 1, 0]])
  '''
  KK = np.array([[520.97092069697146, 0.0, 318.40565581396697],
                 [0.0, 517.85544366622719, 263.46756370601804],
                 [0, 0, 1]])
  m = np.dot(KK, (m))
  #print m.shape
  #print m
  return m

  #-(3,4)
  #p14
def xyz_matrix():
  fx = 588.5168602060173
  fy = 584.73028132692866
  a = -0.0030711
  b = 3.3309495
  cx = 320.22664144213843
  cy = 241.98395817513071
  # -(3,4), 
  mat = np.array([[-1/fx, 0, 0, cx/fx],
                  [0, -1/fy, 0, cy/fy],
                  [0,   0, 0,    1],
                  [0,   0, a,     b]])
  #print mat
  return mat



u,v = np.meshgrid(range(640),range(480))
depth = u*0+900
print "u= ",u
print "v= ",v
depth2xyzuv(depth,u,v)
'''
u,v=np.mgrid[:480,:640]
print u
print v
print " "

X,Y = np.meshgrid(range(640),range(480))
print X
print Y

'''
#image2 = Image.fromarray(a)

'''
while True: 
 mat = cv.fromarray(a) 
'''
'''
def array2cv(a):
    dtype2depth = {
        'uint8':   cv.IPL_DEPTH_8U,
        'int8':    cv.IPL_DEPTH_8S,
        'uint16':  cv.IPL_DEPTH_16U,
        'int16':   cv.IPL_DEPTH_16S,
        'int32':   cv.IPL_DEPTH_32S,
        'float32': cv.IPL_DEPTH_32F,
        'float64': cv.IPL_DEPTH_64F,
        }
    try:
        nChannels = a.shape[2]
    except:
        nChannels = 1

    cv_im = cv.CreateImageHeader((a.shape[1],a.shape[0]),dtype2depth[str(a.dtype)],nChannels)
    cv.SetData(cv_im, a.tostring(),a.dtype.itemsize*nChannels*a.shape[1])
    return cv_im

print type(b)
mat = array2cv(b)
print type(mat), cv.GetDims(mat), cv.CV_MAT_CN(cv.GetElemType(mat))
'''
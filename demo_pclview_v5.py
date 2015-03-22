import os, sys
os.chdir("C:\KinectAR\Freenect_python_wrappers")
sys.path.append(".")


import pykinectwindow as wxwindow
import numpy as np
import cv

import pylab
from OpenGL.GL import *
from OpenGL.GLU import *
import time

import freenect
import calibkinect
import Cross4Pts

# I probably need more help with these!
try: 
  TEXTURE_TARGET = GL_TEXTURE_RECTANGLE
except:
  TEXTURE_TARGET = GL_TEXTURE_RECTANGLE_ARB


if not 'win' in globals(): win = wxwindow.Window(size=(640,480))

def refresh(): win.Refresh()

if not 'rotangles' in globals(): rotangles = [0,0]
if not 'zoomdist' in globals(): zoomdist = 1
if not 'projpts' in globals(): projpts = (None, None)
if not 'rgb' in globals(): rgb = None

def create_texture():
  global rgbtex
  rgbtex = glGenTextures(1)
  glBindTexture(TEXTURE_TARGET, rgbtex)
  glTexImage2D(TEXTURE_TARGET,0,GL_RGB,640,480,0,GL_RGB,GL_UNSIGNED_BYTE,None)


if not '_mpos' in globals(): _mpos = None
@win.eventx
def EVT_LEFT_DOWN(event):
  global _mpos
  _mpos = event.Position
  
@win.eventx
def EVT_LEFT_UP(event):
  global _mpos
  _mpos = None
  
@win.eventx
def EVT_MOTION(event):
  global _mpos
  if event.LeftIsDown():
    if _mpos:
      (x,y),(mx,my) = event.Position,_mpos
      rotangles[0] += y-my
      rotangles[1] += x-mx
      refresh()    
    _mpos = event.Position


@win.eventx
def EVT_MOUSEWHEEL(event):
  global zoomdist
  dy = event.WheelRotation
  #zoomdist *= np.power(0.995, -dy)
  zoomdist *= np.power(0.5, -dy * .001)
  refresh()
  

clearcolor = [0,0,0,0]
@win.event
def on_draw():  
  if not 'rgbtex' in globals():
    create_texture()

  xyz, uv = projpts
  if xyz is None: return

  if not rgb is None:
    #rgb_ = (rgb.astype(np.float32) * 4 + 70).clip(0,255).astype(np.uint8)
    rgb_ = (rgb.astype(np.float32)).clip(0,255).astype(np.uint8)
    glBindTexture(TEXTURE_TARGET, rgbtex)
    #specify a two-dimensional texture subimage
    glTexSubImage2D(TEXTURE_TARGET, 0, 0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, rgb_);

  glClearColor(*clearcolor)
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
  glEnable(GL_DEPTH_TEST)

  # flush that stack in case it's broken from earlier
  glPushMatrix()

  glMatrixMode(GL_PROJECTION)
  glLoadIdentity()
  #set up a perspective projection matrix
  gluPerspective(20, 640/480., 0.3, 200)

  glMatrixMode(GL_MODELVIEW)
  glLoadIdentity()

  def mouse_rotate(xAngle, yAngle, zAngle):
    glRotatef(xAngle, 1.0, 0.0, 0.0);
    glRotatef(yAngle, 0.0, 1.0, 0.0);
    glRotatef(zAngle, 0.0, 0.0, 1.0);
  
  glScale(zoomdist,zoomdist,1)
  glTranslate(0, 0,-1.5)
  mouse_rotate(-rotangles[0], -rotangles[1], 0);
  glTranslate(0,0,1.5)
  #glTranslate(0, 0,-1)

  glMatrixMode(GL_TEXTURE)
  glLoadIdentity()
  glMatrixMode(GL_MODELVIEW)
  glPushMatrix()
  glVertexPointerf(xyz)
  glTexCoordPointerf(uv)

  # Draw the points
  glPointSize(2)
  glEnableClientState(GL_VERTEX_ARRAY)
  glEnableClientState(GL_TEXTURE_COORD_ARRAY)
  glEnable(TEXTURE_TARGET)
  glColor3f(1,1,1)
  glDrawElementsui(GL_POINTS, np.array(range(xyz.shape[0])))
  glDisableClientState(GL_VERTEX_ARRAY)
  glDisableClientState(GL_TEXTURE_COORD_ARRAY)
  glDisable(TEXTURE_TARGET)
  glPopMatrix()

  # Draw some axes
  if 1:
    glBegin(GL_LINES)
    glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(1,0,0)
    glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,1,0)
    glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,1)
    glEnd()
  
  if(sn4_ref is not None and head_virtual is not None):
    glBegin(GL_LINES)
    glColor3f(1,0,0); glVertex3f(sn4_ref[0,0],sn4_ref[0,1],sn4_ref[0,2]); glVertex3f(sn4_ref[1,0],sn4_ref[1,1],sn4_ref[1,2])
    glColor3f(0,1,0); glVertex3f(sn4_ref[1,0],sn4_ref[1,1],sn4_ref[1,2]); glVertex3f(sn4_ref[2,0],sn4_ref[2,1],sn4_ref[2,2])
    glColor3f(0,0,1); glVertex3f(sn4_ref[2,0],sn4_ref[2,1],sn4_ref[2,2]); glVertex3f(sn4_ref[3,0],sn4_ref[3,1],sn4_ref[3,2])
    glColor3f(1,0,1); glVertex3f(sn4_ref[3,0],sn4_ref[3,1],sn4_ref[3,2]); glVertex3f(sn4_ref[0,0],sn4_ref[0,1],sn4_ref[0,2])
    
    glColor3f(1,0,0); glVertex3f(head_virtual[0,0],head_virtual[0,1],head_virtual[0,2]); glVertex3f(sn4_ref[1,0],sn4_ref[1,1],sn4_ref[1,2])
    glColor3f(0,1,0); glVertex3f(head_virtual[0,0],head_virtual[0,1],head_virtual[0,2]); glVertex3f(sn4_ref[2,0],sn4_ref[2,1],sn4_ref[2,2])
    glColor3f(0,0,1); glVertex3f(head_virtual[0,0],head_virtual[0,1],head_virtual[0,2]); glVertex3f(sn4_ref[3,0],sn4_ref[3,1],sn4_ref[3,2])
    glColor3f(1,0,1); glVertex3f(head_virtual[0,0],head_virtual[0,1],head_virtual[0,2]); glVertex3f(sn4_ref[0,0],sn4_ref[0,1],sn4_ref[0,2])
    
    glEnd()
      

      

  # Draw only the points in the near plane
  if 0:
      inds = np.nonzero(xyz[:,2]<0.55)
      glPointSize(3)
      glColor3f(1,1,0)
      glEnableClientState(GL_VERTEX_ARRAY)
      glDrawElementsui(GL_POINTS, np.array(inds))
      glDisableClientState(GL_VERTEX_ARRAY)
      
  if 0:   
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
      glEnable(GL_BLEND)
      glColor(0.9,0.9,1.0,0.8)
      glPushMatrix()
      glTranslate(0,0,0.55)
      glScale(0.6,0.6,1)
      glBegin(GL_QUADS)
      glVertex3f(-1,-1,0); glVertex3f( 1,-1,0);
      glVertex3f( 1, 1,0); glVertex3f(-1, 1,0);
      glEnd()
      glPopMatrix()
      glDisable(GL_BLEND)
      
  glPopMatrix()


# A silly loop that shows you can busy the ipython thread while opengl runs
def playcolors():
  while 1:
    global clearcolor
    clearcolor = [np.random.random(),0,0,0]
    time.sleep(0.1)
    refresh()

# Update the point cloud from the shell or from a background thread!

def update(dt=0):
  global projpts, rgb, depth
  depth,_ = freenect.sync_get_depth()
  rgb,_ = freenect.sync_get_video()
  q = depth
  X,Y = np.meshgrid(range(640),range(480))
  # YOU CAN CHANGE THIS AND RERUN THE PROGRAM!
  # Point cloud downsampling
  d = 1
  projpts = calibkinect.depth2xyzuv(q[::d,::d],X[::d,::d],Y[::d,::d])
  refresh()
  
def update_join():
  update_on()
  try:
    _thread.join()
  except:
    update_off()
  
def update_on():
  global _updating
  if not '_updating' in globals(): _updating = False
  if _updating: return
  
  _updating = True
  from threading import Thread
  global _thread
  def _run():
    while _updating:
      update()
  _thread = Thread(target=_run)
  _thread.start()
  
def update_off():
  global _updating
  _updating = False
  

#==array2cv==================================================#
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
#==array2cv==================================================#







#Mouse click
pt = None
#mr3: store 3 scene points for mirror 
mr3 = None
#sn4: store 4 scene points for screen in the mirror 
sn4 = None
#sn4_ref:  actual scene points for screen reflected by mirror     
sn4_ref = None
#scene4_cross: The projection line will cross 4 pts on the scene
scene4_cross = None                                                  
#head_pos: the view position E
head_pos = None
head_virtual = None

Calibration_mode = 1
Virtual_Mirror_mode = 0
Mapping_mode = 2
mode = Mapping_mode # Calibration_mode # Virtual_Mirror_mode  

if mode == Calibration_mode:
    print "Calibrate parameter!"
elif mode == Virtual_Mirror_mode:
    print "Use predefined parameter!"
    sn4_ref = np.array([[ 0.06936537, -0.12935865,  0.06853867],
                        [-0.18968564, -0.1270214,   0.05850861],
                        [-0.19571873, -0.28810189,  0.11189724],
                        [ 0.07097422, -0.28530721,  0.1143399 ]])
else:
    print "Start with Mapping Range image with Camera View" 
    
#please select a tracking mode
click = 0
nose = 1
camera = 2
tracking = nose #click #camera #  # camera#

def loopcv():
    cv.NamedWindow("Depth_Map", cv.CV_WINDOW_AUTOSIZE)
    cv.SetMouseCallback( "Depth_Map", on_mouse, None)
    print "Execute loopcv: click 3 pts on mirror, click 4 pts at screen corner"
    global mr3
    global sn4
    global sn4_ref 
    
    while 1:
        (depth,_) = freenect.sync_get_depth()
        im = array2cv(depth.astype(np.uint8))
        
        #[warp]add rgb as img
        
        if pt is not None:
            print "=================="
            (x_d,y_d) = pt
            print "x=",x_d, " ,y=",y_d
            #print depth.shape
            #Watch out the indexing for depth col,row = 480,640
            d_raw = np.array([depth[y_d,x_d]])
            u_d = np.array([x_d])
            v_d = np.array([y_d])
        
            print "d_raw= ", d_raw
            print "u_d= ", u_d 
            print "v_d= ", v_d 
            xyz,uv = calibkinect.depth2xyzuv(d_raw,u_d,v_d)
            print "XYZ=", xyz
            print "XYZonRGBplane=", uv
            cv.WaitKey(100)   
            cv.Circle(im, (x_d,y_d), 4, (0, 0, 255, 0), -1, 8, 0)
            cv.Circle(im, (int(uv[0,0]),int(uv[0,1])), 2, (255, 255, 255, 0), -1, 8, 0)
            
            if(mr3 is None):
                mr3=xyz
                #print mr3
            
            elif(mr3.shape[0] <= 2): #append "2"+1= 3pts
                mr3=np.append(mr3,xyz,axis=0)
                #print "append mr3=",mr3 
                if(mr3.shape[0] == 3):
                    print "enough for mirror, click on screen"
                    #print "mr3.shape= ",mr3.shape
                        
            elif(mr3.shape[0] == 3):
                if(sn4 is None):
                    sn4     = xyz
                    sn4_ref = MirrorReflection(mr3,xyz[0])
                    
                elif(sn4.shape[0] <= 3): #append "3"+1= 4pts
                    sn4     = np.append(sn4,     xyz,                   axis=0)
                    sn4_ref = np.append(sn4_ref, MirrorReflection(mr3,xyz[0]), axis=0)
                    
                    
                    if(sn4.shape[0] == 4):#sn4 have 4 pts actually
                        print "Total screen pts before reflection=" ,sn4
                        #print "sn4 shape= ",sn4.shape
                        print "Total screen pts after reflection="  ,sn4_ref
                        #print "sn4_ref shape= ",sn4_ref.shape
                        if(mr3.shape[0]==3 and sn4.shape[0]==4):
                            print "go into Real Game: Virtual Mirror mode"
                            #print "mr3= ", mr3
                            #print "sn4_ref[0:3,:]= ", sn4_ref[0:3,:]
            else:
                print "..."

            
        #for (x,y) in feat:
        #print x, y, velx[y,x], vely[y,x]
        cv.ShowImage("Depth_Map",im)
        if cv.WaitKey(10)==27:
            print "screen size after calibration: "
            print "Corner 1-2 = ", np.linalg.norm(sn4_ref[0]-sn4_ref[1])
            print "Corner 2-3 = ", np.linalg.norm(sn4_ref[1]-sn4_ref[2])
            print "Corner 3-4 = ", np.linalg.norm(sn4_ref[2]-sn4_ref[3])
            print "Corner 4-1 = ", np.linalg.norm(sn4_ref[3]-sn4_ref[0])
            break
    #update()
    if(sn4_ref is not None):
        cv.DestroyWindow("Depth_Map")
        VirtualMirror()
        

    
    

    
def on_mouse(event, x, y, flags, param):  
    global pt
    if event == cv.CV_EVENT_LBUTTONDOWN:
        pt = (x, y)
        #print "pt= ",pt
 
    if event == cv.CV_EVENT_LBUTTONUP:
        pt = None
        

#inputs: 1 pt for reflection, 3 pts for mirror planar 
#outputs: 1 reflective mirror pts 
def MirrorReflection( mirror, pt_for_ref): 
    #print "==="
    #print "sn4 be reflected to sn4_ref"
    
    #print "mirror[0].shape= ",mirror[0].shape
    #print "pt_for_ref.shape= ",pt_for_ref.shape
    
    norm_vec = np.cross(mirror[2]-mirror[1],mirror[0]-mirror[1])
    #print "norm_vec= ", norm_vec
    #print norm_vec.size
    if (norm_vec[2]<0):
        norm_vec=-norm_vec
        #print "keep Z positive, norm_vec= ", norm_vec
         
    #print "mr3pts[0]= ",mirror[0]
    #dis = np.dot(norm_vec,(pt_for_ref[0]-mirror[0]))/np.linalg.norm(norm_vec)
    dis = np.dot(norm_vec,(pt_for_ref-mirror[0]))/np.linalg.norm(norm_vec)
    
    dis = np.absolute(dis)
    
    #print "distance= ", dis
    
    #pt_aft_ref = pt_for_ref[0] - 2*dis*norm_vec/np.linalg.norm(norm_vec)
    pt_aft_ref = pt_for_ref - 2*dis*norm_vec/np.linalg.norm(norm_vec)
    
    #print "pt_aft_ref= ", pt_aft_ref #print pt_aft_ref.shape
    #print "==="
    return np.array([pt_aft_ref])

# contend: Head_detect, MirrorReflection, CrossPts
def VirtualMirror():
    cv.NamedWindow("RGB_remap",cv.CV_WINDOW_NORMAL)
    cv.NamedWindow("Depth_remap",cv.CV_WINDOW_AUTOSIZE)
    cv.NamedWindow('dst', cv.CV_WINDOW_NORMAL)
    cv.SetMouseCallback( "Depth_remap", on_mouse, None)
    print "Virtual Mirror"
    print "Calibrated 4 Screen corner= ", sn4_ref
    print "Corner 1-2 = ", np.linalg.norm(sn4_ref[0]-sn4_ref[1])
    print "Corner 2-3 = ", np.linalg.norm(sn4_ref[1]-sn4_ref[2])
    print "Corner 3-4 = ", np.linalg.norm(sn4_ref[2]-sn4_ref[3])
    print "Corner 4-1 = ", np.linalg.norm(sn4_ref[3]-sn4_ref[0])
    global head_pos
    global head_virtual
    global scene4_cross
    head_pos = np.array([-0.2,-0.2,1.0]) #Head_detect()
    
    while 1:
        (depth,_) = freenect.sync_get_depth()
        (rgb,_)   = freenect.sync_get_video()
        #print type(depth)
        img = array2cv(rgb[:,:,::-1])
        im = array2cv(depth.astype(np.uint8))
        #modulize this part for update_on() and loopcv()
        #q = depth
        X,Y = np.meshgrid(range(640),range(480))
        d = 2 #downsampling if need
        projpts = calibkinect.depth2xyzuv(depth[::d,::d],X[::d,::d],Y[::d,::d])
        xyz,uv = projpts
        
        if tracking == 0:
            #*********************************
            if pt is not None:
                print "=================="
                (x_d,y_d) = pt
                print "x=",x_d, " ,y=",y_d
                #print depth.shape
                #Watch out the indexing for depth col,row = 480,640
                d_raw = np.array([depth[y_d,x_d]])
                u_d = np.array([x_d])
                v_d = np.array([y_d])
            
                print "d_raw= ", d_raw
                print "u_d= ", u_d 
                print "v_d= ", v_d 
                head3D,head2D = calibkinect.depth2xyzuv(d_raw,u_d,v_d)
                print "XYZ=", head3D
                print "XYZonRGBplane=", head2D
             
                head_pos = head3D[0]
                #print "head_pos.shape",head_pos.shape
                print "head_pos= ",head_pos
                cv.WaitKey(100) 
                cv.Circle(im, (x_d,y_d), 4, (0, 0, 255, 0), -1, 8, 0)
                cv.Circle(im, (int(head2D[0,0]),int(head2D[0,1])), 2, (255, 255, 255, 0), -1, 8, 0)
    


            #*********************************
        elif tracking == 1:
            #find the nearest point (nose) as reference for right eye position
            print "nose"
            inds = np.nonzero(xyz[:,2]>0.5)
            #print xyz.shape
            new_xyz = xyz[inds]
            #print new_xyz.shape
            close_ind = np.argmin(new_xyz[:,2])
            head_pos =  new_xyz[close_ind,:]+(0.03,0.04,0.01)
            #print head_pos.shape
            #print head_pos
            
        elif tracking == 2:
            #find the closest point as eye posiiton
            print "camera"
            inds = np.nonzero(xyz[:,2]>0.5)
            #print xyz.shape
            new_xyz = xyz[inds]
            #print new_xyz.shape
            close_ind = np.argmin(new_xyz[:,2])
            head_pos =  new_xyz[close_ind,:]
            #print head_pos.shape
            #print head_pos
               
        else:
            print "please select a tracking mode"
            
        head_virtual = MirrorReflection (sn4_ref[0:3,:],head_pos)
        print "head_virtual= ",head_virtual
        
        rgbK = np.array([[520.97092069697146,0.0,318.40565581396697], [0.0,517.85544366622719,263.46756370601804], [0.0,0.0,1.0]])      
        rgbD = np.array([[0.22464481251757576],[-0.47968370787671893],[0.0],[0.0]])
        irK = np.array([[588.51686020601733,0.0,320.22664144213843], [0.0,584.73028132692866,241.98395817513071], [0.0,0.0,1.0]])
        irD = np.array([[-0.1273506872313161], [0.36672476189160591], [0.0], [0.0]])
        
        mapu = cv.CreateImage((640, 480), cv.IPL_DEPTH_32F, 1)
        mapv = cv.CreateImage((640, 480), cv.IPL_DEPTH_32F, 1)
        mapx = cv.CreateImage((640, 480), cv.IPL_DEPTH_32F, 1)
        mapy = cv.CreateImage((640, 480), cv.IPL_DEPTH_32F, 1)
        
        cv.InitUndistortMap(rgbK, rgbD, mapu, mapv)
        cv.InitUndistortMap(irK, irD, mapx, mapy)
    
        if 1:
            rgb_remap = cv.CloneImage(img)
            cv.Remap(img, rgb_remap, mapu, mapv)
            
            depth_remap = cv.CloneImage(im)
            cv.Remap(im, depth_remap, mapx, mapy)
            

         
        scene4_cross = Cross4Pts.CrossPts(xyz, uv, head_pos, head_virtual ,sn4_ref)        
        #[warp] Add whole warpping code here
        #[warp] points = Scene4Pts() as warpping 4 pts
        #Flip the dst image!!!!!!!!!
        #ShowImage("rgb_warp", dst)
       
        #Within/out of the rgb range
        #Mapping Destination (width, height)=(x,y)
        
        
        #Warning: the order of pts in clockwise: pt1(L-T),pt2(R-T),pt3(R-B),pt4(L-B)
        #points = [(test[0,0],test[0,1]), (630.,300.), (700.,500.), (400.,470.)]
        points = [(scene4_cross[0,0],scene4_cross[0,1]), 
                  (scene4_cross[1,0],scene4_cross[1,1]), 
                  (scene4_cross[2,0],scene4_cross[2,1]), 
                  (scene4_cross[3,0],scene4_cross[3,1])] 
        #Warping the image without flipping (camera image)
        #npoints  = [(0.,0.), (640.,0.), (640.,480.), (0.,480.)]            
        #Warping the image with flipping (mirror flip image)
        npoints  = [(640.,0.), 
                    (0.,0.), 
                    (0.,480.), 
                    (640.,480.)]            
        mat = cv.CreateMat(3, 3, cv.CV_32FC1)
        cv.GetPerspectiveTransform( points, npoints, mat);
    
        #src = cv.CreateImage( cv.GetSize(img), cv.IPL_DEPTH_32F, 3 )
        src = cv.CreateImage( cv.GetSize(rgb_remap), cv.IPL_DEPTH_32F, 3 )
        #cv.ConvertScale(img,src,(1/255.00))
        cv.ConvertScale(rgb_remap,src,(1/255.00))
        
        dst = cv.CloneImage( src );
        cv.Zero(dst);     
        cv.WarpPerspective(src, dst, mat);
        #************************************************************************
    
        #Remap the rgb and depth image
        #Warping will use remap rgb image as src
                
        if 1:
            cv.ShowImage("RGB_remap",rgb_remap)  #rgb[200:440,300:600,::-1]
            cv.ShowImage("Depth_remap",depth_remap)
            cv.ShowImage("dst", dst) #warp rgb image
           
        
        if cv.WaitKey(5)==27:
            cv.DestroyWindow("RGB_remap")
            cv.DestroyWindow("Depth_remap")
            cv.DestroyWindow("dst")
            break

    
    
def Head_detect():
    print "return head position"    

#update() 
#update_on()

if mode == 1:
    loopcv()
elif mode == 0:
    VirtualMirror()
elif mode == 2:
    update()

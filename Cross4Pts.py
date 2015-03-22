#!/usr/bin/env python

import os, sys, platform
os.chdir("C:\KinectAR\Freenect_python_wrappers")
sys.path.append(".")


#new
import calibkinect

from freenect import sync_get_depth as get_depth, sync_get_video as get_video
import cv  
import numpy as np


#==array2cv==================================================#
def array2cv(a):
    #a=np.array(a,'f')
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

    
    #print "shape row,col", a.shape[0],",", a.shape[1]
    cv_im = cv.CreateImageHeader((a.shape[1],a.shape[0]),dtype2depth[str(a.dtype)],nChannels)
    #cv_im = cv.CreateMatHeader(a.shape[0],a.shape[1],cv.CV_32FC1)
    cv.SetData(cv_im, a.tostring(),a.dtype.itemsize*nChannels*a.shape[1])
    #cv.SetData(cv_im, a.tostring(),4*nChannels*a.shape[1])
    return cv_im
#==array2cv==================================================#





def CrossPts(xyz_pt, uv_pt, h_pt,vh_pt,sn4_ref_pt):
    #print "return scene4_cross"
    #4 projection line
    Id3 = np.identity(3) 

    #Watch out the projection line origin is the head_virtual
    #Unit vector from sn4_ref to head_virual
    
    #unit_sn_vh = (sn4_ref_pt[0]-vh_pt)
    unit_sn_vh = (sn4_ref_pt - vh_pt)
    for i in range(4):
        unit_sn_vh[i,:] = unit_sn_vh[i,:]/np.linalg.norm(unit_sn_vh[i,:])
        #print "unit_sn_vh[i,:]= ",unit_sn_vh[i,:]

    scene4_cross = np.ndarray([4,2])    # cross rgb image
    scene4 = np.ndarray([4,3])          # cross depth map 
    
    Realtime_mode = 0
    Precise_mode = 1
    Mode = Realtime_mode # Precise_mode #
      
    if Mode:
        len,_ = xyz_pt.shape
        #xyz_pt = xyz_pt[:,:,:]
        #why = xyz_pt[:,2].argmin(0)
        #print "closest pt= ", xyz_pt[why,:]
        #len,_ = xyz_pt.shape
        dis_xyz = np.matrix(np.ndarray([len,12]))
        #print "1= ",dis_xyz.shape
        
        #P.11/60 Point to line distance 
        #unit_sn_vh = unit_sn_vh/np.linalg.norm(unit_sn_vh)
        #dis_xyz = (np.dot((Id3 - unit_sn_vh*unit_sn_vh.T),((xyz_pt[:,:]-vh_pt).T))).T
        
        proj_line = (xyz_pt[:,:] - vh_pt).T    
        #print "4= ",proj_line.shape
        print "proj_line= ",proj_line
        
        
        #algorithm to find the closest point with minimized distance
        for i in range(4):
            u = (np.asmatrix(unit_sn_vh[i,:])).T
            uuT = np.dot(u,u.T) 
            #print "u*u.T= ", uuT
            #print dis_xyz[:,3*(i):3*(i+1)].shape
            #print (np.dot((Id3 - uuT), proj_line)).shape
            dis_xyz[:,3*(i):3*(i+1)] = (np.dot((Id3 - uuT), proj_line)).T
        
        #Get the real distance (error) by using norm (Euclidean, sum of square root)
        dis_err = np.matrix(np.ndarray([len,4]))
        for j in range(len):
            dis_err[j,0]=np.linalg.norm(dis_xyz[j,0:3])
            dis_err[j,1]=np.linalg.norm(dis_xyz[j,3:6])
            dis_err[j,2]=np.linalg.norm(dis_xyz[j,6:9])
            dis_err[j,3]=np.linalg.norm(dis_xyz[j,9:12])
    
        # for each column: the row index of the minimum value
        ind = dis_err.argmin(0) 
        #scene4_cross = np.ndarray([4,2])    # cross rgb image
        #scene4 = np.ndarray([4,3])          # cross depth map 
        
        for i in range(4):        
            #print xyz_pt[ind[0,i],:]
            scene4[i,:] = xyz_pt[ind[0,i],:]
            #print uv[ind[0,i],:]
            scene4_cross[i,:] = uv_pt[ind[0,i],:]
            #print scene4_cross[i,:]
        
        #**************************************************
        #Check if scene point direction match proj_line
        #If not match, proj_line must be out of image range (show "warning message")
        #Use other point's depth(Z) for calculating XY on the proj_line
        #But XYZ not on depth map, it will use calibkinect.uv_matrix()*"XYZ1" to map back to uv      
        unit_scene4_vh = (scene4 - vh_pt)
        diff_angle = np.ndarray([4,1])
    
        for i in range(4):
            unit_scene4_vh[i,:] = unit_scene4_vh[i,:]/np.linalg.norm(unit_scene4_vh[i,:])
            diff_angle[i,0] = np.dot(unit_scene4_vh[i,:],unit_sn_vh[i,:])
            diff_angle[i,0] = (np.arccos(diff_angle[i,0]))*180/np.pi
            print "Point",i,"out of proj_line angle= ",diff_angle[i,0]
            #Parameter for detection angle difference(not radius)
            if (diff_angle[i,0] > 1): 
                print "Line ",i,": your virtual mirror image is out of camera range!!!"
            
        for i in range (4):
            if (diff_angle[i,0] > 1): 
                good_ref = diff_angle.argmin(0)
                #print "good_ref= ",good_ref
                good_proj = scene4[good_ref,:] - vh_pt
                ratio = good_proj[0,2]/unit_sn_vh[i,2]
                good_scene_pt = (unit_sn_vh[i,:])*ratio + vh_pt
                '''
                #Verification****************
                a = good_scene_pt - vh_pt
                a = a/np.linalg.norm(a)
                b = np.dot(a,unit_sn_vh[i,:])
                b = (np.arccos(b))*180/np.pi
                print "b should be very small or 0", b
                #********************************
                '''
                super_xyz = np.vstack((good_scene_pt.T,0))
                #print super_xyz
                U,V,W = np.dot(calibkinect.uv_matrix(), super_xyz)
                U,V = U/W, V/W
                #print "Old_scene4_cross[i,:]= ",scene4_cross[i,:]
                scene4_cross[i,:] = np.vstack((U,V)).transpose()
                #print "New_scene4_cross[i,:]= ",scene4_cross[i,:]     
            
            
        #**************************************************
    else:
        print "Go into real-time mode, head_position= ", h_pt

        for i in range (4):
            #print "*************************"
            #good_ref = diff_angle.argmin(0)
            #print "good_ref= ",good_ref
            good_proj = h_pt - vh_pt
            ratio = good_proj[0,2]/unit_sn_vh[i,2]
            good_scene_pt = (unit_sn_vh[i,:])*ratio + vh_pt
            scene4[i,:] = good_scene_pt
            #print "scene4[i,:]= ",scene4[i,:]
            '''
            #Verification****************
            a = good_scene_pt - vh_pt
            a = a/np.linalg.norm(a)
            b = np.dot(a,unit_sn_vh[i,:])
            b = (np.arccos(b))*180/np.pi
            print "b should be very small or 0", b
            #********************************
            '''
            super_xyz = np.vstack((good_scene_pt.T,0))
            #print super_xyz
            U,V,W = np.dot(calibkinect.uv_matrix(), super_xyz)
            U,V = U/W, V/W
            #print "Old_scene4_cross[i,:]= ",scene4_cross[i,:]
            scene4_cross[i,:] = np.vstack((U,V)).transpose()
            #print "New_scene4_cross[i,:]= ",scene4_cross[i,:]     
            #print "*************************"
                    
    #print "scene4_cross= ",scene4_cross
    #print "============================="
    return scene4_cross
    
    #Optional: return 4 pts on the rgb image "uv_matrix()"
    #calibkienct: Line 75-78
    #Optional: set normal depth for whole scene
    



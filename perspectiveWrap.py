#!/usr/bin/env python
import cv
import math

cv.NamedWindow('original', cv.CV_WINDOW_NORMAL)
cv.NamedWindow('src', cv.CV_WINDOW_NORMAL)
cv.NamedWindow('dst', cv.CV_WINDOW_NORMAL)
img = cv.LoadImageM("color2.png", cv.CV_LOAD_IMAGE_COLOR)
width, height = cv.GetSize(img)

mat = cv.CreateMat(3, 3, cv.CV_32FC1)
#mat = cv.CreateMat(3, 3, cv.IPL_DEPTH_8U)
gray = cv.CreateImage( (width,height), cv.IPL_DEPTH_8U, 1 )        
cv.CvtColor( img, gray, cv.CV_RGB2GRAY );

#points  = [(50.,50.), (50.,100.), (100.,100.), (100.,50.)]            
#npoints = [(20.,30.), (30.,150.), (160.,170.), (200.,20.)]

#points = [(20.,30.), (30.,150.), (160.,170.), (200.,20.)]
#npoints  = [(50.,50.), (50.,100.), (100.,100.), (100.,50.)]            

points = [(20.,30.), (30.,150.), (160.,170.), (200.,20.)]
npoints  = [(0.,0.), (640.,0), (640.,480.), (640.,480.)]            

cv.GetPerspectiveTransform( points, npoints, mat);
#cv.CvtColor( img, gray, cv.CV_RGB2GRAY );

src = cv.CreateImage( cv.GetSize(img), cv.IPL_DEPTH_32F, 3 )
#fimg = cv.CreateImage( cv.GetSize(img), cv.IPL_DEPTH_8U, 3 )

cv.ConvertScale(img,src,(1/255.00))
#cv.ConvertScale(gray,src,(1/255.00))

dst = cv.CloneImage( src );
cv.Zero(dst);

cv.WarpPerspective(src, dst, mat);

while 1:
    cv.ShowImage("original", img)
    cv.ShowImage("src", src)
    cv.ShowImage("dst", dst)
    if cv.WaitKey() == 27:
        break;


#import optparse
#
#    
#    
#def main( files ):
#    print "main"
#    while True:
#        file = files.pop()
#        
#        img = cv.LoadImage( file )
#        
#        cv.ShowImage( 'x', img )
#        
#        
#        files.insert(0, file);
#        
#        key = cv.WaitKey(100) % 0x100
#        
#        if key == 27:
#            break;
#
#if __name__=="__main__":
#    parser = optparse.OptionParser("<source>");
#    (_, files) = parser.parse_args()
#    if not len(files):
#        parser.error("No files found matching %s" % str(files))
#        
#    files.reverse()
#        
#    main(files)
#
'''
cv.NamedWindow('yes', cv.CV_WINDOW_NORMAL)
cam = cv.CaptureFromCAM(0);

cv.SetCaptureProperty(cam, cv.CV_CAP_PROP_FRAME_WIDTH, 640);
cv.SetCaptureProperty(cam, cv.CV_CAP_PROP_FRAME_HEIGHT, 480);
    

  
if True:
    while True:
        img = cv.QueryFrame(cam)
  
        width, height = cv.GetSize(img)
        print width," ", height
        checkers = (10,7) #(3,3)
  
        gray     = cv.CreateImage( (width,height), cv.IPL_DEPTH_8U, 1 )        
        cv.CvtColor( img, gray, cv.CV_RGB2GRAY );
  
        found_all, corners = cv.FindChessboardCorners( gray, checkers )
  
        if found_all:
            cv.DrawChessboardCorners( img, checkers, corners, found_all )
            n = checkers[1];
  
            points = [corners[0], corners[n-1], corners[-1], corners[-(n)]]            
            cv.PolyLine( img, [points], False, (255,255,0), 2, 8)
            print points
            npoints = [(x, y+50) for x, y in points]
  
            #ys = [y for x,y in points]
            #
            #origin = points[ys.index(min(ys))]
            #points = [(x-origin[0], y-origin[1]) for x,y in points]
            #
            #size = max([math.sqrt(math.pow(x,2)+math.pow(y,2)) for x,y in points])
            #
            #
            #npoints = [(0,0),(size,0),(size,size),(0,size)]
            #npoints = [(x+origin[0], y+origin[1]) for x,y in npoints]
  
            #npoints = points;
            #
            #npoints[1] = (npoints[1][0], npoints[0][1])
            #npoints[2] = (npoints[1][0], npoints[3][1])
            #npoints[3] = (npoints[0][0], npoints[3][1])
  
            cv.PolyLine( img, [npoints], False, (255,0,0), 2, 8)
  
            mat = cv.CreateMat(3, 3, cv.CV_32FC1)
            #mat = cv.CreateMat(3, 3, cv.IPL_DEPTH_8U)
  
            #npoints = [(x,y/2) for x,y in points]
  
            # npoints = [(0,0),(0,1),(1,1),(1,0)]
  
            #cv.GetPerspectiveTransform( points, npoints, mat);
  
            #cv.GetPerspectiveTransform( points, npoints, mat);
            cv.GetPerspectiveTransform( points, npoints, mat);
  
  
            #src = cv.CreateImage( cv.GetSize(img), cv.IPL_DEPTH_32F, 3 )
            #gray = cv.CreateImage( cv.GetSize(img), cv.IPL_DEPTH_8U, 1)
  
            cv.CvtColor( img, gray, cv.CV_RGB2GRAY );
  
            src = cv.CreateImage( cv.GetSize(img), cv.IPL_DEPTH_32F, 3 )
            #fimg = cv.CreateImage( cv.GetSize(img), cv.IPL_DEPTH_8U, 3 )
  
            cv.ConvertScale(img,src,(1/255.00))
            #cv.ConvertScale(gray,src,(1/255.00))
  
            dst = cv.CloneImage( src );
            cv.Zero(dst);
  
            cv.WarpPerspective(src, dst, mat);
  
            cv.ShowImage( "test",dst );
  
  
            #
            #
            #
            ##cv.PolyLine( img, [[(0,0), (100,0), (100, 100)]], False, (255,0,0), 2, 8)
            #
            #
            #
            #
            #npoints = []
            #
            #for x,y in points:
            #    cv.Line( img, (origin), (x+origin[0],y+origin[1]), (255,0,255), 1, 8 );
            #    
            #    if x != 0 and y != 0:
            #        angle = math.atan(x/y)
            #        
            #       
            #        #angle = math.radians(45) + angle;
            #        #
            #        #c = math.sqrt(math.pow(x,2) + math.pow(y,2))
            #        #
            #        #y = math.sin(angle) * c;
            #        #x = math.cos(angle) * c;
            #        
            #        
            #    npoints.append( (x+origin[0],y+origin[1]) )
  
  
            #print npoints;            
            #
            #cv.PolyLine( img, [npoints], False, (0,255,255), 2, 8)
  
  
            #nline = line[:2]
            #
            #a,b = nline;
            #
            #c = (nline[0][0], nline[1][1])
            #
            #nline.append(c);
            #
            ##print nline;
            #
            #
            #
            #
            #
            ##c = (nline[0][0], nline[1][1])
            ##nline.append(c);
            #
            #
            #
            #cv.PolyLine( img, [nline], True, (255,0,0), 2, 8)
            ##           
            ##cv.PolyLine( img, [[a,b,c]], False, (0,0,255), 2, 8)
  
  
  
  
  
  
            #store = cv.CreateMemStorage()
  
            #line = []
            #
            #l = cv.ConvexHull2( corners, line );
  
            #for point in line:
            #    print point;
  
            # cv.PolyLine( img, [line], False, (255,0,0), 1, 8)
  
  
            #line = cv.FitLine( corners )
  
            #n = checkers[0] - 1;
            #
            #print n
            #
            #points = [corners[0], corners[1], corners[-n], corners[-1]]
            #
            #cv.PolyLine(img, [points], False, (255,0,0), 1, 8)
  
  
            #print corners;
            #print points;
  
  
  
  
  
        cv.ShowImage( 'yes', img );
  
        key = ( cv.WaitKey(10) ) % 0x100
        if key == 27:
            break;
'''
'''
#!/usr/bin/env python
import cv
import math
  
pos = None#[(0,0),(0,0),(0,0),(0,0)]
cam = cv.CaptureFromCAM(0);
  
width = 0;
height = 0;
  
cv.NamedWindow('source', 1)
  
  
def main( source ):
    while True:
        f = cv.QueryFrame( source );
  
        #print cv.Size(f);
  
        cv.ShowImage('source', f );
  
        key = ( cv.WaitKey(1) ) % 0x100
        if key == 27:
            break;
  
  
def set_left( num ):
    global pos, width,height;
    a,b,c,d = pos;
  
    a = (num,a[1]);
    d = (num,d[1])
  
    pos = [a,b,c,d]
  
def set_right( num ):
    global pos, width, height;
    a,b,c,d = pos;
  
    b = (width-num, b[1])
    c = (width-num, c[1])
  
    pos = [a,b,c,d]
  
def set_top( num ):
    global pos, width, height;
    a,b,c,d = pos;
  
    a = (a[0], num)
    b = (b[0], num)
    pos = [a,b,c,d]
  
def set_bottom( num ):
    global pos, width, height;
    print pos;
    a,b,c,d = pos;
  
    c = (c[0], height-num)
    d = (d[0], height-num)    
    pos = [a,b,c,d]
  
  
img = cv.QueryFrame(cam)
width, height = cv.GetSize(img)
  
cv.CreateTrackbar('left', 'source', 0, width, set_left)
cv.CreateTrackbar('right', 'source', 0, width, set_right )
cv.CreateTrackbar('top', 'source', 0, height, set_top )
cv.CreateTrackbar('bottom', 'source', 0, height, set_bottom )
  
pos  = [(0,0),(width,0),(width,height),(0,height)]
orig = pos[:];
  
if True:
    while True:
  
        checkers = (3,3)
  
        cv.PolyLine( img, [pos], True, (255,0,0), 2, 8)
  
        mat = cv.CreateMat(3, 3, cv.CV_32FC1)
        cv.GetPerspectiveTransform( orig, pos, mat);
  
  
        src = cv.CreateImage( cv.GetSize(img), cv.IPL_DEPTH_32F, 3 )
        cv.ConvertScale(img,src,(1/255.00))
        dst = cv.CloneImage( src );
        cv.Zero(dst);
  
        cv.WarpPerspective(src, dst, mat);
  
        cv.ShowImage( "test",dst );
        cv.ShowImage( 'source', img );
  
        key = ( cv.WaitKey(10) ) % 0x100
        img = cv.QueryFrame(cam)
        if key == 27:
            break
'''
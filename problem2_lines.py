# Project 2 for 673

import numpy as np
import cv2 as cv
import glob
from matplotlib import pyplot as plt
import helpers


def ransac(data):
    df = np.vstack(data).squeeze() 
    x = []
    y = []
    print(len(df))
    for i in range(0,len(df)-1):
        x.append(df[i][1])
        y.append(df[i][0])

    
    #plot data set points
    plt.scatter(x,y)

    # make x into 3x250 matrix with x**2, x, and 1
    one = ()
    x2 = ()
    for i in x:
        one = np.append(one,1)
        x2 = np.append(x2,i**2)
    X = [x2,x,one] # put x**2, x, and 1 all into the same matrix
    X = np.array(X)

    # matrix math
    Xtran = np.matrix.transpose(X) # take transpose of X
    Xinv = np.linalg.pinv(np.matmul(X,Xtran)) # take inv of X*XT
    XY = np.matmul(y,Xtran)
    B = np.matmul(XY,Xinv)
    return B
    





#get the image
img =[cv.imread(File) for File in glob.glob("./Videos/data_1/data/*.png")]
img2 = np.array(img)

#num = int(input("Enter the number of the picture from 0-302: ")) # uncomment this line to use a different image
num=220 # best image
#Camera Matrix
K= np.array( [[9.037596e+02, 0.000000e+00, 6.957519e+02],[0.000000e+00, 9.019653e+02, 2.242509e+02], [0.000000e+00, 0.000000e+00, 1.000000e+00]])
#distortion coefficients
D = np.array([-3.639558e-01, 1.788651e-01, 6.029694e-04, -3.922424e-04, -5.382460e-02])
while True:
    #show the original image
    file = img2[num]
    # 4 points on the original image's lane lines (two from each line)
    source = np.array([[0,440], [505,280], [725,280], [935,520]])
    # 4 new points line) for where the image should line up
    dest = np.array([[0, 700], [0, 0], [400, 0], [400, 700]])
    cv.line(file, (0,435), (505,285), (0,255,0), 1)
    cv.line(file, (723,280), (935,520), (0, 255, 0), 1)
    cv.imshow("rezied src", file)
    h, w = file.shape[:2]
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
    dst = cv.undistort(file, K, D, None, newcameramtx)
##    cv.imshow("undistorted", dst)

    # find homography H from the given 8 points
    h, status = cv.findHomography(source, dest)
    hinv, status = cv.findHomography(dest, source)
    # create the top down view
    #newimg = cv.resize(file,(1000,500))
    unwarped = cv.warpPerspective(dst, h, (500, 750))
    cv.imshow("unwarped", unwarped)


    # threshold the image in binary
    unwarped_gray = cv.cvtColor(unwarped, cv.COLOR_BGR2GRAY)
    #unwarped_gray = cv.fastNlMeansDenoising(unwarped_gray)
    ret, BW_lanes = cv.threshold(unwarped_gray, 200, 255, cv.THRESH_BINARY)
    laplacian = cv.Laplacian(BW_lanes, cv.CV_64F)
    sobelx = cv.Sobel(BW_lanes, cv.CV_64F, 1, 0, ksize=5)  # x
    sobely = cv.Sobel(BW_lanes, cv.CV_64F, 0, 1, ksize=5)  # y


    #==== Drawing the lines ====
    #first line
    lane_region1 = BW_lanes[:, 80:260]
    # perform Canny edge detection
    edges_region1 = cv.Canny(lane_region1, 50, 100, apertureSize=3)
    contours, hierarchy = cv.findContours(edges_region1, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    B = ransac(contours)
    x_start1 = int(B[2] + 80)
    x_end1 = int(B[0] * 700**2 + B[1] * 700 + B[2] + 80)

    
    #second line
    lane_region2 = BW_lanes[:, 300:450]
    # perform Canny edge detection
    edges_region2 = cv.Canny(lane_region2, 50, 100, apertureSize=3)
    contours, hierarchy = cv.findContours(edges_region2, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    B = ransac(contours)
    x_start2 = int(B[2] + 300)
    x_end2 = int(B[0] * 700**2 + B[1] * 700 + B[2] + 300)

    #draw the lines
    cv.line(BW_lanes, (x_start1,0), (x_end1,700), (255,0,0), thickness=10, lineType=8, shift=0)
    cv.line(BW_lanes, (x_start2,0), (x_end2,700), (255,0,0), thickness=10, lineType=8, shift=0)
    cv.imshow("BW Lanes", BW_lanes)
    
    #rewarp them
##    warp_zero = np.zeros_like(BW_lanes).astype(np.uint8)
##    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
##    actual_lanes = cv.warpPerspective(color_warp,hinv,(file.shape[1], file.shape[0]))
##    result = cv.addWeighted(file, 1, actual_lanes, 0.3, 0)
##    cv.imshow("Actual Lanes", actual_lanes)
    
    
##    cv.imshow("laplacian", laplacian)
##    cv.imshow("Sobelx", sobelx)
##    cv.imshow("Sobely", sobely)
    # quit program if user presses escape or 'q'

    key = cv.waitKey() & 0xFF
    if key == 27 or key == ord("q"):
        break

    

cv.destroyAllWindows()


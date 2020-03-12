# Project 2 for 673

import numpy as np
import cv2 as cv
import glob
import kalman as k
from matplotlib import pyplot as plt
import helpers
from os import path

#  The following script uses a code taken from  https://github.com/lblackhall/pyconau2016.git,
# Thanks to the user: lblackhall, This code computes a Kalman filter which was used to predict
# the direction where the car is going.

# Kalman filter parameters adjusted to predict the dirrection of the car
A = 1  # No process innovation
C = 1  # Measurement
B = 0  # No control input
Q = 0.001  # Process covariance
R = 1  # Measurement covariance
x = 0  # Initial estimate
P = 1  # Initial covariance

if __name__ == '__main__':
    # Verbose: 1 to show only the final result, 2 to show the processed images
    verbose = 2

    # Camera Matrix
    K = np.array([[9.037596e+02, 0.000000e+00, 6.957519e+02], [0.000000e+00, 9.019653e+02, 2.242509e+02],
                  [0.000000e+00, 0.000000e+00, 1.000000e+00]])
    # Distortion coefficients
    D = np.array([-3.639558e-01, 1.788651e-01, 6.029694e-04, -3.922424e-04, -5.382460e-02])
    # Text on screen parameters
    font = cv.FONT_HERSHEY_SIMPLEX
    org = (0, 20)
    fontScale = 1
    color = (255, 0, 0)
    thickness = 2

    # Create and object for the kalman filter
    kalmanFilter = k.SingleStateKalmanFilter(A, B, C, x, P, Q, R)
    # Get the image
    img = [cv.imread(File) for File in glob.glob("./Videos/data_1/data/*.png")]
    img2 = np.array(img)  # opens all the images in an array

    for img_num in range(0, 303):
        imgFile = "./Videos/data_1/data/"+str(img_num).zfill(10)+".png"
        # Show the original image
        if path.exists(imgFile):
            file = cv.imread(imgFile)
        else:
            print("File Not Found:",imgFile)
            break
        # 4 points on the original image's lane lines (two from each line)
        source = np.array([[0, 435], [505, 285], [723, 280], [935, 520]])
        # 4 new points line) for where the image should line up
        dest = np.array([[0, 700], [0, 0], [400, 0], [400, 700]])
        if verbose >1:
            cv.line(file, (0, 435), (505, 285), (0, 255, 0), 1)
            cv.line(file, (723, 280), (935, 520), (0, 255, 0), 1)
            cv.imshow("rezied src", file)

        h, w = file.shape[:2]
        # Calculate the New camera matrix using the given conditions described on line 31
        newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
        # calculate the distortion of the camera
        dst = cv.undistort(file, K, D, None, newCameraMatrix)

        if verbose>1:
            cv.imshow("undistorted", dst)
        # Find homography H from the given 8 points
        h, status = cv.findHomography(source, dest)
        hinv, status = cv.findHomography(dest, source)

        # Create the top down view
        unwarped = cv.warpPerspective(dst, h, (500, 750))

        if verbose > 1:
            cv.imshow("unwarped", unwarped)

        # Threshold the image in binary
        unwarpedGray = cv.cvtColor(unwarped, cv.COLOR_BGR2GRAY)
        # separate the lines of the road on left and right to apply different threshold on them.
        ret1, BW_lanesRight = cv.threshold(unwarpedGray, 200, 255, cv.THRESH_BINARY)
        ret, BW_lanesLeft = cv.threshold(unwarpedGray, 210, 255, cv.THRESH_BINARY)
        if verbose>2:
            laplacian = cv.Laplacian(BW_lanesRight, cv.CV_64F)
            sobelx = cv.Sobel(lane_region2, cv.CV_64F, 1, 0, ksize=5)  # x
            sobely = cv.Sobel(BW_lanesRight, cv.CV_64F, 0, 1, ksize=5)  # y

        # MAKING THE LINES
        # First line
        # lane_region1 = BW_lanes[:, 80:260]
        leftLane1 = 80
        rightLane1 = 260
        lane_region1 = BW_lanesLeft[:, leftLane1:rightLane1]
        if verbose >2:
            cv.imshow("Left Lane", lane_region1)

        # Perform Canny edge detection
        edges_region1 = cv.Canny(lane_region1, 50, 100, apertureSize=3)
        contours, hierarchy = cv.findContours(edges_region1, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # Get lane line coefficients and check if they are too far off from previous frame
        if img_num > 0:
            B1_old = B1

        B1 = helpers.ransac(contours)

        # If they are too far off, replace them with the previous frame
        if img_num > 0:
            if abs(B1[0] - B1_old[0]) > .00004 or abs(B1[1] - B1_old[1]) > .02:
                B1 = B1_old

        # Second line
        # lane_region2 = BW_lanes[:, 300:450]
        leftLane2 = 350
        rightLane2 = 420
        lane_region2 = BW_lanesRight[:, leftLane2:rightLane2]
        if verbose>2:
            cv.imshow("Right Lane", lane_region2)

        # Perform Canny edge detection on the right lane
        edges_region2 = cv.Canny(lane_region2, 50, 100, apertureSize=3)
        contours, hierarchy = cv.findContours(edges_region2, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        if verbose>2:
            # Test sobelx for behaivor
            sobelx = cv.Sobel(lane_region2, cv.CV_64F, 1, 0, ksize=5)  # x

        # Get lane line coefficients and check if they are too far off from pervious frame
        if img_num > 0:
            B2_old = B2
        B2 = helpers.ransac(contours)

        # If they are too far off, replace them with the previous frame
        if img_num > 0:
            if abs(B2[0] - B2_old[0]) > .00004 or abs(B2[1] - B2_old[1]) > .02:
                B2 = B2_old

        # DRAW THE LINES
        polys1 = []
        polys2 = []
        polys3 = []

        # Create the points of the lines made from RANSAC function
        for i in range(0, 700, 5):
            x1 = int(B1[0] * i ** 2 + B1[1] * i + B1[2] + leftLane1)
            x2 = int(B2[0] * i ** 2 + B2[1] * i + B2[2] + leftLane2)
            x3 = int((x1 + x2)/2)

            polys1.append([x1, i])
            polys2.append([x2, i])
            polys3.append([x3, i])

        # Put the points into a usable array for the polylines function
        polys1 = np.array(polys1, np.int32)
        polys1 = polys1.reshape((-1, 1, 2))
        polys2 = np.array(polys2, np.int32)
        polys2 = polys2.reshape((-1, 1, 2))
        polys3 = np.array(polys3, np.int32)
        polys3 = polys3.reshape((-1, 1, 2))

        # Turn prediction
        xPoints = polys3[:, :, 0].reshape(1, -1)[0]
        yPoints = polys3[:, :, 1].reshape(1, -1)[0]

        # calculate the slope of a lane
        xEnd = xPoints[0]
        xInit = xPoints[-1]
        yEnd = yPoints[0]
        yInit = yPoints[-1]

        m = (-yEnd+yInit)/(xEnd-xInit)
        kalmanFilter.step(0, m)
        m = kalmanFilter.current_state()
        # Conditions to considering a turning condition of not
        print(m)
        if 0 < m < 50:
            text = "Turning right"
        elif -50 < m < 0:
            text = "Turning left"
        else:
            text = "Going straight"

        # Have to flip the second line points otherwise the polyfill won't work correctly
        points = np.concatenate((polys1, np.flip(polys2, 0)))

        if verbose > 1:
            cv.imshow("unwarped", unwarped)
        # Fill in the space between the lines
        cv.fillPoly(unwarped, [points], color=[0, 100, 0])
        # Create the actual lines of from the RANSAC
        cv.polylines(unwarped, polys1, True, (0, 0, 255), thickness=20, lineType=8, shift=0)
        cv.polylines(unwarped, polys2, True, (0, 0, 255), thickness=20, lineType=8, shift=0)
        cv.polylines(unwarped, polys3, True, (0, 255, 255), thickness=20, lineType=8, shift=0)
        cv.line(unwarped, (xInit, yInit), (xEnd, yEnd), color, thickness=10)

        if verbose > 1:
            cv.imshow("unwarped&lines", unwarped)

        # Rewarp the lanes back into the image
        rewarped = cv.warpPerspective(unwarped, hinv, (file.shape[1], file.shape[0]))

        if verbose > 2:
            cv.imshow("rewarped", rewarped)

        # Overlap the lane detection on the original image
        final = cv.addWeighted(dst, 0.6, rewarped, 0.4, 0)
        final = final[100:420, 100:1292]

        cv.putText(final, text, org, font, fontScale, (255,255,255), thickness, cv.LINE_AA)

        if verbose > 0:
            cv.imshow("Final", final)
            cv.moveWindow("Final", 20, 20)
        if verbose>2:
            imgAll = np.hstack([ edges_region2, sobelx])
            cv.imshow("imgAll", imgAll)
            cv.moveWindow("imgAll", 10, 10)
            cv.imshow("Canny", edges_region1)
            cv.imshow("laplacian", laplacian)
            cv.imshow("Sobelx", sobelx)
            cv.imshow("Sobely", sobely)

        # Quit program if user presses escape or 'q'
        key = cv.waitKey(20) & 0xFF
        if key == 27 or key == ord("q"):
            break

    del img, img2, final
    cv.destroyAllWindows()

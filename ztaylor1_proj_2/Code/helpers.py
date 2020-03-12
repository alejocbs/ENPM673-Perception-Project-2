import numpy as np
import cv2 as cv2

lastOrientation = "BR"
size = 200
# make an array of the corner positions of a 200x200 square
perspective = np.array([[0, 0], [size - 1, 0],
                        [size - 1, size - 1], [0, size - 1]])


# Compute homography using supplemtary homography method provided by the Professor
# this is a custom function used in place of cv2.findHomography
def get_homograph(points1, points2):
    A = []
    B = []
    for i in range(0, 4):
        x_w, y_w = points2[i][0], points2[i][1]
        x_c, y_c = points1[i][0], points1[i][1]
        A = [[x_w, y_w, 1, 0, 0, 0, -x_c * x_w, -x_c * y_w, -x_c], [0, 0, 0, x_w, y_w, 1, -y_c * x_w, -y_c * y_w, -y_c]]
        B.append(A)
    B = np.reshape(B, (8, 9))
    U, S, D = np.linalg.svd(B)
    fact = (1 / D[-1, -1])
    h = D[-1, :]
    L = fact * h
    hom = np.reshape(L, (3, 3))
    return hom


def warp(matrix, M):
    h, w, z = matrix.shape
    warp = {'x': [], 'y': [], 'xnew': [], 'ynew': [], 'colors': []}
    aux = np.full(matrix.shape, 0, dtype='uint8')
    for j in range(h):
        for i in range(w):
            warp['x'].append(i)
            warp['y'].append(j)
            xpri = (M[0, 0] * i + M[0, 1] * j + M[0, 2]) / (M[2, 0] * i + (M[2, 1] * j) + M[2, 2])
            ypri = (M[1, 0] * i + M[1, 1] * j + M[1, 2]) / (M[2, 0] * i + (M[2, 1] * j) + M[2, 2])
            warp['xnew'].append(xpri)
            warp['ynew'].append(ypri)
            color = matrix[j, i]
            warp['colors'].append(color)
    for i in range(len(warp['x'])):
        xpri = int(warp['xnew'][i])
        ypri = int(warp['ynew'][i])

        if (xpri > 0 and ypri > 0) and (xpri < w and ypri < h):
            aux[ypri, xpri, :] = warp['colors'][i]
    return aux

def ransac(data):
    # put the contour points into a list
    df = np.vstack(data).squeeze()
    x = []
    y = []
    for i in range(0, len(df) - 1):
        x.append(df[i][1])
        y.append(df[i][0])

    # create two additional columns for x for x^2 coef and constant coef
    one = ()
    x2 = ()
    for i in x:
        one = np.append(one, 1)
        x2 = np.append(x2, i ** 2)
    X = [x2, x, one]  # put x**2, x, and 1 all into the same matrix
    X = np.array(X)

    # matrix math
    Xtran = np.matrix.transpose(X)  # take transpose of X
    Xinv = np.linalg.pinv(np.matmul(X, Xtran))  # take inv of X*XT
    XY = np.matmul(y, Xtran)
    # B is the matrix of coeficients (a,b,c in y = a*x^2 + b*x + c)
    B = np.matmul(XY, Xinv)

    return B


#This class exists to handle all processing related to a specific video frame
class frame:
    number = 0

    def __init__(self):
        self.number = 0

    def get_isolated_channels(self):
        # b = self.image.copy()
        # # set green and red channels to 0
        # b[:, :, 1] = 0
        # b[:, :, 2] = 0
        #
        # g = self.image.copy()
        # # set blue and red channels to 0
        # g[:, :, 0] = 0
        # g[:, :, 2] = 0
        #
        # r = self.image.copy()
        # # set blue and green channels to 0
        # r[:, :, 0] = 0
        # r[:, :, 1] = 0
        # return r, g, b
        channels = cv2.split(self.image)
        return channels

    def increment_frame(self):
        """Advances the frame number by one. Returns the updated value."""
        self.number += 1
        return self.number

    def adjust_gamma(self, gamma=1.0):
        # build a lookup table mapping the pixel values [0, 255] to
        # their adjusted gamma values
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
                          for i in np.arange(0, 256)]).astype("uint8")

        # apply gamma correction using the lookup table
        return cv2.LUT(self.image, table), table

    def lowpass(self, bottom):
        """Take a bottom number. Every pixel below bottom will be set to 0 and every pixel above
        will be set to 255. The image will be returned."""
        if self.image.shape()[2] == 3:
            print("Converting from color to grayscale first")
            gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        ret, out = cv2.threshold(gray, bottom, 255, cv2.THRESH_BINARY)

    def equalizeBins(self,cuts):
        """self.image must already be B/W"""
        h,w=np.shape(self.image)
        hIncrement = int(h/cuts)
        wIncrement = int(w/cuts)
        for y in range(cuts):
            for x in range(cuts):
                subset = self.image[hIncrement*x:hIncrement*(x+1),wIncrement*y:wIncrement*(y+1)]
                subset = cv2.equalizeHist(subset)
                self.image[hIncrement * x:hIncrement * (x + 1),wIncrement * y:wIncrement * (y + 1)] = subset[:,:]

    def equalizeRange(self, cuts=5):
        histogram = np.histogram(self.image.ravel(), 256, [0, 256])
        cutSize = 255/cuts
        cutSize = int(cutSize)
        finalLUT =[]
        #print(histogram)

        for cut in range(cuts):
            temp = histogram[0][cut*cutSize:255 if (cut+1)*cutSize > 256 else (cut+1)*cutSize]
            #print(temp)
            total = sum(temp)
            cumulation = []
            for i in range(len(temp)):
                i = int(i)
                if i != 0:
                    cumulation.append(cumulation[i-1]+temp[i])
                elif i == 0:
                    cumulation.append(temp[i])
            for i in range(0,cutSize):
                portion = cumulation[i]/total
                finalLUT.append(int(portion*cutSize+(cutSize*cut)))
        finalLUT.append(255)

        for x in range(len(self.image[:])):
            for y in range(len(self.image[x])):
                value = self.image[x][y]
                newValue=finalLUT[value]
                self.image[x][y]=newValue

    def resize(self, scale):
        """Scales the image using open cv resize function based on the scale parameter.
        To scale down the scale parameter should be less than 1."""
        self.image = cv2.resize(self.image, (0, 0), fx=scale, fy=scale)

    def smooth(self,kernalSize=5):
        """Executes a GaussianBlur filter using a kernal size provided. It should be and odd number"""
        self.image = cv2.GaussianBlur(self.image,(kernalSize,kernalSize),0)
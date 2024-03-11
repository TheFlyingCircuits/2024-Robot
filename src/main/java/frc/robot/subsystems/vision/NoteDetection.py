# OpenCV
import numpy as np
import cv2 as cv

# wpiLib
from cscore import CameraServer
from ntcore import NetworkTableInstance

def initCamera():
    # start the camera
    camera = cv.VideoCapture(0, cv.CAP_V4L2)

    # request MJPG format for video
    desiredFourcc = cv.VideoWriter.fourcc('M','J','P','G')
    camera.set(cv.CAP_PROP_FOURCC, desiredFourcc)

    # set picture size and fps
    # Monochrome Camera (Arducam B0332) can do all settings
    # Color Camera (Arducam b0385) can only do 100, 90, 60, 30, or sometimes 15 fps depending on resolution
    # Both cameras claim a 70 degree horizontal FOV on the datasheet.
    # 1280 x 800 (MJPG 100/120, YUYV 10)
    # 1280 x 720 (MJPG 100/120, YUYV 10)
    #  800 x 600 (MJPG 100/120)
    #  640 x 480 (MJPG 100/120)
    #  320 x 240 (MJPG 100/120)

    camera.set(cv.CAP_PROP_FRAME_WIDTH, 800)
    camera.set(cv.CAP_PROP_FRAME_HEIGHT, 600)
    camera.set(cv.CAP_PROP_FPS, 30)

    # I figured out how to control exposure
    # by reading the Arducam Docs site:
    # https://docs.arducam.com/UVC-Camera/Adjust-the-minimum-exposure-time/
    useAutoExposure = False
    maxExposureMilliseconds = 500
    minExposureMilliseconds = 0.1
    desiredExposureMilliseconds = 5.1
    if (useAutoExposure):
        # Turn on auto exposure
        camera.set(cv.CAP_PROP_AUTO_EXPOSURE, 3)
    else:
        # Turn off auto exposure
        camera.set(cv.CAP_PROP_AUTO_EXPOSURE, 1)

        # Set requested exposure.
        # Arducam docs indicate the value given should
        # be an int with units of [tenths of a milisecond],
        # so a value of 5 would mean 5 tenths of a miliscond
        desiredExposure_TenthsOfAMillisecond = int(desiredExposureMilliseconds / 10)
        camera.set(cv.CAP_PROP_EXPOSURE, desiredExposure_TenthsOfAMillisecond)

    # camera.set(cv.CAP_PROP_AUTO_WB, 0)

    # TODO: other configs
    #       exposure, whiteBalance, gain, etc.
    # camera.set(cv.CAP_PROP_EXPOSURE, 0)

    # read back configs to confirm they were set correctly
    frameWidth = int(camera.get(cv.CAP_PROP_FRAME_WIDTH))
    frameHeight = int(camera.get(cv.CAP_PROP_FRAME_HEIGHT))
    targetFPS = int(camera.get(cv.CAP_PROP_FPS))
    autoExposure = int(camera.get(cv.CAP_PROP_AUTO_EXPOSURE))
    exposure = int(camera.get(cv.CAP_PROP_EXPOSURE))
    whiteBalance = int(camera.get(cv.CAP_PROP_AUTO_WB))
    print("Auto Exposure:", autoExposure)
    print("Exposure:", exposure)
    print("White Balance:", whiteBalance)

    fourccInt = int(camera.get(cv.CAP_PROP_FOURCC))
    fourccBytes = fourccInt.to_bytes(length=4, byteorder='little')
    fourccString = fourccBytes.decode()

    return camera, frameWidth, frameHeight, targetFPS

def initNetworkTables():
    networkTables = NetworkTableInstance.getDefault()

    onRobot = False
    if (onRobot):
        print("Starting CircuitVision in Robot Mode")
        networkTables.startClient4("wpiLibPi")
        networkTables.setServerTeam(1787)
        # networkTables.startDSCLient()
    else:
        print("Starting CircuitVision in Desktop Mode")
        networkTables.startServer()
    
    return networkTables

framePutter = None
hMaxSub = None
hMinSub = None
sMaxSub = None
sMinSub = None
vMaxSub = None
vMinSub = None
exposureSub = None
blurSub = None
edgeStrengthSub = None

def main():
    networkTables = initNetworkTables()
    camera, frameWidth, frameHeight, targetFPS = initCamera()

    # networktables demo
    # Note that I've had frequent disconnects when using
    # shuffleboard 2024, but not with shuffleboard 2023!
    # probably because the pi image is still the 2023 version.
    global hMaxSub, hMinSub, sMaxSub, sMinSub, vMaxSub, vMinSub, exposureSub, blurSub, edgeStrengthSub
    networkTable = networkTables.getTable("noteDetectionParams")
    hMaxPub = networkTable.getDoubleTopic("hMax").publish()
    hMaxSub = networkTable.getDoubleTopic("hMax").subscribe(0)
    hMinPub = networkTable.getDoubleTopic("hMin").publish()
    hMinSub = networkTable.getDoubleTopic("hMin").subscribe(0)

    sMaxPub = networkTable.getDoubleTopic("sMax").publish()
    sMaxSub = networkTable.getDoubleTopic("sMax").subscribe(0)
    sMinPub = networkTable.getDoubleTopic("sMin").publish()
    sMinSub = networkTable.getDoubleTopic("sMin").subscribe(0)

    vMaxPub = networkTable.getDoubleTopic("vMax").publish()
    vMaxSub = networkTable.getDoubleTopic("vMax").subscribe(0)
    vMinPub = networkTable.getDoubleTopic("vMin").publish()
    vMinSub = networkTable.getDoubleTopic("vMin").subscribe(0)

    exposurePub = networkTable.getDoubleTopic("exposureMilliseconds").publish()
    exposureSub = networkTable.getDoubleTopic("exposureMilliseconds").subscribe(0)

    blurPub = networkTable.getDoubleTopic("blur").publish()
    blurSub = networkTable.getDoubleTopic("blur").subscribe(0)

    edgeStrengthPub = networkTable.getDoubleTopic("edgeStrength").publish()
    edgeStrengthSub = networkTable.getDoubleTopic("edgeStrength").subscribe(0)

    hMaxPub.set(179)
    hMinPub.set(0)
    sMaxPub.set(255)
    sMinPub.set(0)
    vMaxPub.set(255)
    vMinPub.set(0)
    exposurePub.set(3.4) # 1.7, 3.4, 5.1 avoids flicker for being close to power line frequency.
    blurPub.set(0)
    edgeStrengthPub.set(0)

    # init camera stream for viz
    global framePutter
    framePutter = CameraServer.putVideo("My Stream", frameWidth, frameHeight)
    framePutter.putFrame(np.full((frameHeight, frameWidth), 255, dtype=np.uint8))

    print("Using "+str(frameWidth)+"x"+str(frameHeight)+" frames @ "+str(targetFPS)+" FPS")
    print("openCV Verison:", cv.__version__)
    print("numpyVersion:", np.__version__)
    # print("CV Build:", cv.getBuildInformation())
    
    # start grabbing frames
    while (True):
        # get the next frame from the camera, and convert to greyscale
        _, frame = camera.read()
        
        showMe = detectNote(frame)

        framePutter.putFrame(showMe)

        desiredExposureMilliseconds = exposureSub.get()
        if (desiredExposureMilliseconds <= 0):
            # Turn on auto exposure
            camera.set(cv.CAP_PROP_AUTO_EXPOSURE, 3)
        else:
            # Turn off auto exposure
            camera.set(cv.CAP_PROP_AUTO_EXPOSURE, 1)

            # Set requested exposure.
            # Arducam docs indicate the value given should
            # be an int with units of [tenths of a milisecond],
            # so a value of 5 will be interpreted by the camera
            # as an exposure time of 5 tenths of a milliscond.
            # Experiment shows that max exposure will also be limited
            # by frame rate. e.g. to capture at 100 fps, each frame
            # can't take more than 10 milliseconds to capture.
            # So if you request an exposure time greater than 10 millieseconds
            # while capturing at 100 fps, your exposure request will be ignored
            # by the camera in order to maintain its frame rate.
            camera.set(cv.CAP_PROP_EXPOSURE, int(desiredExposureMilliseconds * 10))


def getVectorFieldViz(gradMags, gradAngles):
    positiveAngles = np.copy(gradAngles)
    positiveAngles[positiveAngles < 0] += 2*np.pi
    normalizedAngles = positiveAngles / (2*np.pi)
    
    h = (179 * normalizedAngles).astype(np.uint8)
    s = np.full_like(h, 255)
    v = (255 * gradMags).astype(np.uint8)
    hsv = cv.merge([h, s, v])
    bgr = cv.cvtColor(hsv, cv.COLOR_HSV2BGR)
    return bgr



def getGradient(greyscaleImage):
    # init kernels
    kernelX = np.array([-0.5, 0, 0.5], dtype=greyscaleImage.dtype).reshape(1, 3)
    kernelY = np.array([-0.5, 0, 0.5], dtype=greyscaleImage.dtype).reshape(3, 1)

    # calculate each component of the gradient one after the other
    gradientX = cv.filter2D(greyscaleImage, -1, kernelX, borderType=cv.BORDER_REPLICATE)
    gradientY = cv.filter2D(greyscaleImage, -1, kernelY, borderType=cv.BORDER_REPLICATE)
    return gradientX, gradientY


def detectNote(originalImage):
    # extract info from image
    height, width, channels = originalImage.shape
    totalPixels = width * height

    normalizedImage = originalImage.astype(np.float32) / 255

    isRightColor = getColorMask(normalizedImage)
    isWrongColor = np.logical_not(isRightColor)
    colorFilteredImage = np.copy(originalImage)
    colorFilteredImage[isWrongColor] = 0

    # edges = edgeDetection(originalImage)
    # return edges

    # Have to do broadcasting hack to this to work because of 3 color channels
    # idk which is more efficient, the np.copy() approach, or the np.where() approach
    # colorFilteredImage = np.where(isRightColor, originalImage, 0)
    return colorFilteredImage

def getColorMask(normalizedImage):
    # color detection
    hsv = cv.cvtColor(normalizedImage, cv.COLOR_BGR2HSV)
    # hue, saturation, value = cv.split(hsv) <- may need to split for inverted color range, but it seems the ring is firmly above h = 0

    global hMaxSub, hMinSub, sMaxSub, sMinSub, vMaxSub, vMinSub

    hMax = hMaxSub.get()
    hMin = hMinSub.get()
    sMax = sMaxSub.get()
    sMin = sMinSub.get()
    vMax = vMaxSub.get()
    vMin = vMinSub.get()
    lowerBounds = np.array([hMin, sMin, vMin]) # [5-10], 0, 200
    upperBounds = np.array([hMax, sMax, vMax]) # 40, 255, 255
    # keep whole note, at expense of still detecting the tape sometimes
    # h = [2, 20], [2, 30]
    # s = [50, 220], [64, 255]
    # v = [150, 255], [200, 255]

    # Remove everything until the only thing left is the note
    # h = [4, 30]
    # s = [128, 255]
    # v = [128, 255]

    return (cv.inRange(hsv, lowerBounds, upperBounds) > 0)


def edgeDetection(originalImage):
    # Normalization, Greycale, Blur
    normalizedImage = originalImage.astype(np.float32) / 255
    greyscale = cv.cvtColor(normalizedImage, cv.COLOR_BGR2GRAY)
    blur = blurSub.get()
    if (blur > 0):
        greyscale = cv.GaussianBlur(greyscale, (0, 0), sigmaX=blur)

    # Edges
    gradX, gradY = getGradient(greyscale)
    gradMags, gradAngles = cv.cartToPolar(gradX, gradY)
    maxGrad = np.sqrt(2)
    avgGradMag = np.average(gradMags)
    gradsToShow = np.ones_like(gradMags)
    minEdgeStrength = (avgGradMag / maxGrad) #edgeStrengthSub.get()
    gradsToShow[(gradMags / maxGrad) < minEdgeStrength] = 0
    viz = getVectorFieldViz(gradsToShow, gradAngles)
    return viz

main()














# # Let's give it a shot, maybe OpenCV releases the GIL!
# # from threading import Thread
# from concurrent.futures import ThreadPoolExecutor
# import concurrent.futures


# def getDirectionalDerivative(surface, directionX, directionY, threadPool=None):
#     gradX, gradY = getGradient(surface, threadPool)
#     return cv.blendLinear(src1=gradX, src2=gradY, weights1=directionX, weights2=directionY)
#     # return gradX * directionX + gradY * directionY


# def helpfulDivide(numerator, denominator):
#     return np.divide(numerator, denominator, out=np.zeros_like(numerator), where=(denominator != 0))
    





# def shiftUp(img):
#     # originially implemented with numpy roll,
#     # but that's pretty slow so now I'm trying
#     # an opencv function.
#     # upon testing, opencv may be slower?
#     # I'll come back to this later.
#     bottomRow = img[-1,:]
#     shiftedUp = np.roll(img, -1, axis=0)
#     shiftedUp[-1,:] = bottomRow
#     # height, width = img.shape

#     # translationX = 0
#     # translationY = -1
#     # translationMatrix = np.array([[1, 0, translationX], [0, 1, translationY]], dtype=np.float32)

#     # shiftedUp = cv.warpAffine(img, translationMatrix, (width, height), borderMode=cv.BORDER_REPLICATE)
#     return shiftedUp

# def shiftDown(img):
#     topRow = img[0,:]
#     shiftedDown = np.roll(img, 1, axis=0)
#     shiftedDown[0,:] = topRow
#     # height, width = img.shape

#     # translationX = 0
#     # translationY = 1
#     # translationMatrix = np.array([[1, 0, translationX], [0, 1, translationY]], dtype=np.float32)

#     # shiftedDown = cv.warpAffine(img, translationMatrix, (width, height), borderMode=cv.BORDER_REPLICATE)
#     return shiftedDown

# def shiftRight(img):
#     leftColumn = img[:,0]
#     shiftedRight = np.roll(img, 1, axis=1)
#     shiftedRight[:,0] = leftColumn
#     # height, width = img.shape

#     # translationX = 1
#     # translationY = 0
#     # translationMatrix = np.array([[1, 0, translationX], [0, 1, translationY]], dtype=np.float32)

#     # shiftedRight = cv.warpAffine(img, translationMatrix, (width, height), borderMode=cv.BORDER_REPLICATE)
#     return shiftedRight

# def shiftLeft(img):
#     rightColumn = img[:,-1]
#     shiftedLeft = np.roll(img, -1, axis=1)
#     shiftedLeft[:,-1] = rightColumn
#     # height, width = img.shape

#     # translationX = -1
#     # translationY = 0
#     # translationMatrix = np.array([[1, 0, translationX], [0, 1, translationY]], dtype=np.float32)

#     # shiftedLeft = cv.warpAffine(img, translationMatrix, (width, height), borderMode=cv.BORDER_REPLICATE)
#     return shiftedLeft

# def getLevelSets(img, isoValue):
#     # TODO: maybe optimize with filter2D
#     #       and see if opencv has a faster sign() function
#     signs = np.sign(img - isoValue).astype(np.int8)
#     signAbove = shiftDown(signs)
#     signBelow = shiftUp(signs)
#     signLeft = shiftRight(signs)
#     signRight = shiftLeft(signs)

#     crossesNorth = ((signs + signAbove) == 0)
#     crossesSouth = ((signs + signBelow) == 0)
#     crossesEast = ((signs + signRight) == 0)
#     crossesWest = ((signs + signLeft) == 0)
#     isZero = (signs == 0) # unlikely, but here for correctness

#     return crossesNorth | crossesSouth | crossesEast | crossesWest | isZero


# def threadedLevelSets(surface, isoValue, threadPool):
#     signs = np.sign(surface - isoValue).astype(np.int8)

#     futureNorth = threadPool.submit(crossesNorth, signs)
#     futureSouth = threadPool.submit(crossesSouth, signs)
#     futureEast = threadPool.submit(crossesEast, signs)
#     futureWest = threadPool.submit(crossesWest, signs)

#     # An iterator that returns futures in the order they are completed
#     futuresInOrder = concurrent.futures.as_completed((futureNorth, futureSouth, futureEast, futureWest))

#     firstHalf = threadPool.submit(np.logical_or, next(futuresInOrder).result(), next(futuresInOrder).result())
#     secondHalf = threadPool.submit(np.logical_or, next(futuresInOrder).result(), next(futuresInOrder).result())

#     futuresInOrder = concurrent.futures.as_completed((firstHalf, secondHalf))


#     output = (signs == 0)
#     output |= next(futuresInOrder).result()
#     output |= next(futuresInOrder).result()
#     return output

# def crossesNorth(signs):
#     signAbove = shiftDown(signs)
#     return ((signs + signAbove) == 0)

# def crossesSouth(signs):
#     signBelow = shiftUp(signs)
#     return ((signs + signBelow) == 0)

# def crossesEast(signs):
#     signRight = shiftLeft(signs)
#     return ((signs + signRight) == 0)

# def crossesWest(signs):
#     signLeft = shiftRight(signs)
#     return ((signs + signLeft) == 0)

# def numericLevelSets(surface, isoValue):
#     signs = np.sign(surface - isoValue).astype(np.int8)
#     n = signs + shiftDown(signs)
#     s = signs + shiftUp(signs)
#     e = signs + shiftLeft(signs)
#     w = signs + shiftRight(signs)

#     product = signs
#     for d in [n, s, e, w]:
#         product *= d
#     return (product == 0)

# def threadedNumericLevelSets(surface, isoValue, threadPool):
#     if (threadPool is None):
#         return numericLevelSets(surface, isoValue)
#     signs = np.sign(surface - isoValue).astype(np.int8)

#     sumNorth = threadPool.submit(crossesNorthNumeric, signs)
#     sumSouth = threadPool.submit(crossesSouthNumeric, signs)
#     sumEast = threadPool.submit(crossesEastNumeric, signs)
#     sumWest = threadPool.submit(crossesWestNumeric, signs)

#     # An iterator that returns futures in the order they are completed
#     futuresInOrder = concurrent.futures.as_completed((sumNorth, sumSouth, sumEast, sumWest))

#     firstHalf = threadPool.submit(np.multiply, next(futuresInOrder).result(), next(futuresInOrder).result())
#     secondHalf = threadPool.submit(np.multiply, next(futuresInOrder).result(), next(futuresInOrder).result())

#     futuresInOrder = concurrent.futures.as_completed((firstHalf, secondHalf))

#     product = signs
#     for future in futuresInOrder:
#         product *= future.result()
#     return (product == 0)



# def crossesNorthNumeric(signs):
#     signsAbove = shiftDown(signs)
#     return signs + signsAbove
# def crossesSouthNumeric(signs):
#     signsBelow = shiftUp(signs)
#     return signs + signsBelow
# def crossesEastNumeric(signs):
#     signsRight = shiftLeft(signs)
#     return signs + signsRight
# def crossesWestNumeric(signs):
#     signsLeft = shiftRight(signs)
#     return signs + signsLeft

# def crossesLeftRight(signs):
#     return crossesEastNumeric(signs) * crossesWestNumeric(signs)

# def crossesUpDown(signs):
#     return crossesNorthNumeric(signs) * crossesSouthNumeric(signs)

# def lessThreadedNumericLevelSets(surface, isoValue, threadPool):
#     signs = np.sign(surface - isoValue).astype(np.int8)

#     leftRight = threadPool.submit(crossesLeftRight, signs)
#     upDown = threadPool.submit(crossesUpDown, signs)

#     futuresInOrder = concurrent.futures.as_completed((leftRight, upDown))

#     output = signs
#     output *= next(futuresInOrder).result()
#     output *= next(futuresInOrder).result()
#     return (output == 0)

# def shortCircuitLevelSets(surface, isoValue):
#     # TODO: maybe optimize with filter2D
#     #       and see if opencv has a faster sign() function
#     signs = np.sign(surface - isoValue).astype(np.int8)
#     signAbove = shiftDown(signs)
#     signBelow = shiftUp(signs)
#     signLeft = shiftRight(signs)
#     signRight = shiftLeft(signs)

#     crossesNorth = ((signs + signAbove) == 0)
#     crossesSouth = ((signs + signBelow) == 0)
#     crossesEast = ((signs + signRight) == 0)
#     crossesWest = ((signs + signLeft) == 0)
#     isZero = (signs == 0) # unlikely, but here for correctness

#     # manual short circuiting
#     crossesZero = isZero
#     crossesZero[crossesZero == False] |= crossesNorth[crossesZero == False]
#     crossesZero[crossesZero == False] |= crossesSouth[crossesZero == False]
#     crossesZero[crossesZero == False] |= crossesEast[crossesZero == False]
#     crossesZero[crossesZero == False] |= crossesWest[crossesZero == False]

#     return crossesZero




# def simonVision(originalImage, statsDict, threadPool=None,):
#     # extract info from image
#     height, width, channels = originalImage.shape
#     totalPixels = width * height

#     # Normalization, Greycale, Blur
#     t0 = cv.getTickCount()
#     # normalizedImage = originalImage.astype(np.float32) / 255
#     # greyscale = cv.cvtColor(normalizedImage, cv.COLOR_BGR2GRAY)
#     greyscale = originalImage[:,:,0].astype(np.float32) / 255
#     t1 = cv.getTickCount()
#     statsDict["pre-processing"] += (t1-t0)/cv.getTickFrequency()


#     # Calculate gradient, angles, and magnitudes.
#     # grad has dimensions of [intensity / distance]
#     t0 = cv.getTickCount()
#     gradX, gradY = getGradient(greyscale, threadPool)
#     t1 = cv.getTickCount()
#     statsDict["grad"] += (t1-t0)/cv.getTickFrequency()


#     t0 = cv.getTickCount()
#     gradMags, gradAngles = cv.cartToPolar(gradX, gradY)
#     t1 = cv.getTickCount()
#     statsDict["mags and angles"] += (t1-t0)/cv.getTickFrequency()


#     t0 = cv.getTickCount()
#     if (threadPool is None):
#         gradDirectionX = helpfulDivide(gradX, gradMags)
#         gradDirectionY = helpfulDivide(gradY, gradMags)
#     else:
#         gradDirectionX_inProgress = threadPool.submit(helpfulDivide, gradX, gradMags)
#         gradDirectionY_inProgress = threadPool.submit(helpfulDivide, gradY, gradMags)
#         gradDirectionX = gradDirectionX_inProgress.result()
#         gradDirectionY = gradDirectionY_inProgress.result()
#     t1 = cv.getTickCount()
#     statsDict["division"] += (t1-t0)/cv.getTickFrequency()

#     # now directional derivative tests
#     t0 = cv.getTickCount()
#     directionalDerivative = getDirectionalDerivative(gradMags, gradDirectionX, gradDirectionY, threadPool)
#     t1 = cv.getTickCount()
#     statsDict["1st Direcitonal Derivative"] += (t1-t0)/cv.getTickFrequency()

#     # t0 = cv.getTickCount()
#     # directionalConcavity = getDirectionalDerivative(directionalDerivative, gradDirectionX, gradDirectionY, threadPool)
#     # t1 = cv.getTickCount()
#     # statsDict["2nd Direcitonal Derivative THIS ONE"] += (t1-t0)/cv.getTickFrequency()

#     # level sets
#     # t0 = cv.getTickCount()
#     # peaksTroughsFlatlines = getLevelSets(directionalDerivative, 0)
#     # t1 = cv.getTickCount()
#     # print("og level sets:", (t1-t0)/cv.getTickFrequency())

#     # t0 = cv.getTickCount()
#     # peaksTroughsFlatlines = threadedLevelSets(directionalDerivative, 0, threadPool)
#     # t1 = cv.getTickCount()
#     # print("threaded level sets:", (t1-t0)/cv.getTickFrequency())

#     # t0 = cv.getTickCount()
#     # peaksTroughsFlatlines = numericLevelSets(directionalDerivative, 0)
#     # t1 = cv.getTickCount()
#     # print("numeric level sets:", (t1-t0)/cv.getTickFrequency())


#     # t2 = cv.getTickCount()
#     # peaksTroughsFlatlines = threadedNumericLevelSets(directionalDerivative, 0, threadPool)
#     # t3 = cv.getTickCount()
#     # statsDict["threaded numeric level sets THIS ONE"] += (t3-t2)/cv.getTickFrequency()

#     # statsDict["total"] += (t3-t0)/cv.getTickFrequency()

#     t0 = cv.getTickCount()
#     if (threadPool == None):
#         directionalConcavity = getDirectionalDerivative(directionalDerivative, gradDirectionX, gradDirectionY, threadPool)
#         peaksTroughsFlatlines = numericLevelSets(directionalDerivative, 0)
#     else:
#         directionalFuture = threadPool.submit(getDirectionalDerivative, directionalDerivative, gradDirectionX, gradDirectionY, threadPool)
#         levelSetFuture = threadPool.submit(threadedNumericLevelSets, directionalDerivative, 0, threadPool)
#         # levelSetFuture = threadPool.submit(numericLevelSets, directionalDerivative, 0)
#         directionalConcavity = directionalFuture.result()
#         peaksTroughsFlatlines = levelSetFuture.result()
#     t1 = cv.getTickCount()
#     statsDict["combo meal"] += (t1-t0)/cv.getTickFrequency()

#     # t0 = cv.getTickCount()
#     # peaksTroughsFlatlines = lessThreadedNumericLevelSets(directionalDerivative, 0, threadPool)
#     # t1 = cv.getTickCount()
#     # print("less threaded numeric level sets:", (t1-t0)/cv.getTickFrequency())

#     # t0 = cv.getTickCount()
#     # peaksTroughsFlatlines = shortCircuitLevelSets(directionalDerivative, 0)
#     # t1 = cv.getTickCount()
#     # print("short circuit level sets:", (t1-t0)/cv.getTickFrequency())

#     # level sets can happen at the same time as concavity!!!!
#     # gotta check about adding a task from within a thread though...

    


#     # t0 = cv.getTickCount()
#     # directionalDerivative = threadedDirectionalDerivative(gradMags, gradDirectionX, gradDirectionY)
#     # directionalConcavity = threadedDirectionalDerivative(directionalDerivative, gradDirectionX, gradDirectionY)
#     # t1 = cv.getTickCount()
#     # print("threadPooledDirectional:", (t1-t0)/cv.getTickFrequency())

#     # t0 = cv.getTickCount()
#     # directionalDerivative = dumbDirectionalDerivative(gradMags, gradDirectionX, gradDirectionY)
#     # directionalConcavity = dumbDirectionalDerivative(directionalDerivative, gradDirectionX, gradDirectionY)
#     # t1 = cv.getTickCount()
#     # print("dumbDirectional:", (t1-t0)/cv.getTickFrequency())

#     # t0 = cv.getTickCount()
#     # showMe = getVectorFieldViz(gradMags / math.sqrt(0.5), gradAngles)
#     # t1 = cv.getTickCount()
#     # statsDict["viz"] += (t1-t0)/cv.getTickFrequency()
#     showMe = greyscale
#     return showMe
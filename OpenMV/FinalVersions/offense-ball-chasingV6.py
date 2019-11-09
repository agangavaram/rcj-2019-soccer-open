from pyb import Pin
import sensor, image, time

threshold_index = 0
# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)

goalThresholds = [(74, 93, -25, 8, 30, 77), (42, 69, -21, 1, -45, -21)] # threshold index (yellow goal, blue goal)
ballThreshold = [(52, 94, 9, 76, -44, 57)]

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
sensor.set_brightness(-2)
clock = time.clock()

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.

p0 = Pin('P0', Pin.OUT_PP, Pin.PULL_NONE) # p0, p1  11 - straight 10 - left 01 - right 00 - do not see
p1 = Pin('P1', Pin.OUT_PP, Pin.PULL_NONE)
initPin = Pin('P2', Pin.IN, Pin.PULL_NONE) #initialize input
proxValuePin = Pin('P3', Pin.IN, Pin.PULL_NONE) #proximity sensor input
seeBallPin = Pin('P4', Pin.OUT_PP, Pin.PULL_NONE) #have ball or not output
diagPin = Pin('P5', Pin.OUT_PP, Pin.PULL_NONE)



isInitialized = False

def initialize():
    while(True):
        print("Trying to Initialize")
        img = sensor.snapshot()
        goalBlobs = img.find_blobs(goalThresholds, pixels_threshold=100, area_threshold=100, merge=False)
        print(len(goalBlobs))
        if len(goalBlobs) > 0:
            blob = max(goalBlobs, key = lambda b: b.w() - b.x())
            if blob.cy() > 115:
                print("Opponent Goal Set")
                if(blob.code() == 1):
                    print("yellow")
                    return blob.code() >> 1
                else:
                    print("blue")
                    return blob.code() >> 1
        else:
            return None

def goToBall(blob):
     img.draw_rectangle(blob.rect())
     img.draw_cross(blob.cx(), blob.cy())
     cpos = blob.cx()
     if blob.cy() > 95:
        if cpos < 110:
          p0.value(1)
          p1.value(0)
          diagPin.value(0)
        elif cpos > 182:
          p0.value(0)
          p1.value(1)
          diagPin.value(0)
        else:
          p0.value(1)
          p1.value(1)
          diagPin.value(0)
     else:
        p0.value(0)
        p1.value(0)
        diagPin.value(0)


def findGoal():
    while(proxValuePin.value() and seeBall() is None):
        print(proxValuePin.value())
        print("Trying to Find Goal")
        img = sensor.snapshot()
        goalBlobs = img.find_blobs(goalThresholds, pixels_threshold=100, area_threshold=100, merge=True)
        if len(goalBlobs) > 0:
            print("See Opponent Goal")
            blob = max(goalBlobs, key = lambda b: b.w() - b.x())
            if blob.cy() > 115:
                if blob.h() - blob.y() < -100:
                    print(blob.h() - blob.y())
                    img.draw_cross(blob.cx(), blob.cy())
                    img.draw_rectangle(blob.rect())
                    cpos = blob.cx()
                    diagPin.value(0)
                else:
                    diagPin.value(1)
                    print("kick")
        else:
            diagPinv.value(0)
            print(p0.value(), p1.value())


def seeBall():
    img = sensor.snapshot()
    blobs = img.find_blobs([ballThreshold[threshold_index]], pixels_threshold=50, area_threshold=50, merge=True)
    if len(blobs) > 0:
        blob = max(blobs, key = lambda b: b.w() - b.x())
        seeBallPin.value(1)
        return blob
    else:
        seeBallPin.value(0)
        return None

print("Program Start")
clock.tick()
opponentGoalIndex = None
while(True):
    if initPin.value() == 1:
        img = sensor.snapshot()
        blob = seeBall()
        if proxValuePin.value():
            findGoal()
        elif blob is not None:
            goToBall(blob)
        else:
             p0.value(0)
             p1.value(0)
        print(p0.value(), p1.value())
    else:
        while(initPin.value() == 0):
            img = sensor.snapshot()
            print(initPin.value())
            print("Waiting for signal from Arduino")


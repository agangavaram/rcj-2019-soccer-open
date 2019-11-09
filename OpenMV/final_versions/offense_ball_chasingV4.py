from pyb import Pin
import sensor, image, time

threshold_index = 0
# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...

goalThresholds = [(67, 91, -13, 2, 28, 72), (43, 100, -15, 16, -57, -12)] # threshold index (ball, yellow goal, blue goal)
ballThreshold = [(55, 80, 19, 79, 17, 73)]

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
p2 = Pin('P2', Pin.IN, Pin.PULL_NONE)
opos = 150

isInitialized = False

def initialize():
    while(True):
        print("Trying to Initialize")
        img = sensor.snapshot()
        goalBlobs = img.find_blobs(goalThresholds, pixels_threshold=200, area_threshold=200, merge=False)
        if len(goalBlobs) > 0:
            for blob in img.find_blobs(goalThresholds, pixels_threshold=100, area_threshold=100, merge=False):
                if blob.cy() > 115:
                    opponentGoal = blob.code()
                    print("Target Set")
                    if opponentGoal == 1:
                        print("Need to Score in Yellow Goal")
                        return True
                    elif opponentGoal == 2:
                        print("Need to Score in Blue Goal")
                        return True
                    else:
                        print(opponentGoal)
                        return False


print("Program Start")

while(True):
    print(isInitialized)
    print(p2.value())
    if isInitialized:
        while(isInitialized):
          clock.tick()
          img = sensor.snapshot()
          blobs = img.find_blobs([ballThreshold[threshold_index]], pixels_threshold=200, area_threshold=200, merge=False)
          if len(blobs) > 0:
              for blob in img.find_blobs([ballThreshold[threshold_index]], pixels_threshold=200, area_threshold=200, merge=False):
                   img.draw_rectangle(blob.rect())
                   img.draw_cross(blob.cx(), blob.cy())
                   cpos = blob.cx()
                   if blob.cy() > 115:
                      if cpos < 140:
                        p0.value(1)
                        p1.value(0)
                      elif cpos > 164:
                        p0.value(0)
                        p1.value(1)
                      else:
                        p0.value(1)
                        p1.value(1)
          else:
               p0.value(0)
               p1.value(0)
          print(p0.value(), p1.value())
    else:
        while(p2.value() == 0):
            img = sensor.snapshot()
            print(p2.value())
            print("Waiting for signal from Arduino")
        isInitialized = initialize()



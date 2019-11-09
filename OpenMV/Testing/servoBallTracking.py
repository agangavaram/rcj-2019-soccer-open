from pyb import Pin
import sensor, image, time

threshold_index = 0 # 0 for red, 1 for green, 2 for blue

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...
thresholds = [(47, 69, 39, 99, 25, 79)] # generic_blue_thresholds

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.

p0 = Pin('P8', Pin.OUT_OD, Pin.PULL_NONE) # p0, p1  11 - straight; 10 - left; 01 - right``` ; 00 - do not see
p1 = Pin('P9', Pin.OUT_OD, Pin.PULL_NONE)
opos = 150

while(True):
   clock.tick()
   img = sensor.snapshot()
   if img.find_blobs([thresholds[threshold_index]], pixels_threshold=75, area_threshold=75, merge=True):
       for blob in img.find_blobs([thresholds[threshold_index]], pixels_threshold=75, area_threshold=75, merge=True):
           img.draw_rectangle(blob.rect())
           img.draw_cross(blob.cx(), blob.cy())
           cpos = blob.cx()
           print(opos, cpos, (opos - cpos))
           if cpos < 120:
                p0.value(1)
                p1.value(0)
                print("left")
           elif cpos > 204:
                p0.value(0)
                p1.value(1)
                print("right")
           else:
                p0.value(1)
                p1.value(1)
           print(p0.value(), p1.value())
           time.sleep(20)
           opos = cpos
   else:
       print("no blob")
       p0.value(0)
       p1.value(0)


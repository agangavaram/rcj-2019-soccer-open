from pyb import Pin
import sensor, image, time

threshold_index = 0 # 0 for red, 1 for green, 2 for blue

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...
thresholds = [(46, 83, 30, 86, 6, 84)] # generic_blue_thresholds

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
#sensor.set_brightness(-3)
#sensor.set_gainceiling(128)
clock = time.clock()

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.

p0 = Pin('P0', Pin.OUT_PP, Pin.PULL_NONE) # p0, p1  11 - straight 10 - left 01 - right 00 - do not see
p1 = Pin('P1', Pin.OUT_PP, Pin.PULL_NONE)

while(True):
   clock.tick()
   img = sensor.snapshot()
   blobs = img.find_blobs([thresholds[threshold_index]], pixels_threshold=200, area_threshold=200, merge=False)
   if len(blobs) > 0:
       for blob in img.find_blobs([thresholds[threshold_index]], pixels_threshold=200, area_threshold=200, merge=False):
         img.draw_rectangle(blob.rect())
         img.draw_cross(blob.cx(), blob.cy())
         cpos = blob.cx()
         if cpos < 155:
            p0.value(1)
            p1.value(0)
         elif cpos > 220:
            p0.value(0)
            p1.value(1)
         elif 220 > cpos > 155:
            p0.value(1)
            p1.value(1)
         print(p0.value(), p1.value())
   else:
       p0.value(0)
       p1.value(0)
       print(p0.value(), p1.value())

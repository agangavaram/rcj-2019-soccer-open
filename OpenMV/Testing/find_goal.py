# Untitled - By: abiramgupta - Thu Apr 25 2019

import sensor, image, time

goalThresholds = [(16, 59, -12, 27, -91, -9)]

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()
    goalBlobs = img.find_blobs(goalThresholds, pixels_threshold=200, area_threshold=200, merge=False)
    if len(goalBlobs) > 0:
        print("See Goal")
        blob = max(goalBlobs, key = lambda b: b.w() - b.x())
        img.draw_rectangle(blob.rect())
        if((blob.h() - blob.y()) < -65):
            print(blob.h() - blob.y())
            img.draw_cross(blob.cx(), blob.cy())
            cpos = blob.cx()

                    cpos = blob.cx()
                    if blob.cy() > 115:
                        if cpos < 140:
                            print("Left")
                        else:
                            print("Right")
            print("Forward")
        else:
            print("Shoot!")
    else:
        print("dont see goal")

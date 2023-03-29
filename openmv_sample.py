import sensor, image, time

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green things. You may wish to tune them...
thresholds = [(0, 100, -35, 35, 30, 127), (15, 100, -64, -8, -32, 32),(15, 100, 15, 127, 15, 100)]

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()


while(True):
    clock.tick()
    img = sensor.snapshot()
    for blob in img.find_blobs([(0, 100, -35, 35, 30, 127)], pixels_threshold=150, area_threshold=150, merge=False):
        if blob.code() == 1:
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            img.draw_string(blob.x() + 2, blob.y() + 2, "YELLOW")
    for blob in img.find_blobs([(15, 100, -64, -8, -32, 32)], pixels_threshold=150, area_threshold=150, merge=False):
        if blob.code() == 1:
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            img.draw_string(blob.x() + 2, blob.y() + 2, "GREEN")
    for blob in img.find_blobs([(15, 100, 20, 127, 0, 80)], pixels_threshold=150, area_threshold=150, merge=False):
        if blob.code() == 1:
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            img.draw_string(blob.x() + 2, blob.y() + 2, "RED")

    #print(clock.fps())

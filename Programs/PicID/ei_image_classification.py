# Edge Impulse - OpenMV Image Classification Example

import sensor, image, time, os, tf, uos, gc, pyb
from pyb import Pin

p0 = pyb.Pin("P0", pyb.Pin.OUT_PP)  #to communicate with Arduino
p1 = pyb.Pin("P1", pyb.Pin.OUT_PP)  #connect MV pins p0, p1, p2
p2 = pyb.Pin("P2", pyb.Pin.OUT_PP)  #to Arduino digital pins

sensor.reset()                         # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)    # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)      # Set frame size to QVGA (320x240)
sensor.set_windowing((240, 240))       # Set 240x240 window.
sensor.skip_frames(time=2000)          # Let the camera adjust.

net = None
labels = None

try:
    # Load built in model
    labels, net = tf.load_builtin_model('trained')
except Exception as e:
    raise Exception(e)


clock = time.clock()
while(True):
    clock.tick()

    img = sensor.snapshot()

    # default settings just do one detection... change them to search the image...
    for obj in net.classify(img, min_scale=1.0, scale_mul=0.8, x_overlap=0.5, y_overlap=0.5):
        #print("**********\nPredictions at [x=%d,y=%d,w=%d,h=%d]" % obj.rect())
        img.draw_rectangle(obj.rect())
        # This combines the labels and confidence values into a list of tuples
        predictions_list = list(zip(labels, obj.output()))

        ret_name, ret_val = "", 0.0
        for i in range(len(predictions_list)):
            id_name, id_val = predictions_list[i][0], predictions_list[i][1]
            #print("%s = %f" % (id_name, id_val))
            if id_val > ret_val:
                ret_val = id_val
                ret_name = id_name

        debugChk = False
        #print("%s: %f"%(ret_name, ret_val))
        if not ret_name or ret_name == "White" or ret_val < 0.9:
            p0.low()
            p1.low()
            p2.low()
            if debugChk: print("N")
        else:
            if ret_name == "H":
                p0.high()
                p1.high()
                p2.high()
                if debugChk: print("H")
            if ret_name == "S":
                p0.low()
                p1.high()
                p2.high()
                if debugChk: print("S")
            if ret_name == "Red" or ret_name == "Yellow":
                p0.high()
                p1.low()
                p2.high()
                if debugChk: print("R/Y")
            if ret_name == "U" or ret_name == "Green":
                p0.high()
                p1.high()
                p2.low()
                if debugChk: print("U/G")

#import sensor, time, image, pyb, uasyncio

#from pyb import UART
#'''
#Known bugs/issues:
#- Fix bug where camera detects stac up and create issues
#'''

##initialization of the camera, resolution: 160, 120 (QQVGA)
#sensor.reset()
#sensor.set_pixformat(sensor.GRAYSCALE)
#sensor.set_framesize(sensor.QQVGA)
#sensor.skip_frames(time = 2000)

#clock = time.clock()

#class detect: #Detecting class (Using OOP because it's more organized)
    ##init method
    #def __init__(self):
        #self.H = image.Image("/H.pgm")
        #self.S = image.Image("/S.pgm")
        #self.U = image.Image("/U.pgm")
        #self.uart = UART(1, 9600, timeout_char=1000) #Instantiate UART class
        #self.uart.init(9600, bits=8, parity=None, stop=1, timeout_char=1000) #initializing the uart. Remember the arduino settings are default.
        #self.verify = None
        #self.HSU()

    #def HSU(self):
        #'''
        #Checks the frame against a template, then checks if it is already registered.
        #When detected, run self.send() with the parameters of 0, 1 or 2.
        #'''

        #clock.tick()
        #while True:
            #self.img = sensor.snapshot()
            #self.harmed = self.img.find_template(self.H, 0.5, search=image.SEARCH_DS) #finding the template in the storage
            #if self.harmed != None and self.verify == "harmed":
                #self.verify = None
                #self.send("0")
            #elif self.harmed and self.verify != "harmed":
                #self.verify = "harmed"

            #self.safe = self.img.find_template(self.S, 0.5, search=image.SEARCH_DS) #finding the template in the storage
            #if self.safe != None and self.verify == "safe":
                #self.verify = None
                #self.send("1")
            #elif self.safe and self.verify != "safe":
                #self.verify = "safe"


            #self.unharmed = self.img.find_template(self.U, 0.5, search=image.SEARCH_DS) #finding the template in the storage
            #if self.unharmed != None and self.verify == "unharmed":
                #self.verify = None
                #self.send("2")
            #elif self.unharmed and self.verify != "unharmed":
                #self.verify = "unharmed"

            #else:
                #pass

    #def send(self, num):
        #'''
        #Sends whatever value passed into the method to the arduino.
        #Arduino returns a 2 (decimal code 50) to the camera to continue detecting.
        #If nothing is detected, it will continue to run.
        #Note: add a timeout.
        #'''
        #self.uart.write(num)
        #print(num)
        #while True:
            #read = self.uart.readchar()
            #if read == 50 or read == 2:
                #print("Done")
                #self.HSU()
        #print("Error")




#detect = detect() #instantiating a class





#UART test code below
#uart = UART(1, 9600, timeout_char=1000)
#uart.init(9600, bits=8, parity=1, timeout_char=1000)
#while True:
    #red = pyb.LED(1)
    #pyb.LED.on(red)
    #uart.writechar(0)
    #if uart.readchar() == 1:
        #print("Check 1 done")
        #uart.writechar(1)
        #if uart.readchar() == 2:
            #print("Check 2 done")
            #uart.writechar(2)
            #if uart.readchar() == 3:
                #print("Check 3 done")
                #pyb.LED.off(red)
                #print("Check good")



import sensor, time, image, pyb

from pyb import UART

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()
H = image.Image("/H.pgm")
S = image.Image("/S.pgm")
U = image.Image("/U.pgm")
led = pyb.LED(1)
uart = UART(1, 115200, timeout_char=1000) #Instantiate UART class
uart.init(115200, bits=8, parity=None, stop=1, timeout_char=1000) #initializing the uart. Remember the arduino settings are default.
verify = None
def HSU(H, S, U, uart, verify, clock):
        while True:
            clock.tick()
            img = sensor.snapshot()
            harmed = img.find_template(H, 0.5, search=image.SEARCH_DS) #unreliable
            if harmed != None and verify == "harmed":
                verify = None
                uart.write("0")
                led.on()
                uart.any()
                print(uart.readchar())
                while True:
                    if uart.any():
                        read = uart.readchar()
                        if read == 50 or read == 2:
                            print("Done")
                            led.off()
                            break
            elif harmed and verify != "harmed":
                verify = "harmed"

            safe = img.find_template(S, 0.5, search=image.SEARCH_DS) #so damn unreliable
            if safe != None and verify == "safe":
                verify = None
                uart.write("1")
                led.on()
                while True:
                    if uart.any():
                        read = uart.readchar()
                        if read == 50 or read == 2:
                            print("Done")
                            led.off()
                            break
            elif safe and verify != "safe":
                verify = "safe"

            unharmed = img.find_template(U, 0.5, search=image.SEARCH_DS) #unreliable x3
            if unharmed != None and verify == "unharmed":
                verify = None
                uart.write("2")
                led.on()
                while True:
                    if uart.any():
                        read = uart.readchar()
                        if read == 50 or read == 2:
                            print("Done")
                            led.off()
                            break
            elif unharmed and verify != "unharmed":
                verify = "unharmed"

            else:
                pass
HSU(H, S, U, uart, verify, clock)

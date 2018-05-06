# Python programm e-puck explorateur project
# Maxime Marchionno and Nicolas Peslerbe
# Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
# MICRO-315 | École Polytechnique Fédérale de Lausanne

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
from matplotlib import collections as mc
import serial
import struct
import sys
import signal
import time
from threading import Thread
import re
from PIL import Image
import math

global pointRobot, collectionRobot

n = 800
max_value = 800

#handler when closing the window
def handle_close(evt):
    #we stop the serial thread
    reader_thd.stop()

#update the plots
def update_plot():
    if(reader_thd.need_to_update_plot()):
        fig.canvas.draw_idle()
        reader_thd.plot_updated()

#function used to update the plot of the cam data
def update_cam_plot(port):

    cam_data = readUint8Serial(port)
    
    if(len(cam_data)>0):
        cam_plot.set_ydata(cam_data)
        
        graph_cam.relim()
        graph_cam.autoscale()

        reader_thd.tell_to_update_plot()


#reads the data in uint8 from the serial
def readUint8Serial(port):

    state = 0
    
    

    while(state != 5):

        #reads 1 byte
        c1 = port.read(1)
        #timeout condition
        if(c1 == b''):
            return [];

        if(state == 0):
            if(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 1):
            if(c1 == b'T'):
                state = 2
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 2):
            if(c1 == b'A'):
                state = 3
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 3):
            if(c1 == b'R'):
                state = 4
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 4):
            if(c1 == b'T'):
                state = 5
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0

    total = 0
    
    while(1):
        c = port.read(1)
        if(c.decode("utf-8")  == '|'):
            break
        else:
            if(ord(c.decode("utf-8")) == 32):
                continue
            elif(ord(c.decode("utf-8"))< 48 | ord(c.decode("utf-8"))>57):
                return[]
            total = total*10 + int(c.decode("utf-8"))
    
    state = 0

    
    if(port.read(1).decode("utf-8") != '|'):
        return[]
    
    content = ''

    while(1):
        c = port.read(1)
        if(c.decode("utf-8") == '|'):
            break
        else:
           content = content + c.decode("utf-8")

    if(port.read(1).decode("utf-8") != '|'):
        return[]

    toRead = total
    rcv_buffer = b''
    #reads the data
    while(toRead > 0):
        if(toRead > 4000):
            rcv_buffer += port.read(4000)
        else:
            rcv_buffer += port.read(toRead)
        toRead = total - len(rcv_buffer)

    data = []


    if("Message" in content):
        if( "New point computed :" in rcv_buffer.decode("utf-8")):
            pointsText = rcv_buffer.decode("utf-8").split(':')
            #if(len(pointsText) == 4):
                #print("Will plot: ", [int(pointsText[1])]," and: ", [int(pointsText[2])])
                #graph_cam.plot(int(pointsText[1]), int(pointsText[2]), marker='s', linestyle='-', color='k')
                #reader_thd.tell_to_update_plot()
        elif( "PointInEnvironment:" in rcv_buffer.decode("utf-8")):
            pointsText = rcv_buffer.decode("utf-8").split(':')
            #if(len(pointsText) == 4):
                #print("Will plot env: ", [int(pointsText[1])]," and: ", [int(pointsText[2])])
                #graph_cam.plot(int(pointsText[1]), int(pointsText[2]), marker='s', linestyle='-', color='b')
                #reader_thd.tell_to_update_plot()
        elif( "PointClosest:" in rcv_buffer.decode("utf-8")):
            pointsText = rcv_buffer.decode("utf-8").split(':')
            # if(len(pointsText) == 4):
                #print("Will plot close: ", [int(pointsText[1])]," and: ", [int(pointsText[2])])
                #graph_cam.plot(int(pointsText[1]), int(pointsText[2]), marker='s', linestyle='-', color='g')
                #reader_thd.tell_to_update_plot()
        elif( "Walls" in rcv_buffer.decode("utf-8")):
            pointsText = rcv_buffer.decode("utf-8").split(':')
            if(len(pointsText) == 4):
                print("Will plot: ", [int(pointsText[1])]," and: ", [int(pointsText[2])])
                lines = [[(0, 0), (int(pointsText[1]), 0)],
                          [(0, 0), (0, int(pointsText[2]))],
                          [(int(pointsText[1]), 0), (int(pointsText[1]), int(pointsText[2]))],
                          [(0, int(pointsText[2])), (int(pointsText[1]), int(pointsText[2]))]]
                lc = mc.LineCollection(lines, colors='r', linewidths=2)
                graph_cam.add_collection(lc)
                reader_thd.tell_to_update_plot()
        elif("New position:" in rcv_buffer.decode("utf-8")):
            pointsText = rcv_buffer.decode("utf-8").split(':')
            if(len(pointsText) == 5):
                print("Will plot position: ", [int(pointsText[1])]," and: ", [int(pointsText[2])])
                global collectionRobot
                global pointRobot
                pointRobot.remove()
                collectionRobot.remove()
                pointRobot, = graph_cam.plot(int(pointsText[1]), int(pointsText[2]), marker='s', linestyle='-', color='k')
                lines = [[(int(pointsText[1]), int(pointsText[2])),
                       (int(pointsText[1]) + 100*math.cos(float(pointsText[3])+math.pi/2), int(pointsText[2]) + 100*math.sin(float(pointsText[3])+math.pi/2))]]
                lc = mc.LineCollection(lines, colors='r', linewidths=2)
                collectionRobot = graph_cam.add_collection(lc)
                reader_thd.tell_to_update_plot()
        elif("send the map" in rcv_buffer.decode("utf-8")):
            print("Will save the map")
            graph_cam.savefig("/Users/nicolas/epuck/finalMap.png")
        else:
            print(rcv_buffer.decode("utf-8"))
        return []
    elif("Image" in content):
        global imageID
        x= 0
        y= 0
        sizeText = content.split(':')
        if (len(sizeText) == 4):
            x = int(sizeText[1])
            y = int(sizeText[2])
            print("New object : x :", x, "y :", y)
            graph_cam.plot(x, y, marker='s', linestyle='-', color='r')
            graph_cam.annotate("Img " + str(imageID), xy=(x, y))
            reader_thd.tell_to_update_plot()
        else:
            print(content)
            print("False lenght "+ str(len(sizeText)))
            return []
        #if we receive the good amount of data, we convert them in float32
        if(len(rcv_buffer) == total):
            new_buffer = bytearray(len(rcv_buffer))
            new_buffer[0::2] = rcv_buffer[1::2]
            new_buffer[1::2] = rcv_buffer[0::2]
            im = Image.frombytes("RGB", (80, 120), bytes(new_buffer), "raw", "BGR;16")
            nameimg ="Image" + str(imageID) + "_x_" + str(x) + "_y_" + str(y)
            im.show(title=nameimg)
            im.save("/Users/nicolas/epuck/Image" + str(imageID) + "_x_" + str(x) + "_y_" + str(y) + ".png", "PNG")
            imageID += 1
            
            print('received !')
            return []
        else:
            print('Timout...')
            return []
    return []

#thread used to control the communication part
class serial_thread(Thread):

    #init function called when the thread begins
    def __init__(self, port):
        Thread.__init__(self)
        self.contReceive = False
        self.alive = True
        self.need_to_update = False

        print('Connecting to port {}'.format(port))

        try:
            self.port = serial.Serial(port, timeout=0.5)
        except:
            print('Cannot connect to the e-puck2')
            sys.exit(0)
    #function called after the init
    def run(self):
        
        while(self.alive):

            if(self.contReceive):
                update_cam_plot(self.port)
            else:
                #flush the serial
                self.port.read(self.port.inWaiting())
                time.sleep(0.1)

    #enables the continuous reading
    def setContReceive(self, val):  
        self.contReceive = True

    #disables the continuous reading
    def stop_reading(self, val):
        self.contReceive = False

    #tell the plot need to be updated
    def tell_to_update_plot(self):
        self.need_to_update = True

    #tell the plot has been updated
    def plot_updated(self):
        self.need_to_update = False

    #tell if the plot need to be updated
    def need_to_update_plot(self):
        return self.need_to_update

    #clean exit of the thread if we need to stop it
    def stop(self):
        self.alive = False
        self.join()
        if(self.port.isOpen()):
            while(self.port.inWaiting() > 0):
                self.port.read(self.port.inWaiting())
                time.sleep(0.01)
            self.port.close()

        
#test if the serial port as been given as argument in the terminal
if len(sys.argv) == 1:
    print('Please give the serial port to use as argument')
    sys.exit(0)

#serial reader thread config
#begins the serial thread
reader_thd = serial_thread(sys.argv[1])
reader_thd.start()

#figure config
fig, ax = plt.subplots(num=None, figsize=(10, 10), dpi=80)
fig.canvas.set_window_title('Points plot')
plt.subplots_adjust(left=0.1, bottom=0.25)
fig.canvas.mpl_connect('close_event', handle_close) #to detect when the window is closed and if we do a ctrl-c

#cam graph config with initial plot
graph_cam = plt.subplot(111)
graph_cam.set_ylim([0, max_value])
axes = plt.gca()
axes.set_xlim([-20,600])
axes.set_ylim([-20,800])
plt.ylabel("y")
plt.xlabel("x")
imageID = 0
pointRobot, = graph_cam.plot(0, 0, marker='s', linestyle='-', color='k')
lines3 = [[(0,0),(0 ,0)]]
lc3 = mc.LineCollection(lines3, colors='r', linewidths=2)
collectionRobot = graph_cam.add_collection(lc3)


#timer to update the plot from within the state machine of matplotlib
#because matplotlib is not thread safe...
timer = fig.canvas.new_timer(interval=50)
timer.add_callback(update_plot)
timer.start()

#positions of the buttons, sliders and radio buttons
colorAx             = 'lightgoldenrodyellow'
receiveAx           = plt.axes([0.4, 0.025, 0.1, 0.04])

#config of the buttons, sliders and radio buttons
receiveButton           = Button(receiveAx, 'Start reading', color=colorAx, hovercolor='0.975')

#callback config of the buttons, sliders and radio buttons
receiveButton.on_clicked(reader_thd.setContReceive)

#starts the matplotlib main
plt.show()

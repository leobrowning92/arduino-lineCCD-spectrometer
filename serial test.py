import sys, time, os
import serial
import numpy as np
import matplotlib.pyplot as plt
plt.interactive(True)

try:
    s = serial.Serial( baudrate=115200, port='/dev/ttyUSB1' , timeout=.5 )#, parity=serial.PARITY_EVEN )
except:
    print("issue connecting") 
while True:
    #print "step"
    
    time.sleep(1) #INTEGRATION TIME
    s.write('r')
    for i in range(800):
        print(i)
        #print s.read()
        d = s.readline()
        print(d)
        if d=='':
            print('nothing to read')
    
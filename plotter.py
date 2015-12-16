import sys, time, os
import serial
import numpy as np
import matplotlib.pyplot as plt
plt.interactive(True)

try:
    s = serial.Serial( baudrate=115200, port='/dev/ttyUSB1' , timeout=.5 )#, parity=serial.PARITY_EVEN )
except:
    print("issue connecting") 
#os.system('stty -F /dev/ttyUSB0 115200 cs8 cread clocal')

dat = np.zeros((800))
x=range(800)
p = plt.plot( x, dat )
#print type(p)
plt.ylim(0,200)
time.sleep(0.5)

while True:
    #print "step"
    s.write('r')
    time.sleep(0.002) #INTEGRATION TIME
    for i in range(800):
        print(i)
        #print s.read()
        d = s.readline()
        print(d)
        if d=='':
        	print('nothing to read')
        	dat[i] = np.float(d.rstrip())
    		p, = plt.plot( range(800), dat )
    		plt.set_ydata(dat)
    		plt.draw()
    
    



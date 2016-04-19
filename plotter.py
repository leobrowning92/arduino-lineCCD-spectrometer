import serial
import time
import matplotlib.pyplot as plt

plt.interactive(True)
print 'import'


# open up dummy serial to reset the arduino with
s = serial.Serial(port='/dev/ttyUSB1')

# reset the arduino
s.flushInput()
s.setDTR(level=False)
time.sleep(0.5)
# ensure there is no stale data in the buffer
s.flushInput()
s.setDTR()
time.sleep(1)

# now open up a new serial line for communication
s = serial.Serial(baudrate=115200, port='/dev/ttyUSB1', timeout=0.01)

#initializes plotting axis
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

# initializes data
data=[]
# time for system to settle after opening serial port
time.sleep(1)
# initial read command
s.write('r')
#continuous loop that will plot the data anew each time it runs, as well as
#pass the read command to the arduino
while True:
    s.write('r') #read command

    #loop which iterates through the serial being read, only taking
    #non-empty values and appending them to the data set
    while True:
        value=s.readline()
        if value !='':
            data.append(float(value.rstrip()))

            #determines the length of the dataset to observe
            if len(data)==800:
                break

    #plots the dataset
    ax1.clear()
    ax1.plot( range(len(data)), data )
    plt.draw()

    data=[]

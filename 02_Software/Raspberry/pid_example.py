import numpy as np
import time
import struct
import serial
import h5py
import random

# Environment
dt = 0.01

# Controller
pa = 2.2
ia = 0.5
da = 0.2
inta = 0.0

pv = 0.4
iv = 0.08
intv = 0.0

# Preallocate 10MB buffer (writing to disk while controller is running can delay the processor)
# This can make it run for a maximum of 41.6 Minutes
buffer = np.zeros(shape = (250000, 10), dtype = np.float32)
u = np.zeros(2, dtype = np.float32)
i = 0 #counting variable for the columns of the buffer

# Start Serial Port
serialobj = serial.Serial('/dev/ttyAMA0', 1000000)

# Start communication
serialobj.flushInput()
serialobj.write(b'\x00')
sincePrint = time.clock()
sinceStart = time.clock()
try:
    while True:
        inputbuffer_bytes = serialobj.in_waiting
        if inputbuffer_bytes == 4*7:
            # Read data and convert to array
            data_in = serialobj.read(inputbuffer_bytes) #Read bytes as array
            x = np.frombuffer(data_in, dtype=np.float32, count=7) #Convert to numpy array
            buffer[i, 0:7] = x

            # Evaluate the controller
            t = time.clock() - sinceStart
            inta += dt*ia*x[3]
            intv += dt*iv*x[5]
            pid_signal = pv*x[5] + intv + pa*x[3] + inta + da*x[4]

            u[0] = pid_signal
            u[1] = pid_signal
            np.clip(u, -1.0, 1.0, out = u)
            y = np.array([u[0], u[1]] , dtype=np.float32)
            buffer[i, 7:9] = y
            buffer[i, 9] = t
            i += 1

            # Convert Output to bytes and send it back
            data_out = np.getbuffer(y)
            serialobj.write(b'\x08') # 8 bytes to come
            serialobj.write(data_out)

            if time.clock()-sincePrint>1.0:
                sincePrint = time.clock()
                print("State: 1-4: "+str(x[0:4]))
                print("State: 5-7: "+str(x[4:7]))
                print("Action    : "+str(y))


        elif inputbuffer_bytes>4*7:
            #print "Buffer was more than 4*7 bytes, there were "+str(inputbuffer_bytes)+" bytes"
            print("Received too much, had "+str(inputbuffer_bytes)+" bytes")
            serialobj.flushInput()
            serialobj.write(b'\x00')

except KeyboardInterrupt:
    h5f = h5py.File('data.h5', 'w')
    h5f.create_dataset('xu', data=buffer[0:i, :])
    h5f.close()

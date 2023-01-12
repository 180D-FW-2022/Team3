import warnings
import serial
import serial.tools.list_ports
import time

arduino_ports = [
    p.device
    for p in serial.tools.list_ports.comports()
    if ('Arduino' in p.description or 'IOUSBHostDevice' in p.description)  # may need tweaking to match new arduinos
]
if not arduino_ports:
    raise IOError("No Arduino found")
if len(arduino_ports) > 1:
    warnings.warn('Multiple Arduinos found - using the first')

ser = serial.Serial(arduino_ports[0], timeout = 0.5)
time.sleep(4)
print(ser.name)         # check which port was really used

angle = 100
#ser.write(str.encode("rot"+str(i)+"deg"+",100spd"))#send the following value
bytes_val = angle.to_bytes(2, 'big', signed=True)
ser.write(str.encode('d'))
ser.write(bytes_val)
time.sleep(0.1)
if(bytes_val == ser.read(2)):
    print("received")
else:
    #b'\x61'
    ser.write(b'\xFF')
    
    #print(i)
ser.close()             # close port


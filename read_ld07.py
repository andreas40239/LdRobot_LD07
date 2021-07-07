import serial
import time
import binascii
import math

ld07_startMeasurement = [0xAA,0xAA,0xAA,0xAA,0x01,0x02,0x00,0x00,0x00,0x00,0x03]
ld07_stopMeasurement = [0xAA,0xAA,0xAA,0xAA,0x01,0x0F,0x00,0x00,0x00,0x00,0x10]
ld07_header = 0xAA
ld07_distances = list()
ld07_confidences = list()


ser = serial.Serial(port='/dev/ttyUSB0',    
                    baudrate=921600,
                    timeout=10.0,
                    bytesize=8,
                    parity='N',
                    stopbits=1)

ser.write(ld07_startMeasurement)
time.sleep(0.1)

### Search for header ################################
searchHeader = True
while searchHeader:
    
    #read data from interface until header 0xAA received
    headernotFound = True
    while headernotFound:
        b = ser.read(1)
        tmpInt = int.from_bytes(b, 'little')
        if tmpInt == ld07_header:
            #repeat 3 times
            headernotFound = False

    # next 3 times must be 0xAA, too.
    # I am too distracted to code a beautiful algorithm now :-/
    #next byte must be 0xAA
    b = ser.read(1)
    tmpInt = int.from_bytes(b, 'little')
    if tmpInt == ld07_header:
        #next byte must be 0xAA
        b = ser.read(1)
        tmpInt = int.from_bytes(b, 'little')
        if tmpInt == ld07_header:
            #next byte must be 0xAA
            b = ser.read(1)
            tmpInt = int.from_bytes(b, 'little')
            if tmpInt == ld07_header:
                searchHeader = False


### Read data frame ################################

#reading first data ...
ld07_deviceAddress = int.from_bytes(ser.read(1), byteorder='little')
ld07_cmdCode = int.from_bytes(ser.read(1), byteorder='little')
ld07_packetOffsetAddress = int.from_bytes(ser.read(2), byteorder='little')
ld07_dataLength = int.from_bytes(ser.read(2), byteorder='little')
ld07_timestamp = int.from_bytes(ser.read(4), byteorder='little')

#reading all 160 distance measurement points
#first D1-D80 represent right camera, D81-D160 represent left camera (left/right from top view facing scan direction)
#distance are bits 0..8 (9 digits)
#confidence are bits 9..15 (7 digits)
for measurement in range(160):
    tmpMeasurement = int.from_bytes(ser.read(2), byteorder='little')
    tmpConfidence = (tmpMeasurement >> 9) <<1
    tmpDistance = tmpMeasurement & 0x1ff
    ld07_distances.append(tmpDistance)
    ld07_confidences.append(tmpConfidence)
    #print(f"{str(measurement)}:\t{bin(tmpMeasurement)}\t d: {str(tmpDistance)} mm\t Conf: {str(tmpConfidence)}")


#reading and verifying checksum
#ld07_checksum = int.from_bytes(ser.read(1), byteorder='little')
#tmpChecksum = ld07_deviceAddress + ld07_cmdCode + ld07_packetOffsetAddress + ld07_dataLength + ld07_timestamp
#tmpChecksum += ... all measurements
#currently wrong, because tmpChecksum must be uint8, not python-integer


debug = True
if debug:
    interface_data = ser.read(40)
    print("Distance-Data:",interface_data.hex())
    data_hex = binascii.hexlify(interface_data)
    print("data hex", data_hex)
    data_string= str(data_hex)
    print("data string",data_string)




ser.write(ld07_stopMeasurement)
time.sleep(0.1)


debug = True
if debug:
    print(f"ld07_deviceAddress:\t\t{str(ld07_deviceAddress)}")
    print(f"ld07_cmdCode:\t\t\t{str(ld07_cmdCode)} (must be 2)")
    print(f"ld07_packetOffsetAddress:\t{str(ld07_packetOffsetAddress)}")
    print(f"ld07_dataLength:\t\t{str(ld07_dataLength)} (must be 324)")
    print(f"ld07_timestamp:\t\t\t{str(ld07_timestamp)}")
    inputBufferLength = ser.in_waiting
    print(f"Input buffer length:\t\t{inputBufferLength}")

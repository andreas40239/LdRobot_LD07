# Step 1: disconnect and reconnect your TTL-to-USB adapter
# Step 2: Type this command in your linux-shell:   dmesg | grep tty
#         You should see which USB-device was connected and how it is called
# Step 3: Edit the name in line 10
import serial
import time
import binascii

interface_ld07 = serial.Serial('/dev/ttyUSB0',921600)  
#/dev/ttyUSB0   115200 9600 921600
if interface_ld07.isOpen() :
    print("open success")
else :
    print("open failed") 


ld07_DevicesConnectedCount = [0xAA,0xAA,0xAA,0xAA,0x00,0x16,0x00,0x00,0x00,0x00,0x16]
# returns b'\xaa\xaa\xaa\xaa\x01\x16\x00\x00\x00\x00\x17' -> one device connected

ld07_getCorrectionData = [0xAA,0xAA,0xAA,0xAA,0x01,0x12,0x00,0x00,0x00,0x00,0x13]
# returns b'\xaa\xaa\xaa\xaa\x01\x12\x00\x00\x12\x00}\x00\x00\x00\x83\x00\x00\x00\xc4\x14\x00\x00n\x18\x00\x00P\x00\xd3'

ld07_getDistance = [0xAA,0xAA,0xAA,0xAA,0x01,0x02,0x00,0x00,0x00,0x00,0x03]

ld07_stopGetDistance = [0xAA,0xAA,0xAA,0xAA,0x01,0x0F,0x00,0x00,0x00,0x00,0x10]


interface_ld07.write(ld07_getDistance)
time.sleep(0.1)


def main():
    global interface_ld07
    time.sleep(0.2) 
    num=interface_ld07.inWaiting()
            
    if num: 
        try:   #try to read hexadecimal data
            interface_data = interface_ld07.read(num)
            print(interface_data)   

            #data= str(binascii.b2a_hex(interface_data))

        except:
            pass

while True:
    main()


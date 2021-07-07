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
ld07_getDistance = [0xAA,0xAA,0xAA,0xAA,0x01,0x02,0x00,0x00,0x00,0x00,0x03]
ld07_stopGetDistance = [0xAA,0xAA,0xAA,0xAA,0x01,0x0F,0x00,0x00,0x00,0x00,0x10]




interface_ld07.write(ld07_getCorrectionData)
time.sleep(0.1)


time.sleep(0.2) 
num=interface_ld07.inWaiting()
        
if num: 
    try:   #try to read hexadecimal data
        interface_data = interface_ld07.read(num)
        print(interface_data)   
        #data_hex = binascii.hexlify(interface_data)
        #print(data_hex)
        #data_string= str(data_hex)
        #print(data_string)
        #data_list = ['{:02x}'.format(b) for b in interface_data]
        #print(data_list)

        #check for valid header first
        if interface_data[0] == 0xaa and interface_data[1] == 0xaa and interface_data[2] == 0xaa and interface_data[3] == 0xaa:
            #read calibration correction data
            coe_k0_1 = interface_data[10:11].hex()
            coe_k0_2 = interface_data[11:12].hex()
            coe_k0_3 = interface_data[12:13].hex()
            coe_k0_4 = interface_data[13:14].hex()
            coe_k0 = int(coe_k0_1, 16) + int(coe_k0_2, 16)*16 + int(coe_k0_3, 16)*256 + int(coe_k0_4, 16)*512
            k0 = coe_k0 / 10000.0
            
            coe_k1_1 = interface_data[14:15].hex()
            coe_k1_2 = interface_data[15:16].hex()
            coe_k1_3 = interface_data[16:17].hex()
            coe_k1_4 = interface_data[17:18].hex()
            coe_k1 = int(coe_k1_1, 16) + int(coe_k1_2, 16)*16 + int(coe_k1_3, 16)*256 + int(coe_k1_4, 16)*512
            k1 = coe_k1 / 10000.0

            coe_b0_1 = interface_data[18:19].hex()
            coe_b0_2 = interface_data[19:20].hex()
            coe_b0_3 = interface_data[20:21].hex()
            coe_b0_4 = interface_data[21:22].hex()
            coe_b0 = int(coe_b0_1, 16) + int(coe_b0_2, 16)*16 + int(coe_b0_3, 16)*256 + int(coe_b0_4, 16)*512
            b0 = coe_b0 / 10000.0
            
            coe_b1_1 = interface_data[22:23].hex()
            coe_b1_2 = interface_data[23:24].hex()
            coe_b1_3 = interface_data[24:25].hex()
            coe_b1_4 = interface_data[25:26].hex()
            coe_b1 = int(coe_b1_1, 16) + int(coe_b1_2, 16)*16 + int(coe_b1_3, 16)*256 + int(coe_b1_4, 16)*512
            b1 = coe_b1 / 10000.0

            print("Calibration correction: Left camera")
            print ("k0:",k0,"  b0:", b0)
            print("Calibration correction: Right camera")
            print ("k1:",k1,"  b1:", b1)
            points = int(interface_data[26:27].hex(), 16)
            print("Points per camera:", points)

        else:
            print("wrong response")



    except:
        print("wrong again")
        pass

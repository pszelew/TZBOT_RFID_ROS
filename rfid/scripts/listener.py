#!/usr/bin/env python
import rospy
import serial

from rfid.msg import RFID
from std_msgs.msg import String, Bool
import subprocess
import os
import time
import libscrc

"""
 Folder where script is located
"""
path = os.path.dirname(os.path.abspath(__file__))
 
"""
 Search for our converter
 """
port_def = subprocess.check_output([path +"/find_usb.bash"], shell=True)
port_def = port_def.strip()
rospy.Publisher('rfid_result_read', RFID, queue_size=10)
rospy.Publisher('rfid_result_write', Bool, queue_size=10)


"""
Wysylanie wiadomosci do topicu rfid_result_read. Sygnalizuje odczyt danych z tagu.
"""
def send_message_read(message):
    pub = rospy.Publisher('rfid_result_read', RFID, queue_size=10)
    try:
        RFID_msg = RFID(message[5], message[6], message[7], message[8], message[9], message[10], message[11], message[12], message[13], message[14], message[15])
        pub.publish(RFID_msg)
    except:
        rospy.loginfo('RFID not responding') 
    
        
"""
Wysylanie wiadomosci do topicu rfid_result_write. Sygnalizuje status zapisu na tagu.
True - powodzenie
False - niepowodzenie
"""
def send_message_write(message):
    pub = rospy.Publisher('rfid_result_write', Bool, queue_size=10)
    try:
        if(message[6]==0):
            pub.publish(True)
        else:
            pub.publish(False)
    except:
        rospy.loginfo('RFID not responding') 
    
"""
Dokodowanie wiadomosci z typu string na int
"""
def decode_msg(message):
    intRet = []
    for ch in message:
        intRet.append(int(hex(ord(ch)), 16))
    return intRet
    
    
"""
Poszukaj w systemie konwertera RS232 
"""
while(port_def==""):
    print("USB RS232 not found! Press enter to search again")
    raw_input('')
    port_def = subprocess.check_output([path +"/find_usb.bash"], shell=True)
    port_def = port_def.strip()
print("Found USB RS232 converter on port: " + port_def)   
rospy.loginfo('Starting node') 
    

"""
Funkcja oblicza CRC16-modbus i zwraca ich rezultaty jako liczby typu int
"""
def calcCrc(code):
    crc16 = libscrc.modbus(code)
    crc16 = hex(crc16).split('x')[-1]
    crc16H = crc16[0:2]
    crc16L = crc16[2:4]
    return int(crc16H, 16), int(crc16L, 16)

"""
    @brief Funkcja zwrotna, ktrora wysyla dana komende za pomoca adaptera RS232
    @param[in] message: Otrzymana wiadomosc typu String
    r: czytaj dane z tagu RFID
    w*: zapisz dane na tagu RFID, gdzie * to maksymalnie 8 znakow, ktore maja byc zapisane
    l: odczyt danych z tagu RFID w petli
    @param[in] port: Port USB RS232 do ktorego podlaczony jest konwerter RS232
"""
def callback(message, portT = port_def):
    
    code = ""
    # open serial port
    ser = serial.Serial(port=portT, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)  
    message = str(message)
    message = message.replace('data: "', '')
    message = message.replace('"', '')
    try:
        if(message[0] == 'r'):
            #request data
            rospy.loginfo('Read data. Waiting for response.')
            #read data from RFID
            code = bytearray([0x52, 0x43, 0x6f, 0x64, 0x65, 0x00, 0x3f, 0xfd])
            
            ser.write(code)
            #one second to get an answer
            #read values from RFID
            read_val = ser.read(18)
            
            read_int = decode_msg(read_val)
            
            send_message_read(read_int)
            
        if(message[0]=='w'):
            rospy.loginfo('Write data. Asking for response.')
            a = []
            for i in range(0, len(message)-1):
                a.append(message[i+1])
            for i in range(len(message)-1, 8):
                a.append(0x00)
          
            #code = bytearray([0x57, 0x43, 0x6f, 0x64, 0x65, 0x00, a[0], a[1], a[2], a[3],a[4],a[5],a[6],a[7], 0x9a, 0x5a])
            code = bytearray([0x57, 0x43, 0x6f, 0x64, 0x65, 0x00, a[0], a[1], a[2], a[3],a[4],a[5],a[6],a[7]])
            crc16 = calcCrc(code)
            code.append(crc16[1])
            code.append(crc16[0])
            
            ser.write(code)
            #one second to get an answer
            #read values from RFID
            read_val = ser.read(9)
            
            read_int = decode_msg(read_val)
          
            send_message_write(read_int)
            
            
            
        if(message[0] == 'l'):                
            rospy.loginfo('Loop read. Waiting for response.')
            pause_time = int(message[1:])
            while(1):
                #read data from RFID
                code = bytearray([0x52, 0x43, 0x6f, 0x64, 0x65, 0x00, 0x3f, 0xfd])
                ser.write(code)
                #one second to get an answer
                #read values from RFID
                read_val = (ser.read(18))
                
                read_int = decode_msg(read_val)
            
                send_message_read(read_int)
                    
                time.sleep(pause_time/1000.0)
    except: 
        rospy.logwarn('Unknown command')
    ser.close()

"""
    @brief Listener function that inits node ''listener' and create topic 'sound_listener'
    @param[in] message: Given message of type string
    @param[in] port Port USB UART converter is connected to
"""
def listener():
    rospy.Subscriber('rfid_listener', String, callback)
    rospy.init_node('listener', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    listener()

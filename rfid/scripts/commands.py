#!/usr/bin/env python

import rospy
from std_msgs.msg import String

"""
    @brief Function sends given message to ''RFID_listener' topic
    @param[in] message: Given message of type string
"""

def send_message(message):
    pub = rospy.Publisher('rfid_listener', String, queue_size=10) 
    rospy.loginfo('Sending message: %s', message)
    pub.publish(message)
"""
    @brief Main function of program, getting input from console and sending it further
"""

if __name__ == '__main__':
    message = ""
    rospy.init_node('rfid_player', anonymous=True) 
    send_message('')
    while(message != 'q'):
        try:
            print('Commands for TZBOT RFID reader/writer')    
            message = raw_input("Choose an option (type help for more info) ")
    
            if(message[0] == 'h'):
                print('\n type:')
                print('r -- to read data from RFID card')
                print('w* -- to write data to save on RFID card, where * is a thing to be saved. Max 8 bytes')
                print('l* -- to read data in loop from RFID card, where * is reading period in ms')
                print('q -- to quit \n')
            elif(message[0] == 'q'):
                pass
            else:
                send_message(message)
        except:
            print('Something went wrong!')
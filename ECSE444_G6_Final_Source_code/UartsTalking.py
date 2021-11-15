import serial
import json
import time
import sys

# Uart transimit is stable if the bytes < 383
Conv2d_1_Ws = []
Conv2d_1_Bs = []
Conv2d_2_Bs = []
Conv2d_3_Bs = []
animal = []
num_of_packets_animal = 0

musicNotes = ''

try:
    #Define Port
    #bitrate，from these standard：50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200
    bps=115200
    #Timeout : second
    timex=10000
    # Open Port and Check
    ser=serial.Serial("/dev/cu.usbmodem145103",bps,timeout=timex)
    print("Port：", ser)
    print(ser.port)#Get the port
    print(ser.baudrate)#Get the bit rate
    print('Start')#Get the bit rate
    counter = 0
    #Loop. Too tired to think about thread
    while True:
        if ser.in_waiting:
            str=ser.read(ser.in_waiting ).decode("UTF-8")
            if(str=="exit"):
                break
            if (str[0]=='p'):
                musicNotes = [['                              ','                              ','               @@@            ','             *@@,*@           ','             @/   &.          ','             @   @@           ','             & @@@.           ','           .@@@@#             ','         @@@@@/               ','       @@@@   @               ','//////@@@(//(@@@@@@#//////////','      @%   @@&,& (@@@         ',',,,,,,@(,,*@,,,(,,,#@(,,,,,,,,','       (%   %   (  #&         ','          %%.   %%%           ','                 ,            ','         .@@#    %            ','        .@@@@/   &            ','          &&, .@.             ']]

                for i in str[1:-1]:
                    if i is '1':
                        m = ['                              ','                              ','                              ','                              ','                              ','                              ','                              ','                              ','                              ','             &                ','/////////////&////////////////','             &                ',',,,,,,,,,,,,,&,,,,,,,,,,,,,,,,','             &                ','             &                ','             %                ','       .@@@@@@                ','      #@@@@@@.                ','         .                    ']
                        musicNotes.append(m)
                    elif i is '2':
                        m = ['                              ','                              ','                              ','                              ','                              ','                              ','                              ','                              ','              ,               ','              *               ','//////////////#///////////////','              *               ',',,,,,,,,,,,,,,/,,,,,,,,,,,,,,,','              *               ','              *               ','        ,@@@@@@               ','       .@@@@@@                ','                              ','                              ']
                        musicNotes.append(m)
                    elif i is '3':
                        m = ['                              ','                              ','                              ','                              ','                              ','                              ','                              ','           *                  ','           /                  ','           /                  ','///////////#(/////////////////','           /                  ',',,,,,,,,,,,(,,,,,,,,,,,,,,,,,,','           /                  ','    .@@@@@@*                  ','    @@@@@@*                   ','                              ','                              ','                              ']
                        musicNotes.append(m)
                    elif i is '4':
                        m=['                              ','                              ','                              ','                              ','                              ','                              ','                %             ','                %             ','                %             ','                %             ','////////////////&/////////////','                %             ',',,,,,,,,,,,,,,,,%,,,,,,,,,,,,,','          @@@@@@%             ','         #@@@@@.              ','                              ','                              ','                              ','                              ']
                        musicNotes.append(m)
                    elif i is '5':
                        m=['                              ','                              ','                              ','                              ','                              ','                              ','                              ','                              ','                              ','             @@@@(            ','///////////@@@@@@@////////////','          * #&(               ',',,,,,,,,,,(,,,,,,,,,,,,,,,,,,,','          *                   ','          *                   ','          *                   ','          *                   ','          *                   ','          .                   ']
                        musicNotes.append(m)
                    elif i is '0':
                        m=['                              ','                              ','                              ','                              ','                              ','                              ','                              ','                              ','             @@@@@/           ','           %@@@@@@.           ','///////////&//////////////////','           %                  ',',,,,,,,,,,,%,,,,,,,,,,,,,,,,,,','           %                  ','           %                  ','           %                  ','           (                  ','                              ','                              ']
                        musicNotes.append(m)
                    else:
                        continue
                N = len(musicNotes[0])
                C = len(musicNotes)
                final = ''
                for i in range(19):
                    for j in range(C):
                        final+=musicNotes[j][i]
                        if j == C-1:
                            final += '\n'
                print(final)
            else:
                print(str,end ="")

    
    print("---------------")
    ser.close()


except Exception as e:
    print("---Exception---：",e)
#!/usr/bin/env python3

import pygame
from time import *
from gpiozero import Motor, OutputDevice
import numpy as np 

#Buttons (digital 0 or 1)
#0-select, 1-Lstick, 2-Rstick, 3-start, 4-Dup, 5-Dright, 6-Ddown, 7-Dleft, 8-L2, 9-R2, 10-L1, 11-R1
#12-triangle, 13-circle, 14-x, 15-square, 16-PS button

#Axes (-1 to 1, 0 center)
#0-Ls_LR, 1-Ls_UD, 2-L2, 3-Rs_LR, 4-Rs_UD, 5-R2

# Initialise the pygame library
pygame.init()

# Connect to the first JoyStick
controller = pygame.joystick.Joystick(0)
controller.init()

print ('Initialized Joystick : %s' % controller.get_name())

axis_num = controller.get_numaxes()
print ('Number of axis: %s' %axis_num)

#Moto Zero pins (enable, pos, neg)

motor1 = Motor(24,27);
motor1_enable = OutputDevice(5,initial_value=1);
prev_motor1_val = 0.5;

#Arm switch must be on to run motor. Default off if no connection
def arm_bot():
    GPIO.output(armed, 1);

#Button and axis variables (Top = -1, Bottom = 1; Left = -1, Right = 1)
axis_names_list = ['Lstick_leftright','Lstick_updown','L2','Rstick_leftright','Rstick_updown','R2'];
axis_values = [];
but_names_list = ['X','O','TRI','SQ','L1','R1','L2','R2','SEL','SRT','PS','LCK','RCK','UP','DN','L','R'];
but_values = [];
Rstick_updown = 0.0001;
Rstick_leftright = 0.0001;

armed = 1;

while armed == 1:
    #Get controller values
    if pygame.mixer.get_busy() != None:
        for event in pygame.event.get():

            #Axes
            Lstick_leftright, Lstick_updown = controller.get_axis(0), controller.get_axis(1);
            Rstick_leftright, Rstick_updown = controller.get_axis(3), controller.get_axis(4);
            L2, R2 = controller.get_axis(2), controller.get_axis(5);
            
            #Prevent arctan divide by zero
            if Rstick_leftright == 0:
                Rstick_leftright = 0.00001; 

            '''		
            #Buttons
            if event.type == pygame.JOYBUTTONDOWN:
                for i in range(0,len(but_names_list)):
                    button_val = controller.get_button(i);
                    if but_names_list[i] == 'X' and button_val == 1:
                        wheelFL_val = 200;
                        #print(str(but_names_list[i])+ ' button pressed');
                    if but_names_list[i] == 'O' and button_val == 1:
                        circle = True;
                        #print(str(but_names_list[i])+ ' button pressed');\
                    if but_names_list[i] == 'SQ' and button_val == 1:
                        #print(str(but_names_list[i])+ ' button pressed');
                        square = True;
                    if but_names_list[i] == 'TRI' and button_val == 1:
                        #print(str(but_names_list[i])+ ' button pressed');
                        triangle = True;
                    if but_names_list[i] == 'L1' and button_val == 1:
                        #print(str(but_names_list[i])+ ' button pressed');
                        L1 = True;
                    if but_names_list[i] == 'R1' and button_val == 1:
                        #print(str(but_names_list[i])+ ' button pressed');
                        R1 = True;
                    if but_names_list[i] == 'L2' and button_val == 1:
                        #print(str(but_names_list[i])+ ' button pressed');
                        L2 = 1;
                    if but_names_list[i] == 'R2' and button_val == 1:
                        #print(str(but_names_list[i])+ ' button pressed');
                        R2 = 1;
                    if but_names_list[i] == 'SEL' and button_val == 1:
                        print(str(but_names_list[i])+ ' button pressed');
                    if but_names_list[i] == 'SRT' and button_val == 1:
                        print(str(but_names_list[i])+ ' button pressed');
                    if but_names_list[i] == 'L' and button_val == 1:
                        print(str(but_names_list[i])+ ' button pressed');
                    if but_names_list[i] == 'R' and button_val == 1:
                        print(str(but_names_list[i])+ ' button pressed');
                    if but_names_list[i] == 'DN' and button_val == 1:
                        print(str(but_names_list[i])+ ' button pressed');
                    if but_names_list[i] == 'UP' and button_val == 1:
                        print(str(but_names_list[i])+ ' button pressed');
                    if but_names_list[i] == 'RCK' and button_val == 1:
                        print(str(but_names_list[i])+ ' button pressed');
                    if but_names_list[i] == 'LCK' and button_val == 1:
                        print(str(but_names_list[i])+ ' button pressed');
                    if but_names_list[i] == 'PS' and button_val == 1:
                        print(str(but_names_list[i])+ ' button pressed \n');
                        print(np.arctan(Rstick_updown/Rstick_leftright)*(180/np.pi));
                    else:
                        kill_motors(motorFL,motorFR,motorBL,motorBR);
            '''
    #Determine what direction Modias moves. Should set theta to 0-360 value
    if Rstick_updown <= 0 and Rstick_leftright <= 0:
        theta = np.arctan((Rstick_updown/Rstick_leftright));
    elif Rstick_updown <= 0 and Rstick_leftright >= 0:
        theta = np.arctan((Rstick_updown/Rstick_leftright))+np.pi;
    elif Rstick_updown >= 0 and Rstick_leftright >= 0:
        theta = abs(np.arctan((Rstick_updown/Rstick_leftright)))+np.pi;
    elif Rstick_updown >= 0 and Rstick_leftright <= 0:
        theta = np.arctan((Rstick_updown/Rstick_leftright))+2*np.pi;

    #Determine motor values
    theta = np.arctan((Rstick_updown/Rstick_leftright)); #in radians
    if Rstick_leftright < 0:
        mag = -np.sqrt(Rstick_updown**2+Rstick_leftright**2);
    else:
        mag = np.sqrt(Rstick_updown**2+Rstick_leftright**2);
    #motor1.value = mag*np.cos(theta);
    #motorFR_val = 0;
    #motorBL_val = 0;
    #motorBR_val = 0;

    if motor1.value != prev_motor1_val:
        print('Rstick L:',Rstick_leftright,'Rstick_updown:',Rstick_updown);
        print('Angle', theta);
        print(motor1.value);

    #LED position
    #map theta (0-360) to number of LEDs (29?).
    #power LED

    #Correct motors for direction
    #motor_correct()
    #Set motor to last value above
    if mag > 0.20 or mag < -0.20:
        motor1.value = mag*np.cos(theta);
    else:
        motor1.value = 0;
        #motor2.value = 0;
        #motor3.value = 0;
        #motor4.value = 0;

    # Motor values can only be updated every 20ms
    sleep(0.5);

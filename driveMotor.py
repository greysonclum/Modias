#!/usr/bin/env python3

import pygame
from time import *
import RPi.GPIO as GPIO
import numpy as np 

#Buttons (digital 0 or 1)
#0-select, 1-Lstick, 2-Rstick, 3-start, 4-Dup, 5-Dright, 6-Ddown, 7-Dleft, 8-L2, 9-R2, 10-L1, 11-R1
#12-triangle, 13-circle, 14-x, 15-square, 16-PS button

#Axes (-1 to 1, 0 center)
#0-5

# Initialise the pygame library
pygame.init()

# Connect to the first JoyStick
controller = pygame.joystick.Joystick(0)
controller.init()

print ('Initialized Joystick : %s' % controller.get_name())

axis_num = controller.get_numaxes()
print ('Number of axis: %s' %axis_num)

#Set GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

wheelFL = 7;                #Not the pin number. GPIO number 7
wheelFR = 11;
wheelBL = 13;
wheelBR = 15;
armed = 0;
wheelFL_val = 0;
wheelFR_val = 0;
wheelBL_val = 0;
wheelBR_val = 0;

GPIO.setup(armed, GPIO.OUT)
GPIO.setup(motorFL, GPIO.OUT)
GPIO.setup(motorFR, GPIO.OUT)
GPIO.setup(motorBL, GPIO.OUT)
GPIO.setup(motorBR, GPIO.OUT)

#Arm switch must be on to run motor. Default off if no connection
def arm_bot():
    GPIO.output(armed, 1)

def kill_motors():
    GPIO.output(motorFL, 0)
    GPIO.output(motorFR, 0)
    GPIO.output(motorBL, 0)
    GPIO.output(motorBR, 0)
    #print('Motors off');

def map(val, in_min, in_max, out_min, out_max):
    return (val - in_min)*(out_max-out_min) / (in_max - in_min) + out_min;
    

def input_direction():
    get_angle = np.arctan(Rstick_updown/Rstick_leftright);
    print(get_angle)
    sleep(2);

def setmotors(motorFL_val,motorFR_val,motor_BL_val,motorBR_val):
    GPIO.output(motorFL, motorFL_val)
    GPIO.output(motorFR, motorFR_val)
    GPIO.output(motorBL, motorBL_val)
    GPIO.output(motorBR, motorBR_val)

#Button and axis variables (Top = -1, Bottom = 1; Left = -1, Right = 1)
axis_names_list = ['Lstick_leftright','Lstick_updown','L2','Rstick_leftright','Rstick_updown','R2'];
axis_values = [];
but_names_list = ['X','O','TRI','SQ','L1','R1','L2','R2','SEL','SRT','PS','LCK','RCK','UP','DN','L','R'];
but_values = [];
Rstick_updown = .0001;
Rstick_leftright = .0001;

armed = 1;

while armed == 1:
    #Get controller values
    if pygame.mixer.get_busy() != None:
        for event in pygame.event.get():

            #Axes
            for k in range(0,len(axis_names_list)):
                axis_val = controller.get_axis(k);
                if axis_names_list[k] == 'Lstick_updown' and axis_val != 0:
                    #print(str(axis_names_list[k])+ ' axis moved');
                    print(axis_val);
                else:
                    Lstick_updown = .00001; #prevent dividing by zero for arctan
                if axis_names_list[k] == 'Lstick_leftright' and axis_val != 0: #and 0.1 <= axis_val <= -0.1:
                    #print(str(axis_names_list[k])+ ' axis moved');
                    print(axis_val);
                else:
                    Lstick_leftright = .00001;
                if axis_names_list[k] == 'Rstick_updown' and axis_val != 0:
                    Rstick_updown = axis_val;
                    #print(str(axis_names_list[k])+ ' axis moved');
                else:
                    Rstick_updown = .00001;
                if axis_names_list[k] == 'Rstick_leftright' and axis_val != 0:
                    Rstick_leftright = axis_val;
                else:
                    Rstick_updown = .00001;
                    #print(str(axis_names_list[k])+ ' axis moved');
                if axis_names_list[k] == 'L2' and axis_val != -1 and axis_val != 0:
                    #print(str(axis_names_list[k])+ ' axis moved');
                    print(str(axis_val));
                if axis_names_list[k] == 'R2' and axis_val != -1 and axis_val != 0:
                    #print(str(axis_names_list[k])+ ' axis moved');
                    print(str(axis_val));
		
            #Buttons    
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
                    kill_motors();

        #Determine what direction Modias moves. Should set theta to 0-360 value.
        if Rstick_updown <= 0 and Rstick_leftright <= 0;
            theta = np.arctan((Rstick_updown/Rstick_leftright));
        if Rstick_updown <= 0 and Rstick_leftright >= 0;
            theta = np.arctan((Rstick_updown/Rstick_leftright))+180;
        if Rstick_updown >= 0 and Rstick_leftright >= 0;
            theta = abs(np.arctan((Rstick_updown/Rstick_leftright)))+180;
        if Rstick_updown >= 0 and Rstick_leftright <= 0;
            theta = np.arctan((Rstick_updown/Rstick_leftright))+360;

        #Determine motor values
        theta = np.arctan((Rstick_updown/Rstick_leftright)); #in radians
        mag = np.sqrt(Rstick_updown**2+Rstick_leftright**2);
        motorFL_val = 255*mag*cos(theta+(45/180*np.pi));
        motorFR_val = 0;
        motorBL_val = 0;
        motorBR_val = 0;

        #LED position
        #map theta (0-360) to number of LEDs (29?).
        #power LED

    #Correct motors for direction
    #motor_correct()
    #Set motor to last value above
    setmotors(motorFL_val, motorFR_val, motorBL_val, motorBR_val)

    # Motor values can only be updated every 20ms
    sleep(.2);


#!/usr/bin/env python3

import pygame
from time import *
from gpiozero import Motor, OutputDevice
import numpy as np 

#//******************************CONTROLLER LAYOUT**********************************************//
#Buttons (digital 0 or 1)
#0-select, 1-Lstick, 2-Rstick, 3-start, 4-Dup, 5-Dright, 6-Ddown, 7-Dleft, 8-L2, 9-R2, 10-L1, 11-R1
#12-triangle, 13-circle, 14-x, 15-square, 16-PS button

#Axes (-1 to 1, 0 center)
#0-Ls_LR, 1-Ls_UD, 2-L2, 3-Rs_LR, 4-Rs_UD, 5-R2

#//********************************INITIALIZE**********************************************//

# INITIALIZE PYGAME LIBRARY
pygame.init()

# CONNECT TO FIRST AVAILABLE CONTROLLER.        leave disarmed if no controller connected
controller = pygame.joystick.Joystick(0)
controller.init()

if controller.init() == True:
    print ('Initialized Joystick : %s' % controller.get_name())
else:
    print('Controller init failed');

axis_num = controller.get_numaxes()
print ('Number of axis: %s' %axis_num)

#GPIOZERO LIBRARY MOTOR PINOUT (enable, pos, neg)
motor1 = Motor(24,27);
motor1_enable = OutputDevice(5,initial_value=1);
motor2 = Motor(6,22);
motor2_enable = OutputDevice(17, initial_value=1);
prev_motor1_val = 0.1;

#DEFAULT NOT ARMED. PRESS SELECT BUTTON TO ARM
def arm_bot(armed):
    GPIO.output(armed, 1);

Rstick_updown = 0;
Rstick_leftright = 0.0001;
armed = 1;


#//*********************************MAIN LOOP*******************************************//

while armed == 1:
    # GET CONTROLLER INPUT
    if pygame.mixer.get_busy() != None:
        for event in pygame.event.get():

            # GET CONTROLLER AXIS VALUES
            Lstick_leftright, Lstick_updown = controller.get_axis(0), controller.get_axis(1);
            Rstick_leftright, Rstick_updown = controller.get_axis(3), controller.get_axis(4);
            L2, R2 = controller.get_axis(2), controller.get_axis(5);
            
            #Prevent arctan divide by zero
            if Rstick_leftright == 0:
                Rstick_leftright = 0.00001; 
		
            #DEFINE BUTTONS (pressed = 1, else = 0)
            if event.type == pygame.JOYBUTTONDOWN:
                X_button, CIR_button, TRI_button, SQ_button = controller.get_button(0), controller.get_button(1), controller.get_button(2), controller.get_button(3);
                L1, R1, L2, R2 = controller.get_button(4), controller.get_button(5), controller.get_button(6), controller.get_button(7);
                SEL, START, PS = controller.get_button(8), controller.get_button(9), controller.get_button(10);
                L_stick, R_stick = controller.get_button(11), controller.get_button(12);
                UP_arrow, DN_arrow, LF_arrow, RT_arrow = controller.get_button(13), controller.get_button(14), controller.get_button(15), controller.get_button(16);

    # DETERMINE DIRECTION 0-360 USING UP AND DOWN COMPONENTS TO GET VECTOR ANGLE. POS VALUES IN 1ST AND 3RD QUADRANT. NEG IN 2ND AND 4TH.
    '''
    if Rstick_updown <= 0 and Rstick_leftright <= 0:
        theta = np.arctan((Rstick_updown/Rstick_leftright));
    elif Rstick_updown <= 0 and Rstick_leftright >= 0:
        theta = np.arctan((Rstick_updown/Rstick_leftright))+np.pi;          #In radians
    elif Rstick_updown >= 0 and Rstick_leftright >= 0:
        theta = abs(np.arctan((Rstick_updown/Rstick_leftright)))+np.pi;
    elif Rstick_updown >= 0 and Rstick_leftright <= 0:
        theta = np.arctan((Rstick_updown/Rstick_leftright))+2*np.pi;
    '''

    # DETERMINE MAGNITUDE (MOTOR SPEED/DIRECTION)
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
        #print('Rstick L:',Rstick_leftright,'Rstick_updown:',Rstick_updown);
        print('Angle: ', theta);
        print('Magnitude: ', motor1.value);

    #LED position
    #map theta (0-360) to number of LEDs (29?).
    #power LED

    #Correct motors for direction
    #motor_correct()

    # TOLERANCE RANGE FOR MOTORS. REDUCE NOISE
    if mag > 0.20 or mag < -0.20:
        motor1.value = mag*np.cos(theta+(45*np.pi/180)); #add 45 degrees for X formation
        motor2.value = mag*np.cos(theta+(90*np.pi/180)); #add 90 degrees
    else:
        motor1.value = 0;
        #motor2.value = 0;
        #motor3.value = 0;
        #motor4.value = 0;

    # UPDATE MOTORS EVERY __ SECONDS
    sleep(0.2);

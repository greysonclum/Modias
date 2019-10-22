#!/usr/bin/env python3

import pygame
from time import *
from gpiozero import Motor, OutputDevice
import numpy as np 
import pigpio as pi

#//******************************PS3 CONTROLLER LAYOUT**************************************//
#Buttons (digital 0 or 1)
#0-select, 1-Lstick, 2-Rstick, 3-start, 4-Dup, 5-Dright, 6-Ddown, 7-Dleft, 8-L2, 9-R2, 10-L1, 11-R1
#12-triangle, 13-circle, 14-x, 15-square, 16-PS button

#Axes (-1 to 1, 0 center)
#0-Ls_LR, 1-Ls_UD, 2-L2, 3-Rs_LR, 4-Rs_UD, 5-R2

#//********************************INITIALIZE**********************************************//

# INITIALIZE PYGAME LIBRARY
pygame.init()

# CONNECT TO FIRST AVAILABLE CONTROLLER. Raspi needs bluetooth on and controller paired/trusted.
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
motor3 = Motor(23,16);
motor3_enable = OutputDevice(12, initial_value=1);
motor4 = Motor(13,18);
motor4_enable = OutputDevice(25, initial_value=1);
prev_motor1_val = 0.1;

#DEFAULT NOT ARMED. PRESS SELECT BUTTON TO ARM. SEE BELOW

Rstick_updown = 0;
Rstick_leftright = 0.0001;
Lstick_updown = 0;
Lstick_leftright = 0.0001;
arm = False;
run = 1;
SEL = 0;


#//*********************************MAIN LOOP*******************************************//

while run == 1:
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
    
    #************ARM BOT WITH SEL ************/
    if SEL == 1:
        sleep(0.5);
        arm = not arm;
        print ("Is armed?: ", arm); #toggle arming. returns True or False

    # DETERMINE DIRECTION 0-360 USING UP AND DOWN COMPONENTS TO GET VECTOR ANGLE (MOTOR CW OR CCW) 
    theta = np.arctan((Rstick_updown/Rstick_leftright)); #in radians
    
    #DETERINE MAGNITUDE (MOTOR SPEED)
    if Rstick_leftright or Rstick_updown < 0:
        Rmag = -np.sqrt(Rstick_updown**2+Rstick_leftright**2);
        if Rmag < 1:
            Rmag = -1;
    else:
        Rmag = np.sqrt(Rstick_updown**2+Rstick_leftright**2);
        if Rmag > 1:
            Rmag = 1;

    if Lstick_leftright < 0:
        Lmag = np.sqrt(Lstick_updown**2+Lstick_leftright**2);
    elif Lstick_leftright >= 0:
        Lmag = np.sqrt(Lstick_updown**2+Lstick_leftright**2);
    
    #LED position every 360/30 = 12 degrees. Actually 29 LEDs
    active_LED = int(theta/360*30);
    #turn on active_LED +1 at 50%
    #turn on active_LED 
    #turn on active_LED -1 at 50%
    
    #map theta (0-360) to number of LEDs (29?).

    # MOVE BOT WITH RSTICK.
    # TOLERANCE RANGE FOR MOTORS. REDUCE NOISE
    if arm == True and (Rmag > 0.20 or Rmag < -0.20):
        motor1.value = Rmag*np.cos(theta);
        motor2.value = Rmag*np.sin(theta); 
        motor3.value = Rmag*np.cos(theta); 
        motor4.value = Rmag*np.sin(theta);
    else:
        motor1.value = 0;
        motor2.value = 0;
        motor3.value = 0;
        motor4.value = 0;

    #ROTATE (YAW) BOT WITH LEFT STICK

    #PRINT PREV MOTOR1 VALUES
    if motor1.value != prev_motor1_val:
        print('Rstick Angle: ', theta);
        print('Motor1 Output: ', motor1.value);
        prev_motor1_val = motor1.value;

    # UPDATE MOTORS EVERY __ SECONDS
    sleep(0.05);

import pygame
from pygame.locals import *
import time
import numpy as np
import sys
sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import mbot_motor_command_t

LIN_VEL_CMD = 1.0 # 1 m/2
ANG_VEL_CMD = 2.0 # 2 rad/sec

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
pygame.init()
pygame.display.set_caption("MBot TeleOp")
screen = pygame.display.set_mode([100,100])
pygame.key.set_repeat(25)
time.sleep(0.5)
running = True
fwd_vel = 0.0
turn_vel = 0.0
while(running):

    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            running = False
            pygame.quit()
            sys.exit()
        key_input = pygame.key.get_pressed()  
        if(~key_input[pygame.K_LEFT] & ~key_input[pygame.K_LEFT] & ~key_input[pygame.K_LEFT] & ~key_input[pygame.K_LEFT]):
            turn_vel = 0
            fwd_vel = 0
        if key_input[pygame.K_UP]:
            fwd_vel = 1.0
        elif key_input[pygame.K_DOWN]:
            fwd_vel = -1.0
        else:
            fwd_vel = 0
        if key_input[pygame.K_LEFT]:
            turn_vel = 1.0
        elif key_input[pygame.K_RIGHT]:
            turn_vel = -1.0
        else:
            turn_vel = 0.0
    command = mbot_motor_command_t()
    command.trans_v = fwd_vel * LIN_VEL_CMD
    command.angular_v = turn_vel * ANG_VEL_CMD
    lc.publish("MBOT_MOTOR_COMMAND",command.encode())
    time.sleep(0.05)

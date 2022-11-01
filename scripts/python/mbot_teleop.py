import pygame
from pygame.locals import *
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
import numpy as np
import sys
sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import mbot_motor_command_t

LIN_VEL_CMD = 1.5
ANG_VEL_CMD = 6.28

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
pygame.init()
pygame.display.set_caption("MBot TeleOp")
screen = pygame.display.set_mode([640,480])
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.5)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    screen.fill([0,0,0])
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = image.swapaxes(0,1)
    image = cv2.flip(image, -1)
    image = pygame.surfarray.make_surface(image)
    screen.blit(image, (0,0))
    pygame.display.update()
    fwd_vel = 0.0
    turn_vel = 0.0
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            pygame.quit()
            sys.exit()
            cv2.destroyAllWindows()
    key_input = pygame.key.get_pressed()  
    if key_input[pygame.K_LEFT]:
        turn_vel += 1.0
    if key_input[pygame.K_UP]:
        fwd_vel +=1.0
    if key_input[pygame.K_RIGHT]:
        turn_vel -= 1.0
    if key_input[pygame.K_DOWN]:
        fwd_vel -= 1.0
    command = mbot_motor_command_t()
    command.trans_v = fwd_vel * LIN_VEL_CMD
    command.angular_v = turn_vel * ANG_VEL_CMD
    lc.publish("MBOT_MOTOR_COMMAND",command.encode())
    rawCapture.truncate(0)

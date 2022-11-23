import pygame
from pygame.locals import *
# from picamera.array import PiRGBArray
# from picamera import PiCamera
import cv2
import time
import numpy as np
import sys
sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import mbot_motor_command_t, timestamp_t

LIN_VEL_CMD = 0.15
ANG_VEL_CMD = np.pi/5

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
pygame.init()
pygame.display.set_caption("MBot TeleOp")
screen = pygame.display.set_mode([320,240])
# pygame.key.set_repeat(100)
# camera = PiCamera()
# camera.resolution = (640, 480)
# camera.framerate = 32
# rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.5)
key_input = pygame.key.get_pressed()  
fwd_vel = 0.0
turn_vel = 0.0
def current_utime(): return int(time.time() * 1e6)

while True:
    # image = frame.array
    # screen.fill([0,0,0])
    # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # image = image.swapaxes(0,1)
    # image = cv2.flip(image, -1)
    # image = pygame.surfarray.make_surface(image)
    # screen.blit(image, (0,0))
    pygame.display.update()
    # fwd_vel = 0.0
    # turn_vel = 0.0
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            pygame.quit()
            sys.exit()
            # cv2.destroyAllWindows()
    key_input = pygame.key.get_pressed()  
    if key_input[pygame.K_a]:
        turn_vel = 1.0
        # print("left")
        # print(turn_vel)
    elif key_input[pygame.K_w]:
        fwd_vel = 1.0
        # print("fwd")
        # print(fwd_vel)
    elif key_input[pygame.K_d]:
        turn_vel = -1.0
        # print("right")
        # print(turn_vel)
    elif key_input[pygame.K_s]:
        fwd_vel = -1.0
        # print("back")
        # print(fwd_vel)
    elif key_input[pygame.K_f]:
        fwd_vel = 0.0
        turn_vel = 0.0
    elif key_input[pygame.K_e]:
        turn_vel = 0.0
    # else:
    #     if fwd_vel > 0:
    #         fwd_vel -= 1.0
            
    #     elif fwd_vel < 0:
    #         fwd_vel += 1.0
            
    #     if turn_vel > 0:
    #         turn_vel -= 1.0
            
    #     elif turn_vel < 0:
    #         turn_vel += 1
            
    command = mbot_motor_command_t()
    command.utime = current_utime()
    command.trans_v = fwd_vel * LIN_VEL_CMD
    command.angular_v = turn_vel * ANG_VEL_CMD
    # print(command.trans_v,command.angular_v)

    drive_time = timestamp_t()
    drive_time.utime = command.utime
    lc.publish("MBOT_TIMESYNC", drive_time.encode())
    lc.publish("MBOT_MOTOR_COMMAND", command.encode())
    # time.sleep(0.05)
    # rawCapture.truncate(0)

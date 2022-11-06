import numpy as np
import sys
sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import mbot_motor_command_t
from gamepad_reader import Gamepad
from moving_window_filter import MovingWindowFilter

LIN_VEL_CMD = 1.0 # 1 m/2
ANG_VEL_CMD = 2.0 # 2 rad/sec

use_gamepad = True
gamepad = Gamepad(0.3, 0.3, np.pi/2)
lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
# pygame.init()
# pygame.display.set_caption("MBot TeleOp")
# screen = pygame.display.set_mode([100,100])
# pygame.key.set_repeat(25)
# time.sleep(0.5)
running = True
fwd_vel = 0.0
turn_vel = 0.0
filter = MovingWindowFilter(10, dim=2)
while(running):
    lin_speed, ang_speed, e_stop = gamepad.get_command()
    cmd = np.array([lin_speed[0], ang_speed])
    filter.calculate_average(cmd)
    command = mbot_motor_command_t()
    command.trans_v = cmd[0]
    command.angular_v = cmd[1]
    lc.publish("MBOT_MOTOR_COMMAND",command.encode())
    # time.sleep(0.05)

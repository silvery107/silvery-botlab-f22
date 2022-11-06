import lcm

from mbot_lcm_msgs import reset_odometry_t

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

reset_odom_msg = reset_odometry_t()
reset_odom_msg.x = 0.0
reset_odom_msg.y = 0.0
reset_odom_msg.theta = 0.0

lc.publish("RESET_ODOMETRY", reset_odom_msg.encode())

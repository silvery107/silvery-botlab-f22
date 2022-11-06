import lcm

from mbot_lcm_msgs import mbot_encoder_t

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

reset_enc_msg = mbot_encoder_t()


lc.publish("RESET_ENCODERS", reset_enc_msg.encode())

import time
import sys
import json

# sys.path.append("/home/pi/botlab-soln/mbot/mbot_lcm_msgs/lcmtypes")
import lcm
from mbot_lcm_msgs import mbot_system_reset_t, occupancy_grid_t

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

msg = mbot_system_reset_t()
msg.slam_mode = 2  # mapping_only=0, action_only=1, localization_only=2, full_slam=3
# msg.slam_map_location = "/home/pi/botlab-soln/data/convex_10mx10m_5cm_offcenter.map"  # Only relevant if in localization only mode.
f = open("/home/pi/botlab-soln/550lab_map.json")
mapdata = json.load(f)
map_obj = occupancy_grid_t()
map_obj.origin_x = float(mapdata['origin'][0])
map_obj.origin_y = float(mapdata['origin'][1])
map_obj.meters_per_cell = float(mapdata['meters_per_cell'])
map_obj.width = int(mapdata['width'])
map_obj.height = int(mapdata['height'])
map_obj.num_cells = int(mapdata['num_cells'])
for i in range(map_obj.num_cells):
    val = mapdata['cells'][i]
    if val==0.5:
        updated_val = 0
    else:
        updated_val = min(int((val*256)-128), 127)
    map_obj.cells.append(updated_val)
msg.map_obj = map_obj

lc.publish("MBOT_SYSTEM_RESET", msg.encode())

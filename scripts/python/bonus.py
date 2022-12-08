import time
import lcm
import os
import sys
import subprocess
import numpy as np
from threading import Thread
from mbot_lcm_msgs import *
from step_test import *
import collections

class Bonus:
    def __init__(self) -> None:
        # Subscribe data
        self.scan = None
        self.particles = None
        self.slam_pose = None
        self.expState = 0
        self.mctrlConfirm = None
        # Control params
        self.maxTransVel = 0.15
        self.maxAngVel = np.pi / 5
        # Lcm handler indicators
        self.haveScan = False
        self.haveParticle = False
        self.haveSlamPose = False
        self.haveExpState = False
        self.haveConfirm = True
        # Convergence 
        self.particle_x = None
        self.particle_y = None
        self.particle_theta = None
        self.particle_weight = None
        self.maxvariance = 0.04

        self.iter = 0
        self.maxIter = 10
        self.taskDone = False
        self.most_recent_path_time = 0.0

    def handle_lcm(self):
        """
        Function to read the lcm data (and to stop the LCM instance on Cntl + C)
        """
        try:
            while True:
                self.lc.handle()

        except KeyboardInterrupt:
            print("lcm exit!")
            sys.exit()

    def lcm_particles_handler(self, channel, data):
        if not self.haveParticle:
            self.particles = particles_t.decode((data))
            self.haveParticle = True

    def lcm_lidar_handler(self, channel, data):
        if not self.haveScan:
            self.scan = lidar_t.decode(data)
            self.haveScan = True

    def lcm_slam_pose_handler(self, channel, data):
        if not self.haveSlamPose:
            self.slam_pose = pose_xyt_t.decode(data)
            self.haveSlamPose = True

    def lcm_exp_status_handler(self, channel, data):
        msg = exploration_status_t.decode(data)
        self.expState = msg.state
        if self.expState == 3 or self.expState == 4:
            print("Expl finished successed or failed, start a new local expl turn!!!!!!!!")
            self.cleanup_local_exploration()
            time.sleep(0.01)
            self.start_local_exploration()
        
    def lcm_mctrl_confirm_handler(self, channel, data):
        if not self.haveConfirm:
            self.mctrlConfirm = message_received_t.decode(data)
            self.haveConfirm = True

    def particle_adj(self):
        particle_x = []
        particle_y = []
        particle_theta = []
        particle_weight = []
        for tmp in self.particles.particles:
            particle_x.append(tmp.pose.x)
            particle_y.append(tmp.pose.y)
            particle_theta.append(tmp.pose.theta)
            particle_weight.append((tmp.weight))
        self.particle_x = np.array(particle_x)
        self.particle_y = np.array(particle_y)
        self.particle_theta = np.array(particle_theta)
        self.particle_weight = np.array(particle_weight)

        sort_idx = np.argsort(self.particle_weight)[int(len(self.particle_weight)/5):]
        self.particle_x = self.particle_x[sort_idx]
        self.particle_y = self.particle_y[sort_idx]

    def initialize(self):
        print("Creating LCM...")
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

        print("Subscribing to LCM channels...")
        self.lc.subscribe("LIDAR", self.lcm_lidar_handler)
        self.lc.subscribe("SLAM_PARTICLES", self.lcm_particles_handler)
        self.lc.subscribe("EXPLORATION_STATUS", self.lcm_exp_status_handler)
        self.lc.subscribe("MSG_CONFIRM", self.lcm_mctrl_confirm_handler)
        # lc.subscribe("SLAM_POSE", self.lcm_slam_pose_handler)
        
        lcm_kill_thread = Thread(target = self.handle_lcm, args=[], daemon = True)
        lcm_kill_thread.start()
        os.system("pkill motion_planning")
        os.system("pkill motion_controll")

    def send_goal_to_planner(self):
        print("Send goal to planning server")
        goal = pose_xyt_t()
        goal.utime = current_utime()
        goal.x = 0
        goal.y = 0
        goal.theta = 0

        path_request = planner_request_t()
        path_request.utime = goal.utime
        path_request.require_plan = 1
        path_request.goal = goal
        lc.publish("PATH_REQUEST", path_request.encode())
        self.most_recent_path_time = path_request.utime
    
    def start_local_exploration(self):
        print("Start local motion controller, slam and exploration")
        os.system("../bin/mctrl_loc --use-local-channels > /dev/null 2>&1 &")
        os.system("../bin/slam_loc --use-local-channels --num-particles 200 > /dev/null 2>&1 &")
        os.system("../bin/exp_loc --use-local-channels > /dev/null 2>&1 &")
    
    def cleanup_local_exploration(self):
        print("Cleaning up running local motion controller, slam and exploration")
        os.system("pkill mctrl_loc")
        os.system("pkill exp_loc")
        os.system("pkill slam_loc")
    
    def start_global_controller(self):
        print("Start motion controller")
        os.system("../bin/motion_controller > /dev/null 2>&1 &")

    def start_motion_planner(self):
        print("Start motion planning server")
        os.system("../bin/motion_planning_server > /dev/null 2>&1 &")

    def runOnce(self):
        if self.haveParticle:
            self.particle_adj()
            varx = np.var(self.particle_x)
            vary = np.var(self.particle_y)
            # vartheta = np.var(self.particle_theta)
            print(varx + vary)
            if varx + vary  < self.maxvariance:
                self.iter += 1
                print("Iter to Converge: (%d, %d)" % (self.iter, self.maxIter))
            else:
                self.iter = 0
            
            if self.iter == self.maxIter:
                self.taskDone = True

            self.haveParticle = False


    def run(self):
        self.initialize()

        self.start_local_exploration()

        while not self.taskDone:
            self.runOnce()
        
        self.cleanup_local_exploration()
        
        self.start_global_controller()
        self.start_motion_planner()
        time.sleep(0.1)
        self.haveConfirm = False
        while True:
            self.send_goal_to_planner()
            time.sleep(1)
            if self.haveConfirm:
                if abs(self.mctrlConfirm.creation_time - self.most_recent_path_time) < 5:
                    break


# Program entry
if __name__ == "__main__":
    agent = Bonus()
    agent.run()
    # agent.send_goal_to_planner()

    # 1. run exploration with full slam and motion controller in local channels
    # 2. check particles distribution (variance and socre?) in global channels
    # 4. loop until converge (50-70% of particles have a very small variance)
    # Note that global slam is running in localization mode with sampling augmentation,
    # while local slam is running in full slam mode.

    # Local channels: SLAM_POSE_LOCAL, SLAM_MAP_LOCAL
    # slam: publish slam_pose and slam_map
    # exploration: subscribe slam_map, slam_pose; publish controller_path
    # motion_controller: subscribe slam_pose, controller_path
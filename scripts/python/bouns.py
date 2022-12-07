import time
import lcm
import os
import sys
import subprocess
import numpy as np
from threading import Thread
from mbot_lcm_msgs import *
from step_test import *

class Bouns:
    def __init__(self) -> None:
        self.scan = None
        self.particles = None
        self.slam_pose = None
        self.iter = 0
        self.maxTransVel = 0.15
        self.maxAngVel = np.pi / 5
        self.maxvariance = 0.015
        self.haveScan = False
        self.haveParticle = False
        self.haveSlamPose = False
        self.particle_x = None
        self.particle_y = None
        self.iter = 0
        self.maxIter = 5
        self.taskDone = False

    #
    # Function to read the lcm data (and to stop the LCM instance on Cntl + C)
    #
    def handle_lcm(self):
        try:
            while True:
                self.lc.handle()

        except KeyboardInterrupt:
            print("lcm exit!")
            sys.exit()

    def lcm_particle_handler(self, channel, data):
        if not self.haveParticle:
            self.particles = particles_t.decode((data))
            self.haveParticle = True
            # print("have particles")

    def lcm_lidar_handler(self, channel, data):
        if not self.haveScan:
            self.scan = lidar_t.decode(data)
            self.haveScan = True
            # print("have scan")

    # def lcm_slam_pose_handler(self, channel, data):
    #     if not self.haveSlamPose:
    #         self.slam_pose = pose_xyt_t.decode(data)
    #         self.haveSlamPose = True
    #         print("have scan")

    def particle_adj(self):
        particle_x = []
        particle_y = []
        for tmp in self.particles.particles:
            particle_x.append(tmp.pose.x)
            particle_y.append(tmp.pose.y)
        self.particle_x = np.array(particle_x)
        self.particle_y = np.array(particle_y)

    def initialize(self):
        print("creating LCM...")
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

        print("subscribing to LCM channels...")
        self.lc.subscribe("LIDAR", self.lcm_lidar_handler)
        self.lc.subscribe("SLAM_PARTICLES", self.lcm_particle_handler)
        # lc.subscribe("SLAM_POSE", self.lcm_slam_pose_handler)
        
        lcm_kill_thread = Thread(target = self.handle_lcm, args=[], daemon = True)
        lcm_kill_thread.start()

    def send_goal_to_planner(self):
        goal = pose_xyt_t()
        goal.utime = current_utime()
        goal.x = 0
        goal.y = 0
        goal.theta = 0

        path_request = planner_request_t()
        path_request.utime = current_utime()
        path_request.require_plan = 1
        path_request.goal = goal
        lc.publish("PATH_REQUEST", path_request.encode())

    def runOnce(self):
        targetDistance = 0.0
        targetTheta = 0.0

        if self.haveScan:
            maxRange = 0
            rotateSign = 1
            transSign = 1
            for i in range(0, len(self.scan.ranges), int(len(self.scan.ranges)/8)):
                 if self.scan.ranges[i] > maxRange:
                    maxRange = self.scan.ranges[i]
                    targetTheta = self.scan.thetas[i]
                    targetDistance = self.scan.ranges[i] / 5
                    rotateSign = 1 if targetTheta >= 0 else -1
                    transSign = 1 if targetDistance >= 0 else -1
                    if self.scan.thetas[i] > np.pi:
                        print("scan theta exceed pi !!!")
            print("Send trans: %.3f, rotate: %.3f" % (targetDistance, targetTheta))
            rotate(rotateSign * self.maxAngVel, abs(targetTheta / self.maxAngVel), end_stop=False)
            drive_forward(transSign * self.maxTransVel, abs(targetDistance / self.maxTransVel), end_stop=False)

            self.haveScan = False
        
        if self.haveParticle:
            self.particle_adj()
            varx = np.var(self.particle_x)
            vary = np.var(self.particle_y)
            print(varx + vary)
            if varx + vary < self.maxvariance:
                self.iter += 1
                print("Current Iter / Converge Iter: (%d, %d)" % (self.iter, self.maxIter))
            else:
                self.iter = 0
            
            if self.iter > self.maxIter:
                # os.system("../bin/motion_controller")
                # subprocess.Popen(["../bin/motion_controller"])
                self.send_goal_to_planner()

                self.taskDone = True

            self.haveParticle = False

    def run(self):
        self.initialize()
        while not self.taskDone:
            self.runOnce()
            # time.sleep(0.001)


# Program entry
if __name__ == "__main__":
    agent = Bouns()
    # agent.run()
    agent.send_goal_to_planner()
    # 1. search longest scan ray
    # 2. follow 1/10 length of this ray
    # 3. check particles distribution (variance and socre?)
    # 4. loop until converge (50-70% of particles have a very small variance)

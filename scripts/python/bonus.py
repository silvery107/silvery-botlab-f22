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
        # Control params
        self.maxTransVel = 0.15
        self.maxAngVel = np.pi / 5
        # Lcm handler indicators
        self.haveScan = False
        self.haveParticle = False
        self.haveSlamPose = False
        self.haveExpState = False
        # Convergence 
        self.particle_x = None
        self.particle_y = None
        self.maxvariance = 0.01

        self.iter = 0
        self.maxIter = 50
        self.taskDone = False
        # Window processing
        self.windowSize = 40
        self.windowIndex = 10
        self.moveSeg = 1.5
        self.rayStride = 2
        self.windowVarMax = 0.002
        self.minBase = 0.5

    def windowProcess(self, ranges):
        window = collections.deque()
        currentWindow = collections.deque()
        currentMax = 0
        currentMin = np.inf
        windowMax = 0
        for i in range(0, len(ranges), self.rayStride):
            if len(currentWindow) < self.windowSize:
                currentWindow.append(ranges[i])
                if currentMax < ranges[i]:
                    currentMax = ranges[i]
                if currentMin > ranges[i]:
                    currentMin = ranges[i]
                window.append(ranges[i])
                if windowMax < ranges[i]:
                    windowMax = ranges[i]
                ##initialize
            else:
                left = currentWindow.popleft()
                currentWindow.append(ranges[i])
                if left == currentMin:
                    currentMin = min(currentWindow)
                elif left == currentMax:
                    currentMax = max(currentWindow)
                if ranges[i] < currentMin:
                    currentMin = ranges[i]
                elif ranges[i] > currentMax:
                    currentMax = ranges[i]
                currentVar = np.var(currentWindow)
                if currentMax > windowMax and currentVar < self.windowVarMax:
                    # print("Range var: %.4f" % currentVar)
                    window = currentWindow
                    windowMax = currentMax
                    self.windowIndex = i - int(self.windowSize/2)
        if self.windowIndex == 10:
            window = collections.deque()
            currentWindow = collections.deque()
            currentMax = 0
            currentMin = np.inf
            windowMax = 0
            for i in range(0, len(ranges), self.rayStride):
                if len(currentWindow) < self.windowSize:
                    currentWindow.append(ranges[i])
                    if currentMax < ranges[i]:
                        currentMax = ranges[i]
                    if currentMin > ranges[i]:
                        currentMin = ranges[i]
                    window.append(ranges[i])
                    if windowMax < ranges[i]:
                        windowMax = ranges[i]
                    ##initialize
                else:
                    left = currentWindow.popleft()
                    currentWindow.append(ranges[i])
                    if left == currentMin:
                        currentMin = min(currentWindow)
                    elif left == currentMax:
                        currentMax = max(currentWindow)
                    if ranges[i] < currentMin:
                        currentMin = ranges[i]
                    elif ranges[i] > currentMax:
                        currentMax = ranges[i]
                    currentVar = np.var(currentWindow)
                    if currentVar < self.windowVarMax and currentMin > self.minBase :
                        # print("Range var: %.4f" % currentVar)
                        window = currentWindow
                        windowMax = currentMax
                        self.windowIndex = i - int(self.windowSize/2)
                        return

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

    def lcm_exp_status_handlesr(self, channel, data):
        if not self.haveExpState:
            self.expState = exploration_status_t.decode(data)
            self.haveExpState = True

    def particle_adj(self):
        particle_x = []
        particle_y = []
        for tmp in self.particles.particles:
            particle_x.append(tmp.pose.x)
            particle_y.append(tmp.pose.y)
        self.particle_x = np.array(particle_x)
        self.particle_y = np.array(particle_y)

    def initialize(self):
        print("Creating LCM...")
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

        print("Subscribing to LCM channels...")
        self.lc.subscribe("LIDAR", self.lcm_lidar_handler)
        self.lc.subscribe("SLAM_PARTICLES", self.lcm_particles_handler)
        self.lc.subscribe("EXPLORATION_STATUS", self.lcm_exp_status_handlesr)
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
        path_request.utime = goal.utime
        path_request.require_plan = 1
        path_request.goal = goal
        lc.publish("PATH_REQUEST", path_request.encode())
    
    def start_local_exploration(self):
        print("Start local motion controller, slam and exploration")
        os.system("../bin/mctrl_loc --use-local-channels > /dev/null 2>&1 &")
        os.system("../bin/slam_loc --use-local-channels --num-particles 200 > /dev/null 2>&1 &")
        os.system("../bin/exp_loc --use-local-channels > /dev/null 2>&1 &")
    
    def cleanup_local_exploration(self):
        os.system("../cleanup_bonus.sh")

    def runOnce(self):

        # if self.haveScan:
        #     targetDistance = 0.0
        #     targetTheta = 0.0
        #     # maxRange = 0
        #     rotateSign = 1
        #     transSign = 1

        #     self.windowProcess(self.scan.ranges)

        #     targetTheta = self.scan.thetas[self.windowIndex]
        #     if targetTheta > np.pi:
        #         targetTheta -= np.pi * 2
        #     targetDistance = self.scan.ranges[self.windowIndex] / self.moveSeg

        #     print("Window index: %d" % self.windowIndex)

        #     rotateSign = 1 if targetTheta >= 0 else -1
        #     transSign = 1 if targetDistance >= 0 else -1
        #     print("Send trans: %.3f, rotate: %.3f" % (targetDistance, np.rad2deg(targetTheta)))

        #     rotate(rotateSign * self.maxAngVel, abs(targetTheta / self.maxAngVel), end_stop=True)
        #     drive_forward(transSign * self.maxTransVel, abs(targetDistance / self.maxTransVel), end_stop=True)

        #     self.haveScan = False
        
        if self.haveParticle:
            self.particle_adj()
            varx = np.var(self.particle_x)
            vary = np.var(self.particle_y)
            # print(varx + vary)
            if varx + vary < self.maxvariance:
                self.iter += 1
                print("Iter to Converge: (%d, %d)" % (self.iter, self.maxIter))
            else:
                self.iter = 0
            
            if self.iter == self.maxIter:
                print("Send goal to planning server")
                self.send_goal_to_planner()

                self.taskDone = True

            self.haveParticle = False
        
        if self.haveExpState:
            if self.expState == 3 or self.expState == 4:
                self.cleanup_local_exploration()
                time.sleep(0.01)
                self.start_local_exploration()
                
            self.haveExpState = False


    def run(self):
        self.initialize()
        self.start_local_exploration()

        print("Start motion planning server")
        os.system("../bin/motion_planning_server > /dev/null 2>&1 &")

        while not self.taskDone:
            self.runOnce()
            # time.sleep(0.001)
        
        self.cleanup_local_exploration()


# Program entry
if __name__ == "__main__":
    agent = Bonus()
    agent.run()
    # agent.send_goal_to_planner()
    print("Start motion controller")
    os.system("../bin/motion_controller")

    # 1. run exploration with full slam and motion controller in local channels
    # 2. check particles distribution (variance and socre?) in global channels
    # 4. loop until converge (50-70% of particles have a very small variance)
    # Note that global slam is running in localization mode with sampling augmentation,
    # while local slam is running in full slam mode.

    # Local channels: SLAM_POSE_LOCAL, SLAM_MAP_LOCAL
    # slam: publish slam_pose and slam_map
    # exploration: subscribe slam_map, slam_pose; publish controller_path
    # motion_controller: subscribe slam_pose, controller_path
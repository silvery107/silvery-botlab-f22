#!/usr/bin/env python3
#Author: Tom Gao, June 3rd 2021

#Web stuff imports
from flask import Flask, render_template
import logging

#LCM pkg and definitions imports
import lcm
import sys
sys.path.append("/home/pi/botlab-soln/python")
from lcmtypes import omni_motor_command_t
        
#setup code
app = Flask(__name__)
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR) # prevent console-logging all requests

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=2") # LCM instance TODO check udpm ttl number, botlab says 2 but botlab python says 1

#Precomputed constants
COS30 = 0.866025404
MAX_VX = 0.5 #m/s
MAX_VY = 0.5 #m/s
MAX_WZ = 3.14159265 * 2 / 3 #rad/s

@app.route("/")
def hello():
    return render_template("index.html") #make temporary first-time checkbox send to HTML checkboxes
    
@app.route("/motors/<fwd>,<rot>")
def motors(fwd, rot):
    drive_profile(int(fwd) / 100, int(rot) / 100) #TODO calibration cfg?
    return ""

def drive_profile(fwd, rot):
    command = omni_motor_command_t()
    command.vx = fwd * MAX_VX
    command.wz = rot * MAX_WZ
    lc.publish("MBOT_MOTOR_COMMAND", command.encode())
         
if __name__ == "__main__":
    print("Go to 192.168.3.1:5000 to access the web page!")
    app.run(debug=False, host="0.0.0.0", port="5000")

    
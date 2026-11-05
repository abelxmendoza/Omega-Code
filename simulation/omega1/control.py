#!/usr/bin/env python3
"""Bounded Gazebo command adapter. Never imports GPIO or accesses physical hardware."""
import argparse
import json
import math
from pathlib import Path
import shutil
import subprocess
import time

ROOT=Path(__file__).resolve().parent

def bounded(value,limit):
    if not math.isfinite(value): raise ValueError('command must be finite')
    return max(-limit,min(limit,value))

class SimHardware:
    """Command semantics only; Gazebo owns dynamics and sensor observations.

    Wheel odometry is explicitly ideal simulation feedback. This does not emulate
    Pi CPU timing, I2C registers, voltage/current, encoders, or ultrasonic acoustics.
    """
    def __init__(self, parameters, publish, clock=time.monotonic):
        self.p=parameters; self.publish=publish; self.clock=clock
        self.last_command=None; self.moving=False

    def drive(self,linear,angular):
        linear=bounded(linear,self.p['max_linear_speed']); angular=bounded(angular,self.p['max_angular_speed'])
        self.publish('/omega/sim/cmd_vel','gz.msgs.Twist',f'linear: {{x: {linear}}}, angular: {{z: {angular}}}')
        self.last_command=self.clock(); self.moving=bool(linear or angular)
        return linear,angular

    def servo(self,axis,radians):
        if axis not in ('pan','tilt'): raise ValueError('axis must be pan or tilt')
        value=bounded(radians,self.p[axis+'_limit'])
        self.publish('/omega/sim/'+axis,'gz.msgs.Double',f'data: {value}')
        return value

    def tick(self):
        if self.moving and self.clock()-self.last_command >= self.p['watchdog_seconds']:
            self.drive(0,0)

    def stop(self): self.drive(0,0)

def publish(topic,message_type,payload):
    subprocess.run(['gz','topic','-t',topic,'-m',message_type,'-p',payload],check=True,timeout=5)

def main():
    ap=argparse.ArgumentParser(description='Finite-duration, bounded Omega1 simulation command')
    ap.add_argument('--linear',type=float,default=0); ap.add_argument('--angular',type=float,default=0)
    ap.add_argument('--seconds',type=float,default=1); ap.add_argument('--pan',type=float); ap.add_argument('--tilt',type=float)
    a=ap.parse_args()
    if not math.isfinite(a.seconds) or not 0<a.seconds<=30: ap.error('seconds must be in (0,30]')
    if not shutil.which('gz'): ap.error('Gazebo gz command is missing; install Harmonic')
    hw=SimHardware(json.loads((ROOT/'parameters.json').read_text()),publish)
    try:
        for axis in ('pan','tilt'):
            if getattr(a,axis) is not None: hw.servo(axis,getattr(a,axis))
        end=time.monotonic()+a.seconds
        while time.monotonic()<end:
            hw.drive(a.linear,a.angular); time.sleep(0.1)
    finally: hw.stop()

if __name__=='__main__': main()

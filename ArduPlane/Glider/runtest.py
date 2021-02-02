#!/usr/bin/env python

import pexpect, time, sys, os
from pymavlink import mavutil

import argparse
parser = argparse.ArgumentParser(description='run glider test')
parser.add_argument('mission', nargs='?', default=None, help='mission file')
parser.add_argument('--location', default="KEDW", help='location')
parser.add_argument('--speed-scheduling', action='store_true', default=False)
args = parser.parse_args()

def kill_all():
    os.system("pkill mavproxy 2> /dev/null")
    os.system("pkill -9 mavproxy 2> /dev/null")
    os.system("killall -9 xterm 2> /dev/null")
    os.system("killall -9 gdb 2> /dev/null")
    os.system("killall -9 arduplane 2> /dev/null")

def wait_heartbeat(mav, timeout=10):
    '''wait for a heartbeat'''
    start_time = time.time()
    while time.time() < start_time+timeout:
        if mav.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5) is not None:
            return
    raise Exception("Failed to get heartbeat")    

def wait_mode(mav, modes, timeout=10):
    '''wait for one of a set of flight modes'''
    start_time = time.time()
    last_mode = None
    while time.time() < start_time+timeout:
        wait_heartbeat(mav, timeout=10)
        if mav.flightmode != last_mode:
            print("Flightmode %s" % mav.flightmode)
            last_mode = mav.flightmode
        if mav.flightmode in modes:
            return
    print("Failed to get mode from %s" % modes)
    sys.exit(1)

def wait_time(mav, simtime):
    '''wait for simulation time to pass'''
    imu = mav.recv_match(type='RAW_IMU', blocking=True)
    t1 = imu.time_usec*1.0e-6
    while True:
        imu = mav.recv_match(type='RAW_IMU', blocking=True)
        t2 = imu.time_usec*1.0e-6
        if t2 - t1 > simtime:
            break

kill_all()
os.system("rm -rf logs")
time.sleep(3)

cmd = '../../Tools/autotest/sim_vehicle.py -D -f PlaneJSON -G -L %s --aircraft test' % args.location
if sys.version_info[0] >= 3:
    mavproxy = pexpect.spawnu(cmd, logfile=sys.stdout, timeout=300)
else:
    mavproxy = pexpect.spawn(cmd, logfile=sys.stdout, timeout=300)
mavproxy.expect("ArduPilot Ready")
#mavproxy.expect("using GPS")
#mavproxy.expect("using GPS")
#mavproxy.expect("using GPS")
#mavproxy.expect("using GPS")

mav = mavutil.mavlink_connection('127.0.0.1:14550')

mavproxy.send('wp load missions/%s.txt\n' % args.mission)
mavproxy.expect('Flight plan received')
wait_mode(mav, ['MANUAL'])
mavproxy.send('param ftp\n')
mavproxy.expect("Received")
if args.speed_scheduling:
    print("ENABLING speed scheduling")
    mavproxy.send("param set SCR_USER4 1\n")
else:
    print("DISABLNG speed scheduling")
    mavproxy.send("param set SCR_USER4 0\n")

mavproxy.send('arm throttle\n')
mavproxy.expect('Throttle armed')
mavproxy.send('auto\n')
wait_mode(mav, ['AUTO'])
mavproxy.send('rc 6 2000\n')
mavproxy.send('module load map\n')
mavproxy.send('wp list\n')
mavproxy.send('gamslft\n')
mavproxy.expect("Released",timeout=600)
mavproxy.send('disarm force\n')
kill_all()
os.system("ln -f logs/00000001.BIN test_runs/%s.bin" % args.mission)
os.system("ls -l test_runs/%s.bin" % args.mission)
#!/usr/bin/env python3
#%%
from PIL import Image
import gphoto2 as gp
from argparse import ArgumentParser
from pymavlink import mavutil
from pymavlink.mavutil import mavlink as lnk
import pymavlink.dialects.v20.ardupilotmega as mavcmd
from pymavlink.dialects.v20.ardupilotmega import MAVLink as mavlink2
from pymavlink.dialects.v20.ardupilotmega import MAVLink_command_long_message as cmdlong
import sys as sys
import curses as curses
import random as rnd
from time import sleep
from time import time as t_now
import imagecap
import io
def wait_heartbeat(m:mavutil.mavudp):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))


#%%
connection:mavutil.mavudp
source = f"/dev/serial0"
connection = mavutil.mavlink_connection(device=source, source_system=1, source_component=25)
wait_heartbeat(connection)
mav:mavlink2
mav = connection.mav


#%%
i = 0
hb_send_time_ms = 0
cam_control : cmdlong
cam_feedback : mavcmd.MAVLink_camera_feedback_message
both_recvd = 0

camera = gp.check_result(gp.gp_camera_new())
def cam():
    camera_file = gp.check_result(gp.gp_camera_capture_preview(camera))
    file_data = gp.check_result(gp.gp_file_get_data_and_size(camera_file))
    image = Image.open(io.BytesIO(file_data))
    image.show()
    

while True:
       # gp(["--capture-movie --stdout | ffmpeg -re -i pipe:0 -listen 1 -f avi http://localhost:8080/feed.avi"])
    try:
        msg = connection.recv_msg()
        if msg is not None and msg.id == mavcmd.MAVLINK_MSG_ID_COMMAND_LONG:
            # msg : cmdlong
            if msg.command == mavcmd.MAV_CMD_DO_DIGICAM_CONTROL and msg.param5 == 1:
                cam_control = msg
                both_recvd +=1
                
        if msg is not None and msg.id == mavcmd.MAVLINK_MSG_ID_CAMERA_FEEDBACK:
            cam_feedback = msg
            both_recvd +=1
        
        if both_recvd == 2:
                i+=1
                print("Recvd shooting command ", i)
                print("Photoshoot", "lat: ", cam_feedback.lat*1e-7, "lng: ", cam_feedback.lng*1e-7, "alt_msl: ", cam_feedback.alt_msl)
                #gp.check_result(gp.gp_camera_exit(camera))
                r = rnd.randint(0,2)
                imagecap.get_shot()
                print("Simulating delay. Sleeping for ", r, ' sec')
                sleep(r)
                print("Sending answer")
                mav.command_ack_send(mavcmd.MAV_CMD_DO_DIGICAM_CONTROL,mavcmd.MAV_RESULT_ACCEPTED)
                print("Success")
                both_recvd = 0
                


        now = t_now()
        if (now - hb_send_time_ms > 1000):
            mav.heartbeat_send(mavcmd.MAV_TYPE_CAMERA, mavcmd.MAV_AUTOPILOT_INVALID, 0,0, mavcmd.MAV_STATE_ACTIVE)
            hb_send_time_ms = now

    except: 
        connection:mavutil.mavudp
        connection = mavutil.mavlink_connection(device=source, source_system=1, source_component=25)
        wait_heartbeat(connection)
        mav:mavlink2
        mav = connection.mav
        #%%
        i = 0
        hb_send_time_ms = 0
        cam_control : cmdlong
        cam_feedback : mavcmd.MAVLink_camera_feedback_message
        both_recvd = 0
        camera = gp.check_result(gp.gp_camera_new())



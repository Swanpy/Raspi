from argparse import ArgumentParser
from pymavlink import mavutil
from pymavlink.mavutil import mavlink as lnk
import pymavlink.dialects.v20.ardupilotmega as mavcmd
from pymavlink.dialects.v20.ardupilotmega import MAVLink as mavlink2
from pymavlink.dialects.v20.ardupilotmega import MAVLink_command_long_message as cmdlong
import sys as sys
import os
import curses as curses
from datetime import datetime
from time import time as t_now
import uuid
import shutil

import binascii
import time_uuid
import json
"""#import threading import *"""
def time():
    now = datetime.now()
    return str(now.strftime("%H:%M:%S"))
def log(row):
    if not os.path.isfile(f"./rasp_logcopter.txt"):
        with open(f"./rasp_logcopter.txt",'w') as f:
            f.write(f"[{time()}] "+row + '\n')
    else:
        with open(f"./rasp_logcopter.txt", 'a') as f:
            f.write(f"[{time()}] "+row + '\n')


def renameFiles(i, pos, my_uuid, dir):
    shot_time = uuid.uuid1()
    save_folder = my_uuid
    if os.path.isfile("./capt0000.jpg"):
        log('Photo create.')
        data = '{"name":' + f'"{i}.jpg",' + '"pos" :' + str(pos)+ '}'
        if not os.path.exists(f"{dir}/missions/"):
            os.mkdir(f"{dir}/missions")
            log("Create dir.")
        if not os.path.exists(f"{dir}/missions/{save_folder}"):
            os.mkdir(f"{dir}/missions/{save_folder}")
            log("Create dir.")
        if not os.path.isfile(f'{dir}/missions/{save_folder}/mission.json'):
            log('File "mission.json" is create')
            f = open(f'{dir}/missions/{save_folder}/mission.json','a')
            f.close()
        with open(f'{dir}/missions/{save_folder}/mission.json', 'r+') as f:
            row = f.read().rstrip()
            if not row:
                row = '{'+f'"active_mission_guid":"{shot_time}",\n "images":[' + data +']}'
                f.write(row)
            else:
                f.close()
                with open(f'{dir}/missions/{save_folder}/mission.json','w') as f:
                	row = row[:-2] + ',\n \t'+ data +']}'
                	f.write(row)
                	f.close()
        try:
            shutil.copy("./capt0000.jpg", f"{dir}/missions/{save_folder}/{i}.jpg")
        except Exception as ex:
            log(str(ex))
    else:
        log("Photo not make.")

dir = "/home/pi"  # Укажите свой путь здесь

def save_coord(coord,i, dir):
            shot_time = uuid.uuid1()
            save_folder = "missions"
            data = f'"{i}.jpg"' + f':pos "{coord}"'
            if not os.path.exists(f"{dir}/{save_folder}"):
                    os.mkdir(save_folder)
                    log("Create dir.")
            if not os.path.isfile(f'{dir}/{save_folder}/coordinates_mission.json'):
                   log('File "mission.json" is create')
                   f = open(f'{dir}/{save_folder}/coordinates_mission.json','a')
                   f.close()
            with open(f'{dir}/{save_folder}/coordinates_mission.json', 'r+') as f:
                    row = f.read()
                    if not row:
                       row = '{'+f'"active_mission_guid":"{shot_time}",\n "images":[' + data +']}'
                       f.write(row)
                    else:
                       f.close()
                       with open(f'{dir}/{save_folder}/coordinates_mission.json','w') as f:
                           row = row[:-2] + ',\n \t'+ data
                          # print('file update: ', row)
                           f.write(row+']}')
def wait_heartbeat(m:mavutil.mavudp):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))
    log("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))


connection:mavutil.mavudp
connection = mavutil.mavlink_connection(device="udpin:0.0.0.0:63212", source_system=1, source_component=25)
wait_heartbeat(connection)
mav:mavlink2
mav = connection.mav
i = 0
hb_send_time_ms = 0
cam_control : cmdlong
cam_feedback : mavcmd.MAVLink_camera_feedback_message
both_recvd = 0
if os.path.isfile(f"{dir}/active_mission.json"):
   with open(f"{dir}/active_mission.json") as f:
       file_content = f.read()
       templates = json.loads(file_content)
   for section, commands in templates.items():
       log(commands)
   my_uuid = uuid.UUID(commands)

   ts = time_uuid.TimeUUID(bytes=my_uuid.bytes).get_timestamp()
   log(datetime.utcfromtimestamp(ts).strftime('%Y-%m-%dT%H:%M:%SZ'))
   times = datetime.utcfromtimestamp(ts).strftime('%Y-%m-%dT%H:%M:%SZ')
else:
    exit(-1)


while True:
    try:
        msg = connection.recv_msg()
        if msg is not None and msg.id == mavcmd.MAVLINK_MSG_ID_COMMAND_LONG:
            if msg.command == mavcmd.MAV_CMD_DO_DIGICAM_CONTROL and msg.param5 == 1:
                cam_control = msg
                both_recvd +=1
        if msg is not None and msg.id == mavcmd.MAVLINK_MSG_ID_CAMERA_FEEDBACK:
            cam_feedback = msg
            both_recvd +=1
        if both_recvd == 2:
                mav.command_ack_send(mavcmd.MAV_CMD_DO_DIGICAM_CONTROL,mavcmd.MAV_RESULT_ACCEPTED)
                recvd_lat = cam_feedback.lat*1e-7
                recvd_lon = cam_feedback.lng*1e-7
                recvd_alt = cam_feedback.alt_rel
                i+=1
                print(cam_feedback)
                log("Recvd shooting command " + str(i))
                print("Photoshoot", "lat: ", recvd_lat, "lng: ", recvd_lon, "alt_msl: ", recvd_alt)
                log("Photoshoot" + " lat: " +str(recvd_lat) +  " lng: " + str( recvd_lon) +  " alt_msl: " + str( recvd_alt))
                """#live.stop()#"""
                """#shoth.start()#"""
                """#live.join()#"""
                coord =  " lat: " +str(recvd_lat) +  " lng: " + str( recvd_lon) +  " alt_msl: " + str( recvd_alt)
                pos = [recvd_lat,recvd_lon,recvd_alt]
                renameFiles(i=str(i),pos=pos,my_uuid=str(my_uuid), dir=dir)
                log("Sending answer")
                #mav.command_ack_send(mavcmd.MAV_CMD_DO_DIGICAM_CONTROL,mavcmd.MAV_RESULT_ACCEPTED)
                log("Success")
                both_recvd = 0
        now = t_now()
        if (now - hb_send_time_ms > 1000):
            mav.heartbeat_send(mavcmd.MAV_TYPE_CAMERA, mavcmd.MAV_AUTOPILOT_INVALID, 0,0, mavcmd.MAV_STATE_ACTIVE)
            hb_send_time_ms = now
    except Exception as err:
        log(str(err))
        connection:mavutil.mavudp
        connection = mavutil.mavlink_connection(source=source, source_system=1, source_component=25)
        wait_heartbeat(connection)
        mav:mavlink2
        mav = connection.mav
        #%%
        i = 0
        hb_send_time_ms = 0
        cam_control : cmdlong
        cam_feedback : mavcmd.MAVLink_camera_feedback_message
        both_recvd = 1




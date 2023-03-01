from time import sleep
from datetime import datetime
from sh import gphoto2 as gp
import signal, os, subprocess
import shutil
shot_date = datetime.now().strftime("%Y-%m-%d")
shot_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")



triggerCommand = ["--capture-image-and-download"]
save_folder = "image-"+shot_date
def createSaveFolder():
    try:
        os.mkdir(save_folder)
    except:
      pass
    os.chdir(save_folder)

def captureImages():
    gp(triggerCommand)


def renameFiles():
            if os.path.isfile("capt0000.jpg"):
                print('lol')
            try:
                shutil.move("/home/pi/capt0000.jpg", f"/home/pi/{save_folder}/{shot_time}.jpg")
            except Exception as ex:
                print(ex)

def get_shot():
   captureImages()
   createSaveFolder()
   renameFiles()

#get_shot()



# Import the camera server
#!/usr/bin/env python3

# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import json
import time
import sys

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from networktables import NetworkTablesInstance
configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = config.get("stream")

    cam.config = config

    cameraConfigs.append(cam)
    return True

def readSwitchedCameraConfig(config):
    """Read single switched camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read switched camera name")
        return False

    # path
    try:
        cam.key = config["key"]
    except KeyError:
        parseError("switched camera '{}': could not read key".format(cam.name))
        return False

    switchedCameraConfigs.append(cam)
    return True

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    # switched cameras
    if "switched cameras" in j:
        for camera in j["switched cameras"]:
            if not readSwitchedCameraConfig(camera):
                return False

    return True

def startCamera(config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    inst = CameraServer.getInstance()
    camera = UsbCamera(config.name, config.path)
    server = inst.startAutomaticCapture(camera=camera, return_server=True)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))

    return camera

def startSwitchedCamera(config):
    """Start running the switched camera."""
    print("Starting switched camera '{}' on {}".format(config.name, config.key))
    server = CameraServer.getInstance().addSwitchedCamera(config.name)

    def listener(fromobj, key, value, isNew):
        if isinstance(value, float):
            i = int(value)
            if i >= 0 and i < len(cameras):
              server.setSource(cameras[i])
        elif isinstance(value, str):
            for i in range(len(cameraConfigs)):
                if value == cameraConfigs[i].name:
                    server.setSource(cameras[i])
                    break

    NetworkTablesInstance.getDefault().getEntry(config.key).addListener(
        listener,
        NetworkTablesInstance.NotifyFlags.IMMEDIATE |
        NetworkTablesInstance.NotifyFlags.NEW |
        NetworkTablesInstance.NotifyFlags.UPDATE)

    return server

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)
        ntinst.startDSClient()

    # start cameras
    for config in cameraConfigs:
        cameras.append(startCamera(config))

    # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)
    import numpy as np
    import cv2
    locator_table = ntinst.getTable("BallLocator")
    green_profile=[(37,0,0),(67,255,255)]
    blue_profile=[(108,0,0),(131,255,255)]
    yellow_profile=[(14,0,0),(22,255,255)]
    #red_profile=[((0,0,0),(25,255,255))]
    profiles=[green_profile,blue_profile,yellow_profile]
    w=160
    h=120
    image = np.zeros(shape=(w, h, 3), dtype=np.uint8)
    cs = CameraServer.getInstance()
    outputStream = cs.putVideo("Name", w,h)
    contourStreams=list()
    for i,profile in enumerate(profiles):
        contourStreams.append(cs.putVideo("Profile "+str(i), w,h))
    thresholdStreams=list()
    for i,profile in enumerate(profiles):
        thresholdStreams.append(cs.putVideo("Threshold "+str(i), w,h))
    cvSink = cs.getVideo()
    time.sleep(1)
    print(dir(ntinst))
    print(dir(locator_table))
    while True:
        #print("Running")
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, image = cvSink.grabFrame(image)
        img_hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        commands=list()
        for i,profile in enumerate(profiles):
            img_threshold = cv2.inRange(img_hsv, 
                profile[0], 
                profile[1]
            )
            #puts boxes around the contours in img_threshold
            img_contours = img_threshold.copy()
            image_profile=image.copy()
            contours, hierarchy = cv2.findContours(img_contours, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                x,y,w,h = cv2.boundingRect(cnt)
                if cv2.contourArea(cnt)/image.shape[0]/image.shape[1] > 0:
                    commands.append((i,cv2.contourArea(cnt)))
                    cv2.rectangle(image_profile,(x,y),(x+w,y+h),(255,255,255),2)
            contourStreams[i].putFrame(image_profile)
            thresholdStreams[i].putFrame(img_threshold)
        commands_sorted=sorted(commands,key=lambda x:x[1])
        #put the first element of each tuple into a new list
        commands_sorted=list(map(lambda x:x[0],commands_sorted))
        locator_table.putNumber("Command",commands_sorted[-1])
        outputStream.putFrame(image)
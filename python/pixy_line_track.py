#!/usr/bin/env python3
#----------------------------------------------------------------------------
# This is meant to be used in conjuction with WPILib Raspberry Pi image: https://github.com/wpilibsuite/FRCVision-pi-gen
#----------------------------------------------------------------------------

from threading import Thread

from networktables import NetworkTablesInstance
from networktables import NetworkTables

########### SET RESOLUTION TO 256x144 !!!! ############

#Class to examine Frames per second of camera stream. Currently not used.


#################### FRC VISION PI Image Specific #############
configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []

"""Report parse error."""
def parseError(str):
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

"""Read single camera configuration."""
def readCameraConfig(config):
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

    cam.config = config

    cameraConfigs.append(cam)
    return True

"""Read configuration file."""
def readConfig():
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt") as f:
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

    return True

"""Start running the camera."""
def startCamera(config):
    print("Starting camera '{}' on {}".format(config.name, config.path))
    cs = CameraServer.getInstance()
    camera = cs.startAutomaticCapture(name=config.name, path=config.path)

    camera.setConfigJson(json.dumps(config.config))

    return cs, camera

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]
    # read configuration
    if not readConfig():
        sys.exit(1)
        
    print("Waiting 60 seconds before start for networktables")
    time.sleep(1)

    # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    #Name of network table - this is how it communicates with robot. IMPORTANT
    networkTable = NetworkTables.getTable('ChickenVision')

    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)


    # start cameras
    cameras = []
    streams = []
    for cameraConfig in cameraConfigs:
        cs, cameraCapture = startCamera(cameraConfig)
        streams.append(cs)
        cameras.append(cameraCapture)
    #Get the first camera

    webcam = cameras[0]
    cameraServer = streams[0]
    #Start thread reading camera
    cap = WebcamVideoStream(webcam, cameraServer, image_width, image_height).start()
    
    #Get the second camera

    webcam2 = cameras[1]
    cameraServer2 = streams[1]
    #Start thread reading camera
    cap2 = WebcamVideoStream(webcam2, cameraServer2, image_width, image_height).start()

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(image_height, image_width, 3), dtype=np.uint8)
    #Start thread outputing stream
    streamViewer = VideoShow(image_width,image_height, cameraServer, frame=img).start()
    #cap.autoExpose=True
    cap2.autoExpose=True
    tape = False
    #TOTAL_FRAMES = 200;
    # loop forever
    while True:
        
        streamViewer.frame = processed
        #Flushes camera values to reduce latency
        ntinst.flush()

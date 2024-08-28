#!/usr/bin/env python3
import rospy
from my_robot.msg import object_description

import sys
import argparse
import jetson_utils
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, Log


# parse the command line
parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.", 
                                 formatter_class=argparse.RawTextHelpFormatter, 
                                 epilog=detectNet.Usage() + videoSource.Usage() + videoOutput.Usage() + Log.Usage())

parser.add_argument("input", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
parser.add_argument("--threshold", type=float, default=0.2, help="minimum detection threshold to use") 


rospy.init_node('pick_and_place', anonymous=True)

pub = rospy.Publisher('/my_robot/object_description', object_description, queue_size=10)


try:
    args = parser.parse_known_args()[0]
except:
    print("")
    parser.print_help()
    sys.exit(0)

# create video sources and outputs
input = videoSource(args.input, argv=sys.argv)
output = videoOutput(args.output, argv=sys.argv)
    
# load the object detection network
net = detectNet(args.network, sys.argv, args.threshold)

# process frames until EOS or the user exits
tracker = 0
while True:
    # capture the next image
    img = input.Capture()

    if img is None: # timeout
        continue  

    imgInput = img
    crop_factor = 0.3
    crop_border = ((1.0 - crop_factor) * 0.5 * imgInput.width,
                (1.0 - crop_factor) * 0.5 * imgInput.height)
    
    imgOutput = jetson_utils.cudaAllocMapped(width=240.0,
                                         height=200.0,
                                         format=imgInput.format)
    print(crop_border[0])
    print(crop_border[1])
    print(imgInput.width - crop_border[0])
    print(imgInput.height - crop_border[1])
    crop_roi = (580.0, 180.0, 820.0, 380.0)


    jetson_utils.cudaCrop(imgInput, imgOutput, crop_roi)

    # detect objects in the image (with overlay)
    detections = net.Detect(imgOutput, overlay=args.overlay)

    # print the detections
    print("detected {:d} objects in image".format(len(detections)))

    if tracker == 60:
        if detections:
            
            first_detect=detections[0]
            print(dir(first_detect))
            print(first_detect.Center[0])
            msg = object_description()
            msg.x = first_detect.Center[0]
            msg.y = first_detect.Center[1]
            
            msg.object_description = first_detect.ClassID
            pub.publish(msg)
        tracker = 0
            
    tracker = tracker + 1
    
    


    





    # render the image
    output.Render(imgOutput)

    


    # update the title bar
    output.SetStatus("{:s} | Network {:.0f} FPS".format(args.network, net.GetNetworkFPS()))

    # print out performance info
    net.PrintProfilerTimes()

    # exit on input/output EOS
    if not input.IsStreaming() or not output.IsStreaming():
        break
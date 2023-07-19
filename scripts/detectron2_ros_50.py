#!/usr/bin/env python3

# IMPORTS
import rospy
import roslib
import numpy as np
import cv2
import time
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as SensImage
from geometry_msgs.msg import Pose 
import matplotlib.pyplot as plt
import argparse
import os
import sys

from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
print(ROOT)
import torch
import detectron2
print(f"Detectron2 version is {detectron2.__version__}")
from detectron2.engine import DefaultPredictor
from detectron2 import model_zoo
from detectron2.utils.logger import setup_logger
setup_logger()
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import Metadata
from detectron2.utils.visualizer import ColorMode
from detectron2.data import MetadataCatalog, DatasetCatalog

from PIL import Image 

import csv

#GLOBAL VARIABLES
cv_image=None

# FUNCTIONS
def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='default', help='Model path(s)')
    parser.add_argument('--device', nargs='+', type=str, default=['cuda'], help='Device to use: cpu or cuda')
    parser.add_argument('--confidence', nargs='+', type=float, default=[0.7], help='Min confidence to do mask')
    parser.add_argument('--show_segmentation', nargs='+', type=int, default=[0], help='1 or 0(True or False), publish the segmentation or not, without segmentation is faster')
    parser.add_argument('--save_frames', nargs='+', type=int, default=[0], help='put anything but 0 to save segmentation, show_segmentation must be True')
    parser.add_argument('--resize', nargs='+', type=float, default=[1], help='To change resolution of the image between [0, 1]')
    parser.add_argument('--crop', nargs='+', type=int, default=[1], help='1 to crop a square image, 0 to keep the original without crop')
    parser.add_argument('--flip', nargs='+', type=int, default=[0], help='1 to flip a square image aroundthe x axis, 0 to keep the original without flip')
    opt = parser.parse_args()
    return opt

def showSegmentation(visualizer,out,plot=True,pub=None):
    out_vis = visualizer.draw_instance_predictions(out["instances"].to("cpu"))
    if plot:
        cv2.imshow("Detection",out_vis.get_image()[:, :, ::-1])
    if pub is not None:
        cv_bridge=CvBridge()
        pub.publish(cv_bridge.cv2_to_imgmsg(out_vis.get_image()[:, :, ::-1], 'rgb8'))
    return out_vis

def getMask(out):
    masks = np.asarray(out["instances"].pred_masks.to("cpu"))
    if(len(masks[:,0,0])>0):
        total_mask = np.zeros([len(masks[0,:,0]),len(masks[0,0,:]),1],dtype=np.uint8)

        for i in range(len(masks[:,0,0])):
            # Pick an item to mask
            item_mask = masks[i]

            # Create a PIL image out of the mask
            mask = Image.fromarray((item_mask * 255).astype('uint8'))
            mask_imageBlackWhite = np.array(mask)
            mask_image = cv2.cvtColor(mask_imageBlackWhite,cv2.COLOR_GRAY2RGB)
            total_mask=np.add(total_mask,mask_image)
            # Display the image
            mask_i="Mask "+str(i)
            #cv2.imshow(mask_i,mask_image)
            #cv2.waitKey(0)
    
    else:
        total_mask=None
    return(total_mask)

def getOrientedBoxes(mask,plot,pub=None):
    if mask is None:
        return None,None,None
    
    # Convert image to grayscale
    gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    im_height,im_width=gray.shape

    # Convert image to binary
    _, bw = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)
 
    # Find all the contours in the thresholded image
    contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    


    sum_altitudes=0

    center_y_prec = 1000000000000
    center_y = 0

    #print("center_y",center_y)
    #print("center_y_prec",center_y_prec)

    for i, c in enumerate(contours): 

        rect = cv2.minAreaRect(c)
        #center_y = int(rect[0][1])

        center_y=int(1000*(rect[0][1]-im_height/2)/im_height)

        if abs(center_y)<center_y_prec:

            # Calculate the area of each contour
            area = cv2.contourArea(c)
 
            # Ignore contours that are too small or too large
            #if area < 3700 or 100000 < area:
            #   continue

            #cv.minAreaRect returns:
            #(center(x, y), (width, height), angle of rotation) = cv2.minAreaRect(c)

            box = cv2.boxPoints(rect)
            box = np.intp(box)

            # Retrieve the key parameters of the rotated bounding box
            center = (int(rect[0][0]),int(rect[0][1])) 
            width = int(rect[1][0])
            height = int(rect[1][1])
            angle = int(rect[2])


            if width > height:
                
                angle = - angle

            else:

                angle = 90-angle


            if(width<height):
                norm_width=width/im_width
            else:
                norm_width=height/im_width
        
            #sum_altitudes=sum_altitudes+0.75/(norm_width+0.01) # NEED TO BE ADJUSTED, RIGHT NOW IS SETTED FOR THE SIMULATION
            altitude=0.75/(norm_width+0.01) # NEED TO BE ADJUSTED, RIGHT NOW IS SETTED FOR THE SIMULATION

            center_y_prec = abs(center_y)

############
# exclude not centered boxes
# find the closest line to the center of the camera
############


    label = str(angle) + " deg "
    cv2.drawContours(mask,[box],0,(0,0,255),2)
    cv2.putText(mask, label, (center[0]-0, center[1]-25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,200,0), 1, cv2.LINE_AA) 

    mask = cv2.circle(mask, (center[0], center[1]), radius=10, color=(0, 0, 255), thickness=3) # draw the center
    center=[int(1000*(center[0]-im_width/2)/im_width),int(1000*(center[1]-im_height/2)/im_height)]

    if plot:
        cv2.imshow("Mask with boxes", mask)

    if pub is not None:
        cv_bridge=CvBridge()
        pub.publish(cv_bridge.cv2_to_imgmsg(mask, 'rgb8'))

    return center,-angle,altitude


args= parse_opt()

# SUBSCRIBERs CALLBACK
def callback(startImg):
    global cv_image,args
    cv_bridge=CvBridge()
    cv_image = cv_bridge.imgmsg_to_cv2(startImg, 'rgb8')
    #print("image receveid")
    if args.flip[0]:
        cv_image=cv2.flip(cv_image, 0)



def overlay_line_with_point(image, center, angle,frame_name):

    try:
        x=center[0]
        y=center[1]
    except:
        pass

    fig, ax = plt.subplots()
    ax.imshow(image)
    
    # Calculate the line endpoints
    height, width = image.shape[:2]
    slope = np.tan(np.deg2rad(angle))
    print("slope",slope)

    #y1 = 0
    #x1 = int(x + (y1 - y) / slope)
    #y2 = height - 1
    #x2 = int(x + (y2 - y) / slope)
    
    # Overlay the line
    #ax.plot([x1, x2], [y1, y2], 'r-')
    
    # Plot the given point
    print("x, y",x+width/2, height/2-y)
    ax.plot(x+width/2, height/2-y, 'go')
    
    # Add the information box

    # Set the x and y axis limits to match the image dimensions
    ax.set_xlim([0, width - 1])
    ax.set_ylim([height - 1, 0])

    plt.axis('off')
    plt.savefig(frame_name, bbox_inches='tight', pad_inches=0)



def main():
    rospy.init_node('detectron2', anonymous=False)
    save_frames=args.save_frames[0]
    resize=args.resize[0]
    crop=args.crop[0]
    cfg = get_cfg()
    # add project-specific config (e.g., TensorMask) here if you're not running a model in detectron2's core library
    cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = args.confidence[0]  # set threshold for this model
    cfg.MODEL.DEVICE= args.device[0] # Run on cpu/cuda
    # Find a model from detectron2's model zoo. You can use the https://dl.fbaipublicfiles... url as well
    if(args.weights=="default"):
        cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
        my_metadata = MetadataCatalog.get(cfg.DATASETS.TRAIN[0])
        #print(my_metadata)

    else:
        cfg.MODEL.ROI_HEADS.NUM_CLASSES=2
        cfg.MODEL.WEIGHTS = os.path.join(ROOT,"detectron2_weights",args.weights[0])
        my_metadata = Metadata()
        my_metadata.set(thing_classes = ['solar-panel'])
        #print(my_metadata)

    predictor = DefaultPredictor(cfg)
    
    show_segmentation=args.show_segmentation[0]

    
    # For test and debug

    pub_boxes = rospy.Publisher('boxes_and_mask', SensImage, queue_size=1)
    pub_segm = rospy.Publisher('segmentation', SensImage, queue_size=1)
    pub_loc = rospy.Publisher('localization', Pose, queue_size=1)
    
    rospy.Subscriber("output/image_raw", SensImage, callback,queue_size=1)
    directory = str(ROOT)+"/images"
    os.makedirs(directory, exist_ok=True)

    time.sleep(1)

    while not rospy.is_shutdown():
        now = time.time() 

        img=cv_image

        im_height,im_width,_=img.shape

        if(crop==1):
            if(im_width>im_height):
                img=img[0:int(im_height),int((im_width-im_height)/2):int(im_width-(im_width-im_height)/2)]
            else:
                img=img[int((im_height-im_width)/2):int(im_height-(im_height-im_width)/2),0:int(im_width)]
            im_height,im_width,_=img.shape

        dsize = (int(im_width*resize), int(im_height*resize))
        img = cv2.resize(img, dsize, interpolation = cv2.INTER_AREA)

        outputs = predictor(img)
        mask=getMask(outputs)


        #change
        if(save_frames>0):
            #print(segmentation.get_image()[:, :, ::-1])
            frame_name = f"{directory}/img{save_frames}.jpg"

        [center,angle,altitude]=getOrientedBoxes(mask,False,pub_boxes)


        if(show_segmentation==1):
            v = Visualizer(img[:, :, ::-1],
                #metadata=my_metadata, 
                scale=1, 
                instance_mode=ColorMode.IMAGE_BW    # remove the colors of unsegmented pixels. This option is only available for segmentation models
                )
            segmentation=showSegmentation(v,outputs,False,pub_segm)

            if(save_frames>0):
                #print(segmentation.get_image()[:, :, ::-1])
                frame_name = f"{directory}/img{save_frames}.jpg"

                overlay_line_with_point(img, center, angle,frame_name)

                save_frames=save_frames+1
            
        if(center is not None and angle is not None):
            loc=Pose()
            loc.position.x=center[0]
            loc.position.y=-center[1]
            loc.position.z=altitude
            loc.orientation.x=im_width
            loc.orientation.y=im_height
            loc.orientation.z=-angle
            pub_loc.publish(loc)
            with open('data_vis.csv', 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([center[0], -center[1], -angle, time.time()])

        else:
            loc=Pose()
            loc.orientation.w = 42 # Just a way to say that no rail was detected
            loc.orientation.x=im_width
            loc.orientation.y=im_height
            pub_loc.publish(loc)

        print("img dimension: ",img.shape)
        print("img seg time: ", time.time()-now)
        
        print("angle:",loc.orientation.z)
        print("x    :",loc.position.x)
        print("y    :",loc.position.y)

#rospy.spin()


if __name__ == "__main__":
    main()











 
#!/usr/bin/python3

# ****************************************************************************
# Copyright(c) 2017 Intel Corporation. 
# License: MIT See LICENSE file in root directory.
# ****************************************************************************

# Detect objects on a LIVE camera feed using
# Intel® Movidius™ Neural Compute Stick (NCS)

import os
import cv2
import sys
import numpy
import ntpath
import argparse

import mvnc.mvncapi as mvnc

from time import localtime, strftime
from utils import visualize_output
from utils import deserialize_output

import serial

width = 640 #leave None for auto-detection
height = 480 #leave None for auto-detection
middle_w = 2 #Cross Center width 
middle_h = 2 #Cross Center height 

#arduino=serial.Serial('/dev/ttyUSB0',9600)
#arduino=serial.Serial('/dev/ttyAMA0',9600)
arduino=serial.Serial('/dev/ttyS0',9600)   #mini UART

# "Class of interest" - Display detections only if they match this class ID
CLASS_PERSON         = 15
CLASS_CAR            = 7

# Detection threshold: Minimum confidance to tag as valid detection
CONFIDANCE_THRESHOLD = 0.60 # 60% confidant

# Variable to store commandline arguments
ARGS                 = None

# OpenCV object for video capture
camera               = None

# ---- Step 1: Open the enumerated device and get a handle to it -------------

def open_ncs_device():

    # Look for enumerated NCS device(s); quit program if none found.
    devices = mvnc.EnumerateDevices()
    if len( devices ) == 0:
        print( "No devices found" )
        quit()

    # Get a handle to the first enumerated device and open it
    device = mvnc.Device( devices[0] )
    device.OpenDevice()

    return device

# ---- Step 2: Load a graph file onto the NCS device -------------------------

def load_graph( device ):

    # Read the graph file into a buffer
    with open( ARGS.graph, mode='rb' ) as f:
        blob = f.read()

    # Load the graph buffer into the NCS
    graph = device.AllocateGraph( blob )

    return graph

# ---- Step 3: Pre-process the images ----------------------------------------

def pre_process_image( frame ):

    # Resize image [Image size is defined by choosen network, during training]
    img = cv2.resize( frame, tuple( ARGS.dim ) )

    # Convert RGB to BGR [OpenCV reads image in BGR, some networks may need RGB]
    if( ARGS.colormode == "rgb" ):
        img = img[:, :, ::-1]

    # Mean subtraction & scaling [A common technique used to center the data]
    img = img.astype( numpy.float16 )
    img = ( img - numpy.float16( ARGS.mean ) ) * ARGS.scale

    return img

# ---- Step 4: Read & print inference results from the NCS -------------------

def infer_image( graph, img, frame ):

    # Load the image as a half-precision floating point array
    graph.LoadTensor( img, 'user object' )

    # Get the results from NCS
    output, userobj = graph.GetResult()

    # Get execution time
    inference_time = graph.GetGraphOption( mvnc.GraphOption.TIME_TAKEN )

    # Deserialize the output into a python dictionary
    output_dict = deserialize_output.ssd( 
                      output, 
                      CONFIDANCE_THRESHOLD, 
                      frame.shape )

    # Print the results (each image/frame may have multiple objects)
    for i in range( 0, output_dict['num_detections'] ):

        # Filter a specific class/category
        if( ( output_dict.get( 'detection_classes_' + str(i) ) == CLASS_PERSON ) or ( output_dict.get( 'detection_classes_' + str(i) ) == CLASS_CAR ) ):

            cur_time = strftime( "%Y_%m_%d_%H_%M_%S", localtime() )
            print( "Person detected on " + cur_time )

            # Print the results (each image/frame may have multiple objects)
            print( "I found these objects in " + " ( %.2f ms ):" % ( numpy.sum( inference_time ) ) )

            print( "%3.1f%%\t" % output_dict['detection_scores_' + str(i)] 
                    + labels[ int(output_dict['detection_classes_' + str(i)]) ]
                    + ": Top Left: " + str( output_dict['detection_boxes_' + str(i)][0] )
                    + " Bottom Right: " + str( output_dict['detection_boxes_' + str(i)][1] ) )
            

            # Extract top-left & bottom-right coordinates of detected objects 
            (y1, x1) = output_dict.get('detection_boxes_' + str(i))[0]
            (y2, x2) = output_dict.get('detection_boxes_' + str(i))[1]

            #center x, y target point of object box
            cx = x1 + (x2-x1)/2
            cy = y1 + (y2-y1)/2
            print( cx, cy )
            # print target number 終端顯示目標號碼 
            print('#' + str(i+1))

            # 追蹤目標識別率最大#1
            if (i+1) == 1 :
                 #servo motor
                 if cx < width*5/ 10 and cx > width*4/ 10 :                  
                      arduino.write(b'e')                
                      print ('e')
            
                 if cx < width*4/ 10 and cx > width*3/ 10 :                  
                      arduino.write(b'd')                
                      print ('d')
                 if cx < width*3/ 10 and cx > width*2/ 10 :                  
                      arduino.write(b'c')                
                      print ('c')                        
                 if cx < width*2/ 10 and cx > width/ 10 :
                      arduino.write(b'b')
                      print ('b')
                 if cx < width/ 10 :
                      arduino.write(b'a')
                      print ('a')

                 if cx > width*6 / 10 and cx < width*7/ 10 :
                      arduino.write(b'f')
                      print ('f')
                 if cx > width*7/ 10 and cx < width*8/ 10 :
                      arduino.write(b'g')
                      print ('g')
                 if cx > width*8/ 10 and cx < width*9/ 10 :
                      arduino.write(b'h')
                      print ('h')
                 if cx > width*9/ 10 :
                      arduino.write(b'i')
                      print ('i')

                 if cy < height*5/ 10 and cy > height*4/ 10 :
                      arduino.write(b'n')
                      print ('n')
                 if cy < height*4/ 10 and cy > height*3/ 10 :
                      arduino.write(b'm')
                      print ('m')
                 if cy < height*3/ 10 and cy > height*2/ 10 :
                      arduino.write(b'l')
                      print ('l')
                 if cy < height*2/ 10 and cy > height/ 10 :
                      arduino.write(b'k')
                      print ('k')
                 if cy < height/ 10:
                      arduino.write(b'j')
                      print ('j')


                 if cy > height*6 / 10 and cy < height*7 / 10 :
                      arduino.write(b'o')
                      print ('o')
                 if cy > height*7 / 10 and cy < height*8 / 10 :
                      arduino.write(b'p')
                      print ('p')
                 if cy > height*8 / 10 and cy < height*9 / 10 :
                      arduino.write(b'q')
                      print ('q')     
                 if cy > height*9 / 10:
                      arduino.write(b'r')
                      print ('r')  
            '''
            #servo motor
            if cx < width*5/ 10 and cx > width*4/ 10 :                  
                 arduino.write(b'e')                
                 print ('e')
            
            if cx < width*4/ 10 and cx > width*3/ 10 :                  
                 arduino.write(b'd')                
                 print ('d')
            if cx < width*3/ 10 and cx > width*2/ 10 :                  
                 arduino.write(b'c')                
                 print ('c')                        
            if cx < width*2/ 10 and cx > width/ 10 :
                 arduino.write(b'b')
                 print ('b')
            if cx < width/ 10 :
                 arduino.write(b'a')
                 print ('a')

            if cx > width*6 / 10 and cx < width*7/ 10 :
                 arduino.write(b'f')
                 print ('f')
            if cx > width*7/ 10 and cx < width*8/ 10 :
                 arduino.write(b'g')
                 print ('g')
            if cx > width*8/ 10 and cx < width*9/ 10 :
                 arduino.write(b'h')
                 print ('h')
            if cx > width*9/ 10 :
                 arduino.write(b'i')
                 print ('i')

            if cy < height*5/ 10 and cy > height*4/ 10 :
                 arduino.write(b'n')
                 print ('n')
            if cy < height*4/ 10 and cy > height*3/ 10 :
                 arduino.write(b'm')
                 print ('m')
            if cy < height*3/ 10 and cy > height*2/ 10 :
                 arduino.write(b'l')
                 print ('l')
            if cy < height*2/ 10 and cy > height/ 10 :
                 arduino.write(b'k')
                 print ('k')
            if cy < height/ 10:
                 arduino.write(b'j')
                 print ('j')


            if cy > height*6 / 10 and cy < height*7 / 10 :
                 arduino.write(b'o')
                 print ('o')
            if cy > height*7 / 10 and cy < height*8 / 10 :
                 arduino.write(b'p')
                 print ('p')
            if cy > height*8 / 10 and cy < height*9 / 10 :
                 arduino.write(b'q')
                 print ('q')     
            if cy > height*9 / 10:
                 arduino.write(b'r')
                 print ('r')
            '''


            # Prep string to overlay on the image
            display_str = ( 
                labels[output_dict.get('detection_classes_' + str(i))]
                + ": "
                + str( output_dict.get('detection_scores_' + str(i) ) )
                + "%" )

            # Overlay bounding boxes, detection class and scores
            frame = visualize_output.draw_bounding_box( 
                        y1, x1, y2, x2, 
                        frame,
                        thickness=4,
                        color=(255, 255, 0),
                        display_str=display_str )
            # follow box of number, 隨目標框顯示目標號碼(號碼依目標識別率最大依序排列)
            cv2.putText(frame, '#' + str(i+1), (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, 'Targets:' + str(i+1), (10, 100), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 255, 0), 2, cv2.LINE_AA)
            # fire
            if ((x2-x1)*(y2-y1)) >= (width*height)/4 and (i+1) == 1 :
                 arduino.write(b'5')
                 print ('fire') 
                 text = 'FIRE'
                 cv2.putText(frame, text, (500, 40), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 2, cv2.LINE_AA)
                    
                 # Capture snapshots
                 photo = ( os.path.dirname(os.path.realpath(__file__))
                           + "/captures/photo_"
                           + cur_time + ".jpg" )
                 cv2.imwrite( photo, frame )

            '''
            # Capture snapshots
            photo = ( os.path.dirname(os.path.realpath(__file__))
                      + "/captures/photo_"
                      + cur_time + ".jpg" )
            cv2.imwrite( photo, frame )
            '''
    # If a display is available, show the image on which inference was performed
    if 'DISPLAY' in os.environ:
        cv2.imshow( 'NCS live inference', frame )

# ---- Step 5: Unload the graph and close the device -------------------------

def close_ncs_device( device, graph ):
    graph.DeallocateGraph()
    device.CloseDevice()
    camera.release()
    cv2.destroyAllWindows()

# ---- Main function (entry point for this script ) --------------------------

def main():

    device = open_ncs_device()
    graph = load_graph( device )

    # Main loop: Capture live stream & send frames to NCS
    while( True ):
        ret, frame = camera.read()
        img = pre_process_image( frame )
        infer_image( graph, img, frame )

        # Display the frame for 5ms, and close the window so that the next
        # frame can be displayed. Close the window if 'q' or 'Q' is pressed.
        if( cv2.waitKey( 5 ) & 0xFF == ord( 'q' ) ):
            break

    close_ncs_device( device, graph )

# ---- Define 'main' function as the entry point for this script -------------

if __name__ == '__main__':

    parser = argparse.ArgumentParser(
                         description="DIY smart security camera PoC using \
                         Intel® Movidius™ Neural Compute Stick." )

    parser.add_argument( '-g', '--graph', type=str,
                         default='../../caffe/SSD_MobileNet/graph',
                         help="Absolute path to the neural network graph file." )

    parser.add_argument( '-v', '--video', type=int,
                         default=0,
                         help="Index of your computer's V4L2 video device. \
                               ex. 0 for /dev/video0" )

    parser.add_argument( '-l', '--labels', type=str,
                         default='../../caffe/SSD_MobileNet/labels.txt',
                         help="Absolute path to labels file." )

    parser.add_argument( '-M', '--mean', type=float,
                         nargs='+',
                         default=[127.5, 127.5, 127.5],
                         help="',' delimited floating point values for image mean." )

    parser.add_argument( '-S', '--scale', type=float,
                         default=0.00789,
                         help="Absolute path to labels file." )

    parser.add_argument( '-D', '--dim', type=int,
                         nargs='+',
                         default=[300, 300],
                         help="Image dimensions. ex. -D 224 224" )

    parser.add_argument( '-c', '--colormode', type=str,
                         default="bgr",
                         help="RGB vs BGR color sequence. This is network dependent." )

    ARGS = parser.parse_args()

    # Create a VideoCapture object
    camera = cv2.VideoCapture( ARGS.video )

    # Set camera resolution
    camera.set( cv2.CAP_PROP_FRAME_WIDTH, width )
    camera.set( cv2.CAP_PROP_FRAME_HEIGHT, height )

    # Load the labels file
    labels =[ line.rstrip('\n') for line in
              open( ARGS.labels ) if line != 'classes\n']

    main()

# ==== End of file ===========================================================

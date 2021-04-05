# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from matplotlib import pyplot as plt
import numpy as np
import math

# finding distance between coordinates function using Haversine formula
# START OF HAVERSINE FUNCTION
def haversine(coord1, coord2):
    R = 6372800  # Earth radius in meters
    lat1, lon1 = coord1
    lat2, lon2 = coord2
    
    phi1, phi2 = math.radians(lat1), math.radians(lat2) 
    dphi       = math.radians(lat2 - lat1)
    dlambda    = math.radians(lon2 - lon1)
    
    a = math.sin(dphi/2)**2 + \
        math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    
    return 2*R*math.atan2(math.sqrt(a), math.sqrt(1 - a))
# END OF HAVERSINE FUNCTION

# START OF ORIENTATION FUNCTION
def getEllipseRotation(image, cnt):
    try:
        # Gets rotated bounding ellipse of contour
        ellipse = cv2.fitEllipse(cnt)
        centerE = ellipse[0]
        # Gets rotation of ellipse; same as rotation of contour
        rotation = ellipse[2]
        # Gets width and height of rotated ellipse
        widthE = ellipse[1][0]
        heightE = ellipse[1][1]
        # Maps rotation to (-90 to 90). Makes it easier to tell direction of slant
        rotation = translateRotation(rotation, widthE, heightE)

        cv2.ellipse(image, ellipse, (23, 184, 80), 3)
        return rotation
    except:
        # Gets rotated bounding rectangle of contour
        rect = cv2.minAreaRect(cnt)
        # Creates box around that rectangle
        box = cv2.boxPoints(rect)
        # Not exactly sure
        box = np.int0(box)
        # Gets center of rotated rectangle
        center = rect[0]
        # Gets rotation of rectangle; same as rotation of contour
        rotation = rect[2]
        # Gets width and height of rotated rectangle
        width = rect[1][0]
        height = rect[1][1]
        # Maps rotation to (-90 to 90). Makes it easier to tell direction of slant
        rotation = translateRotation(rotation, width, height)
        return rotation
# END OF ORIENTATION FUNCTION

# # START OF GPS 
# #Get input GPS, will be updated in ECEN 404 when we have access to Drone GPS module
# homing_lat = float(input("What is the homing latitude? eg. 30.623585: "));
# homing_long = float(input("What is the homing longitude? eg. -96.340959: "));
# homing_coor = (homing_lat, homing_long)
# #print(homing_gps)
# current_lat = float(input("What is the current latitude? eg. 30.623161: "));
# current_long = float(input("What is the homing longitude? eg. -96.340390: "));
# current_coor = (current_lat, current_long)
# #print(current_gps)
# 
# # Find Distance
# distance_from = haversine(homing_coor, current_coor)
# print(distance_from)
# END OF GPS

# START OF CAMERA
#if distance_from < 100:
distance = 1
if distance == 1:
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    # allow the camera to warmup
    time.sleep(0.1)
    # capture frames from the camera

    #Stop sign detection test
    stop_data = cv2.CascadeClassifier('stop_data.xml')

    # Write to video
    fourcc = cv2.VideoWriter_fourcc(*'XVID') 
    out = cv2.VideoWriter('output.avi', fourcc, 5.0, (640, 480))
    
    # starting image capture
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        
        img = frame.array
        # Canny Edge Detection
        edges = cv2.Canny(img,100,200)
        
        #editing picture
        font = cv2.FONT_HERSHEY_SIMPLEX 
        
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
        found = stop_data.detectMultiScale(img_gray, minSize =(20, 20))
        amount_found = len(found)
        
      
        # Determining left and right
        middleX = 320
        middleY = 240
        th = 10
        if amount_found != 0: 
            for (x, y, width, height) in found:
                # Determining if the drone should move to the left or right to land on center of image
                cv2.rectangle(img, (x, y),  (x + height, y + width), (255, 255, 0), 5)
                centerX = int(x + height/2)
                centerY = int(y + width/2)
                # circle in center of camera
                pt = '('+str(centerX)+','+str(centerY)+')'
                # putting circle in center of iamge
                cv2.circle(img, (centerX,centerY), radius=3, color=(0,255,0), thickness=5)
                cv2.putText(img, pt,(50, 50), font, 1,(0, 255, 0), 2, cv2.LINE_4)
                if centerX < middleX-th:
                    cv2.putText(img, 'X: Right',(50, 100), font, 1,(0, 0, 255), 2, cv2.LINE_4) 
                elif centerX > middleX+th:
                    cv2.putText(img, 'X: Left',(50, 100), font, 1,(0, 0, 255), 2, cv2.LINE_4)
                else:
                    cv2.putText(img, 'X: OK',(50, 100), font, 1,(0, 0, 255), 2, cv2.LINE_4)
                
                if centerY < middleY-th:
                    cv2.putText(img, 'Y: Down',(50, 150), font, 1,(0, 0, 255), 2, cv2.LINE_4) 
                elif centerY > middleY+th:
                    cv2.putText(img, 'Y: Up',(50, 150), font, 1,(0, 0, 255), 2, cv2.LINE_4)
                else:
                    cv2.putText(img, 'Y: OK',(50, 150), font, 1,(0, 0, 255), 2, cv2.LINE_4)
                    
                
        #Find Orientation
        getEllipseRotation(edges, found)
            
        #show the frame
        cv2.circle(img, (middleX,middleY), radius=3, color=(255,255,0), thickness=5)
        out.write(img)  
        cv2.imshow("Frame", img)
        cv2.imshow("Edges", edges);
        
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
        #if key == ord("q"):
        #break
        if cv2.waitKey(1) & 0xFF == ord('a'): 
            break
    # Close the window / Release webcam 
    #img.release() 
      
    # After we release our webcam, we also release the output 
    out.release()  
      
    # De-allocate any associated memory usage  
    cv2.destroyAllWindows()
else:
    # keep video off if not close enough to homing location
    camera = PiCamera(0)
    exit()
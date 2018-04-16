### sudo modprobe bcm2835-v4l2
### Line Tracking ###
### 16/4/2018 ###

### ~~~ Imports ~~~ ###
import cv2
import numpy as np
import serial
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
import time

### ~~~ Variables ~~~ ###
counter = 0

#Pin Definitions
pwmPin = 18

# Pin Setup
GPIO.setup(pwmPin, GPIO.OUT)

# PWM Setup
pwm = GPIO.PWM(pwmPin, 50)  # 50Hz = 20ms
pwm.start(0)
pwm.ChangeDutyCycle(10.0)
time.sleep(1)
pwm.stop()

### ~~~ Camera Setup ~~~ ###
cap = cv2.VideoCapture(0)
cap.set(3, 360)
cap.set(4, 240)
#cap.set(5, 60)
#print (cap.get(3))     #width
#print (cap.get(4))     #height
#print (cap.get(5))     #fps

### ~~~ Serial ~~~ ###
ser = serial.Serial('/dev/ttyUSB0',9600)
#print(ser.name)

### ~~~ User Functions ~~~ ###
# Colour Ranging
#kernal = np.ones((15, 15), "uint8")

def RangeBlack():
    lower_black = np.array([0,0,0])
    #upper_black = np.array([255,255,65])
    upper_black = np.array([255,95,60])
    black = cv2.inRange(hsv, lower_black, upper_black)
    #black=cv2.dilate(black,kernal)
    #res0=cv2.bitwise_and(frame, frame, mask = black)
    return black

def RangeColour():
    # Red #
    lower_red = np.array([160,50,0])
    upper_red = np.array([180,255,255])
    red = cv2.inRange(hsv, lower_red, upper_red)
    #red=cv2.dilate(red, kernal)
    #res1=cv2.bitwise_and(frame, frame, mask = red)

    # Yellow #
    lower_yellow = np.array([20,85,0])
    upper_yellow = np.array([30,255,255])
    yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    #yellow=cv2.dilate(yellow,kernal)
    #res2=cv2.bitwise_and(frame, frame, mask = yellow) 

    # Blue #
    lower_blue = np.array([105,110,0])
    upper_blue = np.array([120,255,255])
    blue = cv2.inRange(hsv, lower_blue, upper_blue)
    #blue=cv2.dilate(blue,kernal)
    #res3=cv2.bitwise_and(frame, frame, mask = blue)

    # Green #
    lower_green = np.array([40,110,0])
    upper_green = np.array([800,255,255])
    green = cv2.inRange(hsv, lower_green, upper_green)
    #green=cv2.dilate(green,kernal)
    #res4=cv2.bitwise_and(frame, frame, mask = green)

    return red, yellow, blue, green

# Colour Track
def CntColour(x):
    (_,contours,hierarchy)=cv2.findContours(x,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    return contours

# Line Following
def LineFollowing():
    c = max(contours, key=cv2.contourArea)
    M = cv2.moments(c)

    try:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        if cx < 220 and cx > 140:
            #command = "c"
            #print ("On Track!")
            ser.write(bytes("c","UTF-8"))
        
        if cx >= 220:
            #command = "r"
            #print ("Turn Right!")
            ser.write(bytes("r","UTF-8"))

        if cx <= 140:
            #command = "l"
            #print ("Turn Left")
            ser.write(bytes("l","UTF-8"))

        #cv2.line(frame,(cx,0),(cx,720),(255,0,0),1)
        #cv2.line(frame,(0,cy),(1280,cy),(255,0,0),1)
        #cv2.drawContours(frame, contours, -1, (0,255,0), 1)
        
    except ZeroDivisionError:
        #command = "x"
        #print("No Line Detected")
        ser.write(bytes("x","UTF-8"))

### ~~~ Circle Detection Function ~~~ ###
def CircleDetect():
    rows = gray.shape[0]
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8, param1=50, param2=30, minRadius=40, maxRadius=60)

    if circles is not None:
        circles = np.uint16(np.around(circles)) 
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            cv2.circle(frame, center, 1, (255, 0, 0), 3)
            # circle outline
            radius = i[2]
            cv2.circle(frame, center, radius, (0, 0, 255), 3)
            counter = 1

    if circles is None:
        counter = 0

    return counter

### ~~~ Main Loop ~~~ ###
while 1:
    # capture frame-by-frame
    ret, frame = cap.read()

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Convert to Gray
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Circle Detect
    circle = CircleDetect()

    # Colour Ranging 
    black = RangeBlack()

    ### Main of Main ###
    if circle == 0:
        #Track Black
        contours = CntColour(black)
        for i in enumerate(contours):
            LineFollowing()

    if circle == 1:
        #STOP
        ser.write(bytes("x","UTF-8"))

    #display frames
    cv2.imshow("frame", frame)
    cv2.imshow("hsv", hsv)
    cv2.imshow("gray", gray)

    # break
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    
### ~~~ Terminate ~~~ ###
ser.write(bytes("x","UTF-8"))
cap.release()
cv2.destroyAllWindows()
ser.close()
pwm.stop()
GPIO.cleanup()
    


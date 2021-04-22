import cv2  # OPENCV FOR VIDEO PROCESSING AND FOR REFERNCE YOU CAN USE THE OPEN CV PYTHON DOCUMENTATION
import numpy as np  # NUMPY
import serial  # SERIAL FOR COMMUNICATION WITH ARDUINO
import time  # FOR GIVING DELAYS


ser = serial.Serial('COM3', 9600, timeout=1)  # MAKING A SERIAL VARIABLE


def shoot():  # DEFINING FUNCTION SHOOT WHICH WRITES(OR SENDS) '1' SERIALLY TO ARDUINO TO WHICH ARDUINO FURTHER ACTS
    ser.write(b'1')  # SENDING DATA TO ARDUINO
    data = ser.readline().decode('ascii')  # RECEIVEING DATA FROM ARDUINO(WHICH ARE STRINGS)
    return data


def hAllignR():  # DEFINING FUNCTION SHOOT WHICH WRITES(OR SENDS) '2' SERIALLY TO ARDUINO TO WHICH ARDUINO FURTHER ACTS
    ser.write(b'2')  # SENDING DATA TO ARDUINO
   # data = ser.readline().decode('ascii')
    return

def hAllignL():  # DEFINING FUNCTION SHOOT WHICH WRITES(OR SENDS) '3' SERIALLY TO ARDUINO TO WHICH ARDUINO FURTHER ACTS
    ser.write(b'3')  # SENDING DATA TO ARDUINO

    #data = ser.readline().decode('ascii')
    return


def vAllignD():  # DEFINING FUNCTION SHOOT WHICH WRITES(OR SENDS) '4' SERIALLY TO ARDUINO TO WHICH ARDUINO FURTHER ACTS
    ser.write(b'4')  # SENDING DATA TO ARDUINO
    #data = ser.readline().decode('ascii')
    return


def vAllignU():  # DEFINING FUNCTION SHOOT WHICH WRITES(OR SENDS) '5' SERIALLY TO ARDUINO TO WHICH ARDUINO FURTHER ACTS
    ser.write(b'5')  # SENDING DATA TO ARDUINO
    #data = ser.readline().decode('ascii')
    return

facecascade=cv2.CascadeClassifier('resources/haarcascade_frontalface_default.xml')
#it is the xml file which has predifined code for trainers and detectors cascade classifier is the method to callsify what we need
#img=cv2.imread('messi5.jpg')
cap=cv2.VideoCapture(0)#capturing the video from  default cam
while True:
    ret,img=cap.read()#reading each frame
    img1=img
    imgrev=cv2.flip(img,1)
    img=imgrev
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # CONVERTED THE COLOURED FRAME IMAGE TO GRAY
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower1 = np.array([94, 93,100])  # AS HSV OF RED COLOR LIES IN THE RANGE OF [(0,100,100) TO (10,255,255)] OR[(160,100,100) TO (179,255,255)]
    upper1 = np.array([176, 255, 255])
    #lower2 = np.array([160, 100, 100])
    #upper2 = np.array([179, 255, 255])

    # I HAVE MADE A MASK THAT CONTAINS ONLY THE HSV RANGES BETWEEN  [(0,100,100) TO (10,255,255)] WHICH CONTAINS COLORS WHICH ARE IN THIS RANGE
    mask1 = cv2.inRange(hsv, lower1, upper1)

    # I HAVE MADE A MASK THAT CONTAINS ONLY THE HSV RANGES BETWEEN  [(160,100,100) TO (179,255,255)] WHICH ARE IN THIS RANGE
  #  mask2 = cv2.inRange(hsv, lower2, upper2)

    # AS I WANTED TWO RANGES SO I PERFORMED " OR " OPERATION OF BOTH OF THEM
    #mask = cv2.bitwise_or(mask1, mask2)
    mask=mask1
    # NOW MASK CONTAINS ONLY RED COLOR IN THE COMPLETE IMAGE

    # NOW DRAWING CONTOURS FOR THIS WHICH IS GOING TO BE ONLY THE RED COLOR
    contours, __ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # DRAWING THE CONTOURS
    for contour in contours:
        (x,y,w,h)=cv2.boundingRect(contour)
        #making contour in the shaoe of rectangle since we will get in form of the object and this function gives the x,y
        #co ordinates and width ,height of rectangle
        #area=cv2.contourArea(contour)#finding the area og=f the rectangle so that we can mark specified object only
        if cv2.contourArea(contour)>2800:
            continue#if the area of rectangle is lss than 200 it skips the iteration
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
    #cv2.drawContours(img, contours, -1, (0, 255, 100), 2)

    # INITAILING TWO VARIABLE 'A' AND 'B' AND THEY ARE GOING TO BE THE CENTER OF THE RED SPOT
    a = 0
    b = 0
    if (len(contours) > 0):  # IF RED WAS DETECTED
        for i in range(len(contours)):  # LOOPING BETWEEN ALL THE CONTOURS
            cnt = contours[i]  # CNT CONTAINS THE CONTOURS[I]

            m = cv2.moments(cnt)  # GETTING THE MOMENTS OF THAT CONTOUR
            if (m['m00'] != 0):
                a = int(m['m10'] / m['m00'])  # THIS GIVES THE CENTER(X COORDINATE) OF THE RED SPOT
                b = int(
                    m['m01'] / m['m00'])  # THIS GIVES THE CENTER(X COORDINATE) OF THE RED SPOT

    faces = facecascade.detectMultiScale(gray, 1.1, 4)
    print(faces)
    for (x,y,w,h) in faces:#we need to iterate the faces bcpz ther might be many faces in  the image
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),3)#drawing the rectangle on the faces

        if ((a > (x)) & (a < (x + w)) & (b > (y) )& (b < (y + h))):  # IF THE LASER SPOT LIES BETWEEN THIS BOX THEN SHOOT AND GIVE 5 SECONDS DELAY
            print("shoot")
            shoot()
            time.sleep(5)
        elif (a < (x )):  # IF THE LASER SPOT LIES TO THE LEFT OF THE BOX MOVE RIGHT
            hAllignR()
        elif (a > (x + w)):  # IF THE LASER SPOT LIES TO THE RIGHT OF THE BOX MOVE LEFT
            hAllignL()

        elif (b < (y )):  # IF THE LASER SPOT LIES BELOW THE RECTANGLE THEN MOVE UP
            vAllignU()
        elif (b > (y - h)):  # IF  LASER THE SPOT LIES ABOVE THE RACTANGLE THEN MOVE DOWN
            vAllignD()

    cv2.imshow('image',img)
    if cv2.waitKey(1)==27:
        break
cap.release()
cv2.destroyAllWindows()

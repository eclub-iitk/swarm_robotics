import numpy as numpy
import cv2
import cv2.aruco as aruco
import math
import datetime
import time
import glob
from socket import *

def make_1080p():
    cap.set(3, 1920)
    cap.set(4, 1080)
    cap.set(cv2.CAP_PROP_AUTOFOCUS,0)
    #i = 0
    #x = (0, 0)
    #y = (0, 0)
    #theta = 0
    #j=0
    #g=0
    #phi = 0
 
    
def draw_circle(event,x,y,flags,param):
    global ix,iy
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img,(x,y),10,(255,0,0),-1)
        ix,iy = x,y 

def distance(pt1, pt2):
        x = pt1[0] - pt2[0]
        y = pt1[1] - pt2[1]
        distance = math.sqrt(x*x + y*y)
        #print("distance: ",distance)
        return distance


def storing_bot_location(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    aruco_frame = aruco.drawDetectedMarkers(gray, corners)
    i=0
    while(i<3):
        for marker in range(len(ids)):
            x_center= int((corners[marker][0][0][0] + corners[marker][0][1][0] + corners[marker][0][2][0] + corners[marker][0][3][0])/4)
            y_center= int((corners[marker][0][0][1] + corners[marker][0][1][1] + corners[marker][0][2][1] + corners[marker][0][3][1])/4)
            if ids[marker] == i:
                botx.append(x_center)
                boty.append(y_center)
                i=i+1

def matching():
    i=0
    j=0
    while(i<3):
        j=i
        while(j<3):
            dist=distance((botx[i],boty[i]),(goalx[j],goaly[j]))
            if(i==j):
                dist1=dist
            if(dist<dist1):
                tempx=goalx[i]
                tempy=goaly[i]
                goalx[i]=goalx[j]
                goaly[i]=goaly[j]
                goalx[j]=tempx
                goaly[j]=tempy
                dist1=dist
            j=j+1
        i=i+1

class movement:   
    def __init__(self,i,x,y,theta,j,g,phi,bot):
        self.i=i
        self.x=x
        self.y=y
        self.theta=theta
        self.j=j
        self.g=g
        self.phi=phi
        self.bot=bot
        self.t = 0
        self.init_dist = 0
        self.t1 = 0
        self.t2 = 0
        self.a = 0
    


    
    def angle_calculate(self,pt1, pt2):

        a = pt2[0]-pt1[0]
        b = pt2[1]-pt1[1]
        angle = math.degrees(math.atan2(b, a))
        if(angle<0):
              angle = 360 + angle
        return int(angle)

    def allignment(self,theta, phi,dist,ip):
        global g
        # global init_dist
        # global t
        # global t1
        # global t2
        clientSocket1 = socket(AF_INET, SOCK_DGRAM)
        clientSocket1.settimeout(1)
        print("angle difference: ",(theta - phi))
        addr1 = (ip, 5007)
        if dist<100:
            clientSocket1.sendto('4'.encode(), addr1)
            print("Bot ", self.bot, " has reached.")
        else:
            if -15<=theta-phi<=15:
                if(self.g<=3):       
                    clientSocket1.sendto('4'.encode(), addr1)   #stop
                    self.g=self.g+1
                    print(4)
                else:
                    #g=0
                    if (dist>100):
                        clientSocket1.sendto('0'.encode(), addr1)    #forward
                        if -5 < self.init_dist - dist < 5:
                            self.t = self.t+1
                        if self.t == 5:
                            clientSocket1.sendto('5'.encode(), addr1)    #impulse
                            print(5)
                            self.t = 0
                        
                        print(0)
                    else:
                        clientSocket1.sendto('4'.encode(), addr1)
                        print("Bot ", self.bot, " has reached.")
                    
            else:
                if  ((0<= theta-phi <=180) | (180 <=phi-theta <= 360)):
                    clientSocket1.sendto('3'.encode(), addr1)   #left
                    if -5 < self.init_dist - dist < 5:
                        self.t1 = self.t1+1
                    if self.t1 == 5:
                        clientSocket1.sendto('5'.encode(), addr1)    #impulse
                        print(5)
                        self.t1 = 0
                    print(3)
                #elif 180<= theta-phi <=360 | -180<= theta-phi <=0:
                else:
                    clientSocket1.sendto('2'.encode(), addr1)   #right
                    if -5 < self.init_dist - dist < 5:
                        self.t2 = self.t2+1
                    if self.t2 == 5:
                        clientSocket1.sendto('5'.encode(), addr1)    #impulse
                        print(5)
                        self.t2 = 0
                    print(2) 
                
                
        self.init_dist = dist

    def aruco_detect(self,frame, ip):
        detect = 0
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()

        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        aruco_frame = aruco.drawDetectedMarkers(gray, corners)
        #print("corners")
        #print(len(corners))
        if len(corners)>0:
            self.a = 0
            for marker in range(len(ids)):
                if (ids[marker] == self.bot):
                    self.a = self.a+1
                    detect = detect + 1
                               
            if self.a != 1:
                clientSocket1 = socket(AF_INET, SOCK_DGRAM)
                clientSocket1.settimeout(1)
                addr1 = (ip, 5007)
                detect = 0
                clientSocket1.sendto('4'.encode(), addr1) 
                cv2.imshow("aruco_frame", frame)
                print(4)
                print(self.bot, "NOT DETECTED");
                return
            
            for marker in range(len(ids)):

                #print(len(corners))
                #print(ids)
                x_center= int((corners[marker][0][0][0] + corners[marker][0][1][0] + corners[marker][0][2][0] + corners[marker][0][3][0])/4)
                y_center= int((corners[marker][0][0][1] + corners[marker][0][1][1] + corners[marker][0][2][1] + corners[marker][0][3][1])/4)

                cv2.circle(frame, (x_center, y_center),2,(0,0,255),2)#red dot at center
                x1 = int(corners[marker][0][0][0])
                x3 = int(corners[marker][0][3][0])
                y1 = int(corners[marker][0][0][1])
                y3 = int(corners[marker][0][3][1])

                pt1 = (x3,y3)
                pt2 = (x1,y1)
                #   Head
                #    ___
                # pt2|      |
                #    |      |
                #    |      |
                # pt1|      |
                #    --------
                cv2.circle(frame, pt1, 2, (0,0,255), 2)#corner
                cv2.circle(frame,pt2, 2, (0,0,255), 2)#corner
                cv2.imshow('aruco_frame', frame)
		
                if ids[marker] == self.bot:
                    pt1 = (x3,y3)
                    pt2 = (x1,y1)
                    self.theta = self.angle_calculate(pt1, pt2)
                    print('bot id:',self.bot)
                    print('theta', self.theta)
                    self.x = (x_center, y_center)
                cv2.imshow('aruco_frame', frame)
                #robot[int(ids[marker])]=(int(x_center), int(y_center), int(self.theta))
                self.phi = self.angle_calculate(self.x, self.y)
    	
            print("phi: ",self.phi)
            dist = distance(self.x,self.y)
            print("distance:",dist)
            if len(ids)>1:
                self.allignment(self.theta, self.phi,dist,ip)
                self.j=2
        start = 0
        start_time_update = time.time()

        cv2.imshow("aruco_frame", frame)
        return
# robot={}
l=0
goalx=[]
goaly=[]
botx=[]
boty=[]
cap = cv2.VideoCapture(0)
make_1080p()
ret,img = cap.read()
cv2.namedWindow('image')
cv2.setMouseCallback('image',draw_circle)
while(1):
    cv2.imshow('image',img)
    if(l<3):
        k = cv2.waitKey(20) & 0xFF
        if k == 27:
            break
        elif k == ord('a'):
            print (ix,iy)
            goalx.append(ix)
            goaly.append(iy)
            l=l+1
    else:
        break

_,img_rgb = cap.read()
storing_bot_location(img_rgb)
matching()
print(goalx[0],goaly[0])
print(goalx[1],goaly[1])
print(goalx[2],goaly[2])
robot1=movement(0,(0,0),(goalx[0],goaly[0]),0,0,0,0,0)
robot2=movement(0,(0,0),(goalx[1],goaly[1]),0,0,0,0,1)
robot3=movement(0,(0,0),(goalx[2],goaly[2]),0,0,0,0,2)
#robot4=movement(0,(0,0),(0,0),0,0,0,0,7,2)
##robot3=movement(0,(0,0),(0,0),0,0,0,0,6,5)
while(1):
    _,img_rgb = cap.read()
   # img_rgb = img_rgb[5:680, 184:1250]
    robot1.aruco_detect(img_rgb,"192.168.0.114")
    robot2.aruco_detect(img_rgb,"192.168.0.125")
    robot3.aruco_detect(img_rgb,"192.168.0.103")
    #robot4.aruco_detect(img_rgb,"192.168.0.128")
##   robot = robot3.aruco_detect(img_rgb,robot,"192.168.0.103")
    k =  cv2.waitKey(1) & 0xFF
    if k == 27:
       cap.release()
       cv2.destroyAllWindow()
       break

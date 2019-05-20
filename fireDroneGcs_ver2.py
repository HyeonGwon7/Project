
import cv2

import numpy as np

import time

from dronekit import connect, Command , LocationGlobal

from pymavlink import mavutil

import random


#video_file = "video_1.mp4"

#video = cv2.VideoCapture("/home/sw-203-09/fire1.mp4")



video = cv2.VideoCapture("/dev/video1") #"/dev/video0"

font = cv2.FONT_HERSHEY_COMPLEX

vehicle = connect('/dev/ttyUSB0',baud=57600)

#370,250 aimpoint

#620,30  battery



def onChange(x):

	pass



def drone_info():

	cv2.namedWindow('DroneInfo')
	background=np.zeros((600,300,3),np.uint8)
	
	cv2.putText(background,"gps : " ,(20,20),font,1,(255,255,255),2,cv2.LINE_AA)
	cv2.putText(background,"%s" % vehicle.gps_0,(100,20),font,1,(255,255,255),2,cv2.LINE_AA)

	cv2.putText(background,"empty1 :    ",(20,40),font,1,(0,0,0))

	cv2.putText(background,"empty1_V",(100,40),font,1,(0,0,0))

	cv2.putText(background,"empty2 :    ",(20,60),font,1,(0,0,0))

	cv2.putText(background,"empty2_V",(100,60),font,1,(0,0,0))

	cv2.putText(background,"empty3 :    ",(20,80),font,1,(0,0,0))

	cv2.putText(background,"empty3_V",(100,80),font,1,(0,0,0))

	cv2.putText(background,"empty4 :    ",(20,100),font,1,(0,0,0))

	cv2.putText(background,"empty4_V",(100,100),font,1,(0,0,0))

	cv2.putText(background,"empty5 :    ",(20,120),font,1,(0,0,0))

	cv2.putText(background,"empty5_V",(100,120),font,1,(0,0,0))

	cv2.putText(background,"empty6 :    ",(20,140),font,1,(0,0,0))

	cv2.putText(background,"empty6_V",(20,60),font,1,(0,0,0))




def setting_bar():

	cv2.namedWindow('HSV_settings')

	cv2.createTrackbar('H_MAX', 'HSV_settings', 0, 255, onChange)

	cv2.setTrackbarPos('H_MAX', 'HSV_settings', 255)

	cv2.createTrackbar('H_MIN', 'HSV_settings', 0, 255, onChange)

	cv2.setTrackbarPos('H_MIN', 'HSV_settings', 0)

	cv2.createTrackbar('S_MAX', 'HSV_settings', 0, 255, onChange)

	cv2.setTrackbarPos('S_MAX', 'HSV_settings', 255)

	cv2.createTrackbar('S_MIN', 'HSV_settings', 0, 255, onChange)

	cv2.setTrackbarPos('S_MIN', 'HSV_settings', 0)

	cv2.createTrackbar('V_MAX', 'HSV_settings', 0, 255, onChange)

	cv2.setTrackbarPos('V_MAX', 'HSV_settings', 255)

	cv2.createTrackbar('V_MIN', 'HSV_settings', 0, 255, onChange)

	cv2.setTrackbarPos('V_MIN', 'HSV_settings', 0)

setting_bar()






while True:

	_, frame = video.read()

	
	hcenter=int(720/2)

	vcenter=int(480/2)

	llen=10

	cv2.line(frame,(hcenter-llen,vcenter),(hcenter+llen,vcenter),(255,0,0),5)

	cv2.line(frame,(hcenter,vcenter-llen),(hcenter,vcenter+llen),(255,0,0),5)





	blur = cv2.GaussianBlur(frame, (25,25), 0)


	mask =np.ones((5,5),np.uint8)



	hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)

	opening_kernel = np.ones((5,5),np.uint8)

	opening_hsv = cv2.morphologyEx(hsv,cv2.MORPH_OPEN,opening_kernel)



#	lower = [0,100,100]

#	upper = [5,255,255]

#hsv



	H_MAX = cv2.getTrackbarPos('H_MAX', 'HSV_settings')

	H_MIN = cv2.getTrackbarPos('H_MIN', 'HSV_settings')

	S_MAX = cv2.getTrackbarPos('S_MAX', 'HSV_settings')

	S_MIN = cv2.getTrackbarPos('S_MIN', 'HSV_settings')

	V_MAX = cv2.getTrackbarPos('V_MAX', 'HSV_settings')

	V_MIN = cv2.getTrackbarPos('V_MIN', 'HSV_settings')

	lower = np.array([H_MIN, S_MIN, V_MIN])

	upper = np.array([H_MAX, S_MAX, V_MAX])







	lower = np.array(lower, dtype="uint8")

	upper = np.array(upper, dtype="uint8")



	mask = cv2.inRange(opening_hsv,lower,upper)

	kernel = np.ones((5,5),np.uint8)



	mask = cv2.erode(mask,kernel)

	

	output = cv2.bitwise_and(frame,opening_hsv,mask=mask)

	no_red = cv2.countNonZero(mask)

	

	contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


	for cnt in contours:



		if int(no_red)>20000:
			area = cv2.contourArea(cnt)
			approx = cv2.approxPolyDP(cnt,0.02*cv2.arcLength(cnt,True),True)

	#		cv2.drawContours(output,[approx],0,(0,255,0),5) #only contourarea

			x,y,w,h = cv2.boundingRect(cnt)


			if (w and h) <= 100:

				pass

			else:

				cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),5)

				if (x<=hcenter and hcenter<=x+w) or (y<=vcenter and vcenter<=y+h):

					cv2.line(frame,(hcenter-llen,vcenter),(hcenter+llen,vcenter),(0,255,0),5)

					cv2.line(frame,(hcenter,vcenter-llen),(hcenter,vcenter+llen),(0,255,0),5)

				#if len(approx) == True:
					print("detecting Firte!!")
					cv2.putText(frame,"fire detected",(x,y),font,1,(255,0,0))

	
	version = vehicle.version
	type = str(vehicle._vehicle_type)
	gps = str(vehicle.gps_0)
	alt = str(vehicle.location.global_relative_frame.alt)
	pitch = str(vehicle.attitude.pitch)
	roll = str(vehicle.attitude.roll)
	yaw = str(vehicle.attitude.yaw)
	gdsp =str(vehicle.groundspeed)
	dist =str(vehicle.rangefinder)
	voltage = str(vehicle.battery)
	
	level = voltage[-3:-1]
	gpsfix =  gps[12:17]
	gps2 = gps[25:30]

#	시나리오1 위로 상승 (타겟 고정)
	dis=random.randint(300,340) #랜덤 난수 생성 ( x~y 사이의 난수)
	distance=str(dis)
#        discance=str(random.randint(100,130))
#	시나리오2 전방으로 이동
#	distance = str(x)            x는 날리기 전 대충 거리 재서 입력 해주세염
#	distance = distance -  y     전방으로 이동하면서 감소할 거리 y값 입력

	cv2.putText(frame,"GPS : %s %s " % (gpsfix, gps2),(500,390),font,0.7,(0,0,0),3) #지피에스
	cv2.putText(frame,"%s" %voltage[len(voltage)-2:len(voltage)],(600,30),font,1.0,(0,0,0),3) # 베터리
	cv2.putText(frame,"%",(645,33),font,1.0,(0,0,0),3)
	cv2.putText(frame,"alt : %s" %alt[0:7],(500,270),font,0.7,(0,0,0),3) # 고도
	cv2.putText(frame,"pitch : %s" %pitch[0:7],(500,300),font,0.7,(0,0,0),3) #피치	
	cv2.putText(frame,"roll : %s" %roll[0:7],(500,330),font,0.7,(0,0,0),3) #롤
	cv2.putText(frame,"yaw : %s" %yaw[0:7],(500,360),font,0.7,(0,0,0),3) #요우
	cv2.putText(frame,"Firmware : %s" %version,(20,30),font,0.5,(0,0,0),3)
	cv2.putText(frame,"distance %s cm" % distance,(500,420),font,0.7,(0,0,0),3)
	cv2.imshow("output",frame)

	
	if cv2.waitKey(1) & 0xFF == ord('q'):

		break


cv2.destroyAllWindows()

video.release()

import cv2
import numpy as np
import rospy
from std_msgs.msg import Bool #추가
from std_msgs.msg import Float64 #추가
from cv_bridge import CvBridge, CvBridgeError	#추가
from sensor_msgs.msg import Image	#추가

rospy.init_node("camera")  
cam_pub1 = rospy.Publisher("cam_pub_find", Bool, queue_size = 10) #추가
cam_pub2 = rospy.Publisher("cam_pub_range",Float64,queue_size = 10) #추가
rate = rospy.Rate(30)추가
bool_turtle=Bool() #추가
bool_turtle=False #추가
        

def listener(): #추가
    cb=CvBridge()
    lower_range=np.array([7,149,127]) #trackbar로 rgb값 조정
    upper_range=np.array([179,255,255]) #trackbar로 rgb값 조정
    while True:
        data=rospy.wait_for_message("camera_image",Image,timeout=5) #추가
        frame=cb.imgmsg_to_cv2(data)
        print("hi")
        frame=cv2.resize(frame,(640,480)) #해상도 640x480
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) #BRG to HSV 변환
        mask=cv2.inRange(hsv,lower_range,upper_range) #색상 범위 제한
        ret2,mask1=cv2.threshold(mask,254,255,cv2.THRESH_BINARY) #cv2.threshold(입력 이미지, 임계값, 임계값을 넘었을 때 적용할 값, type)
        cnts,_=cv2.findContours(mask1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE) #외곽선 검출
        if ret2==0: #추가
    	    bool_turtle=False #추가
        else: #추가
    	    bool_turtle=True #추가
        for c in cnts:
            x=600
            if cv2.contourArea(c)>x:
                x,y,w,h=cv2.boundingRect(c) #bounding box 반환
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2) #bounding box를 이미지에 표시
                cv2.putText(frame,("DETECT"),(10,60),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2) #인식하면 detect 표시
           
                obj_x=Float64()#추가
                obj_x.data=(x+x+w)/2#추가
                print(obj_x.data)#추가
                cam_pub1.publish(bool_turtle) #추가
                cam_pub2.publish(obj_x) #추가
                cv2.circle(frame, (int((x+x+w)/2),int((y+y+h)/2)), 10, (255, 255, 0), -1, cv2.LINE_AA) #추가
                print("x: ", x-320, "y: ", y-240)
        cv2.imshow("FRAME",frame)
        rate.sleep()#추가
        if cv2.waitKey(1)&0xFF==27:
            break
    print("hello")
    cv2.destroyAllWindows()
if __name__ == '__main__': #추가
    listener()#추가

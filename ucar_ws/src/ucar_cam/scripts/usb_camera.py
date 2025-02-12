import numpy as np
import cv2
import time
import rospy

dist=np.array(([[-0.58650416 , 0.59103816, -0.00443272 , 0.00357844 ,-0.27203275]]))
newcameramtx=np.array([[189.076828   ,  0.    ,     361.20126638]
 ,[  0 ,2.01627296e+04 ,4.52759577e+02]
 ,[0, 0, 1]])
mtx=np.array([[398.12724231  , 0.      ,   304.35638757],
 [  0.       ,  345.38259888, 282.49861858],
 [  0.,           0.,           1.        ]])
cap = cv2.VideoCapture("/dev/ucar_video")
weight=320
height=240
cap.set(3, weight)  # 设置分辨率 3和4 分别代表摄像头的属性值。你可以使用函数 cap.get(propId) 来获得视频的一些参数信息。这里propId 可以是 0 到 18 之间的任何整数。每一个数代表视频的一个属性,见表其中的一些值可以使用cap.set(propId,value) 来修改,value 就是你想要设置成的新值。例如,我可以使用 cap.get(3) 和 cap.get(4) 来查看每一帧的宽和高。默认情况下得到的值是 640X480。但是我可以使用 ret=cap.set(3,320)和 ret=cap.set(4,240) 来把宽和高改成 320X240。
cap.set(4, height)
codec = cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')
print(codec)

cap.set(cv2.CAP_PROP_FOURCC, codec)
fps =cap.get(cv2.CAP_PROP_FPS) #获取视频帧数
# cap.set(cv2.CAP_PROP_AUTOFOCUS, False)  # 禁止自动对焦
# cap.set(cv2.CAP_PROP_SETTINGS, 1) 
b_fps=time.time()  #后帧时间全局变量赋值
font = cv2.FONT_HERSHEY_SIMPLEX
while(True):
    # 读取一帧
    f_fps=time.time() #前帧时间
    fps_now=str(round(1/(f_fps-b_fps),2))   #求当前帧率
    b_fps=f_fps #后帧时间
    ret, frame = cap.read()
    frame = cv2.flip(frame,1)   ##图像左右颠倒
#########################################
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
    parameters =  cv2.aruco.DetectorParameters_create()
    dst1 = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    '''
    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
    '''

    #使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点坐标
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
    if ids is not None:

        #rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        # 估计每个标记的姿态并返回值rvet和tvec ---不同
        # from camera coeficcients
        #(rvec-tvec).any() # get rid of that nasty numpy value array error

#        aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.1) #绘制轴
#        aruco.drawDetectedMarkers(frame, corners) #在标记周围画一个正方形

        #for i in range(rvec.shape[0]):
           # cv2.aruco.drawAxis(frame, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.03)
            #cv2.aruco.drawDetectedMarkers(frame, corners)
        ###### DRAW ID #####
        cv2.putText(frame, "Id: " + str(ids), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)


    else:
        ##### DRAW "NO IDS" #####
        cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)


    # 显示结果框架
    #cv2.imshow("frame",frame)
##########################################
    cv2.putText(frame,'FPS:'+' '+fps_now,(10, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1 ,(0,0,255),2,cv2.LINE_AA)
    h,w=frame.shape[:2]
    print(h,w)
    print("获得的帧率:",fps)
    cv2.imshow('Camera_USB', frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break
cap.release()
cv2.destroyAllWindows()

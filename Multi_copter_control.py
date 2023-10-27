from tellocan import Tello
import time
#import os
import cv2
import sys
import threading
import numpy as np
import cv2.aruco as aruco
#import gc
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
#import socket
import math

def distance(goal_x,goal_z,now_x,now_z):
    
    dis=((goal_x-now_x)**2+(goal_z-now_z)**2)**0.5
    return dis

def get_bearing(current_x,current_z,goal_x,goal_z):
    x = current_x-goal_x
    z = current_z-goal_z
    bearing = 90.00 + math.atan2(-x, z) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing

def degree(pos_cam,goal,marker):
    a_vector=np.array([[marker[2]-pos_cam[2],marker[0]-pos_cam[0]]])        #現在與aruco
    b_vector=np.array([[goal[2]-marker[2],goal[0]-marker[0]]])              #目標與aruco
    c_vector=a_vector+b_vector
    angle1 = math.atan2(a_vector[1], a_vector[0])
    angle1 = int(angle1 * 180/math.pi)
    # print(angle1)
    angle2 = math.atan2(c_vector[1], c_vector[0])
    angle2 = int(angle2 * 180/math.pi)
    # print(angle2)
    if angle1*angle2 >= 0:
        included_angle = angle1-angle2
    else:
        included_angle = angle1 + angle2
        if included_angle > 180:
            included_angle = 360 - included_angle
    return included_angle

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

#def main():
#    global tello
#    global frame_read
#    tello = Tello()
#    tello.connect()
#    main_threading=threading.Thread(target=main_1)
#    video_threading=threading.Thread(target=just_video)
#    main_threading.start()
#    video_threading.start()
#    video_threading.join()
#    main_threading.join()
    
def main():
    global tello
    tello = Tello()
    tello.connect()
    tello.takeoff()      ##
    time.sleep(1)        ##
    tello.move_up(120)   ##
    FPS = 30
    
    speed = 20 
    # 建立連線---傳送連線請求
#        s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
#        s.connect(('127.0.0.1',8000))
    
    if not tello.connect():
        print("Tello not connected")
        sys.exit()
    
    if not tello.set_speed(speed):
        print("Not set speed to lowest possible")
        sys.exit()
    
    # In case streaming is on. This happens when we quit this program without the escape key.
    if not tello.streamoff():
        print("Could not stop video stream")
        sys.exit()
    
    if not tello.streamon():
        print("Could not start video stream")
        sys.exit()
    frame_read = tello.get_frame_read()
    #print ("Current battery is " + tello.get_battery())
#    frame_read = tello.get_frame_read()
    stop = False
    numot=0
    while not stop:
    
        frame = frame_read.frame
#        cv2.imshow("Video", frame)
        time.sleep(1)
        
        position_and_concrol(frame,numot)
#            print(self.pos_cam)
#            data = input(self.pos_cam)
#            s.send(data.encode())
        
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        time.sleep(1/FPS)
        numot+=1
    tello.streamoff()
    cv2.destroyAllWindows()
    sys.exit()

def power():
    if tello.get_battery < 30:
        tello.land()

#def video():
##        S = 60
#    

def position_and_concrol(frame,numot):
    ####1#####
    goal_x=154
    goal_z=237
    ####2#####
    turning_point_x=273
    turning_point_z=177
    ####
    
    ids0  = 0
    ids1  = 1
    ids2  = 2
    ids3  = 3
    ids4  = 4
    ids6  = 6
    ids7  = 7
    ids8  = 8
    ids9  = 9
    ids10 = 10
    ##x,z平面   y為高度
    marker_position_0=np.array([0,185,203])#0,185.5,203.4
    marker_position_1=np.array([196,185,0])#196.0,185.5,0
    marker_position_2=np.array([0,185,107])#0,185.5,107
    marker_position_3=np.array([377,185,0])#377.7,185.5
    marker_position_4=np.array([0,185,10 ])#0,185.5,10.45
    marker_position_6=np.array([0,105,204])
    marker_position_7=np.array([377,96,0 ])
    marker_position_8=np.array([0,104,110])
    marker_position_9=np.array([195,99,0 ])
    marker_position_10=np.array([0,105,12])
    pos_cam=[0 ,0 ,0 , 0]
    
    R_flip  = np.zeros((3,3), dtype=np.float32)
    R_flip[0,0] = 1.0
    R_flip[1,1] =-1.0
    R_flip[2,2] =-1.0

    
    marker_size  = 15 #- [cm]
    camera_matrix=np.array([
                            [865.11,     0,      484.579],
                            [0,        858.854,  377.154],
                            [0,           0,           1]])
    camera_distortion=np.array([[0.0529848,-0.792624,0.000128448,-0.0046813,1.01915]])
    
    #--- Define the aruco dictionary
    aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    parameters  = aruco.DetectorParameters_create()
    
    #-- Font for the text in the image
    font = cv2.FONT_HERSHEY_PLAIN
    gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
    
    #-- Find all the aruco markers in the image
    corners, ids, rejected = aruco.detectMarkers(gray, dictionary=aruco_dict, parameters=parameters,
                                                 cameraMatrix=camera_matrix, distCoeff=camera_distortion)
    if ids is not None:
        
        point=np.array([320,240])
        keep=9999
        indexed=0
        for i in range(0,ids.size):
            x=corners[i]
            x=np.squeeze(x)
            x=x[0]
            image_distance=point-x
            distance_point=((int(image_distance[0]))**2+(int(image_distance[1]))**2)**0.5   #dis=((x)**2+(y)**2)**0.5
            num=min(keep,distance_point)
            if keep>num:
                keep=num
                indexed=i
            if keep<num:
                keep=num
                indexed=i-1
        
        rvec,tvec,_=aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
        aruco.drawDetectedMarkers(frame, corners)
        
        
        #-- ret = [rvec, tvec, ?]
        #-- array of rotation and position of each marker in camera frame
        #-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
        #-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
        if ids[indexed]==ids0:
            
#            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

            #-- Unpack the output, get only the first
#            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            rvec=rvec[indexed]
            tvec=tvec[indexed]
            
            #-- Draw the detected marker and put a reference frame over it
#            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
       
            #-- Obtain the rotation matrix tag->camera
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T
    
    
            pos_camera = -R_tc*np.matrix(tvec).T
            transformMatrix_c0 = np.array([[ 0 , 0 ,1 ,  0  ],
                                           [ 0 , 1 ,0 ,185.5],
                                           [-1,  0 ,0 ,203.4],
                                           [ 0,  0 ,0 ,  1  ]])  # Transformation matrix
            pos_c0=np.matrix([[0],[0],[0],[0]], dtype=float)
            pos_c0[0:3,0]  = pos_camera[0:3,0]
            pos_c0[3,0]    = 1
            pos_cam = np.dot(transformMatrix_c0,pos_c0)
            
            pos_cam_x=int(pos_cam[0])
            pos_cam_z=int(pos_cam[2])
            
            roll_camera, yaw_camera, pitch_camera = rotationMatrixToEulerAngles(R_flip*R_tc)       #無人機與marker的角度
            
#            distance_dorne_marker=distance(marker_position_0[0],marker_position_0[2],pos_cam_x,pos_cam_z)
            if abs(goal_x-pos_cam_x)<30:
                 if abs(goal_z-pos_cam_z)<30:
                    tello.move_down(90)
                    time.sleep(1)
                    
#            if distance_dorne_marker>150:
#                distance_dorne_marker=int(distance_dorne_marker)
#                tello.move_forward(int(distance_dorne_marker/2))
#                time.sleep(2)
#                tello.rotate_clockwise()
            
            
            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_cam[0], pos_cam[1], pos_cam[2])
            cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
            
            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                math.degrees(yaw_camera))
            cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
            
        
            
        elif ids[indexed]==ids2:
            
            
#            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
    
            #-- Unpack the output, get only the first
#            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            rvec=rvec[indexed]
            tvec=tvec[indexed]
            
            #-- Draw the detected marker and put a reference frame over it
#            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
         
            #-- Obtain the rotation matrix tag->camera
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T
       
            #-- Now get Position and attitude f the camera respect to the marker
            pos_camera_2 = -R_tc*np.matrix(tvec).T
            transformMatrix_c2 = np.array([[0 , 0 , 1 ,  0  ],
                                          [ 0 , 1 , 0 ,185.5],
                                          [-1 , 0 , 0 , 107 ],
                                          [ 0 , 0 , 0 ,  1  ]])  # Transformation matrix
            pos_c2=np.matrix([[0],[0],[0],[0]], dtype=float)
            pos_c2[0:3,0]  = pos_camera_2[0:3,0]
            pos_c2[3,0]    = 1
            pos_cam = np.dot(transformMatrix_c2,pos_c2)
            
            pos_cam_x=int(pos_cam[0])
            pos_cam_z=int(pos_cam[2])
            
            roll_camera, yaw_camera, pitch_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
            
#            distance_dorne_marker=distance(marker_position_2[0],marker_position_2[2],pos_cam_x,pos_cam_z)    
            
            if abs(goal_x-pos_cam_x)<30:
                 if abs(goal_z-pos_cam_z)<30:
                    tello.move_down(90)
                    time.sleep(1)
                               
#            if distance_dorne_marker>150:
#                distance_dorne_marker=int(distance_dorne_marker)
#                tello.move_forward(int(distance_dorne_marker/2))
#                time.sleep(2)
    
            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_cam[0], pos_cam[1], pos_cam[2])
            cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
            
            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                math.degrees(yaw_camera))
            cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

        elif ids[indexed]==ids4:
            
            
#            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
    
            #-- Unpack the output, get only the first
#            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            rvec=rvec[indexed]
            tvec=tvec[indexed]
            #-- Draw the detected marker and put a reference frame over it
#            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
            
            
            #-- Obtain the rotation matrix tag->camera
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T
       
            #-- Now get Position and attitude f the camera respect to the marker
            pos_camera_4 = -R_tc*np.matrix(tvec).T
            transformMatrix_c4 = np.array([[0 , 0 ,1 ,  0  ],
                                          [ 0 , 1 ,0 ,185.8],
                                          [-1,  0 ,0 ,10.45],
                                          [ 0,  0 ,0 ,  1  ]])  # Transformation matrix
            pos_c4=np.matrix([[0],[0],[0],[0]], dtype=float)
            pos_c4[0:3,0]  = pos_camera_4[0:3,0]
            pos_c4[3,0]    = 1
            pos_cam = np.dot(transformMatrix_c4,pos_c4)
            
            pos_cam_x=int(pos_cam[0])
            pos_cam_z=int(pos_cam[2])
            
            roll_camera, yaw_camera, pitch_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
            
#            distance_dorne_marker=distance(marker_position_4[0],marker_position_4[2],pos_cam_x,pos_cam_z)
            
            if abs(goal_x-pos_cam_x)<30:
                 if abs(goal_z-pos_cam_z)<30:
                    tello.move_down(90)
                    time.sleep(1)

            
#            if distance_dorne_marker>100:
#                distance_dorne_marker=int(distance_dorne_marker)
#                tello.move_forward(int(distance_dorne_marker/2))
#                time.sleep(2)
    
            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_cam[0], pos_cam[1], pos_cam[2])
            cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
            
            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                math.degrees(yaw_camera))
            cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
            
        elif ids[indexed]==ids1:
            
            
#            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
    
            #-- Unpack the output, get only the first
#            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            rvec=rvec[indexed]
            tvec=tvec[indexed]
    
            #-- Draw the detected marker and put a reference frame over it
#            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
    
    
            #-- Obtain the rotation matrix tag->camera
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T
       
            #-- Now get Position and attitude f the camera respect to the marker
            pos_camera_1 = -R_tc*np.matrix(tvec).T
            transformMatrix_c1 = np.array([[1 , 0 ,0 , 196 ],
                                          [ 0 , 1 ,0 ,185.7],
                                          [ 0,  0 ,1 ,  0  ],
                                          [ 0,  0 ,0 ,  1  ]])  # Transformation matrix
            pos_c1=np.matrix([[0],[0],[0],[0]], dtype=float)
            pos_c1[0:3,0]  = pos_camera_1[0:3,0]
            pos_c1[3,0]    = 1
            pos_cam = np.dot(transformMatrix_c1,pos_c1)
            pos_cam_x=int(pos_cam[0])
            pos_cam_z=int(pos_cam[2])
            
            roll_camera, yaw_camera, pitch_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
            
#            distance_dorne_marker=distance(marker_position_1[0],marker_position_1[2],pos_cam_x,pos_cam_z)
            
            if abs(goal_x-pos_cam_x)<30:
                 if abs(goal_z-pos_cam_z)<30:
                    tello.move_down(90)
                    time.sleep(1)

#            if distance_dorne_marker>150:
#                distance_dorne_marker=int(distance_dorne_marker)
#                tello.move_forward(int(distance_dorne_marker/2))
#                time.sleep(2)
    
            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_cam[0], pos_cam[1], pos_cam[2])
            cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
            
            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                math.degrees(yaw_camera))
            cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
        
        elif ids[indexed]==ids3:
            
            
#            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
    
            #-- Unpack the output, get only the first
#            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            rvec=rvec[indexed]
            tvec=tvec[indexed]
    
            #-- Draw the detected marker and put a reference frame over it
#            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
    
            #-- Obtain the rotation matrix tag->camera
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T
       
            #-- Now get Position and attitude f the camera respect to the marker
            pos_camera_3 = -R_tc*np.matrix(tvec).T
            transformMatrix_c3 = np.array([[1 , 0 , 0 ,377.7],
                                          [ 0 , 1 , 0 ,185.4],
                                          [ 0,  0 , 1 ,  0  ],
                                          [ 0,  0 , 0 ,  1  ]])  # Transformation matrix
            pos_c3=np.matrix([[0],[0],[0],[0]], dtype=float)
            pos_c3[0:3,0]  = pos_camera_3[0:3,0]
            pos_c3[3,0]    = 1
            pos_cam = np.dot(transformMatrix_c3,pos_c3)
            
            pos_cam_x=int(pos_cam[0])
            pos_cam_z=int(pos_cam[2])
            
            roll_camera, yaw_camera, pitch_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
                
#            distance_dorne_marker=distance(marker_position_3[0],marker_position_3[2],pos_cam_x,pos_cam_z)
            
            if abs(goal_x-pos_cam_x)<30:
                 if abs(goal_z-pos_cam_z)<30:
                    tello.move_down(90)
                    time.sleep(1)
                    
#            if distance_dorne_marker>150:
#                distance_dorne_marker=int(distance_dorne_marker)
#                tello.move_forward(int(distance_dorne_marker/2))
    
            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_cam[0], pos_cam[1], pos_cam[2])
            cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
            
            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                math.degrees(yaw_camera))
            cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
        
        elif ids[indexed]==ids6:
            
            
#            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
    
            #-- Unpack the output, get only the first
#            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            rvec=rvec[indexed]
            tvec=tvec[indexed]
    
#            -- Draw the detected marker and put a reference frame over it
#            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
    
            #-- Obtain the rotation matrix tag->camera
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T
       
            #-- Now get Position and attitude f the camera respect to the marker
            pos_camera_c6 = -R_tc*np.matrix(tvec).T
            transformMatrix_c6 = np.array([[0 , 0 , 1 ,  0  ],
                                           [ 0 , 1 , 0 ,104.4],
                                           [-1,  0 , 0 ,204.3],
                                           [ 0,  0 , 0 ,  1  ]])  # Transformation matrix
            pos_c6=np.matrix([[0],[0],[0],[0]], dtype=float)
            pos_c6[0:3,0]  = pos_camera_c6[0:3,0]
            pos_c6[3,0]    = 1
            pos_cam = np.dot(transformMatrix_c6,pos_c6)
            
            pos_cam_x=int(pos_cam[0])
            pos_cam_y=int(pos_cam[1])
            pos_cam_z=int(pos_cam[2])
            
            roll_camera, yaw_camera, pitch_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
                
#            distance_dorne_marker=distance(marker_position_6[0],marker_position_6[2],pos_cam_x,pos_cam_z)
            
            if abs(goal_x-pos_cam_x)<20:
                 if abs(goal_z-pos_cam_z)<20:
                     if abs(marker_position_6[1]-pos_cam_y)>30:
                         tello.move_down(90)
                         time.sleep(1)
                     else:
                         tello.land()
                         tello.streamoff()
                         cv2.destroyAllWindows()
                         sys.exit()
    
            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_cam[0], pos_cam[1], pos_cam[2])
            cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
            
            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                math.degrees(yaw_camera))
            cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
            
        elif ids[indexed]==ids8:
            
            
#            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
    
            #-- Unpack the output, get only the first
#            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            rvec=rvec[indexed]
            tvec=tvec[indexed]
    
            #-- Draw the detected marker and put a reference frame over it
#            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
    
            #-- Obtain the rotation matrix tag->camera
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T
       
            #-- Now get Position and attitude f the camera respect to the marker
            pos_camera_8 = -R_tc*np.matrix(tvec).T
            transformMatrix_c8 = np.array([[0 , 0 , 1 ,  0  ],
                                           [ 0 , 1 , 0 ,103.5],
                                           [-1,  0 , 0 ,109.7],
                                           [ 0,  0 , 0 ,  1  ]])  # Transformation matrix
            pos_c8=np.matrix([[0],[0],[0],[0]], dtype=float)
            pos_c8[0:3,0]  = pos_camera_8[0:3,0]
            pos_c8[3,0]    = 1
            pos_cam = np.dot(transformMatrix_c8,pos_c8)
            
            pos_cam_x=int(pos_cam[0])
            pos_cam_y=int(pos_cam[1])
            pos_cam_z=int(pos_cam[2])
            
            roll_camera, yaw_camera, pitch_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
                
#            distance_dorne_marker=distance(marker_position_8[0],marker_position_8[2],pos_cam_x,pos_cam_z)
            
            if abs(goal_x-pos_cam_x)<20:
                 if abs(goal_z-pos_cam_z)<20:
                    if abs(marker_position_8[1]-pos_cam_y)>30:
                         tello.move_down(90)
                         time.sleep(1)
                    else:
                         tello.land()
                         tello.streamoff()
                         cv2.destroyAllWindows()
                         sys.exit()
  
    
            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_cam[0], pos_cam[1], pos_cam[2])
            cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
            
            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                math.degrees(yaw_camera))
            cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
            
        elif ids[indexed]==ids10:
            
            
#            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
    
            #-- Unpack the output, get only the first
#            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            rvec=rvec[indexed]
            tvec=tvec[indexed]
        
            #-- Draw the detected marker and put a reference frame over it
#            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
    
            #-- Obtain the rotation matrix tag->camera
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T
       
            #-- Now get Position and attitude f the camera respect to the marker
            pos_camera_10 = -R_tc*np.matrix(tvec).T
            transformMatrix_c10 = np.array([[0 , 0 , 1 ,  0  ],
                                           [ 0 , 1 , 0 ,104.9],
                                           [-1,  0 , 0 , 11.6],
                                           [ 0,  0 , 0 ,  1  ]])  # Transformation matrix
            pos_c10=np.matrix([[0],[0],[0],[0]], dtype=float)
            pos_c10[0:3,0]  = pos_camera_10[0:3,0]
            pos_c10[3,0]    = 1
            pos_cam = np.dot(transformMatrix_c10,pos_c10)
            
            pos_cam_x=int(pos_cam[0])
            pos_cam_y=int(pos_cam[1])
            pos_cam_z=int(pos_cam[2])
            
            roll_camera, yaw_camera, pitch_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
                
#            distance_dorne_marker=distance(marker_position_10[0],marker_position_10[2],pos_cam_x,pos_cam_z)
            
            if abs(goal_x-pos_cam_x)<20:
                 if abs(goal_z-pos_cam_z)<20:
                    if abs(marker_position_10[1]-pos_cam_y)>30:
                         tello.move_down(90)
                         time.sleep(1)
                    else:
                         tello.land()
                         tello.streamoff()
                         cv2.destroyAllWindows()
                         sys.exit()
    
            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_cam[0], pos_cam[1], pos_cam[2])
            cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
            
            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                math.degrees(yaw_camera))
            cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
            
        elif ids[indexed]==ids7:
            
            
#            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
    
            #-- Unpack the output, get only the first
#            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            rvec=rvec[indexed]
            tvec=tvec[indexed]
    
            #-- Draw the detected marker and put a reference frame over it
#            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
    
            #-- Obtain the rotation matrix tag->camera
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T
       
            #-- Now get Position and attitude f the camera respect to the marker
            pos_camera_7 = -R_tc*np.matrix(tvec).T
            transformMatrix_c7 = np.array([[1 , 0 , 0 , 377],
                                          [ 0 , 1 , 0 , 96 ],
                                          [ 0,  0 , 1 ,  0 ],
                                          [ 0,  0 , 0 ,  1 ]])  # Transformation matrix
            pos_c7=np.matrix([[0],[0],[0],[0]], dtype=float)
            pos_c7[0:3,0]  = pos_camera_7[0:3,0]
            pos_c7[3,0]    = 1
            pos_cam = np.dot(transformMatrix_c7,pos_c7)
            
            pos_cam_x=int(pos_cam[0])
            pos_cam_y=int(pos_cam[1])
            pos_cam_z=int(pos_cam[2])
            
            roll_camera, yaw_camera, pitch_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
                
#            distance_dorne_marker=distance(marker_position_7[0],marker_position_7[2],pos_cam_x,pos_cam_z)
            
            if abs(goal_x-pos_cam_x)<20:
                 if abs(goal_z-pos_cam_z)<20:
                    if abs(marker_position_7[1]-pos_cam_y)>30:
                         tello.move_down(100)
                         time.sleep(1)
                    else:
                         tello.land()
                         tello.streamoff()
                         cv2.destroyAllWindows()
                         sys.exit()
    
            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_cam[0], pos_cam[1], pos_cam[2])
            cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
            
            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                math.degrees(yaw_camera))
            cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
            
        elif ids[indexed]==ids9:
            
            
#            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
    
            #-- Unpack the output, get only the first
#            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            rvec=rvec[indexed]
            tvec=tvec[indexed]
    
            #-- Draw the detected marker and put a reference frame over it
#            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
    
            #-- Obtain the rotation matrix tag->camera
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T
       
            #-- Now get Position and attitude f the camera respect to the marker
            pos_camera_9 = -R_tc*np.matrix(tvec).T
            transformMatrix_c9 = np.array([[1 , 0 , 0 ,195.2],
                                          [ 0 , 1 , 0 ,101.3 ],
                                          [ 0,  0 , 1 ,  0  ],
                                          [ 0,  0 , 0 ,  1  ]])  # Transformation matrix
            pos_c9=np.matrix([[0],[0],[0],[0]], dtype=float)
            pos_c9[0:3,0]  = pos_camera_9[0:3,0]
            pos_c9[3,0]    = 1
            pos_cam = np.dot(transformMatrix_c9,pos_c9)
            
            pos_cam_x=int(pos_cam[0])
            pos_cam_y=int(pos_cam[1])
            pos_cam_z=int(pos_cam[2])
            
            roll_camera, yaw_camera, pitch_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
                
#            distance_dorne_marker=distance(marker_position_9[0],marker_position_9[2],pos_cam_x,pos_cam_z)
            
            if abs(goal_x-pos_cam_x)<20:
                 if abs(goal_z-pos_cam_z)<20:
                    if abs(marker_position_9[1]-pos_cam_y)>30:
                         tello.move_down(90)
                         time.sleep(1)
                    else:
                         tello.land()
                         tello.streamoff()
                         cv2.destroyAllWindows()
                         sys.exit()
    
            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_cam[0], pos_cam[1], pos_cam[2])
            cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
            
            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                math.degrees(yaw_camera))
            cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
        
        
        time.sleep(2)
        yaw=yaw_camera
#        goal=np.array([[154,0,237]])
        yaw=int(math.degrees(yaw))
        time.sleep(5)
        point_x=pos_cam[0]
#        point_y=pos_cam[1]
        point_z=pos_cam[2]
        point_x=int(point_x)
        point_z=int(point_z)
        tello.rotate_counter_clockwise(yaw)
        time.sleep(1)
        
#        elif distance_dorne_marker<150:
        
        if numot<1:
            if (ids[indexed]%2)==0:
                if turning_point_x>point_x:
                    if turning_point_z>point_z:
                       y=turning_point_z-point_z
                       x=-(turning_point_x-point_x)
                       z=0
                       speed=20
                       tello.go_xyz_speed(x,y,z,speed)
                       time.sleep(5)
                        
                if turning_point_x>point_x:
                    if turning_point_z<point_z:
                       y=turning_point_z-point_z
                       x=-(turning_point_x-point_x)
                       z=0
                       speed=20
                       tello.go_xyz_speed(x,y,z,speed)
                       time.sleep(5)
                               
                if turning_point_x<point_x:#
                    if turning_point_z>point_z:
                       y=turning_point_z-point_z
                       x=-(turning_point_x-point_x)
                       z=0
                       speed=20
                       tello.go_xyz_speed(x,y,z,speed)
                       time.sleep(5)
        
                if turning_point_x<point_x: #V
                    if turning_point_z<point_z:
                       y=(turning_point_z-point_z)
                       x=-(turning_point_x-point_x)
                       z=0
                       speed=20
                       tello.go_xyz_speed(x,y,z,speed)
                       time.sleep(5)
                       
            elif (ids[indexed]%2)==1:
                if turning_point_x>point_x:
                    if turning_point_z>point_z:
                       x=-(turning_point_z-point_z)
                       y=-(turning_point_x-point_x)
                       z=0
                       speed=20
                       tello.go_xyz_speed(x,y,z,speed)
                       time.sleep(5)
    
                if turning_point_x>point_x:
                    if turning_point_z<point_z:
                       x=-(turning_point_z-point_z)
                       y=-(turning_point_x-point_x)
                       z=0
                       speed=20
                       tello.go_xyz_speed(x,y,z,speed)
                       time.sleep(5)
    
                if turning_point_x<point_x:
                    if turning_point_z>point_z:
                       x=-(turning_point_z-point_z)
                       y=-(turning_point_x-point_x)
                       z=0
                       speed=20
                       tello.go_xyz_speed(x,y,z,speed)
                       time.sleep(5)
                       
                if turning_point_x<point_x:
                    if turning_point_z<point_z:
                       x=-(turning_point_z-point_z)
                       y=-(turning_point_x-point_x)
                       z=0
                       speed=20
                       tello.go_xyz_speed(x,y,z,speed)
                       time.sleep(5)
        
        elif numot>1:
            if (ids[indexed]%2)==0:
                if goal_x>point_x:
                    if goal_z>point_z:
                       y=goal_z-point_z
                       x=-(goal_x-point_x)
                       z=0
                       speed=20
                       tello.go_xyz_speed(x,y,z,speed)
                       time.sleep(5)
                        
                if goal_x>point_x:
                    if goal_z<point_z:
                       y=goal_z-point_z
                       x=-(goal_x-point_x)
                       z=0
                       speed=20
                       tello.go_xyz_speed(x,y,z,speed)
                       time.sleep(5)
                               
                if goal_x<point_x:#
                    if goal_z>point_z:
                       y=goal_z-point_z
                       x=-(goal_x-point_x)
                       z=0
                       speed=20
                       tello.go_xyz_speed(x,y,z,speed)
                       time.sleep(5)
        
                if goal_x<point_x: #V
                    if goal_z<point_z:
                       y=(goal_z-point_z)
                       x=-(goal_x-point_x)
                       z=0
                       speed=20
                       tello.go_xyz_speed(x,y,z,speed)
                       time.sleep(5)
                       
            elif (ids[indexed]%2)==1:
                if goal_x>point_x:
                    if goal_z>point_z:
                       x=-(goal_z-point_z)
                       y=-(goal_x-point_x)
                       z=0
                       speed=20
                       tello.go_xyz_speed(x,y,z,speed)
                       time.sleep(5)
    
                if goal_x>point_x:
                    if goal_z<point_z:
                       x=-(goal_z-point_z)
                       y=-(goal_x-point_x)
                       z=0
                       speed=20
                       tello.go_xyz_speed(x,y,z,speed)
                       time.sleep(5)
    
                if goal_x<point_x:
                    if goal_z>point_z:
                       x=-(goal_z-point_z)
                       y=-(goal_x-point_x)
                       z=0
                       speed=20
                       tello.go_xyz_speed(x,y,z,speed)
                       time.sleep(5)
                       
                if goal_x<point_x:
                    if goal_z<point_z:
                       x=-(goal_z-point_z)
                       y=-(goal_x-point_x)
                       z=0
                       speed=20
                       tello.go_xyz_speed(x,y,z,speed)
                       time.sleep(5)
        
                   
    elif ids is None :
        tello.rotate_clockwise(20)
        
#    numot+=1
    print(numot)
    
def just_video():
    FPS=30  
    frame_read = tello.get_frame_read()
    stop = False
    while not stop:
    
        frame = frame_read.frame
        cv2.imshow("Video", frame)

        time.sleep(1/FPS)
if __name__ == '__main__':
    main()


    
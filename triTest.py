from scipy.spatial import distance as dist
from imutils.video import VideoStream
from imutils import face_utils
from threading import Thread
import numpy as np
import argparse
import imutils
import time
import dlib
import cv2
import os
import serial

ser = serial.Serial('/dev/ttyACM0', 115200)
ser.reset_input_buffer()



while True:
    if(ser.in_waiting > 0):  
        #insert code here that determines that user is drowsy. if user drowsy then sigToArduino = D.
        def alarm(msg):
            global alarm_status
            #global alarm_status2
            global saying

            while alarm_status:
                print('audio')# Audio alert played from speaker as drowsiness is detected
                s = 'espeak "'+msg+'"'
                os.system(s)

        def eye_aspect_ratio(eye):# calculating EAR of an eye
            A = dist.euclidean(eye[1], eye[5])# vertical
            B = dist.euclidean(eye[2], eye[4])# vertical 2
            C = dist.euclidean(eye[0], eye[3])# horiztonal

            ear = (A + B) / (2.0 * C)
            return ear

        def final_ear(shape):# calculates the overall EAR with both eyes in consideration
            (lStart, lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
            (rStart, rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]
                           
            leftEye = shape[lStart:lEnd]
            rightEye = shape[rStart:rEnd]

            leftEAR = eye_aspect_ratio(leftEye)
            rightEAR = eye_aspect_ratio(rightEye)

            ear = (leftEAR + rightEAR) / 2.0
            return (ear, leftEye, rightEye)


        ap = argparse.ArgumentParser()#calling the rpi cam index
        ap.add_argument("-w", "--webcam", type=int, default=0,help="index of webcam on system")
        args = vars(ap.parse_args())

        EYE_AR_THRESH = 0.3
        EYE_AR_CONSEC_FRAMES = 10
        alarm_status = False
        saying = False
        COUNTER = 0
        TOTAL = 0

        # calls functions from libraries for face detection
        detector = dlib.get_frontal_face_detector()
        predictor = dlib.shape_predictor('shape_predictor_68_face_landmarks.dat')

        (lStart, lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
        (rStart, rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]

        # initializes webcam
        vs = VideoStream(src=args["webcam"]).start()
        time.sleep(1.0)


        while True:
            frame = vs.read()
            frame = imutils.resize(frame, width=600)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            rects = detector(gray, 0)

            for rect in rects:
                shape = predictor(gray, rect)
                shape = face_utils.shape_to_np(shape)

                eye = final_ear(shape)
                ear = eye[0]
                leftEye = eye [1]
                rightEye = eye[2]
            
                leftEAR = eye_aspect_ratio(leftEye)
                rightEAR = eye_aspect_ratio(rightEye)
            
                ear = (leftEAR + rightEAR) / 2.0


                leftEyeHull = cv2.convexHull(leftEye)
                rightEyeHull = cv2.convexHull(rightEye)
                cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
                cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)


                if ear < EYE_AR_THRESH:# eyes closed detection
                    COUNTER += 1

                    if COUNTER >= EYE_AR_CONSEC_FRAMES:# if eyes have been closed for 
                        TOTAL += 1
                        COUNTER = 0
                        #TESTING - VARIFICATION OF DROWSINESS FROM STEERING WHEEL
                        sigToArduino = 'D'
        
                        if (sigToArduino == 'D'):
                            ser.write(b"Drowsy\n")
                            SteeringWFindings = ser.readline()
                            print(SteeringWFindings)
                            if alarm_status == False:
                                alarm_status = True
                                t = Thread(target=alarm, args=('GHESINGH NAH',)) #DROWSINESS DETECTED
                                t.deamon = True
                                t.start()

                else:
                    COUNTER = 0 #delete
                    alarm_status  = False
            
                cv2.putText(frame, "BLINKs: {}".format(TOTAL), (10,50),
                        cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0,0, 255),2)
                cv2.putText(frame, "EAR: {:.2f}".format(ear), (10, 30),
                        cv2.FONT_HERSHEY_TRIPLEX , 0.7, (0, 0, 255), 2)
            
            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

        cv2.destroyAllWindows()
        vs.stop()
        
        
        
    
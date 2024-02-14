#!/usr/bin/env python
# -- coding: utf-8 --

import numpy as np
import cv2
from pathlib import Path
import os
import glob

class CameraCali(object):
    def __init__(self):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        wc = 8 ## 체스 보드 가로 패턴 개수 - 1
        hc = 6  ## 체스 보드 세로 패턴 개수 - 1
        objp = np.zeros((wc * hc, 3), np.float32)
        objp[:, :2] = np.mgrid[0:wc, 0:hc].T.reshape(-1, 2)

        objpoints = []
        imgpoints = [] 

        images = glob.glob(os.path.dirname(__file__) + '/checkerboard_jpg/*.jpg')

        for frame in images:
            img = cv2.imread(frame)
            self.gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) ## gray scale로 바꾸기

            ret, corners = cv2.findChessboardCorners(self.gray, (wc, hc), cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)  ## 체스 보드 찾기
            print(ret, end=' ')
            ## 만약 ret값이 False라면, 체스 보드 이미지의 패턴 개수를 맞게 했는지 확인하거나 (wc, hc)

            if ret == True:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(self.gray, corners, (10, 10), (-1, -1), criteria) ## Canny86 알고리즘으로
                imgpoints.append(corners2)

                ## 찾은 코너 점들을 이용해 체스 보드 이미지에 그려넣는다
                img = cv2.drawChessboardCorners(img, (wc, hc), corners2, ret)
                cv2.imshow('img', img)
                cv2.waitKey()
   
        ret, mtx, dist, rvecs, tvecs  = cv2.calibrateCamera(objpoints, imgpoints, self.gray.shape[::-1], None, None)

        #2차원 영상좌표
        points_2D = np.array([
                              (155, 330), 
        (210, 280),  
        (320, 280),  
        (415, 280),    
                           ], dtype="double")
                            
        #3차원 월드좌표
        points_3D = np.array([
                            (-1.0732, -0.17268, 0),      
                            (-1.6316, -0.17515, 0),        
                            (-1.6426, 0.39314, 0),        
                            (-1.6522, 0.68096, 0)          
                            ], dtype="double") 

        self.cameraMatrix = mtx
        
        self.dist_coeffs = np.array([0,0,0,0,0])

        retval, rvec, tvec = cv2.solvePnP(points_3D, points_2D, self.cameraMatrix, self.dist_coeffs, rvec=None, tvec=None, useExtrinsicGuess=None, flags=None)

        self.rvec, _ = cv2.Rodrigues(rvec)
        self.tvec = tvec

        self.intrisic = np.append(self.cameraMatrix, [[0,0,0]], axis=0)
        self.intrisic = np.append(self.intrisic, [[0],[0],[0],[1]], axis=1)

        self.intrinsic = self.intrisic # 내부 파라미터
        extrinsic = np.append(self.rvec, self.tvec, axis = 1) # 외부 파라미터
        self.extrinsic = np.append(extrinsic, [[0,0,0,1]], axis=0)

        
        print()
        print("intrinsic: ", end='\n')
        print(self.intrinsic)
        print("extrinsic: ", end='\n')
        print(self.extrinsic)
        print("R:", end='\n')
        print(self.rvec)
        print("t:", end='\n')
        print(self.tvec)

if __name__ == '__main__':
    cam_cali = CameraCali()
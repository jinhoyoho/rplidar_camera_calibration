#!/usr/bin/env python
# -- coding: utf-8 --

import numpy as np
import cv2
from pathlib import Path
import os
import glob


class CameraCali(object):
    def __init__(self):
        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        wc = 8  # the number of horizontal checkerboard patterns - 1
        hc = 6  # the number of vertical checkerboard patterns - 1
        objp = np.zeros((wc * hc, 3), np.float32)
        objp[:, :2] = np.mgrid[0:wc, 0:hc].T.reshape(-1, 2)

        objpoints = []
        imgpoints = []

        # import images in directory
        images = glob.glob(os.path.dirname(__file__) +
                           '/checkerboard_jpg/*.jpg')

        for frame in images:
            img = cv2.imread(frame)
            self.gray = cv2.cvtColor(
                img, cv2.COLOR_BGR2GRAY)  # change to gray scale

            # find the checkerboard
            ret, corners = cv2.findChessboardCorners(
                self.gray, (wc, hc), cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)  # 체스 보드 찾기

            # if ret is False, please check your checkerboard (wc, hc)
            print(ret, end=' ')

            if ret == True:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(
                    self.gray, corners, (10, 10), (-1, -1), criteria)
                imgpoints.append(corners2)

                # draw images using corner points
                img = cv2.drawChessboardCorners(img, (wc, hc), corners2, ret)
                cv2.imshow('img', img)
                cv2.waitKey()

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, self.gray.shape[::-1], None, None)

        # 2D-image coordination
        points_2D = np.array([
            (155, 330),
            (210, 280),
            (320, 280),
            (415, 280),
        ], dtype="double")

        # 3D-World coordinations correspond to 2D-image coordinations
        points_3D = np.array([
                            (-1.0732, -0.17268, 0),
                            (-1.6316, -0.17515, 0),
                            (-1.6426, 0.39314, 0),
                            (-1.6522, 0.68096, 0)
        ], dtype="double")

        self.cameraMatrix = mtx

        # distortion coeffiecient in camera matrix
        self.dist_coeffs = np.array([0, 0, 0, 0, 0])

        # using solvePnP to calculate R, t
        retval, rvec, tvec = cv2.solvePnP(points_3D, points_2D, self.cameraMatrix,
                                          self.dist_coeffs, rvec=None, tvec=None, useExtrinsicGuess=None, flags=None)

        self.rvec, _ = cv2.Rodrigues(rvec)
        self.tvec = tvec

        # express homogeneous coordinate
        self.intrisic = np.append(self.cameraMatrix, [[0, 0, 0]], axis=0)
        self.intrisic = np.append(self.intrisic, [[0], [0], [0], [1]], axis=1)

        # intrinsic parameter
        self.intrinsic = self.intrisic
        # extrinic parameter
        extrinsic = np.append(self.rvec, self.tvec,
                              axis=1)
        self.extrinsic = np.append(extrinsic, [[0, 0, 0, 1]], axis=0)

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
